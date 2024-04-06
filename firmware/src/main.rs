#![no_std]
#![no_main]

// TODO - feature gates
use {defmt_rtt as _, panic_probe as _};

use defmt::{debug, error, info};
use embassy_boot_stm32::{AlignedBuffer, FirmwareUpdater, FirmwareUpdaterConfig};
use embassy_embedded_hal::{adapter::BlockingAsync, shared_bus::asynch::i2c::I2cDevice};
use embassy_executor::Spawner;
use embassy_net::{udp, EthernetAddress, Ipv4Address, Stack, StackResources};
use embassy_stm32::{
    bind_interrupts, dma,
    eth::{self, generic_smi::GenericSMI, Ethernet, PacketQueue},
    flash::{Flash, WRITE_SIZE},
    gpio::{Level, Output, Speed},
    i2c::{self, I2c},
    peripherals,
    rng::{self, Rng},
    time::Hertz,
    usart::{self, Uart},
    wdg::IndependentWatchdog,
    Config,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::{
    common::BootloaderState,
    display::Display,
    drivers::pms5003::DefaultPms5003,
    drivers::s8lp::DefaultS8Lp,
    drivers::sgp41::DefaultSgp41,
    drivers::sht31::DefaultSht31,
    reset_reason::ResetReason,
    tasks::data_manager::{data_manager_task, DataManagerTaskState},
    tasks::display::{display_task, DisplayTaskState, MessageChannel as DisplayMessageChannel},
    tasks::led_heartbeat::{led_heartbeat_task, LedHeartbeatTaskState},
    tasks::net::{net_task, Device as NetDevice},
    tasks::pms5003::{pms5003_task, Pms5003TaskState},
    tasks::s8lp::{s8lp_task, S8LpTaskState},
    tasks::sgp41::{sgp41_task, Sgp41TaskState},
    tasks::sht31::{self, sht31_task, Sht31TaskState},
};

mod common;
mod config;
mod display;
mod drivers;
mod reset_reason;
mod tasks;
mod util;

pub mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

static I2C_BUS: StaticCell<
    Mutex<
        NoopRawMutex,
        I2c<'static, peripherals::I2C2, peripherals::DMA1_CH7, peripherals::DMA1_CH2>,
    >,
> = StaticCell::new();
static SHT31_RAW_MEASUREMENT_CHANNEL: StaticCell<sht31::RawMeasurementChannel> = StaticCell::new();
static DISPLAY_MSG_CHANNEL: StaticCell<DisplayMessageChannel> = StaticCell::new();
static DM_MEASUREMENT_CHANNEL: StaticCell<common::MeasurementChannel> = StaticCell::new();

static PACKETS: StaticCell<PacketQueue<8, 8>> = StaticCell::new();
static STACK: StaticCell<Stack<NetDevice>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
static UDP_STORAGE: StaticCell<UdpSocketStorage<{ config::BCAST_PROTO_SOCKET_BUFFER_LEN }>> =
    StaticCell::new();

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    ETH => eth::InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[allow(const_item_mutation)]
    let reset_reason = ResetReason::read_and_clear(&mut embassy_stm32::pac::RCC);

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(12_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV12,
            mul: PllMul::MUL240,
            divp: Some(PllPDiv::DIV2), // 12MHz / 12 * 240 / 2 = 120Mhz
            divq: None,
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    let p = embassy_stm32::init(config);

    let mut wdt = IndependentWatchdog::new(p.IWDG, config::WATCHDOG_TIMEOUT_MS * 1_000);
    wdt.unleash();

    // active-low
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    led.set_high();

    info!("Setup: FLASH");
    Timer::after_millis(10).await;
    let flash_layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let state_flash = Mutex::new(BlockingAsync::new(flash_layout.bank1_region2));
    let dfu_flash = Mutex::new(BlockingAsync::new(flash_layout.bank1_region3));
    let fw_updater_config = FirmwareUpdaterConfig::from_linkerfile(&dfu_flash, &state_flash);
    let mut magic = AlignedBuffer([0; WRITE_SIZE]);
    let mut fw_updater = FirmwareUpdater::new(fw_updater_config, &mut magic.0);
    let bootloader_state = BootloaderState::from(fw_updater.get_state().await.unwrap());

    info!("############################################################");
    info!(
        "{} {} ({})",
        crate::built_info::PKG_NAME,
        config::FIRMWARE_VERSION,
        crate::built_info::PROFILE
    );
    info!("Build date: {}", crate::built_info::BUILT_TIME_UTC);
    info!("Compiler: {}", crate::built_info::RUSTC_VERSION);
    if let Some(gc) = crate::built_info::GIT_COMMIT_HASH {
        info!("Commit: {}", gc);
    }
    info!("Serial number: {:X}", util::read_device_serial_number());
    info!(
        "Device ID: 0x{:X} ({})",
        config::DEVICE_ID,
        config::DEVICE_ID
    );
    info!("IP address: {}", config::IP_CIDR.address());
    info!(
        "MAC address: {}",
        EthernetAddress::from_bytes(&config::MAC_ADDRESS)
    );
    info!("Broadcast protocol port: {}", config::BROADCAST_PORT);
    info!(
        "Broadcast protocol address: {}",
        Ipv4Address(config::BROADCAST_ADDRESS)
    );
    info!("Device protocol port: {}", config::DEVICE_PORT);
    info!("Reset reason: {}", reset_reason);
    info!("Bootloader state: {}", bootloader_state);
    info!("############################################################");

    info!(
        "Setup: startup delay {} seconds",
        config::STARTUP_DELAY_SECONDS
    );
    for _ in 0..config::STARTUP_DELAY_SECONDS {
        for _ in 0..10 {
            wdt.pet();
            led.toggle();
            Timer::after_millis(100).await;
        }
    }
    led.set_low();

    // Setup channels
    let sht31_raw_measurement_channel =
        SHT31_RAW_MEASUREMENT_CHANNEL.init(sht31::RawMeasurementChannel::new());
    let display_msg_channel = DISPLAY_MSG_CHANNEL.init(DisplayMessageChannel::new());
    let dm_measurement_channel = DM_MEASUREMENT_CHANNEL.init(common::MeasurementChannel::new());

    info!("Setup: S8 LP");
    let mut s8lp_serial_config = usart::Config::default();
    s8lp_serial_config.baudrate = 9600;
    let s8lp_serial = Uart::new(
        p.USART3,
        p.PD9,
        p.PD8,
        Irqs,
        dma::NoDma,
        p.DMA1_CH1,
        s8lp_serial_config,
    )
    .unwrap();
    let s8lp_state = S8LpTaskState::new(
        DefaultS8Lp::new(s8lp_serial),
        dm_measurement_channel.sender(),
    );

    info!("Setup: PMS5003");
    let mut pms_serial_config = usart::Config::default();
    pms_serial_config.baudrate = 9600;
    let pms_serial = Uart::new(
        p.USART2,
        p.PD6,
        p.PD5,
        Irqs,
        dma::NoDma,
        p.DMA1_CH5,
        pms_serial_config,
    )
    .unwrap();
    let pms_state = Pms5003TaskState::new(
        DefaultPms5003::new(pms_serial).await.unwrap(),
        dm_measurement_channel.sender(),
    );

    // Shared I2C1 bus, data is only shared between tasks on the same executor
    info!("Setup: I2C2");
    let i2c = I2c::new(
        p.I2C2,
        p.PF1,
        p.PF0,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH2,
        Hertz(100_000),
        Default::default(),
    );
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C_BUS.init(i2c_bus);

    info!("Setup: SHT31");
    let sht31_i2c = I2cDevice::new(i2c_bus);
    let mut sht31 = DefaultSht31::new(sht31_i2c).await.unwrap();
    debug!(
        "SHT31: serial number {}",
        sht31.serial_number().await.unwrap()
    );
    debug!("SHT31: status {}", sht31.status().await.unwrap().bits());
    let sht31_state = Sht31TaskState::new(
        sht31,
        dm_measurement_channel.sender(),
        sht31_raw_measurement_channel.sender(),
    );

    info!("Setup: SGP41");
    let sgp41_i2c = I2cDevice::new(i2c_bus);
    let mut sgp41 = DefaultSgp41::new(sgp41_i2c).await.unwrap();
    debug!(
        "SGP41: serial number {}",
        sgp41.serial_number().await.unwrap()
    );
    let sgp41_state = Sgp41TaskState::new(
        sgp41,
        dm_measurement_channel.sender(),
        sht31_raw_measurement_channel.receiver(),
    );

    info!("Setup: SH1106");
    let sh_i2c = I2cDevice::new(i2c_bus);
    let display = Display::new(sh_i2c).await.unwrap();
    let display_state = DisplayTaskState::new(display, display_msg_channel.receiver());

    info!("Setup: ETH");
    let eth: Ethernet<'_, peripherals::ETH, GenericSMI> = Ethernet::new(
        PACKETS.init(PacketQueue::<8, 8>::new()),
        p.ETH,
        Irqs,
        p.PA1,  // ref_clk
        p.PA2,  // mdio
        p.PC1,  // mdc
        p.PA7,  // crs
        p.PC4,  // rx_d0
        p.PC5,  // rx_d1
        p.PG13, // tx_d0
        p.PG14, // tx_d1
        p.PG11, // tx_en
        GenericSMI::new(0),
        config::MAC_ADDRESS,
    );

    info!("Setup: TCP/IP");
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    let _ = rng.async_fill_bytes(&mut seed).await;
    let seed = u64::from_le_bytes(seed);

    let ip_config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: config::IP_CIDR,
        dns_servers: Default::default(),
        gateway: None,
    });
    let stack = &*STACK.init(Stack::new(
        eth,
        ip_config,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));

    let udp_storage = UDP_STORAGE.init(UdpSocketStorage::new());
    let bcast_socket = udp::UdpSocket::new(
        stack,
        &mut udp_storage.rx_metadata,
        &mut udp_storage.rx_buffer,
        &mut udp_storage.tx_metadata,
        &mut udp_storage.tx_buffer,
    );

    wdt.pet();
    spawner.spawn(net_task(stack)).unwrap();
    // TODO use embassy_futures select with timeout
    stack.wait_config_up().await;

    let dm_state = DataManagerTaskState::new(
        dm_measurement_channel.receiver(),
        display_msg_channel.sender(),
        bcast_socket,
    );

    let hb_state = LedHeartbeatTaskState::new(wdt, led);

    spawner.spawn(led_heartbeat_task(hb_state)).unwrap();
    spawner.spawn(data_manager_task(dm_state)).unwrap();
    spawner.spawn(display_task(display_state)).unwrap();
    spawner.spawn(s8lp_task(s8lp_state)).unwrap();
    spawner.spawn(sht31_task(sht31_state)).unwrap();
    spawner.spawn(sgp41_task(sgp41_state)).unwrap();
    spawner.spawn(pms5003_task(pms_state)).unwrap();

    info!(">>> Initialized <<<");
}

struct UdpSocketStorage<const BL: usize> {
    pub rx_buffer: [u8; BL],
    pub rx_metadata: [udp::PacketMetadata; 1],
    pub tx_buffer: [u8; BL],
    pub tx_metadata: [udp::PacketMetadata; 1],
}

impl<const BL: usize> UdpSocketStorage<BL> {
    pub const fn new() -> Self {
        UdpSocketStorage {
            rx_buffer: [0; BL],
            rx_metadata: [udp::PacketMetadata::EMPTY; 1],
            tx_buffer: [0; BL],
            tx_metadata: [udp::PacketMetadata::EMPTY; 1],
        }
    }
}
