#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::*;
use embassy_stm32::{
    dma,
    flash::{Flash, BANK1_REGION3},
    usart::{self, UartTx},
};
use embassy_sync::blocking_mutex::Mutex;
use log::{debug, error, info};

mod logger;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    /*
    #[cfg(feature = "debug")]
    {
        for _ in 0..10000000 {
            cortex_m::asm::nop();
        }
    }
    */

    let mut serial_config = usart::Config::default();
    serial_config.baudrate = 115_200;
    let logger_serial = UartTx::new(p.USART6, p.PC6, dma::NoDma, serial_config).unwrap();
    unsafe {
        logger::init_logger(logger_serial);
    }

    info!("************************************************************");
    info!(
        "{} {} ({})",
        crate::built_info::PKG_NAME,
        crate::built_info::PKG_VERSION,
        crate::built_info::PROFILE
    );
    info!("Build date: {}", crate::built_info::BUILT_TIME_UTC);
    info!("Compiler: {}", crate::built_info::RUSTC_VERSION);
    if let Some(gc) = crate::built_info::GIT_COMMIT_HASH {
        info!("Commit: {}", gc);
    }
    info!("************************************************************");

    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();

    // TODO testing things out
    // NOTE: can forcefully set the bootloader state to Boot
    //let boot_state: [u8; 4] = [0xD0, 0xD0, 0xD0, 0xD0];
    //layout.bank1_region2.blocking_write(0, &boot_state).unwrap();

    let flash_region2 = Mutex::new(RefCell::new(layout.bank1_region2));
    let flash_region3 = Mutex::new(RefCell::new(layout.bank1_region3));

    // active, DFU, state partitions
    let config =
        BootLoaderConfig::from_linkerfile_blocking(&flash_region3, &flash_region3, &flash_region2);
    let active_offset = config.active.offset();
    let bl = BootLoader::prepare::<_, _, _, 2048>(config);
    debug!("Bootloader state: {:?}", bl.state);

    debug!(
        "Booting to 0x{:X}, base = 0x{:X}, offset = 0x{:X}",
        BANK1_REGION3.base + active_offset,
        BANK1_REGION3.base,
        active_offset
    );

    unsafe {
        logger::flush_disable_logger();

        compiler_fence(SeqCst);

        bl.load(BANK1_REGION3.base + active_offset);
    }

    /*
    // TODO just for testing without the firmware updater in the mix
    debug!("Booting to 0x{:X}", BANK1_REGION3.base,);
    let sp_ptr = BANK1_REGION3.base as *const u32;
    let sp = unsafe { core::ptr::read_volatile(sp_ptr) };
    let reset_vector_ptr = unsafe { sp_ptr.offset(1) };
    let reset_vector = unsafe { core::ptr::read_volatile(reset_vector_ptr) };
    debug!("sp = 0x{sp:X} rv = 0x{reset_vector:X}");
    unsafe {
        logger::flush_disable_logger();
        compiler_fence(SeqCst);
        let mut p = cortex_m::Peripherals::steal();
        p.SCB.invalidate_icache();
        p.SCB.vtor.write(BANK1_REGION3.base);
        cortex_m::asm::bootload(BANK1_REGION3.base as *const u32)
    }
    */
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    error!("HardFault");
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[allow(unused_variables)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    error!("!!!!!!!!!!!\npanic\n{}", info);
    cortex_m::asm::udf();
}
