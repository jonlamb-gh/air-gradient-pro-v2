// NOTE: based on https://github.com/jonlamb-gh/sht3x-rs, my fork of
// https://github.com/miek/sht3x-rs

use bitflags::bitflags;
use core::fmt;
use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_stm32::{i2c, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

// 2.2 Timing Specification for the Sensor System
// Table 4
// TODO: Support longer times needed with lower voltage (Table 5).
const SOFT_RESET_TIME_MS: u64 = 1;

// 4: Operation and Communication
const COMMAND_WAIT_TIME_MS: u64 = 1;

#[derive(Debug)]
pub enum Error {
    /// Wrong CRC
    Crc,
    /// I2C config error
    I2cConfig,
    /// I2C bus error
    I2c(i2c::Error),
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct Measurement {
    /// The temperature in centidegress C
    pub temperature: i32,
    /// The relative humidity in centipercent
    pub humidity: u16,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct RawMeasurement {
    pub temperature_ticks: u16,
    pub humidity_ticks: u16,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct CombinedMeasurement {
    /// The raw temperature
    pub raw_temperature: u16,
    /// The temperature in centidegress C
    pub temperature: i32,

    /// The raw humidity
    pub raw_humidity: u16,
    /// The relative humidity in centipercent
    pub humidity: u16,
}

impl CombinedMeasurement {
    pub fn split(self) -> (Measurement, RawMeasurement) {
        (
            Measurement {
                temperature: self.temperature,
                humidity: self.humidity,
            },
            RawMeasurement {
                temperature_ticks: self.raw_temperature,
                humidity_ticks: self.raw_humidity,
            },
        )
    }
}

pub type DefaultSht31 = Sht31<
    I2cDevice<
        'static,
        NoopRawMutex,
        i2c::I2c<'static, peripherals::I2C2, peripherals::DMA1_CH7, peripherals::DMA1_CH2>,
    >,
>;

pub struct Sht31<I2C> {
    address: Address,
    i2c: I2C,
}

impl<I2C> Sht31<I2C>
where
    I2C: I2c<Error = I2cDeviceError<i2c::Error>>,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error> {
        let mut drv = Self {
            address: Address::Low,
            i2c,
        };
        drv.reset().await?;
        drv.stop().await?;
        Timer::after_millis(20).await;
        drv.clear_status().await?;
        Ok(drv)
    }

    pub async fn serial_number(&mut self) -> Result<u16, Error> {
        self.command(Command::ReadSerialNumber, None).await?;
        let mut buf = [0; 2];
        self.i2c.read(self.address as u8, &mut buf).await?;
        Ok(u16::from_be_bytes(buf))
    }

    pub async fn status(&mut self) -> Result<Status, Error> {
        self.command(Command::Status, None).await?;
        let mut buf = [0; 3];
        self.i2c.read(self.address as u8, &mut buf).await?;

        let status = check_crc([buf[0], buf[1]], buf[2])?;
        Ok(Status::from_bits_truncate(status))
    }

    pub async fn measure(&mut self) -> Result<CombinedMeasurement, Error> {
        let cs = ClockStretch::Enabled;
        let rpt = Repeatability::High;
        self.command(Command::SingleShot(cs, rpt), Some(rpt.max_duration()))
            .await?;
        self.inner_read().await
    }

    async fn clear_status(&mut self) -> Result<(), Error> {
        self.command(Command::ClearStatus, None).await
    }

    async fn reset(&mut self) -> Result<(), Error> {
        self.command(Command::SoftReset, Some(SOFT_RESET_TIME_MS))
            .await
    }

    async fn stop(&mut self) -> Result<(), Error> {
        self.command(Command::Break, Some(Repeatability::High.max_duration()))
            .await?;
        Ok(())
    }

    async fn command(&mut self, command: Command, wait_time: Option<u64>) -> Result<(), Error> {
        let cmd_bytes = command.value().to_be_bytes();
        self.i2c.write(self.address as u8, &cmd_bytes).await?;

        Timer::after_millis(wait_time.unwrap_or(0).max(COMMAND_WAIT_TIME_MS)).await;

        Ok(())
    }

    async fn inner_read(&mut self) -> Result<CombinedMeasurement, Error> {
        let mut buf = [0; 6];
        self.i2c.read(self.address as u8, &mut buf).await?;

        let (raw_temperature, temperature) =
            check_crc([buf[0], buf[1]], buf[2]).map(|t| (t, convert_temperature(t)))?;
        let (raw_humidity, humidity) =
            check_crc([buf[3], buf[4]], buf[5]).map(|h| (h, convert_humidity(h)))?;

        Ok(CombinedMeasurement {
            raw_temperature,
            temperature,
            raw_humidity,
            humidity,
        })
    }
}

const fn convert_temperature(raw: u16) -> i32 {
    -4500 + (17500 * raw as i32) / 65535
}

const fn convert_humidity(raw: u16) -> u16 {
    ((10000 * raw as u32) / 65535) as u16
}

/// Compare the CRC of the input array to the given CRC checksum.
fn check_crc(data: [u8; 2], crc: u8) -> Result<u16, Error> {
    let calculated_crc = crc8(data);

    if calculated_crc == crc {
        Ok(u16::from_be_bytes(data))
    } else {
        Err(Error::Crc)
    }
}

/// Calculate the CRC8 checksum for the given input array.
fn crc8(data: [u8; 2]) -> u8 {
    let mut crc: u8 = 0xff;

    for byte in data {
        crc ^= byte;

        for _ in 0..8 {
            if crc & 0x80 > 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    crc
}

impl fmt::Display for Measurement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SHT31 temperature: {}, humidity: {}",
            self.temperature, self.humidity
        )
    }
}

impl From<i2c::Error> for Error {
    fn from(value: i2c::Error) -> Self {
        Error::I2c(value)
    }
}

impl From<I2cDeviceError<i2c::Error>> for Error {
    fn from(value: I2cDeviceError<i2c::Error>) -> Self {
        match value {
            I2cDeviceError::I2c(e) => Error::I2c(e),
            I2cDeviceError::Config => Error::I2cConfig,
        }
    }
}

bitflags! {
    /// Status register
    #[derive(Debug, Copy, Clone)]
    pub struct Status: u16 {
        /// Alert pending status
        const ALERT_PENDING         = 1 << 15;
        /// Heater status
        const HEATER                = 1 << 13;
        /// RH tracking alert
        const RH_TRACKING_ALERT     = 1 << 11;
        /// T tracking alert
        const T_TRACKING_ALERT      = 1 << 10;
        /// System reset detected
        const SYSTEM_RESET_DETECTED = 1 <<  4;
        /// Command status
        const COMMAND               = 1 <<  1;
        /// Write data checksum status
        const WRITE_DATA_CHECKSUM   = 1 <<  0;
    }
}

/// I2C address
#[derive(Debug, Copy, Clone)]
#[allow(unused)]
enum Address {
    /// Address pin held high
    High = 0x45,
    /// Address pin held low
    Low = 0x44,
}

/// Clock stretching
#[derive(Debug)]
#[allow(unused)]
enum ClockStretch {
    Enabled,
    Disabled,
}

/// Periodic data acquisition rate
#[allow(non_camel_case_types, unused)]
enum Rate {
    /// 0.5 measurements per second
    R0_5,
    /// 1 measurement per second
    R1,
    /// 2 measurements per second
    R2,
    /// 4 measurements per second
    R4,
    /// 10 measurements per second
    R10,
}

#[derive(Copy, Clone)]
#[allow(unused)]
enum Repeatability {
    High,
    Medium,
    Low,
}

impl Repeatability {
    /// Maximum measurement duration in milliseconds
    const fn max_duration(&self) -> u64 {
        match *self {
            Repeatability::Low => 4,
            Repeatability::Medium => 6,
            Repeatability::High => 15,
        }
    }
}

#[allow(unused)]
enum Command {
    SingleShot(ClockStretch, Repeatability),
    Periodic(Rate, Repeatability),
    FetchData,
    PeriodicWithART,
    Break,
    SoftReset,
    HeaterEnable,
    HeaterDisable,
    Status,
    ClearStatus,
    ReadSerialNumber,
}

impl Command {
    const fn value(&self) -> u16 {
        use ClockStretch::Disabled as CSDisabled;
        use ClockStretch::Enabled as CSEnabled;
        use Rate::*;
        use Repeatability::*;
        match *self {
            // 4.3 Measurement Commands for Single Shot Data Acquisition Mode
            // Table 8
            Command::SingleShot(CSEnabled, High) => 0x2C06,
            Command::SingleShot(CSEnabled, Medium) => 0x2C0D,
            Command::SingleShot(CSEnabled, Low) => 0x2C10,
            Command::SingleShot(CSDisabled, High) => 0x2400,
            Command::SingleShot(CSDisabled, Medium) => 0x240B,
            Command::SingleShot(CSDisabled, Low) => 0x2416,

            // 4.5 Measurement Commands for Periodic Data Acquisition Mode
            // Table 9
            Command::Periodic(R0_5, High) => 0x2032,
            Command::Periodic(R0_5, Medium) => 0x2024,
            Command::Periodic(R0_5, Low) => 0x202F,
            Command::Periodic(R1, High) => 0x2130,
            Command::Periodic(R1, Medium) => 0x2126,
            Command::Periodic(R1, Low) => 0x212D,
            Command::Periodic(R2, High) => 0x2236,
            Command::Periodic(R2, Medium) => 0x2220,
            Command::Periodic(R2, Low) => 0x222B,
            Command::Periodic(R4, High) => 0x2334,
            Command::Periodic(R4, Medium) => 0x2322,
            Command::Periodic(R4, Low) => 0x2329,
            Command::Periodic(R10, High) => 0x2737,
            Command::Periodic(R10, Medium) => 0x2721,
            Command::Periodic(R10, Low) => 0x272A,

            // 4.6 Readout of Measurement Results for Periodic Mode
            // Table 10
            Command::FetchData => 0xE000,

            // 4.7 ART command
            // Table 11
            Command::PeriodicWithART => 0x2B32,

            // 4.8 Break command
            // Table 12
            Command::Break => 0x3093,

            // 4.9 Reset
            // Table 13
            Command::SoftReset => 0x30A2,

            // 4.10 Heater
            // Table 15
            Command::HeaterEnable => 0x306D,
            Command::HeaterDisable => 0x3066,

            // 4.11 Status register
            // Table 16
            Command::Status => 0xF32D,
            // Table 18
            Command::ClearStatus => 0x3041,

            Command::ReadSerialNumber => 0x3780,
        }
    }
}
