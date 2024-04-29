use core::fmt;
use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_stm32::{i2c, mode, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Delay, Timer};
use embedded_hal_async::i2c::I2c;
use sht4x::{Precision, Sht4x};

#[derive(Debug)]
pub enum Error {
    /// Wrong CRC
    Crc,
    /// I2C config error
    I2cConfig,
    /// I2C bus error
    I2c(i2c::Error),
    /// Internal error
    Internal,
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

pub type DefaultSht40 =
    Sht40<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, peripherals::I2C2, mode::Async>>>;

pub struct Sht40<I2C> {
    drv: Sht4x<I2C, Delay>,
}

impl<I2C> Sht40<I2C>
where
    I2C: I2c<Error = I2cDeviceError<i2c::Error>>,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error> {
        let mut drv = Self {
            drv: Sht4x::new(i2c),
        };
        drv.drv.soft_reset(&mut Delay).await?;
        Timer::after_millis(100).await;
        Ok(drv)
    }

    pub async fn serial_number(&mut self) -> Result<u32, Error> {
        let sn = self.drv.serial_number(&mut Delay).await?;
        Ok(sn)
    }

    pub async fn measure(&mut self) -> Result<CombinedMeasurement, Error> {
        let raw = self.drv.measure_raw(Precision::High, &mut Delay).await?;
        Ok(CombinedMeasurement {
            raw_temperature: raw.temperature,
            temperature: convert_temperature(raw.temperature),
            raw_humidity: raw.humidity,
            humidity: convert_humidity(raw.humidity),
        })
    }
}

const fn convert_temperature(raw: u16) -> i32 {
    -4500 + (17500 * raw as i32) / 65535
}

const fn convert_humidity(raw: u16) -> u16 {
    ((10000 * raw as u32) / 65535) as u16
}

impl fmt::Display for Measurement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SHT40 temperature: {}, humidity: {}",
            self.temperature, self.humidity
        )
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

impl From<sht4x::Error<I2cDeviceError<i2c::Error>>> for Error {
    fn from(value: sht4x::Error<I2cDeviceError<i2c::Error>>) -> Self {
        match value {
            sht4x::Error::Crc => Error::Crc,
            sht4x::Error::I2c(e) => e.into(),
            _ => Error::Internal,
        }
    }
}
