// NOTE: based on https://github.com/jonlamb-gh/sgp41, my fork of
// https://github.com/aectaan/sgp41

use crate::drivers::sht40;
use core::fmt;
use core::num::NonZeroU16;
use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_stm32::{i2c, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

const SGP41_I2C_ADDRESS: u8 = 0x59;

#[derive(Debug)]
pub enum SelfTestError {
    Voc,
    Nox,
    All,
    Undefined,
}

#[derive(Debug)]
pub enum Error {
    /// CRC checksum validation failed
    Crc,
    /// Self-test measure failure
    SelfTest(SelfTestError),
    /// I2C config error
    I2cConfig,
    /// I2C bus error
    I2c(i2c::Error),
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct GasIndices {
    /// Calculated VOC gas index value.
    /// Zero during initial blackout period and 1..500 afterwards.
    pub voc_index: Option<NonZeroU16>,

    /// Calculated VOC gas index value.
    /// Zero during initial blackout period and 1..500 afterwards.
    pub nox_index: Option<NonZeroU16>,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct Measurement {
    pub voc_ticks: u16,
    pub nox_ticks: u16,
}

pub const fn default_compensation() -> sht40::RawMeasurement {
    sht40::RawMeasurement {
        humidity_ticks: 0x8000,
        temperature_ticks: 0x6666,
    }
}

pub type DefaultSgp41 = Sgp41<
    I2cDevice<
        'static,
        NoopRawMutex,
        i2c::I2c<'static, peripherals::I2C2, peripherals::DMA1_CH7, peripherals::DMA1_CH2>,
    >,
>;

pub struct Sgp41<I2C> {
    i2c: I2C,
}

impl<I2C> Sgp41<I2C>
where
    I2C: I2c<Error = I2cDeviceError<i2c::Error>>,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error> {
        let mut drv: Sgp41<I2C> = Self { i2c };
        drv.turn_heater_off().await?;
        drv.execute_self_test().await?;
        Ok(drv)
    }

    pub async fn serial_number(&mut self) -> Result<u64, Error> {
        let mut buf = [0; 9];
        self.read_cmd(Command::GetSerialNumber, &mut buf).await?;

        let serial = u64::from(buf[0]) << 40
            | u64::from(buf[1]) << 32
            | u64::from(buf[3]) << 24
            | u64::from(buf[4]) << 16
            | u64::from(buf[6]) << 8
            | u64::from(buf[7]);
        Ok(serial)
    }

    /// This command starts the conditioning, i.e., the VOC pixel will be
    /// operated at the default temperature and humidity (+25 deg.C, 50% rH) as it is by calling the
    /// measure_raw command  while  the  NOx  pixel  will
    /// be  operated  at  a  different  temperature  for  conditioning.  This
    /// command returns only the measured raw signal of the VOC pixel SRAW_VOC as u16.
    pub async fn execute_conditioning(&mut self) -> Result<u16, Error> {
        let mut buf = [0; 3];
        self.read_cmd_args(Command::ExecuteConditioning, &[0x8000, 0x6666], &mut buf)
            .await?;
        Ok(u16::from_be_bytes([buf[0], buf[1]]))
    }

    pub async fn measure(
        &mut self,
        compensation: &sht40::RawMeasurement,
    ) -> Result<Measurement, Error> {
        let mut buf = [0; 6];
        self.read_cmd_args(
            Command::MeasureRawSignals,
            &[compensation.humidity_ticks, compensation.temperature_ticks],
            &mut buf,
        )
        .await?;
        let voc_ticks = u16::from_be_bytes([buf[0], buf[1]]);
        let nox_ticks = u16::from_be_bytes([buf[3], buf[4]]);
        let data = Measurement {
            voc_ticks,
            nox_ticks,
        };
        Ok(data)
    }

    /// This command turns the hotplate off and stops the measurement.
    /// Subsequently, the sensor enters the idle mode.
    async fn turn_heater_off(&mut self) -> Result<(), Error> {
        self.write_cmd(Command::TurnHeaterOff).await
    }

    /// This command triggers the built-in self-test checking for integrity
    /// of both hotplate and MOX material and returns the result of this
    /// test as 2 bytes (+ 1 CRC byte).
    async fn execute_self_test(&mut self) -> Result<(), Error> {
        let mut buf: [u8; 3] = [0; 3];
        self.read_cmd(Command::ExecuteSelfTest, &mut buf).await?;
        // There is only two significant bits
        let err = u16::from_be_bytes([buf[0], buf[1]]) & 0b11;
        match err {
            0b00 => Ok(()),
            0b01 => Err(Error::SelfTest(SelfTestError::Voc)),
            0b10 => Err(Error::SelfTest(SelfTestError::Nox)),
            0b11 => Err(Error::SelfTest(SelfTestError::All)),
            _ => Err(Error::SelfTest(SelfTestError::Undefined)),
        }
    }

    /// Writes command without additional arguments.
    async fn write_cmd(&mut self, cmd: Command) -> Result<(), Error> {
        let (command, delay) = cmd.as_tuple();
        self.i2c
            .write(SGP41_I2C_ADDRESS, &command.to_be_bytes())
            .await?;
        Timer::after_millis(delay).await;
        Ok(())
    }

    /// Writes command with additional arguments.
    async fn write_cmd_args(&mut self, cmd: Command, args: &[u16]) -> Result<(), Error> {
        let (command, delay) = cmd.as_tuple();

        let mut buf = [0; 8];
        assert!(command.to_ne_bytes().len() + args.len() * 3 <= buf.len());

        buf[0..2].copy_from_slice(&command.to_be_bytes());

        let mut i = 2;
        for arg in args {
            let end = i + 2;
            let be_arg = arg.to_be_bytes();
            buf[i..end].copy_from_slice(&be_arg);
            buf[end] = sensirion_i2c::crc8::calculate(&be_arg);
            i += 3;
        }

        self.i2c.write(SGP41_I2C_ADDRESS, &buf[0..i]).await?;
        Timer::after_millis(delay).await;
        Ok(())
    }

    /// Read command execution result.
    async fn read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error> {
        // See https://github.com/Sensirion/sensirion-i2c-rs/
        assert!(
            data.len() % 3 == 0,
            "Buffer must hold a multiple of 3 bytes"
        );
        self.write_cmd(cmd).await?;
        self.i2c.read(SGP41_I2C_ADDRESS, data).await?;
        sensirion_i2c::crc8::validate(data).map_err(|_| Error::Crc)?;
        Ok(())
    }

    /// Read command with args execution result.
    async fn read_cmd_args(
        &mut self,
        cmd: Command,
        args: &[u16],
        data: &mut [u8],
    ) -> Result<(), Error> {
        self.write_cmd_args(cmd, args).await?;
        self.i2c.read(SGP41_I2C_ADDRESS, data).await?;
        sensirion_i2c::crc8::validate(data).map_err(|_| Error::Crc)?;
        Ok(())
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

impl fmt::Display for Measurement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SGP41 voc_ticks: {}, nox_ticks: {}",
            self.voc_ticks, self.nox_ticks,
        )
    }
}

/// List of SGP41 sensor commands.
#[derive(Debug, Copy, Clone)]
#[allow(unused)]
enum Command {
    ExecuteConditioning,
    MeasureRawSignals,
    ExecuteSelfTest,
    TurnHeaterOff,
    GetSerialNumber,
    SoftReset,
}

impl Command {
    /// Returns command hex code and measurement delay in ms
    fn as_tuple(self) -> (u16, u64) {
        match self {
            Self::ExecuteConditioning => (0x2612, 50),
            Self::MeasureRawSignals => (0x2619, 50),
            Self::ExecuteSelfTest => (0x280E, 320),
            Self::TurnHeaterOff => (0x3615, 1000),
            Self::GetSerialNumber => (0x3682, 1000),
            Self::SoftReset => (0x0006, 1000),
        }
    }
}
