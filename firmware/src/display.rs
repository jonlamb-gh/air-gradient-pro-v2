use crate::{common::FirmwareUpdateStatus, config};
use core::fmt::{self, Write as FmtWrite};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_net::{EthernetAddress, Ipv4Address};
use embassy_stm32::{i2c, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_graphics::{
    mono_font::{MonoFont, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal_async::i2c::I2c;
use heapless::String;
use ssd1306::{command::AddrMode, mode::BufferedGraphicsMode, prelude::*, Ssd1306};
use wire_protocols::{DeviceId, DeviceSerialNumber, FirmwareVersion};

const DH: i32 = 48 / 4;
const Y_R0: i32 = 0;
const Y_R1: i32 = DH;
const Y_R2: i32 = 2 * DH;
const Y_R3: i32 = 3 * DH;

const DW: i32 = 64 / 2;
const X_C0: i32 = 0;
const X_C1: i32 = DW;

const STATUS_FONT: MonoFont<'_> = profont::PROFONT_7_POINT;
const INFO_FONT: MonoFont<'_> = profont::PROFONT_7_POINT;
const DEVID_FONT: MonoFont<'_> = profont::PROFONT_7_POINT;

const LINE_BUF_CAP: usize = 128 + 2;

#[derive(Debug)]
pub enum Error {
    Infallible,
    Format,
    Display(display_interface::DisplayError),
}

#[derive(Debug, defmt::Format)]
pub struct SystemInfo {
    pub device_id: DeviceId,
    pub firmware_version: FirmwareVersion,
    pub ip: Ipv4Address,
    pub mac: EthernetAddress,
    pub device_serial_number: DeviceSerialNumber,
}

impl SystemInfo {
    pub const fn new() -> Self {
        Self {
            device_id: config::DEVICE_ID,
            firmware_version: config::FIRMWARE_VERSION,
            ip: config::IP_CIDR.address(),
            mac: EthernetAddress(config::MAC_ADDRESS),
            device_serial_number: DeviceSerialNumber::zero(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default, defmt::Format)]
pub struct SystemStatus {
    pub pm2_5: Option<u16>,
    pub co2: Option<u16>,
    pub temp: Option<i32>,
    pub humidity: Option<u16>,
    pub voc_index: Option<u16>,
    pub nox_index: Option<u16>,
    pub msg_seqnum: u32,
}

impl SystemStatus {
    pub const fn new() -> Self {
        Self {
            pm2_5: None,
            co2: None,
            temp: None,
            humidity: None,
            voc_index: None,
            nox_index: None,
            msg_seqnum: 0,
        }
    }

    fn aqi(&self) -> Option<u16> {
        self.pm2_5
            .and_then(|concentration| aqi::pm2_5(f64::from(concentration.clamp(0, 500))).ok())
            .map(|aqi| aqi.aqi() as u16)
    }

    fn temp_c(&self) -> Option<f32> {
        self.temp
            .map(f64::from)
            .map(|centi_deg| centi_deg as f32 / 100.0)
    }

    fn temp_f(&self) -> Option<f32> {
        self.temp_c().map(|c| (c * 1.8) + 32.0)
    }

    fn rel_humidity(&self) -> Option<f32> {
        self.humidity
            .map(f32::from)
            .map(|centi_rh| centi_rh / 100.0)
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, defmt::Format)]
pub struct FirmwareUpdateInfo {
    pub status: FirmwareUpdateStatus,
    pub bytes_written: usize,
    pub progress_percent: u8,
}

pub type DefaultDisplay = Display<
    I2cDevice<
        'static,
        NoopRawMutex,
        i2c::I2c<'static, peripherals::I2C2, peripherals::DMA1_CH7, peripherals::DMA1_CH2>,
    >,
>;

pub struct Display<I2C>
where
    I2C: I2c<Error = I2cDeviceError<i2c::Error>>,
{
    drv: Ssd1306<I2CInterface<I2C>, DisplaySize64x48, BufferedGraphicsMode<DisplaySize64x48>>,
    line_buf: String<LINE_BUF_CAP>,
}

impl<I2C> Display<I2C>
where
    I2C: I2c<Error = I2cDeviceError<i2c::Error>>,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error> {
        let iface = I2CInterface::new(i2c, 0x3C, 0x40);
        let mut drv = Ssd1306::new(iface, DisplaySize64x48, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        drv.init_with_addr_mode_async(AddrMode::Horizontal).await?;
        drv.set_display_on_async(true).await?;
        drv.set_brightness_async(Brightness::BRIGHTEST).await?;
        drv.clear_buffer();
        drv.flush_async().await?;

        Ok(Display {
            drv,
            line_buf: String::new(),
        })
    }

    pub async fn render_system_info(&mut self, view: &SystemInfo) -> Result<(), Error> {
        let text_style = MonoTextStyleBuilder::new()
            .font(&INFO_FONT)
            .text_color(BinaryColor::On)
            .build();

        self.drv.clear_buffer();

        self.line_buf.clear();
        write!(&mut self.line_buf, "ID: {}", view.device_id)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R0),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "V: {}", view.firmware_version)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R1),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "{}", view.ip)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R2),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.drv.flush_async().await?;

        Ok(())
    }

    pub async fn render_system_status(&mut self, view: &SystemStatus) -> Result<(), Error> {
        let text_style = MonoTextStyleBuilder::new()
            .font(&STATUS_FONT)
            .text_color(BinaryColor::On)
            .build();

        self.drv.clear_buffer();

        let val = view.aqi();
        let val = DisplayOption(&val);
        self.line_buf.clear();
        write!(&mut self.line_buf, "AQI: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R0),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        let val = DisplayOption(&view.co2);
        self.line_buf.clear();
        write!(&mut self.line_buf, "CO2: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C1, Y_R0),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        let val = DisplayOption(&view.voc_index);
        self.line_buf.clear();
        write!(&mut self.line_buf, "VOC: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R1),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        let val = DisplayOption(&view.nox_index);
        self.line_buf.clear();
        write!(&mut self.line_buf, "NOX: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C1, Y_R1),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        let val = view.temp_f();
        let val = DisplayOption(&val);
        self.line_buf.clear();
        write!(&mut self.line_buf, "  F: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R2),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        let val = view.rel_humidity();
        let val = DisplayOption(&val);
        self.line_buf.clear();
        write!(&mut self.line_buf, "  H: {}", val)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C1, Y_R2),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "MSG: {}", view.msg_seqnum)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R3),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.drv.flush_async().await?;

        Ok(())
    }

    pub async fn render_firmware_update_info(
        &mut self,
        view: &FirmwareUpdateInfo,
    ) -> Result<(), Error> {
        let text_style = MonoTextStyleBuilder::new()
            .font(&STATUS_FONT)
            .text_color(BinaryColor::On)
            .build();

        self.drv.clear_buffer();

        Text::with_baseline(
            "Updating Firmware",
            Point::new(X_C0, Y_R0),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "KB: {}", view.bytes_written / 1024)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R1),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "Progress: {}%", view.progress_percent)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R2),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.line_buf.clear();
        write!(&mut self.line_buf, "Status: {}", view.status)?;
        Text::with_baseline(
            self.line_buf.as_str(),
            Point::new(X_C0, Y_R3),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.drv)?;

        self.drv.flush_async().await?;

        Ok(())
    }
}

#[repr(transparent)]
struct DisplayOption<'a, T>(pub &'a Option<T>);

impl<'a> fmt::Display for DisplayOption<'a, u16> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            None => f.write_str("NA"),
            Some(v) => v.fmt(f),
        }
    }
}

impl<'a> fmt::Display for DisplayOption<'a, f32> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            None => f.write_str("NA"),
            Some(v) => write!(f, "{v:.0}"),
        }
    }
}

impl From<display_interface::DisplayError> for Error {
    fn from(value: display_interface::DisplayError) -> Self {
        Error::Display(value)
    }
}

impl From<fmt::Error> for Error {
    fn from(_value: fmt::Error) -> Self {
        Error::Format
    }
}

impl From<core::convert::Infallible> for Error {
    fn from(_value: core::convert::Infallible) -> Self {
        Error::Infallible
    }
}
