use crate::{
    drivers::{pms5003, s8lp, sgp41, sht31},
    reset_reason::ResetReason,
};
use embassy_embedded_hal::{adapter::BlockingAsync, flash::partition::Partition};
use embassy_stm32::flash::{self, Bank1Region2, Bank1Region3};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use wire_protocols::{DeviceId, DeviceSerialNumber, FirmwareVersion, ProtocolVersion};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Measurement {
    /// Temperature and humidity measurement from the SHT31 sensor
    Sht31(sht31::Measurement),
    /// VOC and NOx measurement from the SGP41 sensor
    Sgp41(sgp41::Measurement),
    /// VOC and NOx computed indices
    GasIndices(sgp41::GasIndices),
    /// PM2.5 measurement from the PMS5003 sensor
    Pms5003(pms5003::Measurement),
    /// CO2 measurement from the S8 LP sensor
    S8Lp(s8lp::Measurement),
}

pub const MEASUREMENT_CHANNEL_SIZE: usize = 8;
pub type MeasurementChannel = Channel<NoopRawMutex, Measurement, MEASUREMENT_CHANNEL_SIZE>;
pub type MeasurementSender = Sender<'static, NoopRawMutex, Measurement, MEASUREMENT_CHANNEL_SIZE>;
pub type MeasurementReceiver =
    Receiver<'static, NoopRawMutex, Measurement, MEASUREMENT_CHANNEL_SIZE>;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct DeviceInfo {
    pub protocol_version: ProtocolVersion,
    pub firmware_version: FirmwareVersion,
    pub device_id: DeviceId,
    pub device_serial_number: DeviceSerialNumber,
    pub mac_address: [u8; 6],
    pub bootloader_state: BootloaderState,
    pub reset_reason: ResetReason,
    pub built_time_utc: &'static str,
    pub git_commit: &'static str,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum BootloaderState {
    Boot,
    Swap,
    DfuDetach,
}

impl From<embassy_boot_stm32::State> for BootloaderState {
    fn from(value: embassy_boot_stm32::State) -> Self {
        use BootloaderState::*;
        match value {
            embassy_boot_stm32::State::Boot => Boot,
            embassy_boot_stm32::State::Swap => Swap,
            embassy_boot_stm32::State::DfuDetach => DfuDetach,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum FirmwareUpdateStatus {
    InProgress,
    Complete,
    Verifying,
    Aborted,
}

pub type DfuRegion = Mutex<NoopRawMutex, BlockingAsync<Bank1Region3<'static, flash::Blocking>>>;
pub type StateRegion = Mutex<NoopRawMutex, BlockingAsync<Bank1Region2<'static, flash::Blocking>>>;
pub type DfuPartition =
    Partition<'static, NoopRawMutex, BlockingAsync<Bank1Region3<'static, flash::Blocking>>>;
pub type StatePartition =
    Partition<'static, NoopRawMutex, BlockingAsync<Bank1Region2<'static, flash::Blocking>>>;
pub type FirmwareUpdater =
    embassy_boot_stm32::FirmwareUpdater<'static, DfuPartition, StatePartition>;
