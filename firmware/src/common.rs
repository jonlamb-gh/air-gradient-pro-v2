use crate::drivers::{pms5003, s8lp, sgp41, sht31};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};

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
