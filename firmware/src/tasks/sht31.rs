use crate::{
    common::{Measurement, MeasurementSender},
    config,
    drivers::sht31::{DefaultSht31, RawMeasurement},
};
use defmt::debug;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Duration, Ticker};

pub const MEASUREMENT_CHANNEL_SIZE: usize = 2;
pub type RawMeasurementChannel = Channel<NoopRawMutex, RawMeasurement, MEASUREMENT_CHANNEL_SIZE>;
pub type RawMeasurementSender =
    Sender<'static, NoopRawMutex, RawMeasurement, MEASUREMENT_CHANNEL_SIZE>;
pub type RawMeasurementReceiver =
    Receiver<'static, NoopRawMutex, RawMeasurement, MEASUREMENT_CHANNEL_SIZE>;

pub struct Sht31TaskState {
    driver: DefaultSht31,
    measurement_sender: MeasurementSender,
    raw_measurement_sender: RawMeasurementSender,
}

impl Sht31TaskState {
    pub fn new(
        driver: DefaultSht31,
        measurement_sender: MeasurementSender,
        raw_measurement_sender: RawMeasurementSender,
    ) -> Self {
        Self {
            driver,
            measurement_sender,
            raw_measurement_sender,
        }
    }
}

#[embassy_executor::task]
pub async fn sht31_task(state: Sht31TaskState) -> ! {
    let Sht31TaskState {
        mut driver,
        measurement_sender,
        raw_measurement_sender,
    } = state;
    let mut ticker = Ticker::every(Duration::from_millis(config::SHT31_MEASUREMENT_INTERVAL_MS));

    loop {
        let (measurement, raw) = driver.measure().await.unwrap().split();
        debug!("{}", measurement);

        // Send raw measurement to the SGP41 for conditioning data
        raw_measurement_sender.send(raw).await;

        // Send measurement to the data manager
        measurement_sender
            .send(Measurement::Sht31(measurement))
            .await;

        ticker.next().await;
    }
}
