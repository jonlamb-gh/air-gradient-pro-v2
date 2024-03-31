use crate::{
    common::{Measurement, MeasurementSender},
    config,
    drivers::sgp41::{default_compensation, DefaultSgp41, GasIndices},
    tasks::sht31::RawMeasurementReceiver,
};
use core::num::NonZeroU16;
use defmt::{debug, warn};
use embassy_time::Timer;
use embassy_time::{Duration, Ticker};
use gas_index_algorithm::{AlgorithmType, GasIndexAlgorithm};
use static_assertions::{const_assert, const_assert_eq};

// SGP41 task requires 1 second cycles
const_assert_eq!(config::SGP41_MEASUREMENT_INTERVAL_MS, 1000);

/// Number of measurement update cycles to perform conditioning.
/// Run conditioning for the first 10 seconds (based on SGP41_MEASUREMENT_INTERVAL_MS).
const CONDITIONING_ITERS_10S: u64 = (10 * 1000) / config::SGP41_MEASUREMENT_INTERVAL_MS;
const_assert!(CONDITIONING_ITERS_10S <= 255);

pub struct Sgp41TaskState {
    driver: DefaultSgp41,
    measurement_sender: MeasurementSender,
    raw_measurement_receiver: RawMeasurementReceiver,
}

impl Sgp41TaskState {
    pub fn new(
        driver: DefaultSgp41,
        measurement_sender: MeasurementSender,
        raw_measurement_receiver: RawMeasurementReceiver,
    ) -> Self {
        Self {
            driver,
            measurement_sender,
            raw_measurement_receiver,
        }
    }
}

type ItersUntilMeasuring = u8;
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
enum State {
    /// Performing conditioning for N iterations after power cycle.
    /// Starts measuring after.
    Conditioning(ItersUntilMeasuring),
    /// Measuring
    Measuring,
}

#[embassy_executor::task]
pub async fn sgp41_task(state: Sgp41TaskState) -> ! {
    let Sgp41TaskState {
        mut driver,
        measurement_sender,
        raw_measurement_receiver,
    } = state;

    let mut voc_algorithm = GasIndexAlgorithm::new(AlgorithmType::Voc, 1.0);
    let mut nox_algorithm = GasIndexAlgorithm::new(AlgorithmType::Nox, 1.0);
    let default_compensation_data = default_compensation();
    let mut compensation_data = None;
    let mut ticker = Ticker::every(Duration::from_millis(config::SGP41_MEASUREMENT_INTERVAL_MS));

    debug!("SGP41: start conditioning");
    let mut state = State::Conditioning(CONDITIONING_ITERS_10S as u8);

    loop {
        if let Ok(raw_measurement) = raw_measurement_receiver.try_receive() {
            if compensation_data.is_none() {
                debug!("SGP41: received compensation data");
            }
            compensation_data = Some(raw_measurement);
        }

        match state {
            State::Conditioning(iters_until_measuring) => {
                driver.execute_conditioning().await.unwrap();
                let iters_until_measuring = iters_until_measuring.saturating_sub(1);
                if iters_until_measuring == 0 {
                    debug!("SGP41: conditioning complete, starting measurements");
                    state = State::Measuring;
                } else {
                    state = State::Conditioning(iters_until_measuring);
                }
            }
            State::Measuring => {
                let comp_data = if let Some(comp_data) = &compensation_data {
                    comp_data
                } else {
                    warn!("SGP41: no compensation data, using default");
                    &default_compensation_data
                };

                let measurement = driver.measure(comp_data).await.unwrap();
                debug!("{}", measurement);

                let gas_indices = GasIndices {
                    voc_index: NonZeroU16::new(
                        voc_algorithm.process(measurement.voc_ticks as _) as u16
                    ),
                    nox_index: NonZeroU16::new(
                        nox_algorithm.process(measurement.nox_ticks as _) as u16
                    ),
                };

                measurement_sender
                    .send(Measurement::Sgp41(measurement))
                    .await;
                measurement_sender
                    .send(Measurement::GasIndices(gas_indices))
                    .await;
            }
        }

        ticker.next().await;
    }
}
