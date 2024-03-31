use crate::{
    common::{Measurement, MeasurementSender},
    config,
    drivers::pms5003::DefaultPms5003,
};
use defmt::debug;
use embassy_time::{Duration, Ticker, Timer};
use static_assertions::{const_assert, const_assert_eq};

// This task requires 1 second cycles
const_assert_eq!(config::PMS5003_MEASUREMENT_INTERVAL_MS, 1000);

// At least 30 seconds required to warm up
const_assert!(config::PMS5003_WARM_UP_PERIOD_MS >= 30_000);

// Should be in standby for longer that the warmup
const_assert!(config::PMS5003_WAKE_INTERVAL_MS > config::PMS5003_WARM_UP_PERIOD_MS);

pub struct Pms5003TaskState {
    driver: DefaultPms5003,
    sender: MeasurementSender,
}

impl Pms5003TaskState {
    pub fn new(driver: DefaultPms5003, sender: MeasurementSender) -> Self {
        Self { driver, sender }
    }
}

type MillisUntilWakeUp = u64;
type MeasurementsUntilStandby = u8;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
enum State {
    /// The sensor is in standby mode.
    /// Waits for PMS5003_WAKE_INTERVAL_MS and then
    /// enters ready mode to warm up.
    /// Data-carrying so that on startup, it can be set to zero
    /// and go immediately into warming up.
    StandbyMode(MillisUntilWakeUp),

    /// The sensor is in ready mode and warming up.
    /// Once woken up, waits for PMS5003_WARM_UP_PERIOD_MS
    /// and then the tasks starts requesting measurements from the
    /// sensor.
    WarmingUp,

    /// The sensor is in ready mode and requesting a measurement
    /// every PMS5003_MEASUREMENT_INTERVAL_MS.
    /// Once warmed up, starts at PMS5003_MEASUREMENT_COUNT,
    /// decrements until zero, then the sensor goes back into standby mode.
    Measuring(MeasurementsUntilStandby),
}

impl State {
    /// Starts at the end of the StandbyMode state, will start
    /// warming up after initialized.
    const fn begin_warm_up() -> Self {
        State::StandbyMode(0)
    }

    const fn init() -> Self {
        State::StandbyMode(config::PMS5003_WAKE_INTERVAL_MS)
    }
}

#[embassy_executor::task]
pub async fn pms5003_task(state: Pms5003TaskState) -> ! {
    let Pms5003TaskState { mut driver, sender } = state;
    let mut state = State::begin_warm_up();
    let mut ticker = Ticker::every(Duration::from_millis(
        config::PMS5003_MEASUREMENT_INTERVAL_MS,
    ));

    loop {
        match state {
            State::StandbyMode(ms_until_wake_up) => {
                if ms_until_wake_up != 0 {
                    Timer::after_millis(ms_until_wake_up).await;
                }

                debug!("PMS5003: enter ready mode");
                driver.enter_ready_mode().await.unwrap();

                state = State::WarmingUp;
            }
            State::WarmingUp => {
                Timer::after_millis(config::PMS5003_WARM_UP_PERIOD_MS).await;
                debug!("PMS5003: begin measuring");
                state = State::Measuring(config::PMS5003_MEASUREMENT_COUNT);
                ticker.reset();
            }
            State::Measuring(measurements_until_standby) => {
                let measurement = driver.measure().await.unwrap();
                debug!("{}", measurement);

                let measurements_until_standby = measurements_until_standby.saturating_sub(1);
                if measurements_until_standby == 0 {
                    debug!("PMS5003: entering standby mode");
                    driver.enter_standby_mode().await.unwrap();
                    state = State::init();
                } else {
                    state = State::Measuring(measurements_until_standby);
                }

                sender.send(Measurement::Pms5003(measurement)).await;

                ticker.next().await;
            }
        }
    }
}
