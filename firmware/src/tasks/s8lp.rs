use crate::{
    common::{Measurement, MeasurementSender},
    config,
    drivers::s8lp::DefaultS8Lp,
};
use defmt::debug;
use embassy_time::{Duration, Ticker};

pub struct S8LpTaskState {
    driver: DefaultS8Lp,
    sender: MeasurementSender,
}

impl S8LpTaskState {
    pub fn new(driver: DefaultS8Lp, sender: MeasurementSender) -> Self {
        Self { driver, sender }
    }
}

#[embassy_executor::task]
pub async fn s8lp_task(state: S8LpTaskState) -> ! {
    let S8LpTaskState { mut driver, sender } = state;
    let mut ticker = Ticker::every(Duration::from_millis(config::S8LP_MEASUREMENT_INTERVAL_MS));
    loop {
        let m = driver.measure().await.unwrap();
        debug!("{}", m);
        sender.send(Measurement::S8Lp(m)).await;
        ticker.next().await;
    }
}
