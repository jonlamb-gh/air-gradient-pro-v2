use crate::config;
use embassy_stm32::{gpio::Output, peripherals::IWDG, wdg::IndependentWatchdog};
use embassy_time::{Duration, Ticker};

pub struct LedHeartbeatTaskState {
    wdt: IndependentWatchdog<'static, IWDG>,
    led: Output<'static>,
}

impl LedHeartbeatTaskState {
    pub fn new(wdt: IndependentWatchdog<'static, IWDG>, led: Output<'static>) -> Self {
        Self { wdt, led }
    }
}

#[embassy_executor::task]
pub async fn led_heartbeat_task(state: LedHeartbeatTaskState) -> ! {
    let LedHeartbeatTaskState { mut wdt, mut led } = state;
    let mut ticker = Ticker::every(Duration::from_millis(config::WATCHDOG_TASK_INTERVAL_MS));

    led.set_high();
    loop {
        wdt.pet();
        led.toggle();
        ticker.next().await;
    }
}
