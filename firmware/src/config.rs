use embassy_net::{Ipv4Address, Ipv4Cidr};

pub use self::generated_confg::*;
mod generated_confg {
    include!(concat!(env!("OUT_DIR"), "/env_config.rs"));
}

pub const IP_CIDR: Ipv4Cidr = Ipv4Cidr::new(Ipv4Address(IP_ADDRESS), 24);

pub const BCAST_PROTO_SOCKET_BUFFER_LEN: usize = wire_protocols::broadcast::MESSAGE_LEN * 4;
pub const DEVICE_PROTO_SOCKET_BUFFER_LEN: usize = wire_protocols::device::SOCKET_BUFFER_LEN;

pub const STARTUP_DELAY_SECONDS: u8 = 5;

pub const WATCHDOG_TIMEOUT_MS: u32 = 8000;
pub const WATCHDOG_TASK_INTERVAL_MS: u64 = 1000;

pub const SGP41_MEASUREMENT_INTERVAL_MS: u64 = 1000;
pub const SHT40_MEASUREMENT_INTERVAL_MS: u64 = 2500;
pub const S8LP_MEASUREMENT_INTERVAL_MS: u64 = 5000;

/// PMS sensor is woken up for measurements every 3 minutes
/// to conserve lifespan, it also needs to warm up for at
/// least 30 seconds before taking a measurement.
///
/// The measurement task is run once a second to drive the
/// wake/measurement/sleep cycle.
pub const PMS5003_MEASUREMENT_INTERVAL_MS: u64 = 1000;
pub const PMS5003_WAKE_INTERVAL_MS: u64 = (3 * 60) * 1000;
pub const PMS5003_WARM_UP_PERIOD_MS: u64 = 45 * 1000;

/// Number of measurements to perform (one per measurement interval) before
/// going into standby mode.
pub const PMS5003_MEASUREMENT_COUNT: u8 = 10;

/// Number of BCAST_INTERVAL_SEC cycles to wait before starting to send
/// broadcast protocol messages
pub const DATA_MANAGER_WARM_UP_PERIOD_CYCLES: u64 = 12;

pub const BCAST_INTERVAL_SEC: u64 = 5;
