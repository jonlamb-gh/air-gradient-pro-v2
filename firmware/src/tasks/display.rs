use crate::display::{DefaultDisplay, Error, FirmwareUpdateInfo, SystemInfo, SystemStatus};
use defmt::{debug, warn};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{with_timeout, Duration};

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, defmt::Format)]
pub enum Message {
    SystemStatus(SystemStatus),
    FirmwareUpdateInfo(FirmwareUpdateInfo),
}

pub const MESSAGE_CHANNEL_SIZE: usize = 8;
pub type MessageChannel = Channel<NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;
pub type MessageSender = Sender<'static, NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;
pub type MessageReceiver = Receiver<'static, NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;

/// Sometimes the display gets into a weird state if reset during flushing.
/// TODO - figure out if this is an issue with the fork I'm using or an
/// actually expected thing.
const RENDER_TIMEOUT: Duration = Duration::from_secs(2);

/// Number of SystemStatus messages to ignore once we've
/// seen a FirmwareUpdateInfo message
const DEFAULT_IGNORE: usize = 2;

const MAX_TIMEOUTS: usize = 5;

pub struct DisplayTaskState {
    display: DefaultDisplay,
    msg_recvr: MessageReceiver,
}

impl DisplayTaskState {
    pub fn new(display: DefaultDisplay, msg_recvr: MessageReceiver) -> Self {
        Self { display, msg_recvr }
    }
}

#[embassy_executor::task]
pub async fn display_task(state: DisplayTaskState) -> ! {
    let DisplayTaskState {
        mut display,
        msg_recvr,
    } = state;

    let mut requests_to_ignore_while_updating = 0_usize;
    let mut timeouts = 0;

    debug!("Initializing display state");
    let sys_info = SystemInfo::new();
    match with_timeout(RENDER_TIMEOUT, display.render_system_info(&sys_info)).await {
        Ok(res) => res.unwrap(),
        Err(_) => {
            panic!("Display timeout on system info");
        }
    }

    loop {
        match msg_recvr.receive().await {
            Message::SystemStatus(status) => {
                requests_to_ignore_while_updating =
                    requests_to_ignore_while_updating.saturating_sub(1);
                if requests_to_ignore_while_updating == 0 {
                    match with_timeout(RENDER_TIMEOUT, display.render_system_status(&status)).await
                    {
                        Ok(res) => {
                            timeouts = 0;
                            handle_result(res);
                        }
                        Err(_) => {
                            warn!("Display timeout");
                            timeouts += 1;
                            if timeouts >= MAX_TIMEOUTS {
                                panic!("Display timeout limit reached");
                            }
                        }
                    }
                }
            }
            Message::FirmwareUpdateInfo(info) => {
                requests_to_ignore_while_updating = DEFAULT_IGNORE;
                match with_timeout(RENDER_TIMEOUT, display.render_firmware_update_info(&info)).await
                {
                    Ok(res) => {
                        timeouts = 0;
                        handle_result(res);
                    }
                    Err(_) => {
                        warn!("Display timeout");
                        timeouts += 1;
                        if timeouts >= MAX_TIMEOUTS {
                            panic!("Display timeout limit reached");
                        }
                    }
                }
            }
        }
    }
}

fn handle_result(res: Result<(), Error>) {
    match res {
        Ok(()) => (),
        Err(e) => {
            warn!(
                "Display driver returned an error {}",
                defmt::Debug2Format(&e)
            );
            if !e.is_recoverable() {
                panic!("Display error. {:?}", e);
            }
        }
    }
}
