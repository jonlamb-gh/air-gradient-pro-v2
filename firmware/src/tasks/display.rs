use crate::{
    config,
    display::{DefaultDisplay, FirmwareUpdateInfo, SystemInfo, SystemStatus},
    util,
};
use defmt::debug;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, defmt::Format)]
pub enum Message {
    SystemStatus(SystemStatus),
    //FirmwareUpdateInfo(FirmwareUpdateInfo),
}

pub const MESSAGE_CHANNEL_SIZE: usize = 4;
pub type MessageChannel = Channel<NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;
pub type MessageSender = Sender<'static, NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;
pub type MessageReceiver = Receiver<'static, NoopRawMutex, Message, MESSAGE_CHANNEL_SIZE>;

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

    let mut sys_info = SystemInfo::new();

    // TODO constructor can just read this now, doesn't need to be const fn
    debug!("Initializing display state");
    sys_info.device_serial_number = util::read_device_serial_number();

    display.render_system_info(&sys_info).await.unwrap();

    loop {
        match msg_recvr.receive().await {
            Message::SystemStatus(status) => {
                display.render_system_status(&status).await.unwrap();
            }
        }
    }
}
