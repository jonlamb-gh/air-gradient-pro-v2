use crate::{
    common::{DeviceInfo, FirmwareUpdateStatus, FirmwareUpdater},
    tasks::display::{Message as DisplayMessage, MessageSender as DisplayMessageSender},
};
use embassy_net::tcp::TcpSocket;
use embassy_time::Timer;

pub struct UpdateManagerTaskState {
    device_info: DeviceInfo,
    device_socket: TcpSocket<'static>,
    fw_updater: FirmwareUpdater,
    display_sender: DisplayMessageSender,
}

impl UpdateManagerTaskState {
    pub fn new(
        device_info: DeviceInfo,
        device_socket: TcpSocket<'static>,
        fw_updater: FirmwareUpdater,
        display_sender: DisplayMessageSender,
    ) -> Self {
        Self {
            device_info,
            device_socket,
            fw_updater,
            display_sender,
        }
    }
}

#[embassy_executor::task]
pub async fn update_manager_task(state: UpdateManagerTaskState) -> ! {
    loop {
        // TODO
        Timer::after_secs(1).await;
    }
}
