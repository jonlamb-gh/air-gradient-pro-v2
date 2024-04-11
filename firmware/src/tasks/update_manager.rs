use crate::{
    common::{DeviceInfo, FirmwareUpdateStatus, FirmwareUpdater},
    config,
    display::FirmwareUpdateInfo,
    tasks::display::{Message as DisplayMessage, MessageSender as DisplayMessageSender},
};
use core::fmt::{self, Write as FmtWrite};
use defmt::{debug, warn};
use embassy_boot::FirmwareUpdaterError;
use embassy_net::tcp::{self, TcpSocket};
use embassy_time::{with_timeout, Duration, Timer};
use embedded_io_async::{Read, Write};
use heapless::String;
use sha2::Sha256;
use wire_protocols::device::{Command, FirmwareHash, FirmwareUpdateHeader, StatusCode};

#[embassy_executor::task]
pub async fn update_manager_task(mut state: UpdateManagerTaskState) -> ! {
    loop {
        if let Err(e) = state.update().await {
            warn!("UM: encountered an error. {}", e);
            state.reset().await;
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
enum Error {
    Accept(tcp::AcceptError),
    Socket(tcp::Error),
    Fmt,
    Connection,
    Protocol,
    Flash,
    FirmwareSignature,
    BadBootloaderState,
}

type TotalUpdateSizeBytes = usize;

pub const CHUNK_BUFFER_SIZE: usize = FirmwareUpdateHeader::MAX_CHUCK_SIZE;
// TODO make this static buffer
const DEVICE_INFO_STRING_SIZE: usize = 512;

const RXTX_TIMEOUT: Duration = Duration::from_secs(4);

pub struct UpdateManagerTaskState {
    device_info: DeviceInfo,
    socket: TcpSocket<'static>,
    fw_updater: FirmwareUpdater,
    display_sender: DisplayMessageSender,
    chunk_buffer: &'static mut [u8],

    info_string: String<DEVICE_INFO_STRING_SIZE>,
    computed_hash: FirmwareHash,
    update_in_progress: Option<TotalUpdateSizeBytes>,
    bytes_written: usize,
}

impl UpdateManagerTaskState {
    pub fn new(
        device_info: DeviceInfo,
        mut socket: TcpSocket<'static>,
        fw_updater: FirmwareUpdater,
        display_sender: DisplayMessageSender,
        chunk_buffer: &'static mut [u8],
    ) -> Self {
        assert!(chunk_buffer.len() >= CHUNK_BUFFER_SIZE);

        socket.set_timeout(RXTX_TIMEOUT.into());

        Self {
            device_info,
            socket,
            fw_updater,
            display_sender,
            chunk_buffer,
            info_string: String::new(),
            computed_hash: FirmwareHash([0; 32]),
            update_in_progress: None,
            bytes_written: 0,
        }
    }

    async fn update(&mut self) -> Result<(), Error> {
        self.manage_socket().await?;

        let cmd = self.recv_cmd().await?;
        self.process_cmd(cmd).await?;

        Ok(())
    }

    async fn display_info(&self, status: FirmwareUpdateStatus) {
        let progress_percent = self
            .update_in_progress
            .map(|total_size| (self.bytes_written * 100 / total_size) as u8)
            .unwrap_or(0);
        let info = FirmwareUpdateInfo {
            status,
            bytes_written: self.bytes_written,
            progress_percent,
        };
        self.display_sender
            .send(DisplayMessage::FirmwareUpdateInfo(info))
            .await;
    }

    async fn reset(&mut self) {
        if self.update_in_progress.is_some() {
            warn!("In-progress update will be aborted");
            self.display_info(FirmwareUpdateStatus::Aborted).await;
        }

        self.update_in_progress = None;
        self.bytes_written = 0;

        self.socket.abort();
        let _ignored_result = flush_with_timeout(&mut self.socket).await;
    }

    async fn manage_socket(&mut self) -> Result<(), Error> {
        if !self.socket.may_recv() && self.socket.may_send() {
            debug!("UM: closing socket due to lack of recv");
            self.reset().await;
        }

        if self.socket.remote_endpoint().is_none() {
            if self.update_in_progress.is_some() {
                self.reset().await;
            }

            // Wait for a new connection
            debug!("UM: listening on port {}", config::DEVICE_PORT);
            self.socket.accept(config::DEVICE_PORT).await?;
            debug!("UM: client connected");
        }

        Ok(())
    }

    async fn send_status(&mut self, status: StatusCode) -> Result<(), Error> {
        if !self.socket.may_send() {
            warn!("Cannot send status {}, aborting", status);
            self.reset().await;
        } else {
            let bytes = u32::from(status).to_le_bytes();
            self.socket.write_all(&bytes).await?;
            flush_with_timeout(&mut self.socket).await?;
        }
        Ok(())
    }

    async fn recv_cmd(&mut self) -> Result<Command, Error> {
        let mut buf = [0_u8; Command::WIRE_SIZE];
        read_exact_with_timeout(&mut self.socket, &mut buf).await?;
        let cmd = Command::from_le_bytes_unchecked(&buf);
        Ok(cmd)
    }

    async fn recv_fw_update_header(&mut self) -> Result<FirmwareUpdateHeader, Error> {
        let mut buf = [0_u8; 4];
        read_exact_with_timeout(&mut self.socket, &mut buf).await?;
        let total_size = u32::from_le_bytes(buf);
        read_exact_with_timeout(&mut self.socket, &mut buf).await?;
        let offset = u32::from_le_bytes(buf);
        read_exact_with_timeout(&mut self.socket, &mut buf).await?;
        let length = u32::from_le_bytes(buf);
        let hdr = FirmwareUpdateHeader::new_unchecked(total_size, offset, length);
        if let Err(e) = hdr.check_length() {
            self.send_status(e).await?;
            return Err(Error::Protocol);
        }
        Ok(hdr)
    }

    async fn recv_fw_hash(&mut self) -> Result<FirmwareHash, Error> {
        let mut hash = FirmwareHash([0; 32]);
        read_exact_with_timeout(&mut self.socket, &mut hash.0).await?;
        Ok(hash)
    }

    async fn process_cmd(&mut self, cmd: Command) -> Result<(), Error> {
        debug!("UM: processing command {}", cmd);
        match cmd {
            Command::Info => {
                self.send_status(StatusCode::Success).await?;
                self.info_string.clear();
                writeln!(&mut self.info_string, "{{\"protocol_version\": \"{}\", \"firmware_version\": \"{}\", \"device_id\": {}, \"device_serial_number\": \"{:X}\", \"mac_address\": {:?}, \"bootloader_state\": \"{}\", \"reset_reason\": \"{}\", \"built_time_utc\": \"{}\", \"git_commit\": \"{}\"}}",
                    self.device_info.protocol_version,
                    self.device_info.firmware_version,
                    self.device_info.device_id,
                    self.device_info.device_serial_number,
                    self.device_info.mac_address,
                    self.device_info.bootloader_state,
                    self.device_info.reset_reason,
                    self.device_info.built_time_utc,
                    self.device_info.git_commit,
                )?;
                self.socket.write_all(self.info_string.as_bytes()).await?;
                self.socket.close();
                flush_with_timeout(&mut self.socket).await?;
                self.reset().await;
            }
            Command::WriteFirmware => {
                let hdr = self.recv_fw_update_header().await?;
                debug!("FirmwareUpdateHeader {}", hdr);
                self.update_in_progress = Some(hdr.total_size as usize);
                let chunk_size = hdr.length as usize;
                read_exact_with_timeout(&mut self.socket, &mut self.chunk_buffer[..chunk_size])
                    .await?;

                match self
                    .fw_updater
                    .write_firmware(hdr.offset as _, &self.chunk_buffer[..chunk_size])
                    .await
                {
                    Ok(_) => {
                        self.send_status(StatusCode::Success).await?;
                        self.bytes_written += chunk_size;
                        self.display_info(FirmwareUpdateStatus::InProgress).await;
                    }
                    Err(e) => {
                        warn!(
                            "Firmware updater returned an error on write_firmware. {}",
                            e
                        );
                        self.send_status(fw_update_error_to_status_code(&e)).await?;
                        self.reset().await;
                    }
                }
            }
            Command::CompleteAndReboot => {
                debug!("UM: scheduling a reboot");
                if let Some(total_size) = self.update_in_progress {
                    self.display_info(FirmwareUpdateStatus::Verifying).await;

                    let recvd_hash = self.recv_fw_hash().await?;

                    let mut chunk_buf = [0_u8; 4];
                    self.fw_updater
                        .hash::<Sha256>(total_size as _, &mut chunk_buf, &mut self.computed_hash.0)
                        .await?;
                    debug!(
                        "UM: verifying update with hash {=[u8]:X}",
                        self.computed_hash.0
                    );

                    if self.computed_hash == recvd_hash {
                        match self.fw_updater.mark_updated().await {
                            Ok(()) => {
                                self.display_info(FirmwareUpdateStatus::Complete).await;
                                self.update_in_progress = None;
                                self.send_status(StatusCode::Success).await?;

                                self.socket.close();
                                flush_with_timeout(&mut self.socket).await?;
                                self.reset().await;
                                Timer::after_secs(3).await;

                                unsafe {
                                    sw_reset();
                                }
                            }
                            Err(e) => {
                                warn!("Firmware updater returned an error on mark_updated. {}", e);
                                self.send_status(fw_update_error_to_status_code(&e)).await?;
                                self.reset().await;
                                // Don't reboot
                                return Err(e.into());
                            }
                        }
                    } else {
                        self.send_status(StatusCode::Signature).await?;
                    }
                } else {
                    self.send_status(StatusCode::NoUpdatePending).await?;
                }
            }
            Command::ConfirmUpdate => {
                if let Ok(embassy_boot_stm32::State::Swap) = self.fw_updater.get_state().await {
                    match self.fw_updater.mark_booted().await {
                        Ok(()) => {
                            self.send_status(StatusCode::Success).await?;
                            self.device_info.bootloader_state =
                                self.fw_updater.get_state().await?.into();
                        }
                        Err(e) => {
                            warn!("Firmware updater returned an error on mark_booted. {}", e);
                            self.send_status(fw_update_error_to_status_code(&e)).await?;
                            self.reset().await;
                        }
                    }
                } else {
                    self.send_status(StatusCode::BadState).await?;
                    self.reset().await;
                }
            }
            Command::Reboot => {
                self.send_status(StatusCode::Success).await?;
                self.socket.close();
                flush_with_timeout(&mut self.socket).await?;
                self.reset().await;
                Timer::after_secs(3).await;

                unsafe {
                    sw_reset();
                }
            }
            Command::Unknown(_) => {
                self.send_status(StatusCode::UnknownCommand).await?;
            }
        }
        Ok(())
    }
}

async fn read_exact_with_timeout<'a>(
    socket: &mut tcp::TcpSocket<'a>,
    buf: &mut [u8],
) -> Result<(), Error> {
    let res = with_timeout(RXTX_TIMEOUT, socket.read_exact(buf)).await?;
    res?;
    Ok(())
}

async fn flush_with_timeout<'a>(socket: &mut tcp::TcpSocket<'a>) -> Result<(), Error> {
    let res = with_timeout(RXTX_TIMEOUT, socket.flush()).await?;
    res?;
    Ok(())
}

/// Initiate a system reset request to reset the MCU
///
/// # Safety
/// This is a reboot.
unsafe fn sw_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

fn fw_update_error_to_status_code(e: &FirmwareUpdaterError) -> StatusCode {
    match e {
        FirmwareUpdaterError::Flash(_) => StatusCode::FlashError,
        FirmwareUpdaterError::Signature(_) => StatusCode::Signature,
        FirmwareUpdaterError::BadState => StatusCode::BadState,
    }
}

impl From<tcp::AcceptError> for Error {
    fn from(value: tcp::AcceptError) -> Self {
        Error::Accept(value)
    }
}

impl From<tcp::Error> for Error {
    fn from(value: tcp::Error) -> Self {
        Error::Socket(value)
    }
}

impl From<fmt::Error> for Error {
    fn from(_value: fmt::Error) -> Self {
        Error::Fmt
    }
}

impl From<FirmwareUpdaterError> for Error {
    fn from(value: FirmwareUpdaterError) -> Self {
        match value {
            FirmwareUpdaterError::Flash(_) => Error::Flash,
            FirmwareUpdaterError::Signature(_) => Error::FirmwareSignature,
            FirmwareUpdaterError::BadState => Error::BadBootloaderState,
        }
    }
}

impl From<embedded_io_async::ReadExactError<tcp::Error>> for Error {
    fn from(value: embedded_io_async::ReadExactError<tcp::Error>) -> Self {
        use embedded_io_async::ReadExactError::*;
        match value {
            UnexpectedEof => Error::Connection,
            Other(e) => Error::Socket(e),
        }
    }
}

impl From<embassy_time::TimeoutError> for Error {
    fn from(_value: embassy_time::TimeoutError) -> Self {
        Error::Connection
    }
}
