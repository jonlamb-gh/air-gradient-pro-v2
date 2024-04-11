//! The device protocol covers communication related to device information and
//! control and firmware updates, usually over TCP.
//! Everything is little endian.

use byteorder::{ByteOrder, LittleEndian};
use core::fmt;

pub const DEFAULT_PORT: u16 = 32101;
pub const SOCKET_BUFFER_LEN: usize = FirmwareUpdateHeader::MAX_CHUCK_SIZE + 256;

/// Commands are received by the device.
/// The device always responds with a `StatusCode`, possibly
/// followed by a response type.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum Command {
    /// Request device information.
    /// This command also causes the device to reset its connection after sending a response.
    /// It can be used to abort an in-progress update too.
    /// Request type: None
    /// Response type: json string
    Info,

    /// Write a chunk of new firmware data to the devices DFU region.
    /// Request type: FirmwareUpdateHeader followed by [u8] data
    /// Response type: None
    WriteFirmware,

    /// Mark the update as complete and schedule a system reboot if the
    /// firmware hash matches the provided hash.
    /// Request type: FirmwareHash (32 bytes)
    /// Response type: None
    CompleteAndReboot,

    /// Mark firmware boot successful and stop rollback on reset.
    /// If there was no previous update applied, this does nothing.
    /// Request type: None
    /// Response type: None
    ConfirmUpdate,

    /// Reboot the device.
    /// Request type: None
    /// Response type: None
    Reboot,

    /// Unknown command.
    /// The device will always response with StatusCode::UnknownCommand.
    /// Request type: None
    /// Response type: None
    Unknown(u32),
}

impl Command {
    pub const WIRE_SIZE: usize = 4;

    pub fn from_le_bytes_unchecked(value: &[u8]) -> Self {
        Command::from(LittleEndian::read_u32(value))
    }

    pub fn from_le_bytes(value: &[u8]) -> Option<Self> {
        if value.len() >= 4 {
            Some(Self::from_le_bytes_unchecked(value))
        } else {
            None
        }
    }
}

impl From<u32> for Command {
    fn from(value: u32) -> Self {
        use Command::*;
        match value {
            1 => Info,
            2 => WriteFirmware,
            3 => CompleteAndReboot,
            4 => ConfirmUpdate,
            5 => Reboot,
            _ => Unknown(value),
        }
    }
}

impl From<Command> for u32 {
    fn from(value: Command) -> Self {
        use Command::*;
        match value {
            Info => 1,
            WriteFirmware => 2,
            CompleteAndReboot => 3,
            ConfirmUpdate => 4,
            Reboot => 5,
            Unknown(v) => v,
        }
    }
}

impl fmt::Display for Command {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub struct FirmwareUpdateHeader {
    /// The total size of the new firmware binary, in bytes.
    /// NOTE: this is pretty wasteful, currently just used to update a progress bar on-device.
    pub total_size: u32,

    /// The starting offset within the firmware area where data writing should begin
    pub offset: u32,

    /// Size in bytes of the region.
    /// It must be a multiple of NorFlash WRITE_SIZE.
    pub length: u32,
}

impl FirmwareUpdateHeader {
    pub const WIRE_SIZE: usize = 12;
    pub const MAX_CHUCK_SIZE: usize = 1024;

    pub fn new_unchecked(total_size: u32, offset: u32, length: u32) -> Self {
        Self {
            total_size,
            offset,
            length,
        }
    }

    pub fn check_length(&self) -> Result<(), StatusCode> {
        if self.length % 4 != 0 {
            Err(StatusCode::LengthNotMultiple4)
        } else if self.length > Self::MAX_CHUCK_SIZE as u32 {
            Err(StatusCode::LengthTooLong)
        } else if self.length == 0 {
            Err(StatusCode::DataLengthIncorrect)
        } else {
            Ok(())
        }
    }

    pub fn to_le_bytes(self) -> [u8; Self::WIRE_SIZE] {
        let size = self.total_size.to_le_bytes();
        let offset = self.offset.to_le_bytes();
        let len = self.length.to_le_bytes();
        [
            size[0], size[1], size[2], size[3], offset[0], offset[1], offset[2], offset[3], len[0],
            len[1], len[2], len[3],
        ]
    }
}

/// SHA256 hash of the firmware
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub struct FirmwareHash(pub [u8; 32]);

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum StatusCode {
    Success,
    InternalError,
    UnknownCommand,
    LengthNotMultiple4,
    LengthTooLong,
    DataLengthIncorrect,
    FlashError,
    BadState,
    Signature,
    NoUpdatePending,
    Unknown(u32),
}

impl StatusCode {
    pub fn from_le_bytes_unchecked(value: &[u8]) -> Self {
        StatusCode::from(LittleEndian::read_u32(value))
    }

    pub fn from_le_bytes(value: &[u8]) -> Option<Self> {
        if value.len() >= 4 {
            Some(Self::from_le_bytes_unchecked(value))
        } else {
            None
        }
    }

    pub fn is_success(&self) -> bool {
        matches!(self, StatusCode::Success)
    }
}

impl From<u32> for StatusCode {
    fn from(value: u32) -> Self {
        use StatusCode::*;
        match value {
            0 => Success,
            1 => InternalError,
            2 => UnknownCommand,
            3 => LengthNotMultiple4,
            4 => LengthTooLong,
            5 => DataLengthIncorrect,
            6 => FlashError,
            7 => BadState,
            8 => Signature,
            9 => NoUpdatePending,
            _ => Unknown(value),
        }
    }
}

impl From<StatusCode> for u32 {
    fn from(value: StatusCode) -> Self {
        use StatusCode::*;
        match value {
            Success => 0,
            InternalError => 1,
            UnknownCommand => 2,
            LengthNotMultiple4 => 3,
            LengthTooLong => 4,
            DataLengthIncorrect => 5,
            FlashError => 6,
            BadState => 7,
            Signature => 8,
            NoUpdatePending => 9,
            Unknown(v) => v,
        }
    }
}

impl fmt::Display for StatusCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_wire_command() {
        for in_c in 0..0xFF_u32 {
            let in_c_bytes = in_c.to_le_bytes();
            let c = Command::from_le_bytes(&in_c_bytes).unwrap();
            assert_eq!(in_c, u32::from(c));
        }
    }

    #[test]
    fn round_trip_status_code() {
        for in_c in 0..0xFF_u32 {
            let in_c_bytes = in_c.to_le_bytes();
            let c = StatusCode::from_le_bytes(&in_c_bytes).unwrap();
            assert_eq!(in_c, u32::from(c));
        }
    }

    #[test]
    fn fw_update_header() {
        let h = FirmwareUpdateHeader::new_unchecked(
            0,
            1024,
            FirmwareUpdateHeader::MAX_CHUCK_SIZE as u32,
        );
        assert!(h.check_length().is_ok());

        let h = FirmwareUpdateHeader::new_unchecked(0, 0, 13);
        assert_eq!(h.check_length(), Err(StatusCode::LengthNotMultiple4));

        let h = FirmwareUpdateHeader::new_unchecked(
            0,
            1024,
            FirmwareUpdateHeader::MAX_CHUCK_SIZE as u32 + 4,
        );
        assert_eq!(h.check_length(), Err(StatusCode::LengthTooLong));
    }
}
