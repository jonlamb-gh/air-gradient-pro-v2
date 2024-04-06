use crate::{
    built_info,
    common::{BootloaderState, DeviceInfo},
    config,
    reset_reason::ResetReason,
};
use wire_protocols::{DeviceSerialNumber, ProtocolVersion};

const NA: &str = "NA";

pub(crate) fn read_device_serial_number() -> DeviceSerialNumber {
    let word0 = unsafe { *(0x1FFF_7A10 as *const u32) };
    let word1 = unsafe { *(0x1FFF_7A14 as *const u32) };
    let word2 = unsafe { *(0x1FFF_7A18 as *const u32) };
    DeviceSerialNumber::new(word0, word1, word2)
}

pub(crate) fn device_info(
    bootloader_state: BootloaderState,
    reset_reason: ResetReason,
) -> DeviceInfo {
    DeviceInfo {
        protocol_version: ProtocolVersion::v1(),
        firmware_version: config::FIRMWARE_VERSION,
        device_id: config::DEVICE_ID,
        device_serial_number: read_device_serial_number(),
        mac_address: config::MAC_ADDRESS,
        bootloader_state,
        reset_reason,
        built_time_utc: built_info::BUILT_TIME_UTC,
        git_commit: built_info::GIT_COMMIT_HASH.unwrap_or(NA),
    }
}
