use crate::{
    archive_util,
    device_util::{self, DeviceInfo},
    interruptor::Interruptor,
    opts::DeviceUpdate,
};
use anyhow::{anyhow, bail, Result};
use elf::{endian::LittleEndian, ElfBytes};
use std::{fs, io::Write, net};
use tokio::{
    io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader},
    net::TcpStream,
};
use tracing::debug;
use wire_protocols::device::{Command, FirmwareUpdateHeader};

pub async fn update(cmd: DeviceUpdate, _intr: Interruptor) -> Result<()> {
    if !cmd.elf_file.exists() {
        bail!(
            "Firmware ELF file '{}' does not exist",
            cmd.elf_file.display()
        );
    }

    println!(
        "Updating device at {}:{} with firmware '{}'",
        cmd.common.address,
        cmd.common.port,
        cmd.elf_file.display()
    );

    if let Some(c) = cmd.cache_dir.as_ref() {
        debug!("Using image cache dir '{}'", c.display());
        fs::create_dir_all(c)?;
    }

    let elf_data = archive_util::load_elf(&cmd.elf_file)?;
    let elf = ElfBytes::<LittleEndian>::minimal_parse(&elf_data)?;
    let bin_data = archive_util::elf2bin(&elf)?;

    // At this point the ELF looks ok

    let s = net::TcpStream::connect((cmd.common.address.as_str(), cmd.common.port))?;
    s.set_nonblocking(true)?;
    s.set_nodelay(true)?;
    let mut stream = TcpStream::from_std(s)?;

    debug!("Requesting device info");
    device_util::write_command(Command::Info, &mut stream).await?;
    let _status = device_util::read_status(&mut stream).await?;

    let mut buf_stream = BufReader::new(stream);
    let mut info_str = String::new();
    let _info_len = buf_stream.read_line(&mut info_str).await?;
    let info = DeviceInfo::from_json(&info_str)?;
    let stream = buf_stream.into_inner();
    if cmd.common.format.is_text() {
        println!("{info:#?}");
    }

    // TODO - check bootloader_state

    let s = stream.into_std()?;
    s.shutdown(net::Shutdown::Both)?;
    drop(s);

    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    // Re-connect after info command
    let s = net::TcpStream::connect((cmd.common.address.as_str(), cmd.common.port))?;
    s.set_nonblocking(true)?;
    s.set_nodelay(true)?;
    let mut stream = TcpStream::from_std(s)?;

    if let Some(c) = cmd.cache_dir.as_ref() {
        let mut bin_path = c.join(cmd.elf_file.file_stem().unwrap());
        bin_path.set_extension("bin");
        debug!("Writing bin '{}'", bin_path.display());
        fs::write(bin_path, &bin_data)?;
    }

    if cmd.common.format.is_text() {
        println!("Wrting bin to device, {} bytes", bin_data.len());
    }
    let mut write_offset = 0_u32;
    let num_chunks = divide_round_up(bin_data.len(), FirmwareUpdateHeader::MAX_CHUCK_SIZE);
    for (chunk_idx, chunk) in bin_data
        .chunks(FirmwareUpdateHeader::MAX_CHUCK_SIZE)
        .enumerate()
    {
        debug!(
            "Sending bin chunk offset=0x{:X}, len=0x{:X}, {} of {}",
            write_offset,
            chunk.len(),
            chunk_idx + 1,
            num_chunks,
        );

        let mem_region_to_write = FirmwareUpdateHeader::new_unchecked(
            bin_data.len() as u32,
            write_offset,
            chunk.len() as u32,
        );
        mem_region_to_write
            .check_length()
            .map_err(|sc| anyhow!("FirmwareUpdateHeader to write is invalid. {sc}"))?;
        device_util::write_command(Command::WriteFirmware, &mut stream).await?;
        stream.write_all(&mem_region_to_write.to_le_bytes()).await?;
        stream.write_all(chunk).await?;
        let _status = device_util::read_status(&mut stream).await?;

        write_offset += mem_region_to_write.length;
    }

    // TODO verify stuff

    if cmd.common.format.is_text() {
        println!("Update complete, issue reboot command");
    }

    device_util::write_command(Command::CompleteAndReboot, &mut stream).await?;

    let _status = device_util::read_status(&mut stream).await?;

    // TODO read status

    Ok(())
}

// a/b
fn divide_round_up(a: usize, b: usize) -> usize {
    (a + (b - 1)) / b
}
