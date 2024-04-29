// NOTE: based on https://github.com/jonlamb-gh/pms-7003,
// my fork of https://github.com/g-bartoszek/pms-7003
//
// NOTE: This impl is a mess.
// I have two of these sensors, one of them always wakes up in active mode
// (older one on the PRO board)
// and the other wakes up in mode it was in before sleeping (newer one on the DIY board).
// They also differ in timing around receiving commands after wakeup.

use core::fmt;
use defmt::{debug, error, trace, warn, Format};
use embassy_stm32::{
    mode, peripherals,
    usart::{self, RingBufferedUartRx, Uart, UartTx},
};
use embassy_time::{with_timeout, Duration, Timer};
use scroll::{Pread, Pwrite, BE};

const CMD_FRAME_SIZE: usize = 7;
const OUTPUT_FRAME_SIZE: usize = 32;
const RESPONSE_FRAME_SIZE: usize = 8;
const CHECKSUM_SIZE: usize = 2;
const HEADER_SIZE: usize = 4;

type Response = [u8; RESPONSE_FRAME_SIZE];

const MN1: u8 = 0x42;
const MN2: u8 = 0x4D;
const PASSIVE_MODE_RESPONSE: Response = [MN1, MN2, 0x00, 0x04, 0xE1, 0x00, 0x01, 0x74];
const ACTIVE_MODE_RESPONSE: Response = [MN1, MN2, 0x00, 0x04, 0xE1, 0x01, 0x01, 0x75];
const SLEEP_RESPONSE: Response = [MN1, MN2, 0x00, 0x04, 0xE4, 0x00, 0x01, 0x77];

// sleep:   424DE400000173
// wake:    424DE400010174
// active:  424DE100010171
// passive: 424DE100000170
// request: 424DE200000171

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct Measurement {
    /// PM2.5 concentration unit μ g/m3 (under atmospheric environment)
    pub pm2_5_atm: u16,
}

#[derive(Debug)]
pub enum Error {
    /// Timeout waiting for response
    Timeout,
    /// Unexpected response
    Response,
    /// Protocol error
    Protocol,
    /// Bad checksum
    Crc,
    /// Serial error
    Serial(usart::Error),
}

pub const RX_BUFFER_SIZE: usize = OUTPUT_FRAME_SIZE * 8;
const RX_RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);
const MAX_RETRY_COUNT: usize = 6;

pub type DefaultPms5003 = Pms5003<'static, peripherals::USART2>;

pub struct Pms5003<'d, Serial>
where
    Serial: usart::BasicInstance,
{
    reader: FrameReader,
    tx: UartTx<'d, Serial, mode::Async>,
    rx: RingBufferedUartRx<'d, Serial>,
}

impl<'d, Serial> Pms5003<'d, Serial>
where
    Serial: usart::BasicInstance,
{
    pub async fn new(
        serial: Uart<'d, Serial, mode::Async>,
        rx_buffer: &'static mut [u8; RX_BUFFER_SIZE],
    ) -> Result<Self, Error> {
        let (tx, rx) = serial.split();
        let mut drv = Self {
            reader: FrameReader::default(),
            tx,
            rx: rx.into_ring_buffered(rx_buffer),
        };

        drv.rx.start()?;

        // Get into a clean state to deal with reboots and power-on states
        // consistently
        debug!("PMS5003: entering init ready mode");
        drv.enter_ready_mode().await?;

        debug!("PMS5003: entering init standby mode");
        drv.enter_standby_mode().await?;

        Ok(drv)
    }

    pub async fn enter_standby_mode(&mut self) -> Result<(), Error> {
        Timer::after_millis(100).await;
        self.sleep().await?;
        self.rx.start()?;
        Ok(())
    }

    pub async fn enter_ready_mode(&mut self) -> Result<(), Error> {
        Timer::after_millis(100).await;
        self.wake()?;
        Timer::after_millis(100).await;

        let mut cnt = 0;
        loop {
            cnt += 1;
            match self.passive().await {
                Ok(()) => break,
                Err(e) => match e {
                    Error::Timeout => {
                        if cnt >= MAX_RETRY_COUNT {
                            warn!("PMS5003 exceeded max retry count");
                            return Err(Error::Response);
                        }
                        debug!("PMS5003: timeout on passive mode, retrying");
                        continue;
                    }
                    _ => return Err(e),
                },
            }
        }

        // Clears the ring buffer
        self.rx.start()?;

        Ok(())
    }

    pub async fn measure(&mut self) -> Result<Measurement, Error> {
        trace!("PMS5003 sending request");
        self.request()?;
        let bytes_read = self.reader.read_frame(&mut self.rx).await?;

        if bytes_read != OUTPUT_FRAME_SIZE {
            error!(
                "PMS5003: bad output frame response, got {} bytes, {=[u8]:X}",
                bytes_read,
                self.reader.buffer[..bytes_read]
            );
            Err(Error::Response)
        } else {
            let f = OutputFrame::from_buffer(&self.reader.buffer)?;
            Ok(Measurement {
                pm2_5_atm: f.pm2_5_atm,
            })
        }
    }

    /// Requests status in passive mode
    fn request(&mut self) -> Result<(), Error> {
        self.send_cmd(&create_command(0xe2, 0))
    }

    async fn sleep(&mut self) -> Result<(), Error> {
        self.send_cmd(&create_command(0xe4, 0))?;
        self.wait_for_response(&SLEEP_RESPONSE).await
    }

    fn wake(&mut self) -> Result<(), Error> {
        self.send_cmd(&create_command(0xe4, 1))
    }

    /// Passive mode - sensor reports air quality on request
    async fn passive(&mut self) -> Result<(), Error> {
        let cmd = create_command(0xe1, 0);
        self.send_cmd(&cmd)?;
        self.wait_for_response(&PASSIVE_MODE_RESPONSE).await
    }

    /// Active mode - sensor reports air quality continuously
    #[allow(unused)]
    async fn active(&mut self) -> Result<(), Error> {
        let cmd = create_command(0xe1, 1);
        self.send_cmd(&cmd)?;
        self.wait_for_response(&ACTIVE_MODE_RESPONSE).await
    }

    fn send_cmd(&mut self, cmd: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(cmd)?;
        self.tx.blocking_flush()?;
        Ok(())
    }

    // Loops until expected response is found, stops after
    // a full RX_BUFFER_SIZE worth of data has been observed
    async fn wait_for_response(&mut self, expected_response: &Response) -> Result<(), Error> {
        match with_timeout(
            RX_RESPONSE_TIMEOUT,
            self.read_response_retrying(expected_response),
        )
        .await
        {
            Ok(res) => res,
            Err(_timeout) => {
                warn!(
                    "PMS5003: timeout waiting for response {=[u8]:X}",
                    expected_response.as_slice()
                );
                Err(Error::Timeout)
            }
        }
    }

    async fn read_response_retrying(&mut self, expected_response: &Response) -> Result<(), Error> {
        let mut bytes_read = 0;
        loop {
            let frame_size = self.reader.read_frame(&mut self.rx).await?;
            bytes_read += frame_size;

            if &self.reader.buffer[..frame_size] == expected_response {
                return Ok(());
            } else {
                debug!(
                    "PMS5003: bad response, expecting {=[u8]:X}, got {} bytes, {=[u8]:X}",
                    expected_response.as_slice(),
                    frame_size,
                    self.reader.buffer[..frame_size]
                );
            }

            if bytes_read >= RX_BUFFER_SIZE {
                break;
            }
        }

        Err(Error::Timeout)
    }
}

fn create_command(cmd: u8, data: u16) -> [u8; CMD_FRAME_SIZE] {
    let mut buffer = [0_u8; CMD_FRAME_SIZE];
    let mut offset = 0usize;

    buffer.gwrite::<u8>(MN1, &mut offset).unwrap();
    buffer.gwrite::<u8>(MN2, &mut offset).unwrap();
    buffer.gwrite::<u8>(cmd, &mut offset).unwrap();
    buffer.gwrite_with::<u16>(data, &mut offset, BE).unwrap();

    let checksum = buffer
        .iter()
        .take(CMD_FRAME_SIZE - CHECKSUM_SIZE)
        .map(|b| *b as u16)
        .sum::<u16>();
    buffer
        .gwrite_with::<u16>(checksum, &mut offset, BE)
        .unwrap();

    buffer
}

#[derive(Default, Debug)]
struct OutputFrame {
    start1: u8,
    start2: u8,
    frame_length: u16,
    pm1_0: u16,
    pm2_5: u16,
    pm10: u16,
    pm1_0_atm: u16,
    pm2_5_atm: u16,
    pm10_atm: u16,
    beyond_0_3: u16,
    beyond_0_5: u16,
    beyond_1_0: u16,
    beyond_2_5: u16,
    beyond_5_0: u16,
    beyond_10_0: u16,
    reserved: u16,
    check: u16,
}

impl OutputFrame {
    pub fn from_buffer(buffer: &[u8; OUTPUT_FRAME_SIZE]) -> Result<Self, Error> {
        let sum: usize = buffer
            .iter()
            .take(OUTPUT_FRAME_SIZE - CHECKSUM_SIZE)
            .map(|e| *e as usize)
            .sum();

        let mut frame = OutputFrame::default();
        let mut offset = 0usize;

        frame.start1 = buffer.gread::<u8>(&mut offset).unwrap();
        frame.start2 = buffer.gread::<u8>(&mut offset).unwrap();
        frame.frame_length = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm1_0 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm2_5 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm10 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm1_0_atm = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm2_5_atm = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.pm10_atm = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_0_3 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_0_5 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_1_0 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_2_5 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_5_0 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.beyond_10_0 = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.reserved = buffer.gread_with::<u16>(&mut offset, BE).unwrap();
        frame.check = buffer.gread_with::<u16>(&mut offset, BE).unwrap();

        if sum != frame.check as usize {
            return Err(Error::Crc);
        }

        Ok(frame)
    }
}

impl fmt::Display for Measurement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PMS5003 pm2_5_atm: {}", self.pm2_5_atm)
    }
}

impl From<usart::Error> for Error {
    fn from(value: usart::Error) -> Self {
        Error::Serial(value)
    }
}

#[derive(Debug, Default, defmt::Format)]
enum FrameReaderState {
    #[default]
    WaitingForFirstMagicNumber,
    WaitingForSecondMagicNumber,
    WaitingForLenMsb,
    WaitingForLenLsb,
    Reading,
}

#[derive(Debug, Default)]
struct FrameReader {
    state: FrameReaderState,
    index: usize,
    frame_len: usize,
    buffer: [u8; OUTPUT_FRAME_SIZE],
}

impl FrameReader {
    fn reset(&mut self) {
        self.state = FrameReaderState::default();
        self.index = 0;
        self.frame_len = 0;
    }

    // NOTE: doesn't check the checksum
    async fn read_frame<'d, Serial>(
        &mut self,
        serial: &mut RingBufferedUartRx<'d, Serial>,
    ) -> Result<usize, Error>
    where
        Serial: usart::BasicInstance,
    {
        self.reset();
        loop {
            match self.update(serial).await? {
                0 => continue,
                frame_len => return Ok(HEADER_SIZE + frame_len),
            }
        }
    }

    async fn update<'d, Serial>(
        &mut self,
        serial: &mut RingBufferedUartRx<'d, Serial>,
    ) -> Result<usize, Error>
    where
        Serial: usart::BasicInstance,
    {
        match self.state {
            FrameReaderState::WaitingForFirstMagicNumber => {
                let _ = serial.read(&mut self.buffer[0..1]).await;
                if self.buffer[0] == MN1 {
                    self.index += 1;
                    self.state = FrameReaderState::WaitingForSecondMagicNumber;
                }
            }
            FrameReaderState::WaitingForSecondMagicNumber => {
                let _ = serial.read(&mut self.buffer[1..2]).await;
                if self.buffer[1] == MN2 {
                    self.index += 1;
                    self.state = FrameReaderState::WaitingForLenMsb;
                } else {
                    warn!("PMS5003 invalid second magic, reseting reader");
                    self.reset();
                }
            }
            FrameReaderState::WaitingForLenMsb => {
                let _ = serial.read(&mut self.buffer[2..3]).await;
                self.index += 1;
                self.state = FrameReaderState::WaitingForLenLsb;
            }
            FrameReaderState::WaitingForLenLsb => {
                let _ = serial.read(&mut self.buffer[3..4]).await;
                self.index += 1;
                self.frame_len = u16::from_be_bytes([self.buffer[2], self.buffer[3]]) as usize;
                self.state = FrameReaderState::Reading;

                if (HEADER_SIZE + self.frame_len) > self.buffer.len() {
                    warn!("PMS5003 dropping invalid frame len={}", self.frame_len);
                    self.reset();
                }
            }
            FrameReaderState::Reading => {
                let tail_index = self.frame_len + HEADER_SIZE;
                let bytes_read = serial
                    .read(&mut self.buffer[self.index..tail_index])
                    .await?;

                self.index += bytes_read;

                if self.index > self.frame_len + HEADER_SIZE {
                    error!(
                        "PMS5003: recvd excessive data len={}, expected={}",
                        self.index,
                        self.frame_len + HEADER_SIZE
                    );
                    self.reset();
                    return Err(Error::Protocol);
                }

                if self.index == self.frame_len + HEADER_SIZE {
                    let frame_len = self.frame_len;
                    trace!("PMS5003 frame complete frame_len={}", frame_len);
                    self.reset();
                    return Ok(frame_len);
                }
            }
        }

        trace!(
            "PMS5003 state={}, frame_len={}, index={}, {=[u8]:X}",
            self.state,
            self.frame_len,
            self.index,
            &self.buffer[..self.index]
        );

        Ok(0)
    }
}
