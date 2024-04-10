// NOTE: based on https://github.com/jonlamb-gh/pms-7003,
// my fork of https://github.com/g-bartoszek/pms-7003

use core::fmt;
use defmt::{debug, error, Format};
use embassy_stm32::{
    dma::NoDma,
    peripherals,
    usart::{self, Uart},
};
use embassy_time::Timer;
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
const SLEEP_RESPONSE: Response = [MN1, MN2, 0x00, 0x04, 0xE4, 0x00, 0x01, 0x77];

// sleep command: https://github.com/esphome/feature-requests/issues/2033

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct Measurement {
    /// PM2.5 concentration unit μ g/m3 (under atmospheric environment)
    pub pm2_5_atm: u16,
}

#[derive(Debug)]
pub enum Error {
    /// Bad response
    Response,
    /// Bad checksum
    Crc,
    /// Serial error
    Serial(usart::Error),
}

pub type DefaultPms5003 = Pms5003<'static, peripherals::USART2, peripherals::DMA1_CH5>;

pub struct Pms5003<'d, Serial, RxDma>
where
    Serial: usart::BasicInstance,
    RxDma: usart::RxDma<Serial>,
{
    reader: FrameReader,
    serial: Uart<'d, Serial, NoDma, RxDma>,
}

impl<'d, Serial, RxDma> Pms5003<'d, Serial, RxDma>
where
    Serial: usart::BasicInstance,
    RxDma: usart::RxDma<Serial>,
{
    pub async fn new(serial: Uart<'d, Serial, NoDma, RxDma>) -> Result<Self, Error> {
        let mut drv = Self {
            reader: FrameReader::default(),
            serial,
        };

        // Default mode after power up is active mode
        // Wake up and read to flush the line before
        // changing modes and sleeping in case
        // it was just a reboot not power cycle
        // the pms_7003 lib only works this way currently
        drv.wake()?;
        Timer::after_millis(100).await;
        let _ = drv.reader.read_frame(&mut drv.serial).await?;
        drv.passive().await?;

        debug!("PMS5003: entering standby mode");
        drv.enter_standby_mode().await?;

        // TODO
        Ok(drv)
    }

    pub async fn enter_standby_mode(&mut self) -> Result<(), Error> {
        Timer::after_millis(100).await;
        self.sleep().await
    }

    // NOTE: the sensor wakes up from sleep in active mode
    pub async fn enter_ready_mode(&mut self) -> Result<(), Error> {
        self.wake()?;
        Timer::after_millis(100).await;
        let _ = self.reader.read_frame(&mut self.serial).await?;
        self.passive().await?;
        Ok(())
    }

    pub async fn measure(&mut self) -> Result<Measurement, Error> {
        self.request()?;
        let bytes_read = self.reader.read_frame(&mut self.serial).await?;

        if bytes_read != OUTPUT_FRAME_SIZE {
            error!(
                "PMS5003: bad response, got {} bytes, {:X}",
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
        self.receive_response(SLEEP_RESPONSE).await
    }

    fn wake(&mut self) -> Result<(), Error> {
        self.send_cmd(&create_command(0xe4, 1))
    }

    /// Passive mode - sensor reports air quality on request
    async fn passive(&mut self) -> Result<(), Error> {
        self.send_cmd(&create_command(0xe1, 0))?;
        self.receive_response(PASSIVE_MODE_RESPONSE).await
    }

    fn send_cmd(&mut self, cmd: &[u8]) -> Result<(), Error> {
        self.serial.blocking_write(cmd)?;
        self.serial.blocking_flush()?;
        Ok(())
    }

    async fn receive_response(&mut self, expected_response: Response) -> Result<(), Error> {
        let bytes_read = self.reader.read_frame(&mut self.serial).await?;

        if bytes_read != RESPONSE_FRAME_SIZE
            || self.reader.buffer[..RESPONSE_FRAME_SIZE] != expected_response
        {
            error!(
                "PMS5003: bad response, got {} bytes, {:X}",
                bytes_read,
                self.reader.buffer[..bytes_read]
            );
            Err(Error::Response)
        } else {
            Ok(())
        }
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
    WaitingForCrcMsb,
    WaitingForCrcLsb,
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

    fn increment(&mut self, byte: u8) {
        self.buffer[self.index] = byte;
        self.index += 1;
    }

    // NOTE: doesn't check the checksum
    async fn read_frame<'d, Serial, RxDma>(
        &mut self,
        serial: &mut Uart<'d, Serial, NoDma, RxDma>,
    ) -> Result<usize, Error>
    where
        Serial: usart::BasicInstance,
        RxDma: usart::RxDma<Serial>,
    {
        self.reset();
        loop {
            match self.update(serial).await? {
                0 => continue,
                frame_len => return Ok(HEADER_SIZE + frame_len),
            }
        }
    }

    async fn update<'d, Serial, RxDma>(
        &mut self,
        serial: &mut Uart<'d, Serial, NoDma, RxDma>,
    ) -> Result<usize, Error>
    where
        Serial: usart::BasicInstance,
        RxDma: usart::RxDma<Serial>,
    {
        match self.state {
            FrameReaderState::WaitingForFirstMagicNumber => {
                let byte = nb::block!(serial.nb_read())?;
                if byte == MN1 {
                    self.increment(byte);
                    self.state = FrameReaderState::WaitingForSecondMagicNumber;
                }
            }
            FrameReaderState::WaitingForSecondMagicNumber => {
                let byte = nb::block!(serial.nb_read())?;
                if byte == MN2 {
                    self.increment(byte);
                    self.state = FrameReaderState::WaitingForCrcMsb;
                }
            }
            FrameReaderState::WaitingForCrcMsb => {
                let byte = nb::block!(serial.nb_read())?;
                self.increment(byte);
                self.state = FrameReaderState::WaitingForCrcLsb;
            }
            FrameReaderState::WaitingForCrcLsb => {
                let byte = nb::block!(serial.nb_read())?;
                self.increment(byte);
                self.frame_len = u16::from_be_bytes([self.buffer[2], self.buffer[3]]) as usize;
                self.state = FrameReaderState::Reading;
            }
            FrameReaderState::Reading => {
                let bytes_read = serial
                    .read_until_idle(&mut self.buffer[self.index..])
                    .await?;

                self.index += bytes_read;

                if self.index > self.frame_len + HEADER_SIZE {
                    error!(
                        "PMS5003: recvd excessive data len={}, expected={}",
                        self.index,
                        self.frame_len + HEADER_SIZE
                    );
                    self.reset();
                    return Err(Error::Response);
                }

                if self.index == self.frame_len + HEADER_SIZE {
                    let frame_len = self.frame_len;
                    self.reset();
                    return Ok(frame_len);
                }
            }
        }

        Ok(0)
    }
}
