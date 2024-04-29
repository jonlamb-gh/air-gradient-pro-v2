use core::fmt;
use defmt::{error, Format};
use embassy_stm32::{
    mode, peripherals,
    usart::{self, Uart},
};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Format)]
pub struct Measurement {
    /// CO2 ppm
    pub co2: u16,
}

#[derive(Debug)]
pub enum Error {
    /// Bad response
    Response,
    Serial(usart::Error),
}
const RESP_SIZE: usize = 7;
const CMD: &[u8] = &[0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5];

pub type DefaultS8Lp = S8Lp<'static, peripherals::USART3>;

pub struct S8Lp<'d, Serial>
where
    Serial: usart::BasicInstance,
{
    resp_buffer: [u8; RESP_SIZE],
    serial: Uart<'d, Serial, mode::Async>,
}

impl<'d, Serial> S8Lp<'d, Serial>
where
    Serial: usart::BasicInstance,
{
    pub fn new(serial: Uart<'d, Serial, mode::Async>) -> Self {
        Self {
            resp_buffer: [0; 7],
            serial,
        }
    }

    pub async fn measure(&mut self) -> Result<Measurement, Error> {
        self.serial.blocking_write(CMD)?;
        self.serial.blocking_flush()?;

        // TODO - add a parser / frame sync logic
        let bytes_read = self.serial.read_until_idle(&mut self.resp_buffer).await?;
        if bytes_read != RESP_SIZE {
            error!("S8LP: bad response size {}", bytes_read);
            return Err(Error::Response);
        }

        // TODO - surface these error variants
        if self.resp_buffer[0] != 0xFE {
            error!("S8LP: bad address");
            Err(Error::Response)
        } else if self.resp_buffer[1] != 0x04 {
            error!("S8LP: bad function code");
            Err(Error::Response)
        } else if self.resp_buffer[2] != 0x02 {
            error!("S8LP: bad payload length");
            Err(Error::Response)
        } else {
            let crc = u16::from_le_bytes([self.resp_buffer[5], self.resp_buffer[6]]);
            let expected_crc = crc16(&self.resp_buffer, self.resp_buffer.len() as u8 - 2);

            if crc != expected_crc {
                error!("S8LP: bad CRC");
                Err(Error::Response)
            } else {
                let payload = u16::from_be_bytes([self.resp_buffer[3], self.resp_buffer[4]]);
                Ok(Measurement { co2: payload })
            }
        }
    }
}

impl fmt::Display for Measurement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "S8LP CO2: {}", self.co2)
    }
}

impl From<usart::Error> for Error {
    fn from(value: usart::Error) -> Self {
        Error::Serial(value)
    }
}

fn crc16(frame: &[u8], data_length: u8) -> u16 {
    let mut crc: u16 = 0xffff;
    for i in frame.iter().take(data_length as usize) {
        crc ^= u16::from(*i);
        for _ in (0..8).rev() {
            if (crc & 0x0001) == 0 {
                crc >>= 1;
            } else {
                crc >>= 1;
                crc ^= 0xA001;
            }
        }
    }
    crc
}
