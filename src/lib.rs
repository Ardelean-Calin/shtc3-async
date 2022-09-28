#![no_std]

use core::prelude::rust_2021::*;

mod crc;
mod types;

use crc::crc8;
pub use types::*;

#[cfg(feature = "defmt")]
use defmt::write;
#[cfg(feature = "defmt")]
use defmt::Format;
use embedded_hal_async::delay::DelayUs;
use embedded_hal_async::i2c::{self};

/// All possible errors in this crate
#[derive(Debug, PartialEq, Clone)]
pub enum SHTC3Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
    /// Some kind of timing related error.
    TimingError,
}

#[cfg(feature = "defmt")]
impl<E> Format for SHTC3Error<E> {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "{:?}", self);
    }
}

/// The I2C address of the SHTC3 sensor.
const ADDRESS: u8 = 0x70;

/// Whether temperature or humidity is returned first when doing a measurement.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum MeasurementOrder {
    TemperatureFirst,
    #[allow(dead_code)]
    HumidityFirst,
}
use MeasurementOrder::*;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PowerMode {
    /// Normal measurement.
    NormalMode,
    /// Low power measurement: Less energy consumption, but repeatability and
    /// accuracy of measurements are negatively impacted.
    LowPower,
}

/// I²C commands sent to the sensor.
#[derive(Debug, Copy, Clone)]
enum Command {
    /// Go into sleep mode.
    Sleep,
    /// Wake up from sleep mode.
    WakeUp,
    /// Measurement commands.
    Measure {
        power_mode: PowerMode,
        order: MeasurementOrder,
    },
    /// Software reset.
    SoftwareReset,
    /// Read ID register.
    ReadIdRegister,
}

impl Command {
    fn as_bytes(self) -> [u8; 2] {
        match self {
            Command::Sleep => [0xB0, 0x98],
            Command::WakeUp => [0x35, 0x17],
            Command::Measure {
                power_mode: PowerMode::NormalMode,
                order: TemperatureFirst,
            } => [0x78, 0x66],
            Command::Measure {
                power_mode: PowerMode::NormalMode,
                order: HumidityFirst,
            } => [0x58, 0xE0],
            Command::Measure {
                power_mode: PowerMode::LowPower,
                order: TemperatureFirst,
            } => [0x60, 0x9C],
            Command::Measure {
                power_mode: PowerMode::LowPower,
                order: HumidityFirst,
            } => [0x40, 0x1A],
            Command::ReadIdRegister => [0xEF, 0xC8],
            Command::SoftwareReset => [0x80, 0x5D],
        }
    }
}

#[derive(Debug, Default)]
pub struct Shtc3<I2C: i2c::I2c> {
    i2c: I2C,
}

impl<I2C, E> Shtc3<I2C>
where
    I2C: i2c::I2c<Error = E> + 'static,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    fn validate_crc(&self, buf: &[u8]) -> Result<(), SHTC3Error<E>> {
        for chunk in buf.chunks(3) {
            if chunk.len() == 3 && crc8(&[chunk[0], chunk[1]]) != chunk[2] {
                return Err(SHTC3Error::Crc);
            }
        }
        Ok(())
    }

    async fn send_command(&mut self, command: Command) -> Result<(), SHTC3Error<E>> {
        self.i2c
            .write(ADDRESS, &command.as_bytes())
            .await
            .map_err(SHTC3Error::I2c)
    }

    async fn read_with_crc(&mut self, mut buf: &mut [u8]) -> Result<(), SHTC3Error<E>> {
        self.i2c
            .read(ADDRESS, &mut buf)
            .await
            .map_err(SHTC3Error::I2c)?;
        self.validate_crc(buf)
    }

    /// Get the device ID of your SHTC3.
    pub async fn get_device_id(&mut self) -> Result<u8, SHTC3Error<E>> {
        let mut buf = [0u8; 3];

        self.send_command(Command::ReadIdRegister).await?;
        self.read_with_crc(&mut buf).await?;

        let ident = u16::from_be_bytes([buf[0], buf[1]]);

        let lsb = (ident & 0b0011_1111) as u8;
        let msb = ((ident & 0b00001000_00000000) >> 5) as u8;
        Ok(lsb | msb)
    }

    pub async fn reset(&mut self) -> Result<(), SHTC3Error<E>> {
        self.send_command(Command::SoftwareReset).await?;
        Ok(())
    }

    pub async fn wakeup(&mut self, delay: &mut impl DelayUs) -> Result<(), SHTC3Error<E>> {
        const WAKEUP_TIME_US: u32 = 240;

        self.send_command(Command::WakeUp).await?;
        delay
            .delay_us(WAKEUP_TIME_US)
            .await
            .map_err(|_| SHTC3Error::TimingError)
    }

    pub async fn sleep(&mut self) -> Result<(), SHTC3Error<E>> {
        self.send_command(Command::Sleep).await?;
        Ok(())
    }

    pub async fn measure(
        &mut self,
        delay: &mut impl DelayUs,
    ) -> Result<Measurement, SHTC3Error<E>> {
        self.send_command(Command::Measure {
            power_mode: PowerMode::NormalMode,
            order: TemperatureFirst,
        })
        .await?;

        delay
            .delay_us(12100)
            .await
            .map_err(|_| SHTC3Error::TimingError)?;

        let measurement = self.get_measurement_result().await?;

        Ok(measurement)
    }

    /// Read the result of a temperature / humidity measurement.
    pub async fn get_measurement_result(&mut self) -> Result<Measurement, SHTC3Error<E>> {
        let raw = self.get_raw_measurement_result().await?;
        Ok(raw.into())
    }

    /// Read the raw result of a combined temperature / humidity measurement.
    pub async fn get_raw_measurement_result(&mut self) -> Result<RawMeasurement, SHTC3Error<E>> {
        let mut buf = [0; 6];
        self.read_with_crc(&mut buf).await?;

        Ok(RawMeasurement {
            temperature: u16::from_be_bytes([buf[0], buf[1]]),
            humidity: u16::from_be_bytes([buf[3], buf[4]]),
        })
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     // #[test]
//     // fn it_works() {
//     //     let result = add(2, 2);
//     //     assert_eq!(result, 4);
//     // }
// }
