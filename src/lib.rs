#![no_std]
#![feature(async_fn_in_trait)]

use core::prelude::rust_2021::*;

mod crc;
mod types;

use crc::crc8;
pub use types::*;

#[cfg(feature = "defmt")]
use defmt::write;
#[cfg(feature = "defmt")]
use defmt::Format;
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal_async::delay::DelayUs;

/// All possible errors in this crate
#[derive(Debug, PartialEq, Clone)]
pub enum SHTC3Error {
    /// I²C bus error
    I2c,
    /// CRC checksum validation failed
    Crc,
    /// Some kind of timing related error.
    TimingError,
}

#[cfg(feature = "defmt")]
impl Format for SHTC3Error {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "{:?}", self);
    }
}

#[cfg(feature = "defmt")]
impl Format for Measurement {
    fn format(&self, fmt: defmt::Formatter) {
        write!(
            fmt,
            "Temp: {}C\tHum: {}%",
            self.temperature.as_degrees_celsius(),
            self.humidity.as_percent()
        );
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

pub struct Shtc3<I2C> {
    i2c: I2C,
}

impl<I2C, E> Shtc3<I2C>
where
    I2C: Write<Error = E> + Read<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    fn validate_crc(&self, buf: &[u8]) -> Result<(), SHTC3Error> {
        for chunk in buf.chunks(3) {
            if chunk.len() == 3 && crc8(&[chunk[0], chunk[1]]) != chunk[2] {
                return Err(SHTC3Error::Crc);
            }
        }
        Ok(())
    }

    fn send_command(&mut self, command: Command) -> Result<(), SHTC3Error> {
        self.i2c
            .write(ADDRESS, &command.as_bytes())
            .map_err(|_| SHTC3Error::I2c)
    }

    fn read_with_crc(&mut self, mut buf: &mut [u8]) -> Result<(), SHTC3Error> {
        self.i2c
            .read(ADDRESS, &mut buf)
            .map_err(|_| SHTC3Error::I2c)?;
        self.validate_crc(buf)
    }

    /// Get the device ID of your SHTC3.
    pub fn get_device_id(&mut self) -> Result<u8, SHTC3Error> {
        let mut buf = [0u8; 3];

        self.send_command(Command::ReadIdRegister)?;
        self.read_with_crc(&mut buf)?;

        let ident = u16::from_be_bytes([buf[0], buf[1]]);

        let lsb = (ident & 0b0011_1111) as u8;
        let msb = ((ident & 0b00001000_00000000) >> 5) as u8;
        Ok(lsb | msb)
    }

    pub async fn reset(&mut self) -> Result<(), SHTC3Error> {
        self.send_command(Command::SoftwareReset)?;
        Ok(())
    }

    pub async fn wakeup(&mut self, delay: &mut impl DelayUs) -> Result<(), SHTC3Error> {
        const WAKEUP_TIME_US: u32 = 240;

        self.send_command(Command::WakeUp)?;
        delay
            .delay_us(WAKEUP_TIME_US)
            .await
            .map_err(|_| SHTC3Error::TimingError)
    }

    pub fn sleep(&mut self) -> Result<(), SHTC3Error> {
        self.send_command(Command::Sleep)?;
        Ok(())
    }

    pub async fn measure(
        &mut self,
        power_mode: PowerMode,
        delay: &mut impl DelayUs,
    ) -> Result<Measurement, SHTC3Error> {
        // Get the duration of a measurement.
        let delay_us = match power_mode {
            PowerMode::LowPower => 800,
            PowerMode::NormalMode => 12100,
        };

        self.send_command(Command::Measure {
            power_mode,
            order: TemperatureFirst,
        })?;

        // Delay until data is ready.
        delay
            .delay_us(delay_us)
            .await
            .map_err(|_| SHTC3Error::TimingError)?;

        let measurement = self.get_measurement_result()?;

        Ok(measurement)
    }

    /// Read the result of a temperature / humidity measurement.
    pub fn get_measurement_result(&mut self) -> Result<Measurement, SHTC3Error> {
        let raw = self.get_raw_measurement_result()?;
        Ok(raw.into())
    }

    /// Read the raw result of a combined temperature / humidity measurement.
    pub fn get_raw_measurement_result(&mut self) -> Result<RawMeasurement, SHTC3Error> {
        let mut buf = [0; 6];
        self.read_with_crc(&mut buf)?;

        Ok(RawMeasurement {
            temperature: u16::from_be_bytes([buf[0], buf[1]]),
            humidity: u16::from_be_bytes([buf[3], buf[4]]),
        })
    }

    /// Simple async sample function. During this time I can communicate with other sensors on the
    /// I2C bus.
    pub async fn sample(
        &mut self,
        mut delay: &mut impl DelayUs,
    ) -> Result<Measurement, SHTC3Error> {
        self.wakeup(delay).await?;
        self.measure(PowerMode::LowPower, &mut delay).await?;
        let result = self.get_measurement_result();
        // Go to sleep to preserve power!
        self.sleep()?;
        result
    }
}
