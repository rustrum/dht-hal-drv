#![no_std]
// #[macro_use(block)]
// extern crate nb;
extern crate alloc;
use alloc::boxed::Box;
use core::prelude::v1::Result;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::timer::{CountDown};
use embedded_hal::blocking::delay::DelayUs;

///
/// We need to use one pin in input and output mode.
/// Right now embedded_hal does not provide such interface,
/// thus it should be done on application side.
/// 
pub trait GenericPin {

    /// Return InputPin or fail with nothing
    fn input(&self) -> Result<Box<dyn InputPin>, ()>;

    /// Return OutputPin or fail with nothing
    fn output(&self) -> Result<Box<dyn OutputPin>, ()>;
}

///
/// Determines DHT sensor types.
///
#[derive(Debug, Clone)]
pub enum DhtType {
    DHT11,
    DHT21,
    DHT22,
}

#[derive(Debug)]
pub enum DhtError {
    /// Unable to read data from sensor
    Readings,
    /// Data was read but checksum validation falied
    Checksum,
    /// Error reading pin values or acquire pin etc.
    IO
}

///
/// Contains readings from DHT sensor.
///
#[derive(Debug)]
pub struct DhtValue {
    dht_type: DhtType,
    value: [u8; 5],
}

impl DhtValue {
    /// Return temperature readings in Fahrenheit.
    pub fn temperature_f(&self) -> f32 {
        self.temperature() * 1.8 + 32.0
    }

    /// Return temperature readings in Celcius.
    pub fn temperature(&self) -> f32 {
        match &self.dht_type {
            DHT11 => self.value[2] as f32,
            _ => {
                let mut v: f32 = (self.value[2] & 0x7F) as f32;
                v = (v * 256.0 + self.value[3] as f32) * 0.1;
                if self.value[2] & 0x80 > 0 {
                    v *= -1.0;
                }
                v
            }
        }
    }

    /// Return humidity readins in percents.
    pub fn humidity(&self) -> f32 {
        match &self.dht_type {
            DHT11 => self.value[0] as f32,
            _ => {
                let mut v: f32 = self.value[0] as f32;
                v = (v * 256.0 + self.value[1] as f32) * 0.1;
                v
            }
        }
    }
}


pub struct DhtSensor {
    pin: Box<dyn GenericPin>,
    delay: Box<dyn DelayUs<u32>>,
    dht_type: DhtType,
    value: [u8; 5],
}

/// Ideas about DHT reading sensors was found here:
/// - https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
/// - https://github.com/adafruit/Adafruit_Python_DHT/blob/master/source/Raspberry_Pi/pi_dht_read.c
impl DhtSensor {
    pub fn new(pin: Box<dyn GenericPin>, delay: Box<dyn DelayUs<u32>>, dht_type: DhtType) -> DhtSensor {
         DhtSensor {
            pin: pin,
            delay: delay,
            dht_type: dht_type,
            value: [0; 5],
        }
    }

    ///
    /// Return result and data readed from sensor.
    /// This sensors are buggy and errors are usual, sometimes errors are more common than success response.
    /// 
    fn read(&mut self) -> Result<DhtValue, DhtError> {
        // Initialize variables
        let mut err: Option<DhtError> = None;
        let mut data: [u8; 5] = [0; 5]; // Set 40 bits of received data to zero.
        let mut cycles: [u32; 83] = [0; 83];

        // Send start signal.  See DHT datasheet for full signal diagram:
        // http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf
        // Go into high impedence state to let pull-up raise data line level and
        // start the reading process.

        let mut pino = self.pin.output().map_err(|_| DhtError::IO)?;
        
        self.delay.delay_us(250_000);

        // Time critical section begins
        // Voltage  level  from  high to  low.
        // This process must take at least 18ms to ensure DHT’s detection of MCU's signal.
        
        pino.set_low();

        self.delay.delay_us(20_000);

        drop(pino);
        let pini = self.pin.input().map_err(|_| DhtError::IO)?;
        
        // MCU will pull up voltage and wait 20-40us for DHT’s response
        // Delay a bit to let sensor pull data line low.

        // READ to cycles[0] - or skip to next

        // Now start reading the data line to get the value from the DHT sensor.
        // First expect a low signal for ~80 microseconds followed by a high signal
        // for ~80 microseconds again.

        // READ to cycles[1] and cycles[2]

        // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
        // microsecond low pulse followed by a variable length high pulse.  If the
        // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
        // then it's a 1.  We measure the cycle count of the initial 50us low pulse
        // and use that to compare to the cycle count of the high pulse to determine
        // if the bit is a 0 (high state cycle count < low state cycle count), or a
        // 1 (high state cycle count > low state cycle count). Note that for speed all
        // the pulses are read into a array and then examined in a later step.

        // READ to cycles[3+] as low level and cycles[4+] as high level

        let mut i = 0;
        let mut x = 0;
        // Max cycles considering delay
        let max_cycles = 1200;
        while i < 83 {
            let v = pini.is_high();
            if (i % 2 == 0) == v {
                // Instead of reading time we just count number of cycles until next level value
                cycles[i] += 1;
            } else {
                i += 1;
            }
            // Delay value entagled with max_cycles
            self.delay.delay_us(5);

            // Check timeout
            x += 1;
            if x > max_cycles {
                err = Some(DhtError::Readings);
                break;
            }
        }

        drop(pini);

        // Inspect pulses and determine which ones are 0 (high state cycle count < low
        // state cycle count), or 1 (high state cycle count > low state cycle count).
        // We skip first 3 values because there is not data there
        for i in 0..40 {
            let low_cycle = cycles[2 * i + 3];
            let high_cycle = cycles[2 * i + 4];

            data[i / 8] <<= 1;
            if high_cycle > low_cycle {
                // High cycles are greater than 50us low cycle count, must be a 1.
                data[i / 8] |= 1;
            }
            // Else high cycles are less than (or equal to, a weird case) the 50us low
            // cycle count so this must be a zero.  Nothing needs to be changed in the
            // stored data.
        }

        #[cfg(feature = "debug_trace")]
        {
            print!("DHT readings: ");
            print!("{:X} {:X} {:X} {:X}", data[0], data[1], data[2], data[3]);
            println!(
                "  {:X} == {:X} (checksum)",
                data[4],
                (data[0] as u16 + data[1] as u16 + data[2] as u16 + data[3] as u16) & 0xFF
            );
        }

        // Check we read 40 bits and that the checksum matches.
        if data[4] as u16
            == ((data[0] as u16 + data[1] as u16 + data[2] as u16 + data[3] as u16) & 0xFF)
        {
            self.value = data;
            Ok(DhtValue {
                value: data,
                dht_type: self.dht_type.clone(),
            })
        } else {
           Err(err.unwrap_or(DhtError::Checksum))
        }
    }
}

// impl fmt::Debug for DhtSensor {
//     fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//         write!(f, "DHT ({:?} pin:{})", self.dht_type, self.pin)
//     }
// }


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
