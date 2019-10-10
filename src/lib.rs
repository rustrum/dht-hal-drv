//! # HAL based driver for Digital Humidity and Temperature sensors (DHT)
//!
//! Because some limitations in HAL API leads to limitations in HW implementations
//! using this sensor **is some kind of tricky**.
//!
//! DHT using one pin for communication and this pin must be switched between input/output modes.
//! Before reading begins MCU must send signal to DHT which would initiate data transfer from DHT.
//!
//! Also DHT initialization process has some caveats.
//! There have to be near 1 sec delay after previous read.
//! During this delay pull-up resitor in DHT module (as it's required according to dataheet) would pull up data pin.
//!
//! This is why reading process from DHT was splitted in two functions.
//! You have to switch pin to output mode first, initialize DHT, switch pin to input mode and then read DHT data.
//!
//! ## Example
//!
//! I assume that you would wrap reading logic in function or something.
//!
//! ```
//! use dht_hal_drv::{dht_init, dht_read, DhtError, DhtType, DhtValue};
//!
//! // Other code of yours //
//!
//! let delay; // Something that impelements DelayUs trait
//! let pin_in; // Pin configured as input floating
//!
//! // Should create closure with
//! // custom HW specific delay logic that DHT driver is not aware of
//! let mut delay_us = |d| delay.delay_us(d);
//!
//! // pin to output mode
//! let mut pin_out = pin_in.into_push_pull_output();
//!
//! // Initializubg DHT data transfer
//! dht_init(&mut pin_out, &mut delay_us);
//!
//! // You can check dht_init response for errors if you whant
//!
//! // WARNING there should be no additional logic between dht_init and dht_read
//!
//! // Should convert pin back to input
//! let mut pin_in = pin_out.into_floating_input(cr);
//!
//! // Now let's read some data
//! let readings = dht_read(DhtType::DHT11, &mut pin_in, &mut delay_us);
//!
//! // Other code of yours //
//!
//! ```
//!
//! Working examples for particular HW platforms could be found in source repository.
//!
//! ## Inspiration sources
//!
//! - [Adafruit DHT.cpp](https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp)
//! - [Adafruit python lib pi_dht_read.c](https://github.com/adafruit/Adafruit_Python_DHT/blob/master/source/Raspberry_Pi/pi_dht_read.c)
//! - [Full signals diagrams for DHT](http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf).
//!
#![no_std]
use core::prelude::v1::Result;
use embedded_hal::digital::v2::{InputPin, OutputPin};

///
/// You sould receive this in a case of some upleasant situation.
///
#[derive(Debug)]
pub enum DhtError {
    /// Unable to read data from sensor
    Readings,
    /// Data was read but checksum validation falied
    Checksum,
    /// Error reading pin values or acquire pin etc.
    IO,
}

///
/// Describe available DHT sensor types.
///
#[derive(Debug, Clone)]
pub enum DhtType {
    DHT11,
    DHT21,
    DHT22,
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
    /// Return temperature readings in Celsius.
    /// All raw data from DHT sensor are related to this scale.
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

    /// Convert temperature readings from Celsius to Fahrenheit
    /// for those who love to use piffle scales and measurements.
    pub fn temperature_f(&self) -> f32 {
        self.temperature() * 1.8 + 32.0
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

///
/// Initialize DHT sensor (sending start signal) to start readings.
///
/// Notice that there have to be about 1 sec delay before each reading (between calling `dht_init`).
/// At this period data pin should be pulled up by resistor connected to DHT
/// which is default connection scheme for DHT.
/// It implies that pin should be set in input floating mode after previous reading.
///
/// In Adafruit drivers you can see that there is inital delay with hight impedance for about 500-700ms.
/// You do not need this delay if you read sensor not to often and do other logic between readings.
///
/// # Arguments
///
/// * `output_pin` - Output pin trait for DHT data pin.
/// * `delay_us` - Closure where you should call appropriate delay/sleep/whatewer API with microseconds as input.
///
pub fn dht_init<Error>(
    output_pin: &mut dyn OutputPin<Error = Error>,
    delay_us: &mut dyn FnMut(u16) -> (),
) -> Result<(), DhtError> {
    // Voltage  level  from  high to  low.
    // This process must take at least 18ms to ensure DHT’s detection of MCU's signal.
    output_pin.set_low().map_err(|_| DhtError::IO)?;

    delay_us(20_000);
    Ok(())
}

///
/// Call this immediately after [initialization](fn.dht_init.html) to acquire proper sensor readings.
///
/// # Arguments
///
/// * `dht` - Dht sensor type
/// * `input_pin` - Input pin trait for DHT data pin
/// * `delay_us' - Closure with delay/sleep/whatewer API with microseconds as input,
/// NOTE that for low frequency CPUs (about 2Mhz or less) you should pass empty closure.
///
pub fn dht_read<Error>(
    dht: DhtType,
    input_pin: &mut dyn InputPin<Error = Error>,
    delay_us: &mut dyn FnMut(u16) -> (),
) -> Result<DhtValue, DhtError> {
    // Initialize variables
    let mut err: Option<DhtError> = None;
    let mut data: [u8; 5] = [0; 5]; // Set 40 bits of received data to zero.
    let mut cycles: [u32; 83] = [0; 83];

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

    // Maximum acceptable count of reading cycles. Considering that one cycle does not complete less in 1-3us.
    let max_cycles = (50 + 70) * 80 + 80 * 3;

    // Delay in microseconds which should help us to slow down "while" cycles.
    // If "while" cycle would be faster than 1us it would cause errors because of max_cycles constraint.
    let delay_us_value = 3;

    let mut i = 0;
    let mut x = 0;

    // As I can see at least 10 instructions should be executed at each cycle.
    // Approximately if CPU frequency is 8MHz and IPC value is 1 than
    // 28us would match to 22 loop cycles without using any delay.
    // For 1MHz CPU it is about 2 full cycles.
    // NOTICE for slow CPU you should not implement `delay_us` this closure should do nothing.
    while i < 83 {
        let high = input_pin.is_high().map_err(|_| DhtError::IO)?;
        if (i % 2 == 0) == high {
            // Instead of reading time we just count number of cycles until next level value
            cycles[i] += 1;
        } else {
            i += 1;
        }

        // Reasonable delay for fast CPU
        delay_us(delay_us_value);

        // Check timeout
        x += 1;
        if x > max_cycles {
            err = Some(DhtError::Readings);
            break;
        }
    }

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    // We skip first 3 values because there is no\t data there
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

    // #[cfg(feature = "debug_trace")]
    // {
    //     print!("DHT readings: ");
    //     print!("{:X} {:X} {:X} {:X}", data[0], data[1], data[2], data[3]);
    //     println!(
    //         "  {:X} == {:X} (checksum)",
    //         data[4],
    //         (data[0] as u16 + data[1] as u16 + data[2] as u16 + data[3] as u16) & 0xFF
    //     );
    // }

    // Check we read 40 bits and that the checksum matches.
    let checksum = (data[0] as u16 + data[1] as u16 + data[2] as u16 + data[3] as u16) & 0xFF;
    if data[4] as u16 == checksum {
        Ok(DhtValue {
            value: data,
            dht_type: dht,
        })
    } else {
        Err(err.unwrap_or(DhtError::Checksum))
    }
}
