extern crate clap;

use clap::{App, Arg, ArgMatches};
use dht_hal_drv::{dht_read, dht_split_init, dht_split_read, DhtError, DhtType, DhtValue};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rppal::gpio::{Gpio, IoPin, Mode};
use spin_sleep;
use std::{thread, time};
use void;

/**
 * Raspberry pi does not have open drain pins so we have to emulate it.
 */
struct OpenPin {
    iopin: IoPin,
    mode: Mode,
}

impl OpenPin {
    fn new(mut pin: IoPin) -> OpenPin {
        pin.set_mode(Mode::Input);
        OpenPin {
            iopin: pin,
            mode: Mode::Input,
        }
    }

    fn switch_input(&mut self) {
        if self.mode != Mode::Input {
            self.mode = Mode::Input;
            self.iopin.set_mode(Mode::Input);
        }
    }

    fn switch_output(&mut self) {
        if self.mode != Mode::Output {
            self.mode = Mode::Output;
            self.iopin.set_mode(Mode::Output);
        }
    }
}

// Current rppal implementation does not support embedded_hal::gpio::v2 pins API.
impl InputPin for OpenPin {
    type Error = void::Void;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.iopin.is_high())
    }

    /// Is the input pin low?
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.iopin.is_low())
    }
}

// Current rppal implementation does not support embedded_hal::gpio::v2 pins API.
impl OutputPin for OpenPin {
    type Error = void::Void;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.switch_output();
        self.iopin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.iopin.set_high();
        self.switch_input();
        Ok(())
    }
}

fn cli_matches() -> ArgMatches<'static> {
    App::new("DHT tester")
        .author("Rumato Estorsky")
        .about("Performs DHT sensors test on Raspberry Pi")
        .arg(
            Arg::with_name("pin")
                .short("p")
                .long("pin")
                .value_name("DHT_PIN")
                .help("Pin number where DHT sensor connected to")
                .required(true),
        )
        .arg(
            Arg::with_name("type")
                .long("type")
                .short("t")
                .value_name("SENSOR_TYPE")
                .required(true)
                .help("DHT sensor type")
                .possible_values(&["DHT11", "DHT21", "DHT22"]),
        )
        .get_matches()
}

fn main() {
    let matches = cli_matches();

    let dht_ts = matches.value_of("type").unwrap();
    let dht_type = match dht_ts {
        "DHT11" => DhtType::DHT11,
        "DHT21" => DhtType::DHT21,
        "DHT22" => DhtType::DHT22,
        _ => panic!("Can not parse DHT type {}", dht_ts),
    };

    let pin = matches.value_of("pin").unwrap().parse::<u8>().unwrap();

    println!("Initialized as {} at pin {}", dht_ts, pin);

    let gpio = Gpio::new().expect("Can not init Gpio structure");

    let iopin = gpio
        .get(pin)
        .expect("Was not able to get Pin")
        .into_io(Mode::Input);

    let mut opin = OpenPin::new(iopin);

    loop {
        let readings = dht_read(DhtType::DHT11, &mut opin, &mut |d| {
            spin_sleep::sleep(time::Duration::from_micros(d as u64))
        });
        // let readings = read_dht_splitted(&mut opin);

        match readings {
            Ok(res) => {
                println!("DHT readins {}C {}%", res.temperature(), res.humidity());
            }
            Err(err) => {
                println!("DHT ERROR {:?}", err);
            }
        };
        thread::sleep(time::Duration::from_secs(2));
    }
}

/// Example of reading using open drain pin emulation
/// We could pass here `pin` as IoPin struct and change it's state between DHT calls
fn read_dht_splitted(pin: &mut OpenPin) -> Result<DhtValue, DhtError> {
    // Implement custom HW specific delay logic that DHT driver is not aware of
    let mut delay_us = |d: u16| {
        // We are using here more accurate delays than in std library
        spin_sleep::sleep(time::Duration::from_micros(d as u64));
    };

    pin.switch_output();
    let init = dht_split_init(pin, &mut delay_us);
    if init.is_err() {
        // You can skip this error check if you like
        return Err(init.err().unwrap());
    }

    // Should convert pin back to input
    pin.switch_input();

    // Now let's read some data
    // Sometimes delays are takes exact same Duration as it was provided
    // You can pass here empty delay closure as well
    dht_split_read(DhtType::DHT11, pin, &mut delay_us)
}
