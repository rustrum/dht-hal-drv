#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    gpio::{gpiob, Floating, Input, Output, PushPull},
    pac,
    prelude::*,
    timer::Timer,
};

use alloc::boxed::Box;
extern crate alloc;

use dht_hal_drv::{DhtSensor, DhtType, GenericPin};
use embedded_hal::digital::v2::{InputPin, OutputPin};

enum DhtPin {
    IN(Input<Floating>),
    OUT(Output<PushPull>),
}

struct PinWrapper {
    pin: DhtPin,
}

impl GenericPin for PinWrapper {
    // fn pin(&self) -> u8 {
    //     self.pin
    // }

    fn input(&self) -> Result<&dyn InputPin<Error = ()>, ()> {
        let mut res = Err(());
        self.pin = match self.pin {
            DhtPin::IN(ip) => {
                res = Ok(&ip);
                DhtPin::IN(ip)
            },
            DhtPin::OUT(op) => {
                let p = op.into_floaing_input();
                res = OK(&p);
                DhtPin::IN(p)
            },
        };
        return res;
    }

    fn output(&self) -> Result<Box<dyn OutputPin<Error = ()>>, ()> {
        // self.gpio
        //     .get(self.pin)
        //     .map(|it| it.into_output())
        //     .map(|it| Box::new(it) as Box<dyn OutputPin>)
        //     .map_err(|_| ())
    }
}

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store
    // the frozen frequencies in `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let led_pin = gpioc.pc15;

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, 1.hz(), clocks);

    loop {
        block!(timer.wait()).unwrap();
        led.set_high();

        hprintln!("LED is ON {}", "dd");

        block!(timer.wait()).unwrap();
        led.set_low();
        hprintln!("LED is OFF");
    }
}
