#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

// use nb::block;

// extern crate void;
// use void::Void;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    delay::Delay,
    gpio,
    gpio::{Floating, Input, Output, PushPull},
    pac,
    prelude::*,
    // timer::Timer,
};

use dht_hal_drv::{dht_init, dht_read, DhtError, DhtType, DhtValue};
use embedded_hal::digital::v2::{InputPin, OutputPin};

// type DhtHwOutPin = gpio::gpiob::PB9<Output<PushPull>>;
// type DhtHwPinCr = gpio::gpiob::CRH;

type DhtHwOutPin = gpio::gpioc::PC15<Output<PushPull>>;
type DhtHwPinCr = gpio::gpioc::CRH;

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

    hprintln!("Freq sysclk {}Hz  bclk2 {}Hz", clocks.sysclk().0, clocks.pclk2().0);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    // let mut timer = Timer::syst(cp.SYST, 1.hz(), clocks);
    let mut delay = Delay::new(cp.SYST, clocks);

    // Configure DHT pins
    // let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    // let mut dht_pin: DhtHwOutPin = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

    let mut dht_pin: DhtHwOutPin = gpioc.pc15.into_push_pull_output(&mut gpioc.crh);
    loop {
        // Prepare to read DHT
        // let dht_value = dht_read(DhtType::DHT11, &mut dht, &mut delay);

        // block!(timer.wait()).unwrap();
        delay.delay_ms(1000_u32);
        led.set_high();

        hprintln!("LED is ON {}", "dd");

        let (result, pout) = read_dht(dht_pin, &mut gpioc.crh, &mut delay);
        dht_pin = pout;
        dht_pin.set_high();
        // dht_pin = read_dht(dht_pin, &mut gpiob.crh, &mut delay);

        match result {
            Ok(res) => hprintln!("DHT readins {:?}", res),
            Err(err) => hprintln!("DHT ERROR {:?}", err),
        };

        // block!(timer.wait()).unwrap();
        delay.delay_ms(1000_u32);
        led.set_low();
        hprintln!("LED is OFF");
    }
}

fn read_dht(
    mut pin: DhtHwOutPin,
    cr: &mut DhtHwPinCr,
    delay: &mut Delay,
) -> (Result<DhtValue, DhtError>, DhtHwOutPin) {
    // Implement custom HW specific delay logic that DHT libraray is not aware of
    let mut delay_us = |d| delay.delay_us(d);

    let init = dht_init(&mut pin, true, &mut delay_us);
    //You can skip this error check if you like
    if init.is_err() {
        return (Err(init.err().unwrap()), pin);
    }
    let mut input = pin.into_floating_input(cr);
    (
        dht_read(DhtType::DHT11, &mut input, &mut delay_us),
        input.into_push_pull_output(cr),
    )
}
