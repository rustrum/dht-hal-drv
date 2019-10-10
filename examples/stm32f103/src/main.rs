#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    delay::Delay,
    gpio,
    gpio::{Floating, Input},
    pac,
    prelude::*,
};

use dht_hal_drv::{dht_init, dht_read, DhtError, DhtType, DhtValue};
use embedded_hal::digital::v2::OutputPin;

// Define types for DHT interface
type DhtHwPin = gpio::gpiob::PB9<Input<Floating>>;
type DhtHwPinCr = gpio::gpiob::CRH;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    // dp.RCC.cfgr.sysclk(1.mhz());
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store
    // the frozen frequencies in `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    // let mut timer = Timer::syst(cp.SYST, 1.hz(), clocks);
    let mut delay = Delay::new(cp.SYST, clocks);

    // DHT pin config
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut dht_pin: DhtHwPin = gpiob.pb9.into_floating_input(&mut gpiob.crh);

    loop {
        let (readings, pout) = read_dht(dht_pin, &mut gpiob.crh, &mut delay);
        dht_pin = pout;

        match readings {
            Ok(res) => {
                // Long blinks if everything is OK
                led_blink(&mut led, &mut delay, 250);
                hprintln!("DHT readins {}C {}%", res.temperature(), res.humidity());
            }
            Err(err) => {
                // Short blinks on errors
                for _ in 0..10 {
                    led_blink(&mut led, &mut delay, 25);
                }
                hprintln!("DHT ERROR {:?}", err);
            }
        };
    }
}

fn read_dht(
    pin: DhtHwPin,
    cr: &mut DhtHwPinCr,
    delay: &mut Delay,
) -> (Result<DhtValue, DhtError>, DhtHwPin) {
    // Implement custom HW specific delay logic that DHT driver is not aware of
    let mut delay_us = |d| delay.delay_us(d);
    // Convert pin to input
    let mut pin_out = pin.into_push_pull_output(cr);
    // Initialize DHT data transfer
    let init = dht_init(&mut pin_out, false, &mut delay_us);
    if init.is_err() {
        // You can skip this error check if you like
        return (Err(init.err().unwrap()), pin_out.into_floating_input(cr));
    }

    // WARNING there should be no additional logic between dht_init and dht_read

    // Should convert pin back to input
    let mut pin_in = pin_out.into_floating_input(cr);
    // Now let's read some data
    let readings = dht_read(DhtType::DHT11, &mut pin_in, &mut delay_us);
    // We must return reading + pin together
    // because of tricky stm32f1xx_hal implementation
    // where you can have only one pin instance at a time
    (readings, pin_in)
}

fn led_blink<Error>(pin: &mut dyn OutputPin<Error = Error>, delay: &mut Delay, ms: u32) {
    pin.set_high();
    delay.delay_ms(ms);
    pin.set_low();
    delay.delay_ms(ms);
}
