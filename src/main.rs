//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W
//!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

// GPIO traits

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use embedded_hal::PwmPin;

/// Entry point
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The minimum and max PWM values
    const LOW: u16 = 0;
    const HIGH: u16 = 65535;

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM7
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_ph_correct();
    pwm.set_div_int(255u8); // To set integer part of clock divider
    pwm.enable();

    // Output channel B on PWM7 to pin GPIO15
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio15);

    // Main loop forever
    loop {
        info!("Ramp up !");
        // Ramp duty factor up
        for i in LOW..=HIGH {
            delay.delay_us(100);
            channel.set_duty(i);
        }
        info!("Ramp down");
        // Ramp duty factor down
        for i in (LOW..=HIGH).rev() {
            delay.delay_us(100);
            channel.set_duty(i);
        }

    }
}

// End of file
