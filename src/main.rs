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

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

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

    // Set the pin to be an output
    let mut test_pin = pins.gpio0.into_push_pull_output();
    let mut pump_main = pins.gpio12.into_push_pull_output();
    let mut pump_bladder = pins.gpio11.into_push_pull_output();
    let mut htr_center = pins.gpio15.into_push_pull_output();
    let mut htr_fb = pins.gpio14.into_push_pull_output();
    let mut htr_lr = pins.gpio13.into_push_pull_output();


    // Blink the LED at 1 Hz
    loop {
        test_pin.set_high().unwrap();
        pump_main.set_high().unwrap();
        pump_bladder.set_high().unwrap();
        htr_center.set_high().unwrap();
        htr_fb.set_high().unwrap();
        htr_lr.set_high().unwrap();
        delay.delay_ms(50);

        test_pin.set_low().unwrap();
        pump_main.set_low().unwrap();
        pump_bladder.set_low().unwrap();
        htr_center.set_low().unwrap();
        htr_fb.set_low().unwrap();
        htr_lr.set_low().unwrap();
        delay.delay_ms(50);
    }
}

// End of file
