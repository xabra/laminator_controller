//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W


#![no_std]
#![no_main]

pub mod valve_controller;

use rp_pico::entry;     // Entry point macro
use panic_halt as _;    // panic fuctionality

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::bank0::{Gpio11,Gpio12};
//use rp_pico::hal::spi;
//use fugit::RateExtU32;
use valve_controller::ValveState;
use valve_controller::ValveController;


/// Entry point
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks - default is 125 MHz system clock
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

    // Delay object
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

    // Setup two output pins for the valve controller
    let pump_main_pin = pins.gpio12.into_push_pull_output();
    let pump_bladder_pin = pins.gpio11.into_push_pull_output();

    //Create the valve controller and initialize it.
    let mut valve_controller = ValveController::new(pump_main_pin, pump_bladder_pin);
    valve_controller.init();

    //let mut spi = spi::Spi::<_, _, 16>::new(pac.SPI0).init(&mut pac.RESETS, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);
    //let mut spi_ctrl = ValveController::new(chip_select);


    // Main loop forever
    loop {
        valve_controller.set_main_valve(ValveState::Open);
        valve_controller.set_bladder_valve(ValveState::Closed);
        delay.delay_ms(3);

        valve_controller.set_main_valve(ValveState::Closed);
        valve_controller.set_bladder_valve(ValveState::Open);
        delay.delay_ms(10);

    }
}

// End of file
