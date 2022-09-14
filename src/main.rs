//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W

#![no_std]
#![no_main]
use panic_halt as _;    // panic fuctionality

use rp_pico::entry;     // Entry point macro
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use rp_pico::hal::gpio::{Pwm, Pin, PinId, Function, FunctionPwm, Disabled, PullDown, Output, PushPull};
// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};
use rp_pico::hal::rom_data::float_funcs::{fdiv, fmul, int_to_float};
//use rp_pico::hal::spi;
use fugit::RateExtU32;

// My Valve Controller:
pub mod valve_controller;
use valve_controller::ValveState;
use valve_controller::ValveController;

// My Heater Controller
// pub mod heater_controller;
// use heater_controller::HeaterController;


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

    // ----------- VALVE CONTROLLER SETUP ------------
    // Create the valve controllers and initialize them.
    // Need to check logic polarity
    // Might chain init()...
    let mut main_chamber_valve = ValveController::new(pins.gpio12.into_push_pull_output());
    main_chamber_valve.init();
    let mut bladder_valve = ValveController::new(pins.gpio11.into_push_pull_output());
    bladder_valve.init();

    // Main loop forever
    loop {
        main_chamber_valve.set_state(ValveState::Vent);
        bladder_valve.set_state(ValveState::Pump);
        delay.delay_ms(2000);

        main_chamber_valve.set_state(ValveState::Pump);
        bladder_valve.set_state(ValveState::Vent);
        delay.delay_ms(1000);

    }
}


// End of file
