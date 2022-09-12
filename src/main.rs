//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W


#![no_std]
#![no_main]

pub mod valve_controller;

use hal::pwm::FreeRunning;
use hal::pwm::Pwm7;
use rp_pico::entry;     // Entry point macro
use panic_halt as _;    // panic fuctionality

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use rp2040_hal::pwm::Slice;
use rp2040_hal::gpio::Pwm;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::bank0::{Gpio11,Gpio12};
//use rp_pico::hal::pwm::{Gpio11,Gpio12};
use crate::hal::pwm::{Channel, B};
use crate::gpio::Pin;

//use rp_pico::hal::spi;
//use fugit::RateExtU32;

// My use statments:
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

    // ----------- VALVE CONTROLLER SETUP ------------
    // Setup two output pins for the valve controller
    let pump_main_pin = pins.gpio12.into_push_pull_output();
    let pump_bladder_pin = pins.gpio11.into_push_pull_output();

    //Create the valve controller and initialize it.
    let mut valve_controller = ValveController::new(pump_main_pin, pump_bladder_pin);
    // Seems kinda silly to pass in praticular gpio pins when they are already built into the ValveController struct via types.
    // Should be one ot her other?
    valve_controller.init();


    // ----------- PWM HEATER CONTROLLER SETUP ------------
    // PWMs
    // let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    // let pwm = &mut pwm_slices.pwm7;
    // pwm.set_ph_correct();
    // pwm.set_div_int(255u8); // To set integer part of clock divider
    // pwm.enable();

    // // Output channel B on PWM7 to pin GPIO15
    // let channel = &mut pwm.channel_b;
    // channel.output_to(pins.gpio15);


    // channel.set_duty(6000);

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

// pub struct HeaterController<'a> {
//     pwm: &'a mut Slice<Pwm7, FreeRunning>,
//     channel: &'a mut Channel<Pwm7, FreeRunning, B>
// }

// impl<I,M> HeaterController {
//     pub fn new(pwm: &mut Slice<Pwm7,FreeRunning>, pwm_pin:Pin<I,M>) -> HeaterController {
//         pwm.set_ph_correct();
//         pwm.set_div_int(255u8); // To set integer part of clock divider
//         pwm.enable();

//         // Output channel B on PWM7 to pin GPIO15
//         let channel = &mut pwm.channel_b;
//         channel.output_to(pwm_pin);
//         HeaterController { pwm: pwm }
//     }
// }

// End of file
