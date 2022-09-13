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
use embedded_hal::PwmPin;
use rp_pico::hal::pwm::{Slice, SliceId, FreeRunning, Channel, B};
use rp_pico::hal::gpio::{Pwm, Pin, PinId, Function};

// My use statments:
pub mod valve_controller;
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
    // Create the valve controllers and initialize them.
    // Need to check logic polarity
    // Might chain init()...
    let mut main_chamber_valve = ValveController::new(pins.gpio12.into_push_pull_output());
    main_chamber_valve.init();
    let mut bladder_valve = ValveController::new(pins.gpio11.into_push_pull_output());
    bladder_valve.init();


    // ----------- PWM HEATER CONTROLLER SETUP ------------
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Get a pwm slice (PWM7) from Slices and configure
    let pwm_slice = &mut pwm_slices.pwm7;       
    pwm_slice.set_ph_correct();
    pwm_slice.set_div_int(255u8); // To set integer part of clock divider
    pwm_slice.enable();

    //Get a channel (B) from the slice
    let channel = &mut pwm_slice.channel_b;   

    // Attach the channel outout to a pin (gpio15)
    let pin = pins.gpio15;
    channel.output_to(pin);

    channel.set_duty(12000);

    

    // Main loop forever
    loop {
        main_chamber_valve.set_state(ValveState::Vent);
        bladder_valve.set_state(ValveState::Pump);
        delay.delay_ms(3);

        main_chamber_valve.set_state(ValveState::Pump);
        bladder_valve.set_state(ValveState::Vent);
        delay.delay_ms(10);

    }
}

// pub struct HeaterController<'a, S: SliceId, I:PinId + rp2040_hal::gpio::bank0::BankPinId> {
//     pwm_slice: &'a mut Slice<S, FreeRunning>,
//     pwm_channel: &'a  mut Channel<S, FreeRunning, B>,        // Temporarily hardcoded channel B...
//     pwm_pin: Pin<I, Function<Pwm>>,
// }

// impl<S: SliceId,  I:PinId + rp2040_hal::gpio::bank0::BankPinId + rp2040_hal::pwm::ValidPwmOutputPin<S, rp2040_hal::pwm::B>> HeaterController<'_, S, I> {
//     pub fn new(pwm_slice: &mut Slice<S,FreeRunning>, pwm_pin: Pin<I, Function<Pwm>>) -> HeaterController<S, I> {
//         pwm_slice.set_ph_correct();
//         pwm_slice.set_div_int(255u8); // To set integer part of clock divider
//         pwm_slice.enable();

//         // // Output channel B on PWM7 to pin GPIO15
//         let pwm_channel = &mut pwm_slice.channel_b;
//         //let pin = pins.gpio15.into_mode::<rp_pico::hal::gpio::FunctionPwm>();
//         pwm_channel.output_to(pwm_pin);

//         HeaterController { 
//             pwm_slice, 
//             pwm_channel,
//             pwm_pin,
//         }
//     }
//     pub fn set_duty(&mut self, df: i32) {
//         //self.pwm_channel.set_duty(df);
//     }
// }

// End of file
