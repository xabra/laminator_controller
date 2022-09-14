//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W

#![no_std]
#![no_main]
//use hal::gpio::PinMode;
//use hal::pll::Disabled;
use panic_halt as _;    // panic fuctionality

use rp_pico::entry;     // Entry point macro
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use embedded_hal::PwmPin;
use rp_pico::hal::pwm::{Slice, SliceId, FreeRunning, Channel, ChannelId, B};
use rp_pico::hal::gpio::{Pwm, Pin, PinId, Function, FunctionPwm, Disabled, PullDown, Output, PushPull};

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
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Get individual pwm slices (PWM6, PWM7) from Slices
    let pwm6_slice = &mut pwm_slices.pwm6;
    let pwm7_slice = &mut pwm_slices.pwm7;

    // Create three pwm heater controlllers
    // ?? Here I pass a mut reference pwm7_slice to two different functions.  Why no complaint from the borrow chechker???
    let mut heater_ctr  = HeaterController::new(pwm7_slice, pwm7_slice.channel_b, pins.gpio15);
    let mut heater_fb  = HeaterController::new(pwm7_slice, pwm7_slice.channel_a, pins.gpio14);
    let mut heater_lr  = HeaterController::new(pwm6_slice, pwm6_slice.channel_b, pins.gpio13);

    heater_ctr.init();
    heater_fb.init();
    heater_lr.init();
    heater_ctr.set_duty(2000);
    heater_fb.set_duty(3000);
    heater_lr.set_duty(4000);


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

// A controller to encapsulate and manage a pwm heater channel
pub struct HeaterController<'a, S: SliceId, C: ChannelId, I: PinId> {
    pwm_slice: &'a mut Slice<S, FreeRunning>,
    pwm_channel: Channel<S, FreeRunning, C>,
    //pwm_pin: Pin<I, Disabled<PullDown>>,    // <===== IS THIS THE RIGHT PINMODE 'M' FOR PWM HERE? 
    pwm_pin: Pin<I, FunctionPwm>,    // <===== IS THIS THE RIGHT PINMODE 'M' FOR PWM HERE? 
}

impl<S: SliceId, C: ChannelId, I: PinId> HeaterController<'_, S, C, I> {  

    pub fn new(pwm_slice: &mut Slice<S,FreeRunning>, pwm_channel:Channel<S, FreeRunning, C>, pwm_pin:Pin<I, Disabled<PullDown>>) -> HeaterController<S, C, I> {
        HeaterController { 
            pwm_slice, 
            pwm_channel,
            pwm_pin,
        }
    }

    pub fn init(&mut self){
        self.pwm_slice.set_ph_correct();
        self.pwm_slice.set_div_int(255u8); // To set integer part of clock divider
        self.pwm_slice.enable();

        self.pwm_channel.output_to(self.pwm_pin);  // <- rustc says no method names output_to.  Why?
        self.pwm_channel.set_duty(0);               // <- rustc says no method names output_to.  Why?
    }

    pub fn set_duty(&mut self, df: i32){
        self.pwm_channel.set_duty(df);              // <- rustc says no method names output_to.  Why?
    }
}

// End of file
