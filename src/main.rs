//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W

#![no_std]
#![no_main]
use panic_halt as _;    // panic fuctionality

use embedded_hal::digital::v2::{InputPin, OutputPin};
use rp_pico::entry;     // Entry point macro
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::{Pwm, Pin, Pins, PinId, Function, FunctionPwm, Disabled, PullDown, Output, PushPull};
// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};
use rp_pico::hal::spi;
use fugit::RateExtU32;

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

// Valve Controller:
pub mod valve_controller;
//use valve_controller::ValveState;
use valve_controller::ValveController;

// Thermocouple Controller
pub mod thermocouple_controller;
use thermocouple_controller::{ThermocoupleController, TCError, TCChannel};

// Pressure Sensor Controller
pub mod pressure_sensor_controller;
use pressure_sensor_controller::PressureSensorController;

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
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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

    // ----------- THERMOCOUPLE CONTROLER SETUP ------------
    // Chip select pins
    let cs_ctr = pins.gpio17.into_push_pull_output();
    let cs_lr = pins.gpio19.into_push_pull_output();
    let cs_fb = pins.gpio20.into_push_pull_output();

    // Set up SPI CLK and DataIn Lines.  These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio16.into_mode::<gpio::FunctionSpi>();

    // Set up spi
    let spi = spi::Spi::<_, _, 16>::new(pac.SPI0).init(&mut pac.RESETS, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);

    let mut tc_controller = ThermocoupleController::new(cs_ctr, cs_lr, cs_fb, spi);
    tc_controller.init();

    // --------------- PRESSURE SENSOR CONTROLLER -----------
    // Configure the auxiliary pins
    let mut ad_start_pin = pins.gpio6.into_push_pull_output();      // Active low??
    let mut ad_busy_pin = pins.gpio7.into_floating_input();        // AD Alert/Busy pin

    // Configure sda & scl pins for I2C
    let sda_pin = pins.gpio4.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio5.into_mode::<gpio::FunctionI2C>();

    // Configure the I2C0 device
    let i2c = rp2040_hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Create new PressureSensorController
    let pressure_sensor_controller = PressureSensorController::new(ad_start_pin, ad_busy_pin, i2c);

    let mut temps = tc_controller.read_temps(TCChannel::Center);
    // Main loop forever
    loop {
        temps = tc_controller.read_temps(TCChannel::Center);
        info!("Channel: {:?} \t\tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
        temps = tc_controller.read_temps(TCChannel::LeftRight);
        info!("Channel: {:?} \tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
        temps = tc_controller.read_temps(TCChannel::FrontBack);
        info!("Channel: {:?} \tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
        println!("------");
        delay.delay_ms(1000);

    }
}


// End of file
