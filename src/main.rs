//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W

#![no_std]
#![no_main]
use thermocouple_controller::ChipSelectState;
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
use rp_pico::hal::rom_data::float_funcs::{fdiv, fmul, int_to_float};
use rp_pico::hal::spi;
use fugit::RateExtU32;

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

// My Valve Controller:
pub mod valve_controller;
//use valve_controller::ValveState;
use valve_controller::ValveController;

// My Thermocouple Controller
 pub mod thermocouple_controller;
 use thermocouple_controller::{ThermocoupleController, TCError, TCId};


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

    // ----------- SPI, DriverMax31855 SETUP ------------
    // Chip select pins
    let mut cs_ctr = pins.gpio17.into_push_pull_output();
    cs_ctr.set_high().unwrap();
    let mut cs_lr = pins.gpio19.into_push_pull_output();
    cs_lr.set_high().unwrap();
    let mut cs_fb = pins.gpio20.into_push_pull_output();
    cs_fb.set_high().unwrap();

    // Set up SPI CLK and DataIn Lines.  These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio16.into_mode::<gpio::FunctionSpi>();

    // Set up spi
    let spi = spi::Spi::<_, _, 16>::new(pac.SPI0).init(&mut pac.RESETS, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);

    let mut tc_controller = ThermocoupleController::new(cs_ctr, cs_lr, cs_fb, spi);


    // Main loop forever
    loop {
        // let temp_result = tc_controller.read_temps();
        // match temp_result {
        //     Ok(temps) => info!("Temp: {=f32}   Ref Temp:{=f32}", temps.tc_temp, temps.ref_temp),
        //     Err(e) => match e {
        //         TCError::TempSensorShortToVCC => {info!("Shorted to VCC");}
        //         TCError::TempSensorShortToGND => {info!("Shorted to GND");}
        //         TCError::TempSensorOpenCircuit => {info!("Open Circuit");}
        //     }
        // }
        tc_controller.set_exclusive_chip_select(TCId::Center, ChipSelectState::Selected);
        delay.delay_ms(1000);
        tc_controller.set_exclusive_chip_select(TCId::LeftRight, ChipSelectState::Selected);
        delay.delay_ms(1000);
        tc_controller.set_exclusive_chip_select(TCId::FrontBack, ChipSelectState::Selected);
        delay.delay_ms(1000);
        tc_controller.set_exclusive_chip_select(TCId::FrontBack, ChipSelectState::Deselected);
    }
}


// End of file
