//! # Controller
//!
//! Runs the custom laminator controller powered by the Pico W
//!


#![no_std]
#![no_main]


use rp_pico::entry;     // Entry point macro
use panic_halt as _;    // panic fuctionality

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::bank0::Gpio12;
use rp_pico::hal::spi;
use fugit::RateExtU32;

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

    // Set the pin to be an output
    let chip_select = pins.gpio12.into_push_pull_output();
    //let mut spi = spi::Spi::<_, _, 16>::new(pac.SPI0).init(&mut pac.RESETS, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);

    let mut spi_ctrl = SpiController::new(chip_select);

    // Main loop forever
    loop {
        //chip_select.set_high().unwrap();
        spi_ctrl.pin_high();
        delay.delay_ms(2);

        //chip_select.set_low().unwrap();
        spi_ctrl.pin_low();
        delay.delay_ms(2);
    }
}

pub struct SpiController {
    cs_pin: gpio::Pin<Gpio12, Output<PushPull>>, // What is the type?
    //spi:  // What is the type?
}

impl SpiController {
    fn new(cs:gpio::Pin<Gpio12, Output<PushPull>> ) -> SpiController {
        SpiController{
            cs_pin:cs
        }
    }
    fn pin_high(&mut self){
        self.cs_pin.set_high().unwrap();
    }

    fn pin_low(&mut self){
        self.cs_pin.set_low().unwrap();
    }
}
// // End of file
