//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W
//!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
// The macro for our start-up function
use rp_pico::entry;

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rp2040_hal::rom_data::float_funcs;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

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

    // =============  SPECIFIC FUNCTIONS UNDER TEST ===========================
    // Init T/C Chip select GPIOs
    let mut tc_ctr_select_n = pins.gpio17.into_push_pull_output();
    let mut tc_lr_select_n = pins.gpio19.into_push_pull_output();
    let mut tc_fb_select_n = pins.gpio20.into_push_pull_output();
    // Initial TC chip select settings --> high
    tc_ctr_select_n.set_high().unwrap();
    tc_fb_select_n.set_high().unwrap();
    tc_lr_select_n.set_high().unwrap();

    // Set up SPI CLK and DataIn Lines.  These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio16.into_mode::<gpio::FunctionSpi>();

    let mut spi = spi::Spi::<_, _, 16>::new(pac.SPI0).init(&mut pac.RESETS, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);

    let dummy_data = &mut [0x88FFu16, 0x88FFu16];
    // Main loop forever
    loop {
        
        // Set chip select low (active)
        tc_fb_select_n.set_low().unwrap();

        // transfer 16 bits out and in.
        let res = spi.transfer( dummy_data).unwrap();
        let high_word = res[0];
        let low_word = res[1];
        let temp = temperature_degc(high_word);
        info!("High Word: {=u16}, As signed int: {=f32}", high_word, temp);
        info!("Low Word: {=u16}", low_word);
        if is_thermocouple_fault(high_word){
            info!("-----------TC Fault !")
        }


        delay.delay_ms(1);

        // Set chip select high (inactive)
        tc_fb_select_n.set_high().unwrap();
        delay.delay_ms(500);
    }
}


// ------- STRUCTS & FUNCTIONS ----------------
enum TempSensorFaultType {
    TempSensorShortToVCC,
    TempSensorShortToGND,
    TempSensorOpenCircuit,
    NoFault,
}
fn is_thermocouple_fault(high_word: u16) -> bool { 
    let bit0_mask = 0x0001_u16;
    return high_word & bit0_mask == 1;
}

fn thermocouple_fault_type(low_word: u16) -> TempSensorFaultType { 
    const OC_MASK:u16 = 0x0001_u16;
    const SCG_MASK:u16 = 0x0002_u16;
    const SCV_MASK:u16 = 0x0004_u16;

    if OC_MASK & low_word != 0 {
        return TempSensorFaultType::TempSensorOpenCircuit;
    }else if SCG_MASK & low_word != 0 {
        return TempSensorFaultType::TempSensorShortToGND;
    } else if SCV_MASK & low_word != 0 {
        return TempSensorFaultType::TempSensorShortToVCC
    } else {
        return TempSensorFaultType::NoFault;
    }
}

fn temperature_degc(high_word: u16) -> f32 { 
    let mask:u16 = 0b0000_0000_0000_0011;

    // Zero out the lowest two bits (reserved and fault bit)
    let masked:i16 = (high_word & !mask) as i16;
    let temp = float_funcs::fdiv(float_funcs::int_to_float(masked as i32), 16.0);
    
    return temp;
}

enum TempSensor {
    TempSensorCenter,
    TempSensorLeftRight,
    TempSensorFrontBack,
}

struct TempSensorController {
    
}

impl TempSensorController {
    // Associated function
    fn new() -> TempSensorController {
        TempSensorController {  }
    }

    fn disable_all(&self) { 
        
    }
    }
// End of file
