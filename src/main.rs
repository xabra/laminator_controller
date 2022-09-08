//! # Laminator Controller
//!
//! Runs the custom laminator controller powered by the Pico W
//!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};
//use embedded_hal::blocking::i2c::SevenBitAddress;
// The macro for our start-up function
use rp_pico::entry;

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rp2040_hal::rom_data::float_funcs::{fdiv, fmul, int_to_float};

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

//use rp_pico::hal::spi;
use fugit::RateExtU32;

/// Entry point
#[entry]
fn main() -> ! {        // '!' means never returns
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

    // =============  I2C Control ===========================

    // Init the AD converter auxiliary GPIO pins
    let mut ad_convert_start = pins.gpio6.into_push_pull_output();      // Active low??
    let mut ad_alert = pins.gpio7.into_floating_input();        // AD Alert/Busy pin
    // Initially set TC chip select settings --> high
    ad_convert_start.set_low().unwrap();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio4.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio5.into_mode::<hal::gpio::FunctionI2C>();

    let mut i2c = rp2040_hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    const I2C_BUS_ADDRESS:u8 = 0b_0010_0000; // AD7994 i2c bus address
    const I2C_CONFIG_REG_ADDRESS:u8 = 0b_0000_0010;  // Select the Config register for write
    const I2C_CONFIG_REG_DATA:u8 = 0b_0101_1011; // Set config to convert channels 1 & 3 and filter on, busy output
    const I2C_CONVERSION_RESULT_REG_ADDRESS:u8 = 0b_0000_0000;   //Select the conversion result register for subsequent reads.
    const I2C_CONVERSION_RESULT_DATA_MASK:u16 = 0b_0000_1111_1111_1111; // Mask off the highest four bits
    const I2C_CONVERSION_RESULT_CHANNEL_MASK:u8 = 0b_0000_0011; // Mask off 
    const PRESSURE_SENSE_VREF:f32 = 5.0;

    // Write to the AD7994 Configuration register
    i2c.write(I2C_BUS_ADDRESS, &[I2C_CONFIG_REG_ADDRESS, I2C_CONFIG_REG_DATA]).unwrap();

     // Write to the AD7994 address register for subsequent conversion result reads in the loop
     i2c.write(I2C_BUS_ADDRESS, &[I2C_CONVERSION_RESULT_REG_ADDRESS]).unwrap();   
    
    let mut read_result = [0; 2];
    
    // Main loop forever
    loop {
        // Generate conversion start pulse.
        ad_convert_start.set_high().unwrap();   // Power up the converter
        delay.delay_us(3);    // Remain high for minimum pulse width (~1us)
        ad_convert_start.set_low().unwrap();    // Falling edge starts the conversion
        delay.delay_us(3);    // Delay for conversion to finish before i2c reading (~2us)

        // Do i2c read from conversion
        i2c.read(I2C_BUS_ADDRESS, &mut read_result).unwrap();
        let value = (((read_result[0] as u16) << 8) | read_result[1] as u16) & I2C_CONVERSION_RESULT_DATA_MASK;

        // Scale the raw pressure signal
        let pressure_volts = fmul(fdiv(int_to_float(value as i32), 4095.0), PRESSURE_SENSE_VREF);

        // Get channel number (zero based, 0-3)
        let channel = (read_result[0] >>4) & I2C_CONVERSION_RESULT_CHANNEL_MASK;

        info!("Channel: {=u8},      Value:{=f32}", channel, pressure_volts);

        delay.delay_ms(1000);    // Delay to set overall loop rate
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

fn debug_print_tc_fault_type (low_word: u16, name: &str){
    let ft = thermocouple_fault_type(low_word);
    match ft {
        TempSensorFaultType::TempSensorShortToVCC => {info!("{=str} Shorted to VCC", name);}
        TempSensorFaultType::TempSensorShortToGND => {info!("{=str} Shorted to GND", name);}
        TempSensorFaultType::TempSensorOpenCircuit => {info!("{=str} Open Circuit", name);}
        TempSensorFaultType::NoFault => {info!("{=str} No fault", name);}
    }
}

fn debug_print_tc (tc_raw: &[u16], name: &str) {
    if  is_thermocouple_fault(tc_raw[0]) {
        //info!("TC Fault:");
        debug_print_tc_fault_type(tc_raw[1], name);
    } else {
        let temp = temperature_degc(tc_raw[0]);
        let ref_temp = reference_junction_temperature_degc(tc_raw[1]);
        info!("{=str} Temp: {=f32}   Ref Temp:{=f32}", name, temp, ref_temp);
    }
}

fn temperature_degc(high_word: u16) -> f32 { 
    const MASK:u16 = 0b0000_0000_0000_0011;

    // Clear the lowest two bits (reserved and fault bit)
    // and convert to signed int using the same length word
    let masked:i16 = (high_word & !MASK) as i16;

    // Divide by 16.0 to scale the raw value to a temperature in deg C
    let temp = fdiv(int_to_float(masked as i32), 16.0);
    
    return temp;
}

fn reference_junction_temperature_degc(low_word: u16) -> f32 { 
    const MASK:u16 = 0b0000_0000_0000_0111;

    // Clear the lowest three bits (fault code bits)
    // and convert to signed int using the same length word
    let masked:i16 = (low_word & !MASK) as i16;

    // Divide by 256.0 to scale the raw value to a temperature in deg C
    let temp = fdiv(int_to_float(masked as i32), 256.0);
    
    return temp;
}

// ------------ Temp Controller ----------------
enum TempSensor {
    TempSensorCenter,
    TempSensorLeftRight,
    TempSensorFrontBack,
}

struct TempSensorController {
    // Chip select gpio pins
    //tc_ctr_select_n<rp_pico::Pins>
   // tc_lr_select_n:rp_pico::Pins::Pin<rp2040_hal::Gpio19,Output<PushPull>>,
    //c_fb_select_n:rp_pico::Pins::Pin<rp2040_hal::Gpio20,Output<PushPull>>,
}

impl TempSensorController {
    // Associated function
    // fn new(pins:rp_pico::Pins) -> TempSensorController {
    //     TempSensorController { 
    //         tc_ctr_select_n: pins.gpio17.into_push_pull_output(),
    //         tc_lr_select_n: pins.gpio19.into_push_pull_output(),
    //         tc_fb_select_n: pins.gpio20.into_push_pull_output(),   
    //     }
    // }
}
// End of file
