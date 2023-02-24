use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::{DynPin};
use rp2040_hal::spi::SpiDevice;
use rp_pico::hal::spi::{Spi, Enabled};

// Logging stuff..
use defmt::*;
use defmt_rtt as _;

use ChipSelectState::{Selected, Deselected};
use serde::{Serialize, Deserialize};
use heapless::Vec;

// Constants
const HIGH_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1100;
const LOW_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1000; // Unused for now.
const TC_ERROR_MASK:u16 = 0b0000_0000_0000_0001;      // Some TC error.  Unused for now.
const OC_ERROR_MASK:u16 = 0b0000_0000_0000_0001;      // Open Circuit error
const SCG_ERROR_MASK:u16 = 0b0000_0000_0000_0010;     // Short Circuit to GND error
const SCV_ERROR_MASK:u16 = 0b0000_0000_0000_0100;     // Short Circuit to VCC error


#[derive(Debug, Copy, Clone)]
pub enum ChipSelectState {
    Selected,
    Deselected,
}

#[derive(Debug, Copy, Clone, Format, PartialEq, Serialize, Deserialize)]
// Thermocouple errors reported by each TC controller chip
pub enum TCError {
    TempSensorShortToVCC,
    TempSensorShortToGND,
    TempSensorOpenCircuit,
    NoTCError,
}

// A Thermocouple channel encapsulates a chip select pin 
// and a boolean that describes whether the T/C is grounded or not
pub struct TCChannelDescriptor {
    cs_pin: DynPin,
    grounded: bool,
}

// A board level thermocouple controller which owns multiple T/C channels
// in an array, and an SPI bus to control them all
pub struct ThermocoupleController<D: SpiDevice, const N: usize> {
    channels: Vec<TCChannelDescriptor, N>,
    spi: Spi<Enabled, D, 16>,
}

impl <D: SpiDevice, const N: usize> ThermocoupleController<D, N> {
    // Create a new T/C controller
    pub fn new(spi: Spi<Enabled, D, 16>) -> ThermocoupleController<D, N> {
        let channels = Vec::new();
        ThermocoupleController {
            channels,
            spi,
        }
    }

    // Init: set all chip-selects to Deselected
    pub fn init(&mut self){
        self.deselect_all ();
    }

    pub fn add_channel(&mut self, cs_pin: DynPin, grounded: bool) {
        let channel: TCChannelDescriptor = TCChannelDescriptor { cs_pin, grounded };
        let result = self.channels.push(channel);
        match result {
            Err(_) => {info!("Failed to add channel")},
            Ok(_) => {()},
        }
    }

    // Acquires one temp measurement from the selected channel
    pub fn acquire(&mut self, channel: usize) -> Result<f32,TCError> {
        // Read raw data
        let (w0,w1) = self.read_raw(channel);

        // Check for an error.  GND errors on grounded TCs are ignored.  Data is still valid
        match self.get_tc_error(w1) {
            Some(error) => {    // If there is an error reported by the IC...
                if self.channels[channel].grounded == true && error == TCError::TempSensorShortToGND {
                    // If the error is a grounding error and the TC is grounded (intentionally)
                    return Ok(self.tc_temperature_degc(w0));    // Return Ok(temp)
                } else {    // We have a real TC error
                    return Err(error);
                }
            },
            None => {   // If no error return Ok(temp)
                Ok(self.tc_temperature_degc(w0))
            }
        }
    }
    
    // Deselect all chip selects
    fn deselect_all (&mut self) {
        for ch in 0..self.channels.len() {
            self.set_chip_select(ch, Deselected);
        }
    }

    // Low-level function to set the chip select line of one tc to specified state
    // !!! Does NOT enforce exclusivity.!!!
    fn set_chip_select(&mut self, channel: usize, state:ChipSelectState){
        match state {
            Deselected => self.channels[channel].cs_pin.set_high().unwrap(),
            Selected => self.channels[channel].cs_pin.set_low().unwrap(),
        }
    }

    // Select chip select, SPI read both temp words, deselect chip select
    fn read_raw(&mut self, channel: usize) -> (u16, u16) {
        let buf = &mut [0x0000u16, 0x0000u16];   // Must write any data to chip to read data.
        self.set_chip_select(channel, Selected);       // Assert chip select 
        let raw = self.spi.transfer( buf).unwrap(); // transfer 2 16 bit words out/in.
        self.set_chip_select(channel, Deselected);     // De-assert chip select
       
        (raw[0], raw[1])
    }
    // Compute the temp from raw temp
    fn tc_temperature_degc(&self, high_word: u16) -> f32 { 
        // Clear the lowest two bits (reserved and fault bit)
        // and convert to signed int using the same length word
        let masked:i16 = (high_word & HIGH_WORD_DATA_MASK) as i16;

        // Divide by 16.0 to scale the raw value to a temperature in deg C
        let temp = (masked as f32)/16.0;
        //let temp = float_funcs::fdiv(float_funcs::int_to_float(masked as i32), 16.0);
        
        return temp;
    }
    
    // Compute the ref temp from raw ref temp
    fn ref_temperature_degc(&self, low_word: u16) -> f32 { 
        // Clear the lowest three bits (fault code bits)
        // and convert to signed int using the same length word
        let masked:i16 = (low_word & LOW_WORD_DATA_MASK) as i16;
        // Divide by 256.0 to scale the raw value to a temperature in deg C
        let temp = (masked as f32)/256.0;
        //let temp = float_funcs::fdiv(float_funcs::int_to_float(masked as i32), 256.0);
        
        return temp;
    }
    // Test one bit in high word to determine if any TC error ocurred
    // Useful in cases where only one (high) word is read for speed
    fn is_tc_error(&self, high_word: u16) -> bool { 
        return high_word & TC_ERROR_MASK == 1;
    }
    
    // Tests low word for detailed TC error report
    fn get_tc_error(&self, low_word: u16) -> Option<TCError> {     
        if OC_ERROR_MASK & low_word != 0 {
            return Some(TCError::TempSensorOpenCircuit);
        }else if SCG_ERROR_MASK & low_word != 0 {
            return Some(TCError::TempSensorShortToGND);
        } else if SCV_ERROR_MASK & low_word != 0 {
            return Some(TCError::TempSensorShortToVCC);
        }
        None  // Default: return no error
    }
    
    
}