use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::spi::SpiDevice;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::{PinId, Output, PushPull};
use rp_pico::hal::spi::{Spi, Enabled};



// Logging stuff..
use defmt::*;
use defmt_rtt as _;

use ChipSelectState::{Selected, Deselected};
use TCChannel::{Center, LeftRight, FrontBack};
use serde::{Serialize, Deserialize};

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

// Give the T/Cs names
#[derive(Debug, Copy, Clone, Format)]
// Gives names to each board level T/C channel
// This seems very ad hoc...not general
pub enum TCChannel {
    Center,
    LeftRight,
    FrontBack,
}
#[derive(Debug, Copy, Clone, Format, PartialEq, Serialize, Deserialize)]
// Thermocouple errors reported by each TC controller chip
pub enum TCError {
    TempSensorShortToVCC,
    TempSensorShortToGND,
    TempSensorOpenCircuit,
    NoTCError,
}
// Struct containing a channel, a temp and the error state for that channel
// The T/C cold-junction refererence temp is not used right now.
// It should be called TemperatureRecord or something...
#[derive(Debug, Copy, Clone, Format)]
pub struct Temperatures {
    pub channel: TCChannel,
    pub tc_temp: f32,
    //pub ref_temp: f32,
    pub error: Option<TCError>,
}

// Encapsulates the board level controller which owns three TC controller ICs
// and an SPI bus to control them all
// Lousy code...?
pub struct ThermocoupleController<I1: PinId, I2: PinId, I3: PinId, D: SpiDevice> {
    cs_ctr: gpio::Pin<I1, Output<PushPull>>,
    cs_lr: gpio::Pin<I2, Output<PushPull>>,
    cs_fb: gpio::Pin<I3, Output<PushPull>>,
    spi: Spi<Enabled, D, 16>,
}

impl <I1: PinId, I2: PinId, I3: PinId, D: SpiDevice> ThermocoupleController<I1, I2, I3, D> {
    // Create a new T/C controller
    pub fn new(
        cs_ctr: gpio::Pin<I1, Output<PushPull>>, 
        cs_lr: gpio::Pin<I2, Output<PushPull>>,
        cs_fb: gpio::Pin<I3, Output<PushPull>>,
        spi: Spi<Enabled, D, 16>) 
        -> ThermocoupleController<I1, I2, I3, D> {

        ThermocoupleController{
            cs_ctr,
            cs_lr,
            cs_fb,
            spi,
        }
    }

    // Init: set all chip-selects to Deselected
    pub fn init(&mut self){
        self.deselect_all ();
    }

    // This is the main public interface to this module
    // Acquires one temp measurement from the selected channel
    pub fn acquire(&mut self, channel: TCChannel) -> Result<f32,TCError> {
        // Read raw data
        let (w0,w1) = self.read_raw(channel);

        // Check for an error
        match self.get_tc_error(w1) {
            Some(error) => {
                return Err(error);
            },
            None => {
                Ok(self.tc_temperature_degc(w0))
            }
        }
    }
    
    // Deselect all chip selects
    fn deselect_all (&mut self) {
        self.set_chip_select(Center, Deselected);
        self.set_chip_select(LeftRight, Deselected);
        self.set_chip_select(FrontBack, Deselected);
    }

    // Low-level function to set the chip select line of one tc to specified state
    // Does NOT enforce exclusivity.
    // Lousy code --> 
    fn set_chip_select(&mut self, channel: TCChannel, state:ChipSelectState){
        match channel {
            Center => {
                match state {
                    Deselected => self.cs_ctr.set_high().unwrap(),
                    Selected => self.cs_ctr.set_low().unwrap(),
                } 
            },
            LeftRight => {
                match state {
                    Deselected => self.cs_lr.set_high().unwrap(),
                    Selected => self.cs_lr.set_low().unwrap(),
                } 
            },
            FrontBack => {
                match state {
                    Deselected => self.cs_fb.set_high().unwrap(),
                    Selected => self.cs_fb.set_low().unwrap(),
                } 
            },
        }

    }

    // Select chip select, SPI read both temp words, deselect chip select
    fn read_raw(&mut self, channel: TCChannel) -> (u16, u16) {
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
    fn _is_tc_error(&self, high_word: u16) -> bool { 
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