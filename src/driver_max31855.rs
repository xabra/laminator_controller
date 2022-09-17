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
use TCId::{Center, LeftRight, FrontBack};

// Constants
const HIGH_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1100;
const LOW_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1000;
//const TC_ERROR_MASK:u16 = 0b0000_0000_0000_0001;// Some TC error.  Unused for now.
const OC_MASK:u16 = 0b0000_0000_0000_0001;      // Open Circuit error
const SCG_MASK:u16 = 0b0000_0000_0000_0010;     // Short Circuit to GND error
const SCV_MASK:u16 = 0b0000_0000_0000_0100;     // Short Circuit to VCC error

pub enum ChipSelectState {
    Selected,
    Deselected,
}

// Give the T/Cs names
pub enum TCId {
    Center,
    LeftRight,
    FrontBack,
}
pub enum TCError {
    TempSensorShortToVCC,
    TempSensorShortToGND,
    TempSensorOpenCircuit,
}
pub struct Temperatures {
    pub tc_temp: f32,
    pub ref_temp: f32,
}
// Lousy code- should use anypin...
pub struct DriverMax31855<I1: PinId, I2: PinId, I3: PinId, D: SpiDevice> {
    cs_ctr: gpio::Pin<I1, Output<PushPull>>,
    cs_lr: gpio::Pin<I2, Output<PushPull>>,
    cs_fb: gpio::Pin<I3, Output<PushPull>>,
    spi: Spi<Enabled, D, 16>,
}

impl <I1: PinId, I2: PinId, I3: PinId, D: SpiDevice> DriverMax31855<I1, I2, I3, D> {
    pub fn new(
        cs_ctr: gpio::Pin<I1, Output<PushPull>>, 
        cs_lr: gpio::Pin<I2, Output<PushPull>>,
        cs_fb: gpio::Pin<I3, Output<PushPull>>,
        spi: Spi<Enabled, D, 16>) 
        -> DriverMax31855<I1, I2, I3, D> {
        DriverMax31855{
            cs_ctr,
            cs_lr,
            cs_fb,
            spi,
        }
    }

    pub fn read_temps(&mut self) -> Result<Temperatures, TCError> {
        let (w0,w1) = self.read_raw();
        info!("RAW ::: Temp: {=u16}   Ref Temp:{=u16}", w0, w1);
        match self.get_tc_error(w1) {
            Some(err) => return Err(err),
            None => {
                let tc_temp = self.tc_temperature_degc(w0);
                let ref_temp = self.ref_temperature_degc(w1);
                info!("READ TEMPS::: Temp: {=f32}   Ref Temp:{=f32}", tc_temp, ref_temp);
                Ok(Temperatures { tc_temp, ref_temp })
            }
        }
    }

    // Select only one chip select, by first deselecting all to prevent bus contention
    pub fn set_exclusive_chip_select (&mut self, tc: TCId, state:ChipSelectState) {
        self.deselect_all();
        self.set_chip_select(tc, Selected);
    }
    
    // Deselect all chip selects
    fn deselect_all (&mut self) {
        self.set_chip_select(Center, Deselected);
        self.set_chip_select(LeftRight, Deselected);
        self.set_chip_select(FrontBack, Deselected);
    }

    // Low-level function to set the chip select line of one tc to specified state
    // Does NOT enforce exclusivity.
    // Lousy code --> use anypin or something...
    fn set_chip_select(&mut self, tc: TCId, state:ChipSelectState){
        match tc {
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

    fn read_raw(&mut self) -> (u16, u16) {
        let buf = &mut [0x0000u16, 0x0000u16];   // Must write any data to chip to read data.
        self.set_chip_select(Center, Selected);       // Assert chip select 
        let raw = self.spi.transfer( buf).unwrap(); // transfer 2 16 bit words out/in.
        self.set_chip_select(Center, Deselected);     // De-assert chip select
       
        (raw[0], raw[1])
    }

    fn tc_temperature_degc(&self, high_word: u16) -> f32 { 
        // Clear the lowest two bits (reserved and fault bit)
        // and convert to signed int using the same length word
        let masked:i16 = (high_word & HIGH_WORD_DATA_MASK) as i16;

        // Divide by 16.0 to scale the raw value to a temperature in deg C
        let temp = (masked as f32)/16.0;
        //let temp = float_funcs::fdiv(float_funcs::int_to_float(masked as i32), 16.0);
        
        return temp;
    }
    
    fn ref_temperature_degc(&self, low_word: u16) -> f32 { 
        // Clear the lowest three bits (fault code bits)
        // and convert to signed int using the same length word
        let masked:i16 = (low_word & LOW_WORD_DATA_MASK) as i16;
        // Divide by 256.0 to scale the raw value to a temperature in deg C
        let temp = (masked as f32)/256.0;
        //let temp = float_funcs::fdiv(float_funcs::int_to_float(masked as i32), 256.0);
        
        return temp;
    }

    fn is_tc_error(&self, high_word: u16) -> bool { 
        return high_word & TC_ERROR_MASK == 1;
    }
    
    fn get_tc_error(&self, low_word: u16) -> Option<TCError> {     
        if OC_MASK & low_word != 0 {
            return Some(TCError::TempSensorOpenCircuit);
        }else if SCG_MASK & low_word != 0 {
            return Some(TCError::TempSensorShortToGND);
        } else if SCV_MASK & low_word != 0 {
            return Some(TCError::TempSensorShortToVCC);
        }
        
        None  // Default: return no error
    }
    
    
}