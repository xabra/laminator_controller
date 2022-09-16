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

// Constants
const HIGH_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1100;
const LOW_WORD_DATA_MASK:u16 = 0b1111_1111_1111_1000;
const TC_ERROR_MASK:u16 = 0x0001_u16;
const OC_MASK:u16 = 0x0001_u16;
const SCG_MASK:u16 = 0x0002_u16;
const SCV_MASK:u16 = 0x0004_u16;

pub enum ChipSelectState {
    Selected,
    Deselected,
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
pub struct DriverMax31855<I: PinId, D: SpiDevice> {
    cs: gpio::Pin<I, Output<PushPull>>,
    spi: Spi<Enabled, D, 16>,
}

impl <I: PinId, D: SpiDevice> DriverMax31855<I, D> {
    pub fn new(cs: gpio::Pin<I, Output<PushPull>>, spi: Spi<Enabled, D, 16>) -> DriverMax31855<I, D> {
        DriverMax31855{
            cs,
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

    fn set_chip_select(&mut self, state:ChipSelectState){
        match state {
            ChipSelectState::Deselected => self.cs.set_high().unwrap(),
            ChipSelectState::Selected => self.cs.set_low().unwrap(),
        } 
    }

    fn read_raw(&mut self) -> (u16, u16) {
        let buf = &mut [0x0000u16, 0x0000u16];   // Must write any data to chip to read data.
        self.set_chip_select(Selected);       // Assert chip select 
        let raw = self.spi.transfer( buf).unwrap(); // transfer 2 16 bit words out/in.
        self.set_chip_select(Deselected);     // De-assert chip select
       
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