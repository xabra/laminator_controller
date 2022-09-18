// =============  I2C Control ===========================

use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::i2c::I2C;
use rp_pico::hal::gpio::{PinId, Pin, Input, Output, PushPull, FunctionI2C, Floating};
use fugit::RateExtU32;

const I2C_BUS_ADDRESS:u8 = 0b_0010_0000; // AD7994 i2c bus address
const I2C_CONFIG_REG_ADDRESS:u8 = 0b_0000_0010;  // Select the Config register for write
const I2C_CONFIG_REG_DATA:u8 = 0b_0101_1011; // Set config to convert channels 1 & 3 and filter on, busy output
const I2C_CONVERSION_RESULT_REG_ADDRESS:u8 = 0b_0000_0000;   //Select the conversion result register for subsequent reads.
const I2C_CONVERSION_RESULT_DATA_MASK:u16 = 0b_0000_1111_1111_1111; // Mask off the highest four bits
const I2C_CONVERSION_RESULT_CHANNEL_MASK:u8 = 0b_0000_0011; // Mask off 
const PRESSURE_SENSE_VREF:f32 = 5.0;

pub struct PressureSensorController<I1: PinId, I2: PinId, P: Read + Write> {    // <????>
    ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
    ad_busy_pin: gpio::Pin<I2, Input<Floating>>,
    i2c: P,       // <????>
}

 impl <I1: PinId, I2: PinId, P: Read+Write> PressureSensorController<I1, I2, P>  // <????>
 {
    pub fn new (
        ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
        ad_busy_pin: gpio::Pin<I2, Input<Floating>>,
        i2c: P,      // <????>
    ) -> PressureSensorController<I1, I2, P> {  // <????>

        PressureSensorController {
            ad_start_pin,
            ad_busy_pin,
            i2c,
        }
    }
    pub fn init(&self) {
        // Set ad start pin low
        self.ad_start_pin.set_low().unwrap();

        // Write to the AD7994 Configuration register
        self.i2c.write(I2C_BUS_ADDRESS, &[I2C_CONFIG_REG_ADDRESS, I2C_CONFIG_REG_DATA]).unwrap();

        // Write to the AD7994 address register for subsequent conversion result reads in the loop
        self.i2c.write(I2C_BUS_ADDRESS, &[I2C_CONVERSION_RESULT_REG_ADDRESS]).unwrap();   
    
    }
    fn start_conversion(&self) {
        // Generate conversion start pulse.
        self.ad_start_pin.set_high().unwrap();   // Power up the converter
        delay.delay_us(3);    // Remain high for minimum pulse width (~1us)
        self.ad_start_pin.set_low().unwrap();    // Falling edge starts the conversion
        delay.delay_us(3);    // Delay for conversion to finish before i2c reading (~2us)
    }

    pub fn read_pressures(&self) {
        let mut read_result = [0; 2];
        self.start_conversion();

        // Do i2c read from conversion
        self.i2c.read(I2C_BUS_ADDRESS, &mut read_result).unwrap();
        let value = (((read_result[0] as u16) << 8) | read_result[1] as u16) & I2C_CONVERSION_RESULT_DATA_MASK;

        // Scale the raw pressure signal
        // delete: let pressure_volts = int_to_float(value as i32)/4095.0 * PRESSURE_SENSE_VREF;
        let pressure_volts = ((value as i32) as f32)/4095.0 * PRESSURE_SENSE_VREF;

        // Get channel number (zero based, 0-3)
        let channel = (read_result[0] >>4) & I2C_CONVERSION_RESULT_CHANNEL_MASK;
    }

 }



