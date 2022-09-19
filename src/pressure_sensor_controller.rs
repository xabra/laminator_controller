// =============  I2C Control ===========================

use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::{PinId, Input, Output, PushPull, Floating};

const I2C_BUS_ADDRESS:u8 = 0b_0010_0000; // AD7994 i2c bus address
const I2C_CONFIG_REG_ADDRESS:u8 = 0b_0000_0010;  // Select the Config register for write
const I2C_CONFIG_REG_DATA:u8 = 0b_0101_1011; // Set config to convert channels 1 & 3 and filter on, busy output
const I2C_CONVERSION_RESULT_REG_ADDRESS:u8 = 0b_0000_0000;   //Select the conversion result register for subsequent reads.
const I2C_CONVERSION_RESULT_DATA_MASK:u16 = 0b_0000_1111_1111_1111; // Mask off the highest four bits
const I2C_CONVERSION_RESULT_CHANNEL_MASK:u8 = 0b_0000_0011; // Mask off 
const PRESSURE_SENSE_VREF:f32 = 5.0;

pub struct PressureMeasurement {
    channel: u8,
    pressure_volts: f32,
}
pub struct PressureSensorController<I1, I2, P> where
    I1: PinId,
    I2: PinId,
    P: Read+Write,
    { 
    ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
    ad_busy_pin: gpio::Pin<I2, Input<Floating>>,
    i2c: P,      
}

 impl <I1, I2, P> PressureSensorController<I1, I2, P> where
    I1: PinId,
    I2: PinId,
    P: Read+Write,
    {
    pub fn new (
        ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
        ad_busy_pin: gpio::Pin<I2, Input<Floating>>,
        i2c: P,
    ) -> PressureSensorController<I1, I2, P> {  

        PressureSensorController {
            ad_start_pin,
            ad_busy_pin,
            i2c,
        }
    }
    pub fn init(&mut self)  {
        // Set ad start pin low
        self.ad_start_pin.set_low().unwrap();

        // Write to the AD7994 Configuration register
        //let mut res = self.i2c.write(I2C_BUS_ADDRESS, &[I2C_CONFIG_REG_ADDRESS, I2C_CONFIG_REG_DATA]);//.unwrap();
        match self.i2c.write(I2C_BUS_ADDRESS, &[I2C_CONFIG_REG_ADDRESS, I2C_CONFIG_REG_DATA]) {
            Ok(()) => (),
            Err(_) => panic!("I2C Write error"),
        };


        // Write to the AD7994 address register for subsequent conversion result reads in the loop
        let res = self.i2c.write(I2C_BUS_ADDRESS, &[I2C_CONVERSION_RESULT_REG_ADDRESS]);     //.unwrap();   
        match res {
            Ok(()) => (),
            Err(_) => panic!("I2C Write error"),
        };
    }
    fn start_conversion(&mut self, mut delay: cortex_m::delay::Delay) {
        // Generate conversion start pulse.
        self.ad_start_pin.set_high().unwrap();   // Power up the converter
        delay.delay_us(3);    // Remain high for minimum pulse width (~1us)
        self.ad_start_pin.set_low().unwrap();    // Falling edge starts the conversion
        delay.delay_us(3);    // Delay for conversion to finish before i2c reading (~2us)
    }

    pub fn read_pressures(&mut self, delay: cortex_m::delay::Delay) -> PressureMeasurement {
        let mut read_result = [0; 2];
        self.start_conversion(delay);

        // Do i2c read from conversion
        let res = self.i2c.read(I2C_BUS_ADDRESS, &mut read_result);  //.unwrap();
        match res {
            Ok(()) => (),
            Err(_) => panic!("I2C Read error"),
        };

        let value = (((read_result[0] as u16) << 8) | read_result[1] as u16) & I2C_CONVERSION_RESULT_DATA_MASK;

        // Scale the raw pressure signal
        // delete: let pressure_volts = int_to_float(value as i32)/4095.0 * PRESSURE_SENSE_VREF;
        let pressure_volts = ((value as i32) as f32)/4095.0 * PRESSURE_SENSE_VREF;

        // Get channel number (zero based, 0-3)
        let channel = (read_result[0] >>4) & I2C_CONVERSION_RESULT_CHANNEL_MASK;

        PressureMeasurement { channel, pressure_volts } // Return pressure measurement struct
    }

 }



