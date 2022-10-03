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

// Sensor scaling from AD count to volts per Vref
const PRESSURE_SENSE_VREF:f32 = 5.0;

// Sensor scaling from Volts to Pa per spec sheet PSE54x
const SCALE_VA:f32 = 1.0;   // Volts
const SCALE_VB:f32 = 5.0;   // Volts
const SCALE_PA:f32 = 0.0;   // Pa
const SCALE_PB:f32 = -101_000.0;  // Pa
const SCALE_SLOPE:f32 = (SCALE_PB-SCALE_PA)/(SCALE_VB-SCALE_VA);
const SCALE_OFFSET:f32 = SCALE_PA-SCALE_VA*SCALE_SLOPE;

pub struct PressureMeasurement {
    pub channel_index: u8,
    pub pressure_pa: f32,
}
pub struct PressureSensorController<I1, I2, P> where
    I1: PinId,
    I2: PinId,
    P: Read+Write,
    {
    ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
    ad_busy_pin: gpio::Pin<I2, Input<Floating>>,   // Not used for now
    i2c: P,  
    pressures: [f32;4],  // A place to cache the results
}

impl <I1, I2, P> PressureSensorController<I1, I2, P> where
    I1: PinId,
    I2: PinId,
    P: Read+Write,
    {
    pub fn new(
        ad_start_pin: gpio::Pin<I1, Output<PushPull>>,
        ad_busy_pin: gpio::Pin<I2, Input<Floating>>,
        i2c: P,
    ) -> PressureSensorController<I1, I2, P> {  

        PressureSensorController {
            ad_start_pin,
            ad_busy_pin,
            i2c,
            pressures: [0.0_f32; 4],
        }
    }
    pub fn 
    init(&mut self)  {
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
    fn trigger_conversion(&mut self) {
        // Generate conversion start pulse.
        self.ad_start_pin.set_high().unwrap();   // Power up the converter
        cortex_m::asm::delay(300);// Remain high for minimum pulse width (~1us)
        self.ad_start_pin.set_low().unwrap();    // Falling edge starts the conversion
        cortex_m::asm::delay(300);  // Delay for conversion to finish before i2c reading (~2us)
    }

    pub fn acquire_one_channel(&mut self) -> (usize, f32) {
        let mut buf = [0; 2];
        self.trigger_conversion();

        // Do i2c read from conversion
        let res = self.i2c.read(I2C_BUS_ADDRESS, &mut buf);  //.unwrap() fails...
        match res {
            Ok(()) => (),
            Err(_) => panic!("I2C Read error"),
        };
        let channel_index = raw_to_channel_index(buf[0]) as usize;
        let counts = raw_to_counts (buf[0], buf[1]);
        let volts = counts_to_volts (counts);
        let pressure_pa = volts_to_pressure_pa(volts);

        (channel_index, pressure_pa ) // Return pressure measurement tuple
    }

    pub fn acquire_all(&mut self) {

        let (ch, p) = self.acquire_one_channel();
        self.pressures[ch] = p;
        let (ch, p) = self.acquire_one_channel();
        self.pressures[ch] = p;

    }

    pub fn get_pressure(&mut self, channel: usize) -> f32 {
        self.pressures[channel]
    }
}

// helper functions
pub fn raw_to_channel_index (high_word: u8) -> u8 {
    (high_word >>4) & I2C_CONVERSION_RESULT_CHANNEL_MASK
}
pub fn raw_to_counts (high_word: u8, low_word:u8) -> u16 {
    (((high_word as u16) << 8) | low_word as u16) & I2C_CONVERSION_RESULT_DATA_MASK
}
pub fn counts_to_volts (counts:u16) -> f32 {
    ((counts as i32) as f32)/4095.0 * PRESSURE_SENSE_VREF
}
pub fn volts_to_pressure_pa(p_volts: f32) -> f32 {
    p_volts*SCALE_SLOPE + SCALE_OFFSET
}