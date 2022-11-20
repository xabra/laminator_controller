use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;

pub struct PWM <I: PinId> {
    pin: gpio::Pin<I, Output<PushPull>>,
    counter_max: u16,
    duty_thresh: u16,
}

impl <I: PinId> PWM <I> {
    pub fn new(pin: gpio::Pin<I, Output<PushPull>>, counter_max: u16) -> PWM<I> {
        PWM {
            pin,
            counter_max,
            duty_thresh: 0,
        }
    }
    pub fn set_duty_factor(&mut self, df: f32){
        if df >= 1.0 {
            self.duty_thresh = self.counter_max;        // Saturate at 100%
        } else {
            self.duty_thresh = (df*(self.counter_max as f32)) as u16;
        }
        
    }
    pub fn get_duty_factor (&self) -> f32 {
        (self.duty_thresh as f32)/(self.counter_max as f32)
    }
    pub fn get_duty_threshold (&self) -> u16 {
        self.duty_thresh
    }
    pub fn set_high(&mut self) {
        self.pin.set_high().unwrap();
    }
    pub fn set_low(&mut self) {
        self.pin.set_low().unwrap();
    }

}