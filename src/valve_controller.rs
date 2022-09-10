
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::bank0::{Gpio11,Gpio12};

pub struct ValveController {
    pump_main_pin: gpio::Pin<Gpio12, Output<PushPull>>, 
    pump_bladder_pin: gpio::Pin<Gpio11, Output<PushPull>>, 
}
pub enum ValveState {
    Open,
    Closed,
}
impl ValveController {
    pub fn new(
        pump_main_pin:gpio::Pin<Gpio12, Output<PushPull>>, 
        pump_bladder_pin: gpio::Pin<Gpio11, Output<PushPull>>
    ) -> ValveController {
        ValveController{
            pump_main_pin:pump_main_pin,
            pump_bladder_pin:pump_bladder_pin
        }
    }
    pub fn init(&mut self) {
        self.set_main_valve(ValveState::Closed);
        self.set_bladder_valve(ValveState::Closed);
    }
    pub fn set_main_valve(&mut self, state:ValveState){
        match state {
            ValveState::Open => self.pump_main_pin.set_high().unwrap(),
            ValveState::Closed => self.pump_main_pin.set_low().unwrap(),
        } 
    }

    pub fn set_bladder_valve(&mut self, state:ValveState){
        match state {
            ValveState::Open => self.pump_bladder_pin.set_high().unwrap(),
            ValveState::Closed => self.pump_bladder_pin.set_low().unwrap(),
        } 
    }

}