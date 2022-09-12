
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;

pub struct ValveController<I: PinId> {
    valve_pin: gpio::Pin<I, Output<PushPull>>, 
}
pub enum ValveState {
    Vent,
    Pump,
}
impl <I: PinId> ValveController<I> {
    pub fn new(
        valve_pin:gpio::Pin<I, Output<PushPull>>, 
    ) -> ValveController<I> {
        ValveController{
            valve_pin:valve_pin,
        }
    }
    pub fn init(&mut self) {
        self.set_state(ValveState::Pump);
    }
    pub fn set_state(&mut self, state:ValveState){
        match state {
            ValveState::Vent => self.valve_pin.set_high().unwrap(),
            ValveState::Pump => self.valve_pin.set_low().unwrap(),
        } 
    }


}