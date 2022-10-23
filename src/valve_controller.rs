
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;
use serde::{Serialize, Deserialize};

#[derive(Debug, Copy, Clone, defmt::Format, PartialEq, Serialize, Deserialize)]
pub enum ValveState {
    Vent,
    Pump,
}

pub struct ValveController<I: PinId> {
    valve_pin: gpio::Pin<I, Output<PushPull>>,
    }

impl <I: PinId> ValveController<I> {
    pub fn new(
        valve_pin: gpio::Pin<I, Output<PushPull>>,
    ) -> ValveController<I> {
        ValveController{
            valve_pin,
        }
    }

    pub fn set_valve_state(&mut self, state:ValveState){
        match state {
            ValveState::Vent => self.valve_pin.set_high().unwrap(),
            ValveState::Pump => self.valve_pin.set_low().unwrap(),
        } 
    }

}