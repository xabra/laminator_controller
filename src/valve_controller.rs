
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
    current_valve_state: ValveState,
    }

impl <I: PinId> ValveController<I> {
    pub fn new(
        valve_pin: gpio::Pin<I, Output<PushPull>>,
    ) -> ValveController<I> {
        ValveController{
            valve_pin,
            current_valve_state: ValveState::Pump,  // Initial state on creation  TODO pass this in?
        }
    }

    pub fn set_valve_state(&mut self, new_valve_state:ValveState){
        if new_valve_state != self.current_valve_state {      // If state changed...
            self.current_valve_state = new_valve_state;           // Update internal state variable
            match new_valve_state {                               // drive the pin appropriately 
                ValveState::Vent => {
                    self.valve_pin.set_high().unwrap();
                },
                ValveState::Pump => {
                    self.valve_pin.set_low().unwrap();
                },
            } 
        }

    }
    pub fn get_valve_state(&self) -> ValveState{
        self.current_valve_state    // Return current state
    }

}