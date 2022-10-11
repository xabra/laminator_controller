
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::PushPull;

#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum ValveState {
    Vent,
    Pump,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum PressureState {
    Vented,
    Intermediate,
    Evacuated,
}
#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum VacuumState {
    Vented, 
    Venting,
    Evacuated, 
    Pumping,
}

pub struct ValveController<I: PinId> {
    valve_pin: gpio::Pin<I, Output<PushPull>>,
    p_atm_threshold: f32,       // Pa
    p_vacuum_threshold: f32,    // Pa
    }

impl <I: PinId> ValveController<I> {
    pub fn new(
        valve_pin: gpio::Pin<I, Output<PushPull>>,
        p_atm_threshold: f32, 
        p_vacuum_threshold: f32 
    ) -> ValveController<I> {
        ValveController{
            valve_pin,
            p_atm_threshold,       // Pa
            p_vacuum_threshold,    // Pa
        }
    }

    pub fn set_valve_state(&mut self, state:ValveState){
        match state {
            ValveState::Vent => self.valve_pin.set_high().unwrap(),
            ValveState::Pump => self.valve_pin.set_low().unwrap(),
        } 
    }

    pub fn get_pressure_state(&self, p:f32) -> PressureState {
        if p> self.p_atm_threshold {return PressureState::Vented;}
        if p< self.p_vacuum_threshold {return PressureState::Evacuated;}
        PressureState::Intermediate
    }

}