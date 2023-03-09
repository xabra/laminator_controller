
// This object probably needs to know about a pressure sensor or pressure measurement
// and a valve.

use serde::{Serialize, Deserialize};

// Same constants are in the pressure sensor controller ?!
const P_ATM: f32 = 0.0;            // Pa.  Atmospheric pressure
const P_VAC: f32 = -101_000.0;     // Pa.  Full vacuum pressure

#[derive(Debug, Copy, Clone, defmt::Format, Serialize, Deserialize)]
pub enum PressureState {
    Vented,
    Intermediate,
    Evacuated,
}
#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum ChamberState {
    Vented,         // Valve set to vent, pressure above p_atm_threshold
    Venting,        // Valve set to vent, pressure below p_atm_threshold
    Evacuated,      // Valve set to pump, pressure above p_vacuum_threshold
    Pumping,        // Valve set to pump, pressure above p_vacuum_threshold
}

pub struct ChamberController {
    p_atm_threshold: f32,       // Pa
    p_vacuum_threshold: f32,    // Pa
    }

impl ChamberController {
    pub fn new(
        p_atm_threshold: f32, 
        p_vacuum_threshold: f32,
    ) -> ChamberController {
        ChamberController{
            p_atm_threshold,       // Pa
            p_vacuum_threshold,    // Pa
        }
    }

    pub fn set_atm_threshold(&mut self, p_offset:f32) {
        self.p_atm_threshold = P_ATM - p_offset;
    }

    pub fn set_vacuum_threshold(&mut self, p_offset:f32){
        self.p_vacuum_threshold = P_VAC + p_offset;
    }

    pub fn get_pressure_state(&self, p:f32) -> PressureState {
        if p> self.p_atm_threshold {return PressureState::Vented;}
        if p< self.p_vacuum_threshold {return PressureState::Evacuated;}
        PressureState::Intermediate
    }
}