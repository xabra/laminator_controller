use crate::{thermocouple_controller::TCError, valve_controller::ValveState};
use serde::{Serialize, Deserialize};
// use serde_json_core;
use defmt_rtt as _;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Measurement {
    pub temp_ctr: f32,
    pub temp_lr: f32,
    pub temp_fb: f32, 
    pub temp_avg: f32,
    pub temp_err_ctr: TCError,
    pub temp_err_lr: TCError,
    pub temp_err_fb: TCError,  
    pub p_chamber: f32,
    pub p_bladder: f32,
    pub duty_factor_ctr: f32,
    pub duty_factor_lr: f32,
    pub duty_factor_fb: f32,

    // Setpoints
    pub temp_sp: f32,   // Current temp setpoint
    pub temp_trim_lr_sp: f32,
    pub temp_trim_fb_sp: f32,
    pub valve_state_chbr: ValveState,
    pub valve_state_bladder: ValveState,
    pub time_elapsed: u32, // Recipe elapsed time.

}
