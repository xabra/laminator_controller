use crate::{thermocouple_controller::TCError, valve_controller::ValveState, machine_mode::MachineMode};
use serde::{Serialize, Deserialize};
// use serde_json_core;
use defmt_rtt as _;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Measurement {
    pub tt_c: f32,
    pub tt_l: f32,
    pub tt_f: f32, 
    pub tt_avg: f32,
    pub tt_err_c: TCError,
    pub tt_err_l: TCError,
    pub tt_err_f: TCError,  
    pub p_ch: f32,
    pub p_bl: f32,
    pub df_c: f32,
    pub df_l: f32,
    pub df_f: f32,
    pub pwr: bool,
    pub t_ela: u32, // Recipe elapsed time.
    pub t_rcp: u32,
    pub seg: usize,
    pub mode: MachineMode,

    // Setpoints
    pub pwr_sp: bool,
    pub tt_sp: f32,   // Current temp setpoint
    pub tt_trim_l_sp: f32,
    pub tt_trim_f_sp: f32,
    pub vlv_ch: ValveState,
    pub vlv_bl: ValveState,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, defmt::Format)]
pub struct Command<'a> {
    pub cmd: &'a str,   
    pub value: &'a str,
}