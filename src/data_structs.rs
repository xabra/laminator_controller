use crate::{thermocouple_controller::TCError, valve_controller::ValveState};
use serde::{Serialize, Deserialize};
// use serde_json_core;
use defmt_rtt as _;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Measurement {
    // Measurements, PVs, internal state
    // Sent to UI Pico.
    pub tt_c: f32,
    pub tt_l: f32,
    pub tt_f: f32, 
    pub tt_avg: f32,
    pub tt_sp: f32,
    pub tt_delta: f32,
    pub tt_err_c: TCError,
    pub tt_err_l: TCError,
    pub tt_err_f: TCError,  
    pub p_ch: f32,
    pub p_bl: f32,
    pub df_c: f32,
    pub df_l: f32,
    pub df_f: f32,
    pub vlv_ch: ValveState, // From recipe if running, otherwise input from UI
    pub vlv_bl: ValveState, // From recipe if running, otherwise input from UI
    pub t_ela: u32,     // Owned by measurement struct.
    pub t_rcp: u32,     // Owned by recipe mgr. Recipe elapsed time.
    pub seg: usize,     // Owned by recipe mgr.
    pub isrun: bool,    // Owned by recipe mgr
    pub pwr: bool,      // Input from UI

    // User input requested-- not needed in the struct- not need to send to UI
    pub tt_sp_in: f32,   // Input from UI
    pub tt_trim_l_sp: f32,  // Input from UI
    pub tt_trim_f_sp: f32,  // Input from UI
    pub vlv_ch_in: ValveState,
    pub vlv_bl_in: ValveState,
}


#[derive(Debug, Copy, Clone, Serialize, Deserialize, defmt::Format)]
pub struct Command<'a> {
    pub cmd: &'a str,   
    pub value: &'a str,
}