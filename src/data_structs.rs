use crate::{thermocouple_controller::TCError, valve_controller::ValveState, chamber_controller::PressureState};
use serde::{Serialize, Deserialize};
// use serde_json_core;
use defmt_rtt as _;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Measurement {
    // Pure OUTPUTS to UI .
    pub tc: f32,    // temp center
    pub tl: f32,    // temp left
    pub tf: f32,    // temp front
    pub tav: f32,   // temp avg
    pub tsp: f32,   // temp set point
    pub td: f32,    // temp delta (pid error)
    pub terc: TCError,  // TC errors
    pub terl: TCError,
    pub terf: TCError,  
    pub pch: f32,   // Pressure chamber
    pub pbl: f32,   // pressure bladder
    pub psch: PressureState,    // pressure state chamber
    pub psbl: PressureState,    // pressure state bladder
    pub dfc: f32,   // Duty factors
    pub dfl: f32,
    pub dff: f32,
    pub vch: ValveState, // From recipe if running, otherwise input from UI
    pub vbl: ValveState, // From recipe if running, otherwise input from UI
    pub te: u32,     // Time elapsed.  Owned by measurement struct.
    pub tr: u32,     // Time recipe.  Owned by recipe mgr. Recipe elapsed time.
    pub seg: usize,     // Current recipe segment. Owned by recipe mgr.
    pub isrun: bool,    // Is recipe currently running.  Owned by recipe mgr
    pub pwr: bool,      // Is 'power' on.  Unused?  State
    pub trl: f32,    // Heater trim factors
    pub trf: f32,
    pub cchm: f32,   // Pressure cal factors
    pub cch0: f32,
    pub cblm: f32,
    pub cbl0: f32,


}
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct UiInputs {
    // Pure user INPUTS from UI
    pub pwr_in: bool,          // Input from UI
    pub tt_sp_in: f32,         // Input from UI
    pub tt_trim_l_sp: f32,     // Input from UI
    pub tt_trim_f_sp: f32,     // Input from UI
    pub vlv_ch_in: ValveState, // Input from UI
    pub vlv_bl_in: ValveState, // Input from UI
    pub ch_vnt_th: f32, // Pressure state thresholds
    pub ch_vac_th: f32,
    pub bl_vnt_th: f32,
    pub bl_vac_th: f32,
    pub ch_cal_vnt: f32,    // Pressure cal factors
    pub ch_cal_vac: f32,
    pub bl_cal_vnt: f32,
    pub bl_cal_vac: f32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, defmt::Format)]
pub struct Command<'a> {
    pub cmd: &'a str,   
    pub value: &'a str,
}
