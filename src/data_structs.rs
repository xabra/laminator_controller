use crate::thermocouple_controller::TCError;

use defmt_rtt as _;

#[derive(Debug, Copy, Clone)]
pub struct Measurement {
    pub temp_ctr: f32,
    pub temp_lr: f32,
    pub temp_fb: f32, 
    pub temp_err_ctr: TCError,
    pub temp_err_lr: TCError,
    pub temp_err_fb: TCError,  
    pub p_chamber: f32,
    pub p_bladder: f32,
}
