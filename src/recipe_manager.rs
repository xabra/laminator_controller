
use VacuumSetpoint::{Evacuated, Vented};
use defmt::info;

pub enum VacuumSetpoint {
    Evacuated,
    Vented,
}

pub struct SetPoint {
    pub t_sec: u32,       // Elapsed time
    pub sp_temp: f32,     // setpoint temp
    pub sp_chbr_pressure: VacuumSetpoint,
    pub sp_bladder_pressure: VacuumSetpoint,
}


pub struct Recipe<'a> {
    pub set_point_array: &'a [SetPoint],
}

impl<'a> Recipe<'a> {
    pub fn new(set_point_array: &'a[SetPoint]) -> Self {
        Self {
            set_point_array,
        }
    }

    pub fn list_setpoints(&self) {
        for (i, sp)  in self.set_point_array.iter().enumerate() {
            info!("SetPoint {:?}, sp_temp = {:?}", i, sp.sp_temp);
        }
    }
}