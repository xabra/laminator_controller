use serde::{Serialize, Deserialize};
use defmt_rtt as _;

#[derive(Debug, Serialize, Clone, Copy, Deserialize, defmt::Format, PartialEq)]
pub enum MachineMode {
    Auto,
    Manual,
    Stopped,
}

pub struct MachineModeController {
    mode: MachineMode,
}

impl MachineModeController {
    // Initialize
    pub fn new() -> MachineModeController {
        MachineModeController {
            mode: MachineMode::Stopped,
        }
    }

    // Start recipe, go to auto mode
    pub fn start(&mut self) {
        if self.mode == MachineMode::Manual {
            //   TODO initi recipe here...
            self.mode = MachineMode::Auto;
        }  
    }

    // Stop recipe and go to manual mode
    pub fn stop(&mut self) {
        if self.mode == MachineMode::Auto {
            //   TODO half recipe here...
            self.mode = MachineMode::Manual;
        }  
    }

    // Power off
    pub fn off(&mut self) {
        if self.mode == MachineMode::Manual || self.mode == MachineMode::Auto {
            //   TODO ..
            self.mode = MachineMode::Stopped;
        }  
    }

    // Power on, go to manual mode.
    pub fn on(&mut self) {
        if self.mode == MachineMode::Stopped {
            //   TODO ..
            self.mode = MachineMode::Manual;
        }  
    }

    pub fn get_mode(self) -> MachineMode {
        self.mode
    }

}