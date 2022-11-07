use serde::{Serialize, Deserialize};
use defmt_rtt as _;

#[derive(Debug, Serialize, Clone, Copy, Deserialize, defmt::Format, PartialEq)]
pub enum MachineMode {
    mode_auto,
    mode_manual,
    mode_stopped,
}

pub struct MachineModeController {
    mode: MachineMode,
}

impl MachineModeController {
    // Initialize
    pub fn new() -> MachineModeController {
        MachineModeController {
            mode: MachineMode::mode_stopped,
        }
    }

    // Start recipe, go to auto mode
    pub fn start(&mut self) {
        if self.mode == MachineMode::mode_manual {
            //   TODO initi recipe here...
            self.mode = MachineMode::mode_auto;
        }  
    }

    // Stop recipe and go to manual mode
    pub fn stop(&mut self) {
        if self.mode == MachineMode::mode_auto {
            //   TODO half recipe here...
            self.mode = MachineMode::mode_manual;
        }  
    }

    // Power off
    pub fn off(&mut self) {
        if self.mode == MachineMode::mode_manual || self.mode == MachineMode::mode_auto {
            //   TODO ..
            self.mode = MachineMode::mode_stopped;
        }  
    }

    // Power on, go to manual mode.
    pub fn on(&mut self) {
        if self.mode == MachineMode::mode_stopped {
            //   TODO ..
            self.mode = MachineMode::mode_manual;
        }  
    }

    pub fn get_modes(self) -> MachineMode {
        self.mode
    }

}