use crate::{data_structs::{Command, Measurement, UiInputs}, valve_controller::ValveState, recipe_manager::Recipe};
use crate::app::N_RECIPE_SETPOINTS;
use defmt_rtt as _;
use defmt::*;


pub fn handle_command(command: Command, m: &mut Measurement, u: &mut UiInputs, r: &mut Recipe<N_RECIPE_SETPOINTS>) {
    match command.cmd {
        // --- TEMP SETPOINT
        "set_temp" => {
            let v = command.value.parse::<f32>().unwrap();
            u.tt_sp_in = v;
            info!("Setting temp to: {:?}", v);    
        }
        // --- HEATER TRIM SETPOINTS
        "set_heater_trim_lr" => {
            // This input works even if a recipe is running
            let v = command.value.parse::<f32>().unwrap();
            u.tt_trim_l_sp = v;
            info!("Setting heater lr trim to: {:?}", v);
        }
        "set_heater_trim_fb" => {
            // This input works even if a recipe is running
            let v = command.value.parse::<f32>().unwrap();
            u.tt_trim_f_sp = v;
            info!("Setting heater fb trim to: {:?}", v);
        }

        // --- VALVE STATE INPUTS
        "set_valve_state_chbr" => {
            info!("Setting chamber valve: {:?}", command.value);
            match command.value {
                "Pump" => {u.vlv_ch_in = ValveState::Pump},
                "Vent" => {u.vlv_ch_in = ValveState::Vent},
                _ => {},
            }
        }
        "set_valve_state_bladder" => {
            if !r.is_running() {     // Only works if NOT running a recipe
                info!("Setting bladder valve: {:?}", command.value);
                match command.value {
                    "Pump" => {u.vlv_bl_in = ValveState::Pump},
                    "Vent" => {u.vlv_bl_in = ValveState::Vent},
                    _ => {},
                }    
            }
        }

        // --- START/STOP RECIPE 
        "recipe" => {
            match command.value {
                "start" => {
                    if !r.is_running() {
                        r.reset();
                        r.run(true);
                        info!("Recipe Start");
                    }
                },
                "stop" => {
                    if r.is_running() {
                        r.run(false);
                        info!("Recipe Stop");
                    }
                },
                _ => {},
            }
            
        }

        // --- POWER ON/OFF
        "set_system_pwr" => {
            match command.value {
                "on" => {
                        u.pwr_in = true;
                        // TODO Initialize everything here for full restart....
                        info!("Setting power: {:?}", m.pwr);
                },
                "off" => {
                        u.pwr_in = false;
                        // TODO clear everything neede here for safe 'shutdown'
                        info!("Setting power: {:?}", m.pwr);    
                },
                _ => {},
            }
        }

        // --- Default case
        _ => {info!("No command found");}
    }
}