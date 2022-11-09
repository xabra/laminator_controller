use crate::{data_structs::{Command, Measurement}, valve_controller::ValveState, recipe_manager::Recipe};
use crate::app::N_RECIPE_SETPOINTS;
use defmt_rtt as _;
use defmt::*;


pub fn handle_command(command: Command, m: &mut Measurement, r: &mut Recipe<N_RECIPE_SETPOINTS>) {
    match command.cmd {
        "set_temp" => {
            let v = command.value.parse::<f32>().unwrap();
            info!("Setting temp to: {:?}", v);
        }
        "set_heater_trim_lr" => {
            let v = command.value.parse::<f32>().unwrap();
            m.tt_trim_l_sp = v;
            info!("Setting heater lr trim to: {:?}", v);
        }
        "set_heater_trim_fb" => {
            let v = command.value.parse::<f32>().unwrap();
            m.tt_trim_f_sp = v;
            info!("Setting heater fb trim to: {:?}", v);
        }
        "set_valve_state_chbr" => {
            info!("Setting chamber valve: {:?}", command.value);
            match command.value {
                "Pump" => {m.vlv_ch = ValveState::Pump},
                "Vent" => {m.vlv_ch = ValveState::Vent},
                _ => {},
            }
        }
        "set_valve_state_bladder" => {
            info!("Setting bladder valve: {:?}", command.value);
        }
        "recipe" => {
            match command.value {
                "start" => {
                    if r.is_running() == false {
                        r.reset();
                        r.run(true);
                        info!("Recipe Start");
                    }
                },
                "stop" => {
                    if r.is_running() == true {
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
                    if m.pwr == false {
                        m.pwr = true;
                        // TODO Initialize everything here for full restart....
                        info!("Setting power: {:?}", m.pwr);
                    }
                },
                "off" => {
                    if m.pwr == true {
                        m.pwr = false;
                        // TODO clear everything neede here for safe 'shutdown'
                        info!("Setting power: {:?}", m.pwr);    
                    }
                },
                _ => {},
            }
        }

        // Default case
        _ => {info!("No command found");}
    }
}