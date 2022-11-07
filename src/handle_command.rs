use crate::{data_structs::{Command, Measurement}, valve_controller::ValveState};
use defmt_rtt as _;
use defmt::*;


pub fn handle_command(command: Command, m: &mut Measurement) {
    match command.cmd {
        "set_temp" => {
            let v = command.value.parse::<f32>().unwrap();
            info!("Setting temp to: {:?}", v);
        }
        "set_heater_trim_lr" => {
            let v = command.value.parse::<f32>().unwrap();
            info!("Setting heater lr trim to: {:?}", v);
        }
        "set_heater_trim_fb" => {
            let v = command.value.parse::<f32>().unwrap();
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
        "set_system_pwr" => {
            info!("Setting power: {:?}", command.value);
            match command.value {
                "on" => {
                    m.pwr_sp = true;
                    info!("Setting power: {:?}", m.pwr_sp);
                },
                "off" => {
                    m.pwr_sp = false;
                    info!("Setting power: {:?}", m.pwr_sp);
                },
                _ => {},
            }
        }

        // Default case
        _ => {info!("No command found");}
    }
}