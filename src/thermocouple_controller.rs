use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio::bank0::Gpio17;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::{PinId, Pin, Output, PushPull};
use rp_pico::Pins;



pub struct ThermocoupleController< {
    tc_ctr_select_n: gpio::Pin<Gpio17, Output<PushPull>>,
    //tc_lr_select_n: gpio::Pin<PinId, Output<PushPull>>,
    //tc_fb_select_n: gpio::Pin<PinId, Output<PushPull>>, 
}

impl ThermocoupleController {
    pub fn new(pins: Pins) -> ThermocoupleController {
        let tc_ctr_select_n = pins.gpio17.into_push_pull_output();
        ThermocoupleController{
            tc_ctr_select_n,
        }
    }
    // pub fn init(&mut self)  {
    //     self.set_state(ValveState::Pump);
    // }
    // pub fn set_state(&mut self, state:ValveState){
    //     match state {
    //         ValveState::Vent => self.valve_pin.set_high().unwrap(),
    //         ValveState::Pump => self.valve_pin.set_low().unwrap(),
    //     } 
    // }
}