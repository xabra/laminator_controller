 // pub struct Pwm {
    //     period: Duration<u32,1,1_000_000>,
    //     duty_factor: u8,
    //     enabled: bool,
    // }

    // impl Pwm {
    //     pub fn new(period:Duration<u32,1,1_000_000>) -> Pwm {
    //         Pwm {
    //             period,
    //             duty_factor: 0,
    //             enabled: false,
    //         }
    //     }

    //     pub fn set_duty_factor (&mut self, duty_factor: u8){

    //         self.duty_factor = duty_factor;
    //     }       

    //     pub fn set_enabled (&mut self, en: bool) {
    //         self.enabled = en;
    //     }

    //     pub fn get_pulse_width (&self) -> MicrosDurationU64{
    //         let pulse_width_microsec = (((self.period.to_micros() as u32)*(self.duty_factor as u32))/255_u32) as u64;
    //         //info!("get pulse width: {:?}", pulse_width_microsec);
    //         MicrosDurationU64::micros(pulse_width_microsec)
    //     }
    // }

