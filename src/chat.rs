// ----- THIS IS ESSENTIALLY THE REPO EXAMPLE CODE - IT WORKS PERFECTLY:
main() {

    // ... boilerplate here...

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Get a pwm slice (PWM7) from Slices and configure
    let pwm_slice = &mut pwm_slices.pwm7;    // <-- Hardcoded pwm7 slice   
    pwm_slice.set_ph_correct();
    pwm_slice.set_div_int(255u8); // To set integer part of clock divider
    pwm_slice.enable();

    //Get a channel (B) from the slice
    let channel = &mut pwm_slice.channel_b;      // <-- Hardcoded channel b 

    // Attach the channel outout to a pin (gpio15)
    let pin = pins.gpio15;      // <-- Hardcoded gpio pin 15 
    channel.output_to(pin);

    channel.set_duty(12000);
    // loop forever ...
}  // -------------------------------------------------------

// ------ THIS MY CODE
```
// A controller to encapsulate and manage a pwm heater
pub struct HeaterController<'a, S: SliceId, C: ChannelId, I: PinId> {
    pwm_slice: &'a mut Slice<S, FreeRunning>,
    pwm_channel: Channel<S, FreeRunning, C>,
    pwm_pin: Pin<I, Disabled<PullDown>>,    // <===== ? IS THIS THE RIGHT PINMODE 'M' FOR PWM HERE? 
}

impl<S: SliceId, C: ChannelId, I: PinId> HeaterController<'_, S, C, I> {  

    pub fn new(pwm_slice: &mut Slice<S,FreeRunning>, pwm_channel:Channel<S, FreeRunning, C>, pwm_pin:Pin<I, Disabled<PullDown>>) -> HeaterController<S, C, I> {
        HeaterController { 
            pwm_slice, 
            pwm_channel,
            pwm_pin,
        }
    }

    pub fn init(&mut self){
        self.pwm_slice.set_ph_correct();
        self.pwm_slice.set_div_int(255u8); // To set integer part of clock divider
        self.pwm_slice.enable();

        self.pwm_channel.output_to(self.pwm_pin);  // <- rustc says no method names output_to.  Why?
        self.pwm_channel.set_duty(0);               // <- rustc says no method names output_to.  Why?
    }

    pub fn set_duty(&mut self, df: i32){
        self.pwm_channel.set_duty(df);              // <- rustc says no method names output_to.  Why?
    }
}

main() {
    // ... boilerplate here...

    // ----------- SETUP PWM HEATER CONTROLLERS ------------
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Get individual pwm slices (PWM6, PWM7) from Slices
    let pwm6_slice = &mut pwm_slices.pwm6;
    let pwm7_slice = &mut pwm_slices.pwm7;

    // Create three pwm heater controlllers
    // ?? Here I pass a mut reference pwm7_slice to two different functions.  
    // Why no complaint from the borrow checker???
    let mut heater_ctr  = HeaterController::new(pwm7_slice, pwm7_slice.channel_b, pins.gpio15);
    let mut heater_fb  = HeaterController::new(pwm7_slice, pwm7_slice.channel_a, pins.gpio14);
    let mut heater_lr  = HeaterController::new(pwm6_slice, pwm6_slice.channel_b, pins.gpio13);

    heater_ctr.init();
    heater_fb.init();
    heater_lr.init();
    heater_ctr.set_duty(2000);
    heater_fb.set_duty(3000);
    heater_lr.set_duty(4000);

    // loop forever ...
}
```