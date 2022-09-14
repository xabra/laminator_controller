use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::Function;
use rp_pico::hal::pwm::{Slice, SliceId, FreeRunning, Channel, ChannelId, B};
use rp_pico::hal::gpio::{Pin, PinId, FunctionPwm, Pwm,Disabled, PullDown};

// A controller to encapsulate and manage a pwm heater channel
pub struct HeaterController<'a, S: SliceId, C: ChannelId, I: PinId> {
    pwm_slice: &'a mut Slice<S, FreeRunning>,
    pwm_channel: Channel<S, FreeRunning, C>,
    //pwm_pin: Pin<I, Disabled<PullDown>>,    // <===== IS THIS THE RIGHT PINMODE 'M' FOR PWM HERE? 
    pwm_pin: Pin<I, FunctionPwm>,    // <===== IS THIS THE RIGHT PINMODE 'M' FOR PWM HERE? 
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
