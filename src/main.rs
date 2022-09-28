#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;

/*
This is 
*/

// Place unused IRQs in the dispatchers list.  You need one IRQ for each priority level in your code
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{Duration, MicrosDurationU32, MicrosDurationU64};
    use defmt::*;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::{monotonic::Monotonic, Alarm, Alarm0}, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    // PWM cycle period. Hardware tasks (alarms) are scheduled using time represented by a MicrosDurationU32
    // That is, it must be a Duration<u32, 1, 1000000>:  a u32 representing time in MICROSECONDS.
    const PWM_PERIOD_US: MicrosDurationU32 = MicrosDurationU32::millis(1000);

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        // Heater PWM stuff
        alarm1: hal::timer::Alarm1,     // HW IRQ 
        htr_ctr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::PushPullOutput>,
        htr_ctr_pwm: Pwm,
        htr_lr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio13, hal::gpio::PushPullOutput>,
        htr_lr_pwm: Pwm,
        htr_fb_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio14, hal::gpio::PushPullOutput>,
        htr_fb_pwm: Pwm,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // --- Init boilerplate ---
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        // --- End of init boilerplate ---

        // Create new timer.  Provides four alarm interrupts (0-3)
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);

        // Create alarm0 dedicated to rtic monotonic.
        // Any number of RTIC scheduled software tasks will be handled by monotonic 
        let alarm0 = timer.alarm_0().unwrap();     

        // Create alarm1. When alarm1 triggers, it generates interrupt TIMER_IRQ_1
        // This interrupt will be bound to a hardware task that services the interrupt.
        let mut alarm1 = timer.alarm_1().unwrap();      

        // Create a CENTER PWM, set its duty factor and enable it.
        let mut htr_ctr_pwm = Pwm::new(PWM_PERIOD_US);
        htr_ctr_pwm.set_duty_factor(127);
        let mut htr_ctr_pin = pins.gpio15.into_push_pull_output();
        htr_ctr_pin.set_low().unwrap();
        htr_ctr_pwm.set_enabled(true);

        // Create a LR PWM, set its duty factor and enable it.
        let mut htr_lr_pwm = Pwm::new(PWM_PERIOD_US);
        htr_lr_pwm.set_duty_factor(1);
        let mut htr_lr_pin = pins.gpio13.into_push_pull_output();
        htr_lr_pin.set_low().unwrap();
        htr_lr_pwm.set_enabled(true);

        // Create a FB PWM, set its duty factor and enable it.
        let mut htr_fb_pwm = Pwm::new(PWM_PERIOD_US);
        htr_fb_pwm.set_duty_factor(254);
        let mut htr_fb_pin = pins.gpio14.into_push_pull_output();
        htr_fb_pin.set_low().unwrap();
        htr_fb_pwm.set_enabled(true);

        // Schedule the first HW interrupt task.
        let _ = alarm1.schedule(PWM_PERIOD_US);
        alarm1.enable_interrupt();

        // Init and return the Shared data structure
        (Shared { alarm1, htr_ctr_pin, htr_ctr_pwm, htr_lr_pin, htr_lr_pwm, htr_fb_pin, htr_fb_pwm}, Local {}, init::Monotonics(Monotonic::new(timer, alarm0)))
    }

    // -- TASK: Hardware task coupled to Alarm1/TIMER_IRQ_1.  It starts the PWM cycle by:
    //     - Set the output pin high
    //     - Clear the interrupt for this task
    //     - Schedule a the next iteration of the PWM cycle (this task) based on PWM period
    //     - Schedule the output pin to be set low after a time corresponding to the PWM pulse width
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_1,  
        shared = [alarm1, htr_ctr_pin, htr_ctr_pwm, htr_lr_pin, htr_lr_pwm, htr_fb_pin, htr_fb_pwm])]
    fn pwm_period_task (c: pwm_period_task::Context) { 
        let alarm = c.shared.alarm1;
        let ctr_pin = c.shared.htr_ctr_pin;
        let lr_pin = c.shared.htr_lr_pin;
        let fb_pin = c.shared.htr_fb_pin;
        
        let ctr_pwm = c.shared.htr_ctr_pwm;
        let lr_pwm = c.shared.htr_lr_pwm;
        let fb_pwm = c.shared.htr_fb_pwm;
        (alarm, ctr_pin, lr_pin, fb_pin, ctr_pwm, lr_pwm, fb_pwm).lock(|a, ctp, lrp, fbp, ctpwm, lrpwm, fbpwm| {
            a.clear_interrupt();
            ctp.set_high().unwrap();
            lrp.set_high().unwrap();
            fbp.set_high().unwrap();
            let _ = a.schedule(PWM_PERIOD_US);
            end_ctr_pulse_task::spawn_after(ctpwm.get_pulse_width()).unwrap();
            end_lr_pulse_task::spawn_after(lrpwm.get_pulse_width()).unwrap();
            end_fb_pulse_task::spawn_after(fbpwm.get_pulse_width()).unwrap();
        });
        
    }

    // -- TASK: Software task to set the pin low at end of duty cycle
    #[task(
        priority = 1,   
        shared = [htr_ctr_pin])]
    fn end_ctr_pulse_task(c: end_ctr_pulse_task::Context) { 
        let mut pin = c.shared.htr_ctr_pin;
        (pin).lock(|p| {
            p.set_low().unwrap();
        });
    }

    // -- TASK: Software task to set the pin low at end of duty cycle
    #[task(
        priority = 1,   
        shared = [htr_lr_pin])]
    fn end_lr_pulse_task(c: end_lr_pulse_task::Context) { 
        let mut pin = c.shared.htr_lr_pin;
        (pin).lock(|p| {
            p.set_low().unwrap();
        });
    }

    // -- TASK: Software task to set the pin low at end of duty cycle
    #[task(
        priority = 1,   
        shared = [htr_fb_pin])]
    fn end_fb_pulse_task(c: end_fb_pulse_task::Context) { 
        let mut pin = c.shared.htr_fb_pin;
        (pin).lock(|p| {
            p.set_low().unwrap();
        });
    }
    pub struct Pwm {
        period: Duration<u32,1,1_000_000>,
        duty_factor: u8,
        enabled: bool,
    }

    impl Pwm {
        pub fn new(period:Duration<u32,1,1_000_000>) -> Pwm {
            Pwm {
                period,
                duty_factor: 0,
                enabled: false,
            }
        }

        pub fn set_duty_factor (&mut self, duty_factor: u8){

            self.duty_factor = duty_factor;
        }       

        pub fn set_enabled (&mut self, en: bool) {
            self.enabled = en;
        }

        pub fn get_pulse_width (&self) -> MicrosDurationU64{
            let pulse_width_microsec = (((self.period.to_micros() as u32)*(self.duty_factor as u32))/255_u32) as u64;
            //info!("get pulse width: {:?}", pulse_width_microsec);
            MicrosDurationU64::micros(pulse_width_microsec)
        }
    }

}