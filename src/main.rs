#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;

/*
This is RTIC version
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

    // PWM cycle period.
    const PWM_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(4167);
    // PWM Period = TOP*PWM_TICK_US.   Max value for duty-factor:  [0-TOP].  0= off, TOP = 100% on
    const TOP: u16 = 480;  // Ticks per period

    // Sample period for Alarm2.
    const SAMPLE_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(2000);

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        // Heater PWM stuff
        alarm1: hal::timer::Alarm1,     // For PWM tick
        
        htr_ctr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::PushPullOutput>,
        htr_ctr_df: u16,
        htr_fb_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio14, hal::gpio::PushPullOutput>,
        htr_fb_df: u16,
        htr_lr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio13, hal::gpio::PushPullOutput>,
        htr_lr_df: u16,

        // Sample measurement
        alarm2: hal::timer::Alarm2,     // For sample tick
        debug_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio8, hal::gpio::PushPullOutput>,
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
        let mut alarm1 = timer.alarm_1().unwrap();   

        // Create alarm2. When alarm1 triggers, it generates interrupt TIMER_IRQ_2
        let mut alarm2 = timer.alarm_2().unwrap();   
        
        // ----- Set up heater PWMs -----
        let htr_ctr_pin = pins.gpio15.into_push_pull_output();
        let htr_ctr_df = 240;
        let htr_fb_pin = pins.gpio14.into_push_pull_output();
        let htr_fb_df = 48;
        let htr_lr_pin = pins.gpio13.into_push_pull_output();
        let htr_lr_df = 480-48;

        // Schedule the first HW interrupt task.
        let _ = alarm1.schedule(PWM_TICK_US);
        alarm1.enable_interrupt();

        // Schedule the first HW interrupt task.
        let _ = alarm2.schedule(SAMPLE_TICK_US);
        alarm2.enable_interrupt();

        let debug_pin = pins.gpio8.into_push_pull_output();
        // Init and return the Shared data structure
        (Shared { 
            alarm1, htr_ctr_pin, htr_ctr_df, htr_fb_pin, htr_fb_df, htr_lr_pin, htr_lr_df,
            alarm2, debug_pin,
            }, 
        Local {}, 
        init::Monotonics(Monotonic::new(timer, alarm0))
        )
    }

    // -- TASK: Hardware task coupled to Alarm1/TIMER_IRQ_1.
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_1,  
        shared = [alarm1, htr_ctr_pin, htr_ctr_df, htr_fb_pin, htr_fb_df, htr_lr_pin, htr_lr_df ],
        local = [counter: u16 = 0],
    )]
    fn pwm_period_task (c: pwm_period_task::Context) { 

        let mut alarm = c.shared.alarm1;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(PWM_TICK_US);
        });

        // Increment and wrap counter if necesary
        *c.local.counter += 1;
        if *c.local.counter == TOP {*c.local.counter =0;}

        // Center heater pwm
        (c.shared.htr_ctr_df,c.shared.htr_ctr_pin).lock(|d,p| {
            if *c.local.counter < *d { p.set_high().unwrap(); } 
            else { p.set_low().unwrap(); }    
        });

        // Front-Back heater pwm
        (c.shared.htr_fb_df,c.shared.htr_fb_pin).lock(|d,p| {
            if *c.local.counter < *d { p.set_high().unwrap(); } 
            else { p.set_low().unwrap(); }    
        });

        // Left-Right heater pwm
        (c.shared.htr_lr_df,c.shared.htr_lr_pin).lock(|d,p| {
            if *c.local.counter < *d { p.set_high().unwrap(); } 
            else { p.set_low().unwrap(); }    
        });

    }

    // -- TASK: Hardware task coupled to Alarm2/TIMER_IRQ_2.
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_2,  
        shared = [alarm2, debug_pin],
        local = [toggle: bool = true],
    )]
    fn sample_period_task (mut c: sample_period_task::Context) { 
        if *c.local.toggle {
            c.shared.debug_pin.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.debug_pin.lock(|l| l.set_low().unwrap());
        }
        *c.local.toggle = !*c.local.toggle;


        let mut alarm = c.shared.alarm2;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(SAMPLE_TICK_US);
        });
    }
}
