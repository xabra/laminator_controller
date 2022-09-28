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
    const PWM_PERIOD_US: MicrosDurationU32 = MicrosDurationU32::micros(7800);

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        // Heater PWM stuff
        alarm1: hal::timer::Alarm1,     // HW IRQ 
        htr_ctr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::PushPullOutput>,
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

            // Grab our singleton objects

        // --- End of init boilerplate ---

        // Create new timer.  Provides four alarm interrupts (0-3)
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);

        // Create alarm0 dedicated to rtic monotonic.
        // Any number of RTIC scheduled software tasks will be handled by monotonic 
        let alarm0 = timer.alarm_0().unwrap();     

        // Create alarm1. When alarm1 triggers, it generates interrupt TIMER_IRQ_1
        // This interrupt will be bound to a hardware task that services the interrupt.
        let mut alarm1 = timer.alarm_1().unwrap();  
        
        // The delay object lets us wait for specified amounts of time (in milliseconds)
        //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());  
        
        // Grab a pin
        let htr_ctr_pin = pins.gpio15.into_push_pull_output();

        // Schedule the first HW interrupt task.
        let _ = alarm1.schedule(PWM_PERIOD_US);
        alarm1.enable_interrupt();

        // Init and return the Shared data structure
        (Shared { alarm1, htr_ctr_pin }, Local {}, init::Monotonics(Monotonic::new(timer, alarm0)))
    }

    // -- TASK: Hardware task coupled to Alarm1/TIMER_IRQ_1.  It starts the PWM cycle by:
    //     - Set the output pin high
    //     - Clear the interrupt for this task
    //     - Schedule a the next iteration of the PWM cycle (this task) based on PWM period
    //     - Schedule the output pin to be set low after a time corresponding to the PWM pulse width
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_1,  
        shared = [alarm1, htr_ctr_pin ],
        local = [toggle: bool = true, counter: u8 = 0],
    )]
    fn pwm_period_task (mut c: pwm_period_task::Context) { 

        //const TOP: u8 = 255;
        const TRIG: u8 = 25;

        *c.local.counter += 1;

        if *c.local.counter < TRIG {
            c.shared.htr_ctr_pin.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.htr_ctr_pin.lock(|l| l.set_low().unwrap());
        }

        let mut alarm = c.shared.alarm1;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(PWM_PERIOD_US);
        });
    }
}

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

