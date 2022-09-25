#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    // Unused IRQs in the dispatch list
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use rp2040_monotonic::*;
    use fugit::{MicrosDurationU64};
    use defmt::*;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    // PWM timebase
    const PWM_PERIOD: MicrosDurationU64  = MicrosDurationU64::millis(2000);

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        master_cycle_trigger_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio8, hal::gpio::PushPullOutput>,  // Debug only.
        htr_ctr_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::PushPullOutput>,
        htr_ctr_pulse_width: MicrosDurationU64,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
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
        // Create new monotonic
        let mono = Rp2040Monotonic::new(c.device.TIMER);


        // Heater PWMs setup
        let mut htr_ctr_pin = pins.gpio15.into_push_pull_output();
        htr_ctr_pin.set_low().unwrap();
  
        // Init the pwm pulse width
        let htr_ctr_pulse_width = MicrosDurationU64::millis(200);

        let mut master_cycle_trigger_pin = pins.gpio8.into_push_pull_output();
        master_cycle_trigger_pin.set_low().unwrap();

        pwm_period_timer::spawn().unwrap();
        sample_timer::spawn().unwrap();

        (Shared { htr_ctr_pulse_width, htr_ctr_pin, master_cycle_trigger_pin}, Local {}, init::Monotonics(mono))
    }

    // -- TASK: High speed measurement & processing cycle start
    #[task(priority = 3, shared = [master_cycle_trigger_pin])]
    fn sample_timer(mut c: sample_timer::Context) { 

        c.shared.master_cycle_trigger_pin.lock(|l| l.set_high().unwrap()); 
        sample_timer::spawn_after(1.millis()).unwrap();
        c.shared.master_cycle_trigger_pin.lock(|l| l.set_low().unwrap()); 
    }

    // -- TASK: Low speed PWM period timebase
    #[task(priority = 1, shared = [htr_ctr_pulse_width, htr_ctr_pin])]
    fn pwm_period_timer(mut c: pwm_period_timer::Context) { 

        // Schedule a restart of this task first 
        pwm_period_timer::spawn_after(PWM_PERIOD).unwrap();
        
        // Get convenient pwm output pin and pulse width variables
        let ctr  = c.shared.htr_ctr_pin;
        let pw = c.shared.htr_ctr_pulse_width;

        // Lock the shared values and do critical stuff in closure.
        // Set pins high to start duty factor pulse and schedule a task to end the duty-factor pulse
        (ctr, pw).lock(|ctr, pw| 
            {ctr.set_high().unwrap();
            pwm1_set_low::spawn_after(*pw).unwrap();}
        ); 
    }

    // -- TASK: End the PWM duty-factor pulse
    #[task(priority = 1, shared = [htr_ctr_pin])]
    fn pwm1_set_low(mut c: pwm1_set_low::Context) {
        c.shared.htr_ctr_pin.lock(|l| l.set_low().unwrap());
    }
}