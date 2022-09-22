#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;


#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::MicrosDurationU32;
    use defmt::*;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::Alarm, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    const PWM_PERIOD_US: MicrosDurationU32 = MicrosDurationU32::millis(2000);

    #[shared]
    struct Shared {
        timer: hal::Timer,
        pwm_period_alarm: hal::timer::Alarm0,
        pwm1_alarm:hal::timer::Alarm1,
        pin0: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio0, hal::gpio::PushPullOutput>,
        pwm1_pulse_width: MicrosDurationU32,     // microseconds
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
        // Setup pins
        let mut pin0 = pins.gpio0.into_push_pull_output();
        pin0.set_low().unwrap();

        // Init the pwm pulse width
        let pwm1_pulse_width = MicrosDurationU32::millis(500);

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);

        let mut pwm_period_alarm = timer.alarm_0().unwrap();
        let mut pwm1_alarm = timer.alarm_1().unwrap();

        let _ = pwm_period_alarm.schedule(PWM_PERIOD_US);
        let _ = pwm1_alarm.schedule(pwm1_pulse_width);

        pwm_period_alarm.enable_interrupt();
        pwm1_alarm.enable_interrupt();

        info!("RTIC Init Complete");

        (Shared { timer, pwm_period_alarm, pwm1_alarm, pwm1_pulse_width, pin0}, Local {}, init::Monotonics())
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 2,
        shared = [timer, pwm_period_alarm, pwm1_alarm, pwm1_pulse_width, pin0],
    )]
    fn pwm_period_timer_irq(mut c: pwm_period_timer_irq::Context) { 

        // Get both alarm objects
        let mut pwm_period_alarm = c.shared.pwm_period_alarm;
        let pwm1_alarm = c.shared.pwm1_alarm;
        let pw = c.shared.pwm1_pulse_width;

        // Retrigger the period alarm
        (pwm_period_alarm).lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(PWM_PERIOD_US);
        });

        // Retrigger pwm1 pulsewidth alarm
        (pwm1_alarm, pw).lock(|a, pw| {
            a.clear_interrupt();
            let _ = a.schedule(*pw);
        });

        // Set pwm1 pin high
        c.shared.pin0.lock(|l| l.set_high().unwrap());     
        
    }

    #[task(
        binds = TIMER_IRQ_1,
        priority = 1,
        shared = [timer, pwm1_alarm, pin0],
    )]
    fn pwm1_pulse_timer_irq(mut c: pwm1_pulse_timer_irq::Context) {

        let mut pwm1_alarm = c.shared.pwm1_alarm;

        // Clear the interrupt
        (pwm1_alarm).lock(|a| {
            a.clear_interrupt();
        });

        // Set pwm1 pin low
        c.shared.pin0.lock(|l| l.set_low().unwrap());
        
    }
}