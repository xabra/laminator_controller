#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;

/*
This is an RTIC example for the Raspberry Pi Pico or other RP2040 based boards.
It shows the simultaneous use of hardware interrupt-based tasks 
and scheduled software tasks.  It implements a simple pulse-width modulation (PWM)
output on a gpio pin.  The hardware task is triggered by Alarm1/TIMER_IRQ_1.  It controls
the overall period of the PWM.  The width of the pulse is controlled by the software task which
is spawned at the start of each PWM period by the hardware task.

Scheduled software tasks in RTIC - those that are scheduled to be spawned at some point in the future -
require their own monotonic timebase which in this case is bound to Alarm0/TIMER_IRQ_0.

Thus, is it possible to have up to three precision hardware timer-based interrupts and unlimited
scheduled software tasks using the pico/rp2040 timer peripheral.

In this example, the PWM period is a const.  The duty factor (period) is a shared variable which
could be driven from another section of code.

! Note that no checking is done to insure that duty period < PWM_PERIOD.
*/

// Place unused IRQs in the dispatchers list.  You need one IRQ for each priority level in your code
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{MicrosDurationU32, MicrosDurationU64};
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
        alarm1: hal::timer::Alarm1,     // HW IRQ 
        output_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio15, hal::gpio::PushPullOutput>,
        duty_period: MicrosDurationU64, // Time duration that output pin is high
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

        // Create a pin to observe output.  Could be any gpio or the led
        let mut output_pin = pins.gpio15.into_push_pull_output();
        output_pin.set_low().unwrap();

        //  Set the duty factor (actually the time) to output is high
        // Unlike hardware tasks, software tasks accept any u64 backed duration.
        let duty_period = MicrosDurationU64::millis(1);

        // Schedule the first HW interrupt task.
        let _ = alarm1.schedule(PWM_PERIOD_US);
        alarm1.enable_interrupt();

        // Init and return the Shared data structure
        (Shared { alarm1, output_pin, duty_period}, Local {}, init::Monotonics(Monotonic::new(timer, alarm0)))
    }

    // -- TASK: Hardware task coupled to Alarm1/TIMER_IRQ_1.  It starts the PWM cycle by:
    //     - Set the output pin high
    //     - Clear the interrupt for this task
    //     - Schedule a the next iteration of the PWM cycle (this task) based on PWM period
    //     - Schedule the output pin to be set low after a time corresponding to the PWM pulse width
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_1,  
        shared = [output_pin, alarm1, duty_period])]
    fn pwm_period_task (c: pwm_period_task::Context) { 
        let pin = c.shared.output_pin;
        let alarm = c.shared.alarm1;
        let dp = c.shared.duty_period;
        (alarm, pin, dp).lock(|a, p, dp| {
            a.clear_interrupt();
            p.set_high().unwrap();
            let _ = a.schedule(PWM_PERIOD_US);
            end_pulse_task::spawn_after(*dp).unwrap();
        });
        
    }

    // -- TASK: Software task to set the pin low at end of duty cycle
    #[task(
        priority = 1,   
        shared = [output_pin])]
    fn end_pulse_task(c: end_pulse_task::Context) { 
        let mut pin = c.shared.output_pin;
        (pin).lock(|p| {
            p.set_low().unwrap();
        });
    }


}