#![no_std]
#![no_main]

use panic_halt as _;
use defmt_rtt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    // Unused IRQs in the dispatch list
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{MicrosDurationU32,MicrosDurationU64};
    use defmt::*;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::{monotonic::Monotonic, Alarm, Alarm0}, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    // PWM timebase
    const PWM_PERIOD: MicrosDurationU64  = MicrosDurationU64::millis(2000);
    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::secs(1);

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        //timer: hal::Timer,
        alarm1: hal::timer::Alarm1,     // HW IRQ 
        //alarm2: hal::timer::Alarm2,   // Unused for now
        //alarm3: hal::timer::Alarm3,   // Unused for now
        measure_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio8, hal::gpio::PushPullOutput>,
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
        // Create new timer and alarm
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let alarm0 = timer.alarm_0().unwrap();          // Alarm0 reserved for monotonic/ SW tasks
        let mut alarm1 = timer.alarm_1().unwrap();      // Use for HW IRQ Task
        //let mut _alarm2 = timer.alarm_2().unwrap();      // Unused for now
        //let mut _alarm3 = timer.alarm_3().unwrap();      // Unused for now

        let mut measure_pin = pins.gpio8.into_push_pull_output();
        measure_pin.set_low().unwrap();

        // Fast measurement HW interrupt
        let _ = alarm1.schedule(SCAN_TIME_US);
        alarm1.enable_interrupt();

        (Shared { alarm1, measure_pin}, Local {}, init::Monotonics(Monotonic::new(timer, alarm0)))
    }

    // -- TASK: High speed measurement & processing cycle start
    #[task(
        priority = 1, 
        binds = TIMER_IRQ_1,
        local = [tog: bool = true], 
        shared = [measure_pin, alarm1])]
    fn sample_timer(mut c: sample_timer::Context) { 
        if *c.local.tog {
            c.shared.measure_pin.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.measure_pin.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

        let mut alarm = c.shared.alarm1;
        (alarm).lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US);
        });

    }
}