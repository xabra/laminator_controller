#![no_std]
#![no_main]

pub mod pwm_controller;
pub mod valve_controller;
pub mod thermocouple_controller;
pub mod pressure_sensor_controller;

use panic_halt as _;
use defmt_rtt as _;

/*
This is RTIC version
*/

// Place unused IRQs in the dispatchers list.  You need one IRQ for each priority level in your code
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{MicrosDurationU32, MicrosDurationU64 ,RateExtU32};
    use defmt::*;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::{monotonic::Monotonic, Alarm, Alarm0}, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };
    use rp_pico::hal::gpio::pin::bank0::{Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio17, Gpio19, Gpio20};
    use rp_pico::hal::spi;
    use rp_pico::hal::pac::SPI0;
    use rp_pico::hal::pac::I2C0;
    use rp_pico::hal::I2C;
    use rp_pico::hal::gpio::Pin;
    use rp_pico::hal::gpio::Function;

    use crate::pwm_controller::PWM;
    use crate::valve_controller::{ValveController, ValveState};
    use crate::thermocouple_controller::{ThermocoupleController, TCChannel};
    use crate::pressure_sensor_controller::PressureSensorController;


    // PWM cycle period.
    const PWM_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(4167);
    // PWM Period = TOP*PWM_TICK_US.   Max value for duty-factor:  [0-TOP].  0= off, TOP = 100% on
    const TOP: u16 = 480;  // Ticks per period

    // Sample period for Alarm2.
    const SAMPLE_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(2_000_000);

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        // Heater PWM stuff
        alarm1: hal::timer::Alarm1,     // For PWM tick
        
        // Sample measurement
        alarm2: hal::timer::Alarm2,     // For sample tick
        debug_pin: hal::gpio::Pin<Gpio8, hal::gpio::PushPullOutput>,

        // Heater PWMs
        pwm_ctr: PWM<Gpio15>,
        pwm_lr: PWM<Gpio13>,
        pwm_fb: PWM<Gpio14>,

        // Valves
        main_chamber_valve: ValveController<Gpio12>,
        bladder_valve: ValveController<Gpio11>,

        // Thermocouple
        tc_controller: ThermocoupleController<Gpio17, Gpio19, Gpio20, SPI0>,

        // Pressure Sensor Controller
        pressure_sensor_controller: PressureSensorController<Gpio6, Gpio7, I2C<I2C0, (Pin<Gpio4, Function<I2C>>, Pin<Gpio5, Function<I2C>>)>>,
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
        let clocks = init_clocks_and_plls(
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
        
        // ----------- HEATER PWM CONTROLLER SETUP ------------
        let mut pwm_ctr = PWM::new(pins.gpio15.into_push_pull_output());
        pwm_ctr.set_duty_factor(0.5, TOP);

        let mut pwm_lr = PWM::new(pins.gpio13.into_push_pull_output());
        pwm_lr.set_duty_factor(0.1, TOP);

        let mut pwm_fb = PWM::new(pins.gpio14.into_push_pull_output());
        pwm_fb.set_duty_factor(0.9, TOP);

        // ----------- VALVE CONTROLLER SETUP ------------
        // Create the valve controllers and initialize them. Need to check logic polarity
        let mut main_chamber_valve = ValveController::new(pins.gpio12.into_push_pull_output());
        main_chamber_valve.set_state(ValveState::Pump);
        let mut bladder_valve = ValveController::new(pins.gpio11.into_push_pull_output());
        bladder_valve.set_state(ValveState::Pump);

        // ------------- THERMOCOUPLE CONTROLLER ---------
        // Chip select pins
        let cs_ctr = pins.gpio17.into_push_pull_output();
        let cs_lr = pins.gpio19.into_push_pull_output();
        let cs_fb = pins.gpio20.into_push_pull_output();

        // Set up SPI CLK and DataIn Lines.  These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio18.into_mode::<rp2040_hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio3.into_mode::<rp2040_hal::gpio::FunctionSpi>();
        let _spi_miso = pins.gpio16.into_mode::<rp2040_hal::gpio::FunctionSpi>();

        // Set up spi
        let spi = spi::Spi::<_, _, 16>::new(c.device.SPI0).init(&mut resets, 125_000_000u32.Hz(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0,);

        let mut tc_controller = ThermocoupleController::new(cs_ctr, cs_lr, cs_fb, spi);
        tc_controller.init();

        // --------------- PRESSURE SENSOR CONTROLLER -----------
        // Configure the auxiliary pins
        let ad_start_pin = pins.gpio6.into_push_pull_output();      // Active low??
        let ad_busy_pin = pins.gpio7.into_floating_input();        // AD Alert/Busy pin

        // Configure sda & scl pins for I2C
        let sda_pin = pins.gpio4.into_mode::<rp2040_hal::gpio::FunctionI2C>();
        let scl_pin = pins.gpio5.into_mode::<rp2040_hal::gpio::FunctionI2C>();

        // Configure the I2C0 device
        let i2c = rp2040_hal::I2C::i2c0(
            c.device.pac.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut c.device.pac.RESETS,
            &clocks.peripheral_clock,
    );

        // Create new PressureSensorController
        let mut pressure_sensor_controller = PressureSensorController::new(ad_start_pin, ad_busy_pin, i2c);
        pressure_sensor_controller.init();


        // --------- DEBUG PIN
        let debug_pin = pins.gpio8.into_push_pull_output();


        // Schedule the first HW interrupt task.
        let _ = alarm1.schedule(PWM_TICK_US);
        alarm1.enable_interrupt();

        // Schedule the first HW interrupt task.
        let _ = alarm2.schedule(SAMPLE_TICK_US);
        alarm2.enable_interrupt();

        // Spawn the valve toggle task
        valve_test_task::spawn_after(MicrosDurationU64::secs(5)).unwrap();
        
        // Init and return the Shared data structure
        (Shared { 
            alarm1,
            alarm2,
            debug_pin,
            pwm_ctr, pwm_lr, pwm_fb,
            main_chamber_valve, bladder_valve,
            tc_controller,
            pressure_sensor_controller,
            }, 
        Local {}, 
        init::Monotonics(Monotonic::new(timer, alarm0))
        )
    }

    // -- HEATER PWM TASK: Hardware task coupled to Alarm1/TIMER_IRQ_1.
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_1,  
        shared = [alarm1, pwm_ctr, pwm_lr, pwm_fb ],
        local = [counter: u16 = 0],
    )]
    fn pwm_period_task (mut c: pwm_period_task::Context) { 

        let mut alarm = c.shared.alarm1;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(PWM_TICK_US);
        });

        // Increment and wrap counter if necesary
        *c.local.counter += 1;
        if *c.local.counter == TOP {*c.local.counter =0;}

        // Center heater pwm
        (c.shared.pwm_ctr).lock(|p| {
            if *c.local.counter < p.get_duty_threshold() { 
                p.set_high(); 
            } else { p.set_low(); }    
        });

        // Front-Back heater pwm
        (c.shared.pwm_fb).lock(|p| {
            if *c.local.counter < p.get_duty_threshold() { 
                p.set_high(); 
            } else { p.set_low(); }    
        });

        // Left-Right heater pwm
        (c.shared.pwm_lr).lock(|p| {
            if *c.local.counter < p.get_duty_threshold() { 
                p.set_high(); 
            } else { p.set_low(); }    
        });

    }

    // -- SIGNAL PROCESSING TASK: Hardware task coupled to Alarm2/TIMER_IRQ_2.
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_2,  
        shared = [alarm2, debug_pin, tc_controller, pressure_sensor_controller],
        local = [toggle: bool = true],
    )]
    fn sample_period_task (mut c: sample_period_task::Context) { 
        if *c.local.toggle {
            c.shared.debug_pin.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.debug_pin.lock(|l| l.set_low().unwrap());
        }
        *c.local.toggle = !*c.local.toggle;

        c.shared.tc_controller.lock(|tcc: _| {
            let mut temps = tcc.read_temps(TCChannel::Center);
            info!("Channel: {:?} \t\tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
            temps = tcc.read_temps(TCChannel::LeftRight);
            info!("Channel: {:?} \tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
            temps = tcc.read_temps(TCChannel::FrontBack);
            info!("Channel: {:?} \tTemp: {=f32}\tRef Temp: {=f32}   \tError: {:?}", temps.channel, temps.tc_temp, temps.ref_temp, temps.error);
        });
        

        c.shared.pressure_sensor_controller.lock(|psc: _| {
            let mut measurement = psc.read_pressures();
            info!("PRESSURE----Channel: {:?} \tPressure: {=f32}", measurement.channel_index, measurement.pressure_pa);
        });
        

        println!("------");

        let mut alarm = c.shared.alarm2;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(SAMPLE_TICK_US);
        });
    }

    // ------- VALVE CONTROLLER TEST TASK: -----------.
    #[task(
        priority = 1, 
        shared = [main_chamber_valve, bladder_valve],
        local = [toggle: bool = true],
    )]
    fn valve_test_task (mut c: valve_test_task::Context) { 
        if *c.local.toggle {
            c.shared.main_chamber_valve.lock(|l| l.set_state(ValveState::Pump));
            c.shared.bladder_valve.lock(|l| l.set_state(ValveState::Pump));
        } else {
            c.shared.main_chamber_valve.lock(|l| l.set_state(ValveState::Vent));
            c.shared.bladder_valve.lock(|l| l.set_state(ValveState::Vent));
        }
        *c.local.toggle = !*c.local.toggle;


        valve_test_task::spawn_after(MicrosDurationU64::secs(5)).unwrap();
    }

}
