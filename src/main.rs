#![no_std]
#![no_main]

pub mod pwm_controller;
pub mod valve_controller;
pub mod thermocouple_controller;
pub mod pressure_sensor_controller;
pub mod signal_processing;
pub mod data_structs;
pub mod time_util;
pub mod recipe_manager;

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
    use hal::rtc::{RealTimeClock, DateTime, DayOfWeek};
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
    use rp_pico::hal::gpio::FunctionI2C;

    use crate::{pwm_controller::PWM, thermocouple_controller::TCError};
    use crate::valve_controller::{ValveController, ValveState};
    use crate::thermocouple_controller::{ThermocoupleController, TCChannel};
    use crate::pressure_sensor_controller::PressureSensorController;
    use crate::signal_processing::{MovingAverageFilter, PIDController};
    use crate::data_structs::Measurement;
    use crate::recipe_manager::{SetPoint, VacuumSetpoint::{Vented, Evacuated}, Recipe};


    // PWM cycle period.
    const PWM_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(4167);
    // PWM Period = TOP*PWM_TICK_US.   Max value for duty-factor:  [0-TOP].  0= off, TOP = 100% on
    const TOP: u16 = 480;  // Ticks per period

    // Sample period for Alarm2.
    const SAMPLE_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(200_000);

    // Control loop period
    const CONTROL_LOOP_TICK_MS: u64 = 1000;

    // Length of the low pass averaging filter
    const FILTER_LENGTH: usize = 50;

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MonotonicType = Monotonic<Alarm0>;

    // ----------------------------------------------------------------
    // -----  SHARED DATA
    // ----------------------------------------------------------------
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
        pressure_sensor_controller: PressureSensorController<Gpio6, Gpio7, I2C<I2C0, (Pin<Gpio4, FunctionI2C>, Pin<Gpio5, FunctionI2C>)>>,

        // Measurement data
        measurement: Measurement,
    }

    // ----------------------------------------------------------------
    // -----  LOCAL DATA
    // ----------------------------------------------------------------
    #[local]
    struct Local {
        temp_ctr_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        temp_lr_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        temp_fb_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        p_chamber_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        p_bladder_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        pid_controller: PIDController,
        rtc: RealTimeClock,
        start_time: u32,
    }

    // ----------------------------------------------------------------
    // -----  APP INIT
    // ----------------------------------------------------------------
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

        let initial_date_time = DateTime {
            year: 2022,
            month: 10,
            day: 7,
            day_of_week: DayOfWeek::Friday,
            hour: 23,
            minute: 30,
            second: 0,
        };
        
        let rtc =  RealTimeClock::new(c.device.RTC, clocks.rtc_clock , &mut resets, initial_date_time).expect("ERROR IN NEW RTC");

        // --------------- CREATE RECIPE --------------
        const recipe_array:[SetPoint; 4] = [
        SetPoint{t_sec:10, sp_temp: 25.0, sp_chbr_pressure: Vented, sp_bladder_pressure: Vented},
        SetPoint{t_sec:40, sp_temp: 45.0, sp_chbr_pressure: Vented, sp_bladder_pressure: Evacuated},
        SetPoint{t_sec:70, sp_temp: 95.0, sp_chbr_pressure: Vented, sp_bladder_pressure: Vented},
        SetPoint{t_sec:80, sp_temp: 2.0, sp_chbr_pressure: Vented, sp_bladder_pressure: Vented},
        ];

        let recipe = Recipe::new(&recipe_array);
        recipe.list_setpoints();

        
        // ----------- HEATER PWM CONTROLLER SETUP ------------
        let mut pwm_ctr = PWM::new(pins.gpio15.into_push_pull_output(), TOP);
        pwm_ctr.set_duty_factor(0.5);

        let mut pwm_lr = PWM::new(pins.gpio13.into_push_pull_output(), TOP);
        pwm_lr.set_duty_factor(0.1);

        let mut pwm_fb = PWM::new(pins.gpio14.into_push_pull_output(), TOP);
        pwm_fb.set_duty_factor(0.9);

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
            c.device.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut resets,
            &clocks.peripheral_clock,
    );

        // Create new PressureSensorController
        let mut pressure_sensor_controller = PressureSensorController::new(ad_start_pin, ad_busy_pin, i2c);
        pressure_sensor_controller.init();

        // ----- SIGNAL PROCESSING ------
        let temp_ctr_filter = MovingAverageFilter::<f32, 50>::new();
        let temp_lr_filter = MovingAverageFilter::<f32, 50>::new();
        let temp_fb_filter = MovingAverageFilter::<f32, 50>::new();
        let p_chamber_filter = MovingAverageFilter::<f32, 50>::new();
        let p_bladder_filter = MovingAverageFilter::<f32, 50>::new();

        let measurement = Measurement {
            temp_ctr: 0.0,
            temp_lr: 0.0,
            temp_fb: 0.0,   
            temp_err_ctr: TCError::NoTCError,
            temp_err_lr: TCError::NoTCError,
            temp_err_fb: TCError::NoTCError,
            p_chamber: 0.0,
            p_bladder: 0.0,
        };

        // ----- PID Controller -----
        let t_sample_sec: f32 = (CONTROL_LOOP_TICK_MS/1000) as f32;
        let pid_controller = PIDController::new(t_sample_sec, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0);

        // --------- DEBUG PIN
        let debug_pin = pins.gpio8.into_push_pull_output();


        // Schedule the PWM tick HW interrupt task.
        let _ = alarm1.schedule(PWM_TICK_US);
        alarm1.enable_interrupt();

        // Schedule the high speed data sampling HW interrupt task.
        let _ = alarm2.schedule(SAMPLE_TICK_US);
        alarm2.enable_interrupt();

        // Initialize the elapsed time counter (RTC)
        let now = rtc.now().expect("Error in RTC now");
        let start_time = crate::time_util::date_time_to_seconds(now);

        // Spawn the valve toggle task
        control_loop_task::spawn_after(MicrosDurationU64::secs(5)).unwrap();

        let monotonic: MonotonicType = Monotonic::new(timer, alarm0);
        
        // Init and return the Shared data structure
        (Shared { 
            alarm1,
            alarm2,
            debug_pin,
            pwm_ctr, pwm_lr, pwm_fb,
            main_chamber_valve, bladder_valve,
            tc_controller,
            pressure_sensor_controller,
            measurement,
            //mono,
            }, 
        Local {
            temp_ctr_filter,
            temp_lr_filter,
            temp_fb_filter,
            p_chamber_filter,
            p_bladder_filter,
            pid_controller,
            rtc,
            start_time,
            }, 
        init::Monotonics(monotonic)
        )
    }

    // ----------------------------------------------------------------
    // -- HEATER PWM TASK
    // ----------------------------------------------------------------
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

     // ----------------------------------------------------------------
    // -- DATA ACQUISITION & SIGNAL PROCESSING TASK
     // ----------------------------------------------------------------
    #[task(
        priority = 2, 
        binds = TIMER_IRQ_2,  
        shared = [alarm2, debug_pin, tc_controller, pressure_sensor_controller, measurement],
        local = [
            toggle: bool = true,
            temp_ctr_filter,
            temp_lr_filter,
            temp_fb_filter,
            p_chamber_filter,
            p_bladder_filter, 
        ],
    )]
    fn sample_period_task (mut c: sample_period_task::Context) { 

        c.shared.debug_pin.lock(|l| l.set_high().unwrap());

        // Acquire temperature measurements
        let mut temp_ctr:f32 = 0.0;
        let mut temp_lr:f32 = 0.0;
        let mut temp_fb:f32 = 0.0;
        let mut temp_err_ctr: TCError = TCError::NoTCError;
        let mut temp_err_lr: TCError = TCError::NoTCError;
        let mut temp_err_fb: TCError = TCError::NoTCError;

        c.shared.tc_controller.lock(|tcc: _| {

            match tcc.acquire(TCChannel::Center) {
                Err(e) => {temp_err_ctr = e;},
                Ok(t) => { temp_ctr = t;},
            }

            match tcc.acquire(TCChannel::LeftRight) {
                Err(e) => {temp_err_lr = e;},
                Ok(t) => { temp_lr = t;},
            }

            match tcc.acquire(TCChannel::FrontBack) {
                Err(e) => {temp_err_fb = e;},
                Ok(t) => {temp_fb = t;},
            };

        });
        
        // Acquire pressure measurements
        let mut p_chamber:f32 = 0.0;
        let mut p_bladder:f32 = 0.0;
        (c.shared.pressure_sensor_controller).lock(|psc: _| {
            psc.acquire_all();
            p_chamber = psc.get_pressure(0);
            p_bladder = psc.get_pressure(2);
        });

        // Filter all input signals
        let temp_ctr_filtered = c.local.temp_ctr_filter.push(temp_ctr);
        let temp_lr_filtered = c.local.temp_lr_filter.push(temp_lr);
        let temp_fb_filtered = c.local.temp_fb_filter.push(temp_fb);
        let p_chamber_filtered = c.local.p_chamber_filter.push(p_chamber);
        let p_bladder_filtered = c.local.p_bladder_filter.push(p_bladder);

        // Lock shared.measurement and write data to shared measurements data structure
        (c.shared.measurement).lock(|m: _| {
            m.temp_ctr = temp_ctr_filtered;
            m.temp_lr = temp_lr_filtered;
            m.temp_fb = temp_fb_filtered;
            m.temp_err_ctr = temp_err_ctr;
            m.temp_err_lr = temp_err_lr;
            m.temp_err_fb = temp_err_fb;
            m.p_chamber = p_chamber_filtered;
            m.p_bladder = p_bladder_filtered;
        });

        c.shared.debug_pin.lock(|l| l.set_low().unwrap());


        // Schedule next iteration
        let mut alarm = c.shared.alarm2;
        (alarm).lock(|a|{
            a.clear_interrupt();
            let _ = a.schedule(SAMPLE_TICK_US);
        });
    }

    // ----------------------------------------------------------------
    // ------- CONTROL LOOP TASK 
    // ----------------------------------------------------------------
    #[task(
        priority = 1, 
        shared = [
            main_chamber_valve, 
            bladder_valve, 
            measurement, 
            pwm_ctr, pwm_lr, pwm_fb,
            ],
        local = [
            toggle: bool = true, 
            pid_controller,
            rtc,
            start_time,
            ],
    )]
    fn control_loop_task (c: control_loop_task::Context) { 

        // c.shared.main_chamber_valve.lock(|l| l.set_state(ValveState::Pump));
        // c.shared.bladder_valve.lock(|l| l.set_state(ValveState::Pump));

        (c.shared.measurement, c.shared.pwm_ctr, c.shared.pwm_lr, c.shared.pwm_fb).lock(|m: _, pwm_ctr: _ , pwm_lr: _ , pwm_fb: _| {
            // Get setpoint temperature
            let temp_sp: f32 = 30.4;     
            
            // Compute average measured temperature
            let temp_avg: f32 = average_temp(m); 

            // Get overall duty_factor from PID controller
            let pwm_out: f32 =  c.local.pid_controller.update(temp_avg, temp_sp);

            // Set the duty_factors for the 3 sets of heaters
            pwm_ctr.set_duty_factor(pwm_out);
            pwm_lr.set_duty_factor(pwm_out*0.9);
            pwm_fb.set_duty_factor(pwm_out*0.5);

            let now = crate::time_util::date_time_to_seconds(c.local.rtc.now().unwrap());
            let elapsed = now - *c.local.start_time;

            info!("Elapsed time: {:?}, \t{:?}, \t{:?}, \t{:?}, \t{:?},  \t{:?}, PWM_DF: \t{:?} , AVG TEMP: \t{:?}", 
            elapsed, m.temp_ctr, m.temp_lr, m.temp_fb, m.p_chamber, m.p_bladder, pwm_out, temp_avg);            

        });


        control_loop_task::spawn_after(MicrosDurationU64::millis(CONTROL_LOOP_TICK_MS)).unwrap();
    }

    // Computes an average temp using only the good (no error) TCs
    // TODO: should return an error is good_tc_count == 0
    fn average_temp(m: &mut Measurement) -> f32 {
        let mut good_tc_count:u32 = 0;
        let mut temp_sum: f32 = 0.0;

        if m.temp_err_ctr == TCError::NoTCError {
            temp_sum += m.temp_ctr;
            good_tc_count += 1;
        }
        if m.temp_err_lr == TCError::NoTCError {
            temp_sum += m.temp_lr;
            good_tc_count += 1;
        }
        if m.temp_err_fb == TCError::NoTCError {
            temp_sum += m.temp_fb;
            good_tc_count += 1;
        }

        temp_sum/(good_tc_count as f32)
    }

}
