#![no_std]
#![no_main]

pub mod pwm_controller;
pub mod valve_controller;
pub mod chamber_controller;
pub mod thermocouple_controller;
pub mod pressure_sensor_controller;
pub mod signal_processing;
pub mod data_structs;
pub mod time_util;
pub mod recipe_manager;
pub mod handle_command;
//pub mod machine_mode;

use panic_halt as _;
use defmt_rtt as _;

/*
===== Possible TO DOs =====
> Use stateful output pins as the authority on pinstate?
*/

// Place unused IRQs in the dispatchers list.  You need one IRQ for each priority level in your code
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [ADC_IRQ_FIFO, UART1_IRQ, DMA_IRQ_1])]    
mod app {

    use core::fmt::Error;

    //use data_structs::Measurement;
    use embedded_hal::digital::v2::OutputPin;
    use fugit::{MicrosDurationU32, MicrosDurationU64 ,RateExtU32};
    use defmt::*;
    use hal::rtc::{RealTimeClock, DateTime, DayOfWeek};
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::{monotonic::Monotonic, Alarm, Alarm0}, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };
    use rp_pico::hal::gpio::pin::bank0::{Gpio0, Gpio1, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio17, Gpio19, Gpio20};
    use rp_pico::hal::spi;
    use rp_pico::hal::pac::SPI0;
    use rp_pico::hal::pac::I2C0;
    use rp_pico::hal::I2C;
    use rp_pico::hal::Clock;
    use rp_pico::hal::gpio::Pin;
    use rp_pico::hal::gpio::FunctionI2C;
    use hal::uart::{DataBits, StopBits, UartConfig};
    use embedded_hal::serial::Read;
    use serde_json_core;

    use crate::{pwm_controller::PWM, thermocouple_controller::TCError};
    use crate::valve_controller::{ValveController, ValveState};
    use crate::thermocouple_controller::{ThermocoupleController, TCChannel};
    use crate::pressure_sensor_controller::PressureSensorController;
    use crate::signal_processing::{MovingAverageFilter, PIDController};
    use crate::data_structs::{Measurement, Command};
    use crate::recipe_manager::{SetPoint, VacuumSetpoint::{Vented, Evacuated}, Recipe};
    //use crate::machine_mode::MachineMode;


    // PWM cycle period.
    const PWM_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(4167);
    // PWM Period = TOP*PWM_TICK_US.   Max value for duty-factor:  [0-TOP].  0= off, TOP = 100% on
    const TOP: u16 = 480;  // Ticks per period

    // Sample period for Alarm2.
    const SAMPLE_TICK_US: MicrosDurationU32 = MicrosDurationU32::micros(50_000);

    // Control loop period
    const CONTROL_LOOP_TICK_MS: u64 = 1000;

    // Length of the low pass averaging filter
    const FILTER_LENGTH: usize = 50;

    pub const N_RECIPE_SETPOINTS: usize = 4;

    const UART_RX_BUF_MAX: usize = 80;
    const UART_TX_BUF_MAX: usize = 600;
    const UART_BAUDRATE: u32 = 57600;   // 28800, 57600, 115200 

    const P_ATM_THRESHOLD: f32 = -200.0;            // Pa.  Above this pressure is considered atmosphere
    const P_VACUUM_THRESHOLD: f32 = -100_000.0;     // Pa  Below this pressure is considered vacuum

    // Alarm0 which generates interrupt request TIMER_IRQ_0 is used by monotonic
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MonotonicType = Monotonic<Alarm0>;

    // Alias the type for our UART pins to make things clearer.
    type UartPins = (
        hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    );

    // Alias the type for our UART to make things clearer.
    type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART0, UartPins>;
    type UartReader = hal::uart::Reader<hal::pac::UART0, UartPins>;
    type UartWriter = hal::uart::Writer<hal::pac::UART0, UartPins>;

    // ----------------------------------------------------------------
    // -----  SHARED DATA
    // ----------------------------------------------------------------
    #[shared]
    struct Shared {
        // Sample measurement
        debug_pin: hal::gpio::Pin<Gpio8, hal::gpio::PushPullOutput>,

        // Heater PWMs - Shared with the PWM task and the main control loop task
        pwm_ctr: PWM<Gpio15>,
        pwm_lr: PWM<Gpio13>,
        pwm_fb: PWM<Gpio14>,

        // Measurement data
        measurement: Measurement,
        recipe: Recipe::<N_RECIPE_SETPOINTS>,

    }

    // ----------------------------------------------------------------
    // -----  LOCAL DATA
    // ----------------------------------------------------------------
    #[local]
    struct Local {
        // Heater PWM stuff
        alarm1: hal::timer::Alarm1,     // For PWM tick
        alarm2: hal::timer::Alarm2,     // For sample tick
        
        // Thermocouple
        tc_controller: ThermocoupleController<Gpio17, Gpio19, Gpio20, SPI0>,

        // Pressure Sensor Controller
        pressure_sensor_controller: PressureSensorController<Gpio6, Gpio7, I2C<I2C0, (Pin<Gpio4, FunctionI2C>, Pin<Gpio5, FunctionI2C>)>>,

        temp_ctr_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        temp_lr_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        temp_fb_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        temp_avg_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        p_chamber_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        p_bladder_filter: MovingAverageFilter<f32, FILTER_LENGTH>,
        // Valves
        chamber_valve: ValveController<Gpio12>,
        bladder_valve: ValveController<Gpio11>,
        pid_controller: PIDController,
        rtc: RealTimeClock,
        start_time: u32,
        //recipe: Recipe::<N_RECIPE_SETPOINTS>,
        uart_rx_msg_len: usize, // UART Receive message length
        uart_rx_buffer: [u8; UART_RX_BUF_MAX], // Place to hold received bytes (should be in local or shared???)
        uart_writer: UartWriter,
        uart_reader: UartReader,
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

        // Create alarm2. When alarm2 triggers, it generates interrupt TIMER_IRQ_2
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
        

         // --------------- INIT REALTIME CLOCK --------------

        let rtc =  RealTimeClock::new(c.device.RTC, clocks.rtc_clock , &mut resets, initial_date_time).expect("ERROR IN NEW RTC");

        
         // --------------- INIT UART --------------
        // Get the UART pins.  UART0 Tx = Gpio0, UART0 Rx = Gpio1
        let uart_pins: UartPins = ( pins.gpio0.into_mode::<hal::gpio::FunctionUart>(), pins.gpio1.into_mode::<hal::gpio::FunctionUart>());
    
        // Init a new UART using the given pins            // &mut c.device.RESETS
        let uart: Uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable( UartConfig::new(UART_BAUDRATE.Hz(), DataBits::Eight, None, StopBits::One),clocks.peripheral_clock.freq() ).unwrap();
        
        // Split the UART into Reader and Writer.
        let (mut uart_reader, uart_writer) = uart.split();
        uart_reader.enable_rx_interrupt();

        let uart_rx_buffer = [0_u8; UART_RX_BUF_MAX];  // Place to hold received bytes
        let uart_rx_msg_len = 0;        // Number of received bytes.  


        // --------------- CREATE RECIPE --------------
        
        static RECIPE_ARRAY:[SetPoint; N_RECIPE_SETPOINTS] = [
        SetPoint{t:0, temp: 10.0, p_chamber: Evacuated, p_bladder: Vented},
        SetPoint{t:40, temp: 50.0, p_chamber: Vented, p_bladder: Evacuated},
        SetPoint{t:80, temp: 50.0, p_chamber: Evacuated, p_bladder: Vented},
        SetPoint{t:120, temp: 10.0, p_chamber: Vented, p_bladder: Evacuated},
        ];

        let recipe = Recipe::<N_RECIPE_SETPOINTS>::new(RECIPE_ARRAY);// new sets the recipe time to 0 and is_running to false
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
        let mut chamber_valve = ValveController::new(pins.gpio12.into_push_pull_output()); //, P_ATM_THRESHOLD, P_VACUUM_THRESHOLD
        chamber_valve.set_valve_state(ValveState::Pump);
        let mut bladder_valve = ValveController::new(pins.gpio11.into_push_pull_output());  // , P_ATM_THRESHOLD, P_VACUUM_THRESHOLD
        bladder_valve.set_valve_state(ValveState::Pump);

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
        let temp_avg_filter = MovingAverageFilter::<f32, 50>::new();
        let p_chamber_filter = MovingAverageFilter::<f32, 50>::new();
        let p_bladder_filter = MovingAverageFilter::<f32, 50>::new();

        let mut measurement = Measurement {
            tt_c: 0.0,
            tt_l: 0.0,
            tt_f: 0.0,   
            tt_avg: 0.0,
            tt_sp: 0.0,   // Current temp setpoint
            tt_delta: 0.0,
            tt_err_c: TCError::NoTCError,
            tt_err_l: TCError::NoTCError,
            tt_err_f: TCError::NoTCError,
            p_ch: 0.0,
            p_bl: 0.0,
            df_c: 0.0,
            df_l: 0.0,
            df_f: 0.0,
            vlv_ch: ValveState::Pump,   // Owned by valve controllers
            vlv_bl: ValveState::Pump, 
            pwr: false,
            t_ela: 0, // Recipe elapsed time.
            seg: 0,
            t_rcp: 0,
            isrun: false,       // Recipe is running.

            // Setpoints
            tt_sp_in: 0.0,   // Current temp setpoint
            tt_trim_l_sp: 1.0,
            tt_trim_f_sp: 1.0,
            vlv_ch_in: ValveState::Pump,
            vlv_bl_in: ValveState::Pump,
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
            debug_pin,
            pwm_ctr, pwm_lr, pwm_fb,
            measurement,
            recipe,
            }, 
        Local {
            alarm1,
            alarm2,
            tc_controller,
            pressure_sensor_controller,
            temp_ctr_filter,
            temp_lr_filter,
            temp_fb_filter,
            temp_avg_filter,
            p_chamber_filter,
            p_bladder_filter,
            // Valves
            chamber_valve,
            bladder_valve,
            pid_controller,
            rtc,
            start_time,
            //recipe,
            uart_rx_msg_len,
            uart_rx_buffer,
            uart_writer,
            uart_reader,
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
        shared = [pwm_ctr, pwm_lr, pwm_fb ],
        local = [alarm1, counter: u16 = 0],
    )]
    fn pwm_period_task (mut c: pwm_period_task::Context) { 

        // Restart the task
        let alarm = c.local.alarm1;
        alarm.clear_interrupt();
        let _ = alarm.schedule(PWM_TICK_US);


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
        shared = [debug_pin, measurement],
        local = [
            alarm2, 
            tc_controller,
            pressure_sensor_controller,
            toggle: bool = true,
            temp_ctr_filter,
            temp_lr_filter,
            temp_fb_filter,
            temp_avg_filter,
            p_chamber_filter,
            p_bladder_filter, 
        ],
    )]
    fn sample_period_task (mut c: sample_period_task::Context) { 

        // Debug pin for checking timing...
        c.shared.debug_pin.lock(|l| l.set_high().unwrap());

        // Acquire temperature measurements
        let mut temp_ctr:f32 = 0.0;
        let mut temp_lr:f32 = 0.0;
        let mut temp_fb:f32 = 0.0;
        let mut temp_avg: f32 = 0.0;
        let mut temp_err_ctr: TCError = TCError::NoTCError;
        let mut temp_err_lr: TCError = TCError::NoTCError;
        let mut temp_err_fb: TCError = TCError::NoTCError;
        
        let tcc = c.local.tc_controller;
        let mut good_tc_count:u32 = 0;      // Number of good T/Cs for the average
        let mut temp_sum: f32 = 0.0;        // for the average temp

        match tcc.acquire(TCChannel::Center) {
            Err(e) => {temp_err_ctr = e;},
            Ok(t) => { 
                temp_ctr = t;
                temp_sum += t;
                good_tc_count += 1;
            },
        }

        match tcc.acquire(TCChannel::LeftRight) {
            Err(e) => {temp_err_lr = e;},
            Ok(t) => { temp_lr = t;
                temp_sum += t;
                good_tc_count += 1;
            },
        }

        match tcc.acquire(TCChannel::FrontBack) {
            Err(e) => {temp_err_fb = e;},
            Ok(t) => {
                temp_fb = t;
                temp_sum += t;
                good_tc_count += 1;
            },
        };
        temp_avg = temp_sum/(good_tc_count as f32);

        
        // Acquire pressure measurements
        let mut p_chamber:f32 = 0.0;
        let mut p_bladder:f32 = 0.0;
        let psc = c.local.pressure_sensor_controller;
        psc.acquire_all();
        p_chamber = psc.get_pressure(0);
        p_bladder = psc.get_pressure(2);


        // Filter all input signals
        let temp_ctr_filtered = c.local.temp_ctr_filter.push(temp_ctr);
        let temp_lr_filtered = c.local.temp_lr_filter.push(temp_lr);
        let temp_fb_filtered = c.local.temp_fb_filter.push(temp_fb);
        let temp_avg_filtered = c.local.temp_avg_filter.push(temp_avg);
        let p_chamber_filtered = c.local.p_chamber_filter.push(p_chamber);
        let p_bladder_filtered = c.local.p_bladder_filter.push(p_bladder);

        // Lock shared.measurement and write data to shared measurements data structure
        (c.shared.measurement).lock(|m: _| {
            m.tt_c = temp_ctr_filtered;
            m.tt_l = temp_lr_filtered;
            m.tt_f = temp_fb_filtered;
            m.tt_avg = temp_avg_filtered;
            m.tt_err_c = temp_err_ctr;
            m.tt_err_l = temp_err_lr;
            m.tt_err_f = temp_err_fb;
            m.p_ch = p_chamber_filtered;
            m.p_bl = p_bladder_filtered;
        });

        // Debug pin for checking timing...
        c.shared.debug_pin.lock(|l| l.set_low().unwrap());


        // Schedule next iteration
        let mut alarm = c.local.alarm2;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SAMPLE_TICK_US);

    }

    // ----------------------------------------------------------------
    // ------- MAIN CONTROL LOOP TASK 
    // ----------------------------------------------------------------
    #[task(
        priority = 1, 
        shared = [
            measurement, 
            pwm_ctr, pwm_lr, pwm_fb,
            recipe,
            ],
        local = [
            toggle: bool = true, 
            chamber_valve, 
            bladder_valve, 
            pid_controller,
            rtc,
            start_time,
            uart_writer,
            ],
    )]
    fn control_loop_task (c: control_loop_task::Context) { 

        (c.shared.measurement, c.shared.pwm_ctr, c.shared.pwm_lr, c.shared.pwm_fb, c.shared.recipe).lock(|m: _, pwm_ctr: _ , pwm_lr: _ , pwm_fb: _, r: _| {
            
            // Get elapsed time since power on
            let now = crate::time_util::date_time_to_seconds(c.local.rtc.now().unwrap());
            let elapsed = now - *c.local.start_time;

            let sp: SetPoint;
            let setpoint_temp;

            if r.is_running() {     // If recipe is running...
                r.update(); // Update the recipe setpoints, if running. TODO: combine into one fn?
                (sp, m.seg) = r.get_current_setpoint();  
                setpoint_temp = sp.temp; // Get setpoint temp from recipe.

                // Set the valves per the setpoints for each chamber
                match sp.p_chamber {
                    Evacuated => {c.local.chamber_valve.set_valve_state(ValveState::Pump);}
                    Vented => {c.local.chamber_valve.set_valve_state(ValveState::Vent);}
                }

                match sp.p_bladder {
                    Evacuated => {c.local.bladder_valve.set_valve_state(ValveState::Pump);}
                    Vented => {c.local.bladder_valve.set_valve_state(ValveState::Vent);}
                }
            } else{     // Not running, manual mode....
                setpoint_temp = m.tt_sp_in;     // Get setpoint temp from user input.
                c.local.bladder_valve.set_valve_state(m.vlv_bl_in);
                c.local.chamber_valve.set_valve_state(m.vlv_ch_in);
            }

            // Get overall duty_factor from PID controller
            let pwm_out: f32 =  c.local.pid_controller.update(m.tt_avg, setpoint_temp);        

            // Determine chamber pressure states (informational only)
            //let ps_chbr: PressureState = c.local.chamber_valve.get_pressure_state(m.p_chamber);
            //let ps_bladder: PressureState = c.local.chamber_valve.get_pressure_state(m.p_chamber);
        

            // Set the duty_factors for the 3 sets of heaters
            let df_ctr = pwm_out;
            let df_lr = pwm_out*m.tt_trim_l_sp;
            let df_fb = pwm_out*m.tt_trim_f_sp;
            pwm_ctr.set_duty_factor(df_ctr);
            pwm_lr.set_duty_factor(df_lr);
            pwm_fb.set_duty_factor(df_fb); 

            //  --- FILL IN STRUCT VALUES TO SEND TO UI
            m.t_rcp = r.recipe_current_time(); // Put the current recipe time in the data struct
            m.isrun = r.is_running(); // Copy recipe manager running state to pv variables for the UI
            m.tt_sp = setpoint_temp;   // Current temp setpoint output to UI
            m.tt_delta = setpoint_temp-m.tt_avg;  // Current temp delta (error)
            m.vlv_ch = c.local.chamber_valve.get_valve_state();
            m.vlv_bl = c.local.bladder_valve.get_valve_state();
            m.t_ela = elapsed; // Recipe elapsed time.
            m.df_c = df_ctr;
            m.df_l = df_lr;
            m.df_f = df_fb;

            // ------------------ UART SEND ---------------
            let mut json_buf = [0_u8; UART_TX_BUF_MAX];     // Create oversized buffer to hold JSON string
            let buf_len = serde_json_core::ser::to_slice(m, &mut json_buf).unwrap();    // Serialize struct m into buffer/slice
            json_buf[buf_len] = 0x0a;   // Append newline 'char'
            c.local.uart_writer.write_full_blocking(&json_buf[..=buf_len]);        // Write buffer/string to UART

            info!("Sent bytes = {:?}, incl newline", buf_len+1);
        });

        // --- Reschedule this task
        control_loop_task::spawn_after(MicrosDurationU64::millis(CONTROL_LOOP_TICK_MS)).unwrap();
    }


    // ----------------------------------------------------------------
    // -- UART RECEIVE TASK
    // ----------------------------------------------------------------
     #[task(
        priority = 2, 
        binds = UART0_IRQ,  
        shared = [measurement, recipe],
        local = [uart_rx_buffer, uart_rx_msg_len, uart_reader],
    )]
    fn uart_receive_task (c: uart_receive_task::Context) { 
        let uart_reader = c.local.uart_reader; 

        let mut cm = c.shared.measurement;
        let mut rec = c.shared.recipe;

        while let Ok(byte) = uart_reader.read() {                 // Read bytes until none avail in FIFO
            if byte != 0x0a {   // If byte is not a newline, 
                c.local.uart_rx_buffer[*c.local.uart_rx_msg_len] = byte;     // Accumulate byte into buffer
                *c.local.uart_rx_msg_len += 1;           // Increment the message length
            } else {        // Otherwise, if we found a newline char...
                //let sslice = core::str::from_utf8(&c.local.buffer[0..*c.local.msg_len]).unwrap();   // Convert buffer to string slice for debug
                
                //info!("UI >> Controller Received Message {}", sslice);        // Print the msg contents
                let result: Result<(Command, usize), serde_json_core::de::Error> = serde_json_core::from_slice(&c.local.uart_rx_buffer[0..*c.local.uart_rx_msg_len]);
                if let Ok((command, _)) = result  {
                    //info!("Parsed command object: {:?}, {:?}", command, byte_count);
                    cm.lock(|m| {       // Dont understand why I couldnt lock multiple variables with a tuple...??
                        rec.lock(|r| {
                            crate::handle_command::handle_command(command, m, r);
                        });
                    })
                    
                } else {
                    info!("Deserialize failed");
                }
                *c.local.uart_rx_msg_len = 0;    // reset the message length counter
            }
        }

    }
}
