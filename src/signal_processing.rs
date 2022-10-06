use core::fmt::Debug;
use num_traits::float::Float;
use defmt::*;

pub enum SignalProcError {
    ZeroLengthArray,
    LengthExceedsCapacity,
}

pub struct RingArray<T: Float, const N: usize> {
    tail: usize,  // Index of the most recently added element
    data: [T; N], // Array containing the data.
}

impl<T: Float, const N: usize> RingArray<T, N> {

    /// Creates a new RingArray of with length N
    pub fn new() -> Self {
        Self {
            tail: N - 1,           // raw index to the last element
            // Initialize the data array to 0.0
            data: [T::from(0.0).unwrap(); N], 
        }
    }

    /// Returns the length
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns the element at index, where 0 is the head of the vector (first in)
    pub fn get_at(&self, index: usize) -> T {
        self.data[self.circular_index(self.tail + 1 + index)]
    }

    /// Returns the value of the head element (first in)
    pub fn get_head(&self) -> T {
        let head_index = self.circular_index(self.tail + 1); // Head index always follows the tail index
        self.data[head_index]
    }

    /// Returns the value of the tail element (last in)
    pub fn get_tail(&self) -> T {
        self.data[self.tail]
    }

    /// Pushes an element at the end (tail).  Head element is popped off and returned
    pub fn push(&mut self, x: T) -> T {
        let head_index = self.circular_index(self.tail + 1); // Head index always follows the tail index
        let head_value = self.data[head_index]; // Save head value to return later
        self.data[head_index] = x; // Overwrite the head with the new tail value
        self.tail = head_index; // Update the tail index to point to the new location
        head_value // Return the old (popped) head value
    }

    /// Used internally to compute the index modulo length into the raw data
    fn circular_index(&self, i: usize) -> usize {
        i % self.len() // Might want to cache the length for speed
    }
    
}

/// A moving average filter of length N for f32, f64 types
/// 
/// Optimized for speed by maintaining a running sum of elements.
/// A new elements is added to sum and the popped element is subtracted.
/// This eliminates the need to do N multiply-accumulate operations as in 
/// other FIR filters
pub struct MovingAverageFilter<T: Float, const N: usize> {
    ring_array: RingArray<T, N>,
    sum: T,
}

impl <T: Float, const N: usize> MovingAverageFilter<T, N> {
    /// Create a new filter
    pub fn new() -> MovingAverageFilter<T, N>  {
        let ring_array = RingArray::<T, N>::new();
        Self {
            ring_array,
            sum: T::from(0.0).unwrap(),   // Must be initialized to zero.
        }
    }

    /// Push a new element onto the tailand return the average of all elements.
    pub fn push(&mut self, input: T) -> T {

        // Push the input and pop the head.
        let head = self.ring_array.push(input);

        // Add input to the sum and subtract the head
        self.sum = self.sum + input - head; 

        let length = T::from(self.ring_array.len()).unwrap();

        // Want to cast length to type T. How?
        self.sum/length  //  
    }

    /// Returns the length
    pub fn len(&self) -> usize {
        self.ring_array.len()
    }
}

pub struct PIDController {
    t_sample_sec: f32,  // Sample time.  Could make this a duration....
    kp: f32,            // Proportional gain
    ki: f32,            // Integral gain
    kd: f32,            // Differential gai    umin: f32, umax: f32,   // Output clampsn
    umin: f32, umax: f32,   // Output clamps
    tau_sec: f32,       // Differentiator rolloff time constant (sec)
    prev_e: f32,        // Previous error
    prev_x: f32,        // Previous measurement
    i: f32,
    d: f32,
}

impl PIDController{

    pub fn new(
        t_sample_sec: f32,          // Sample period in seconds
        kp: f32, ki: f32, kd: f32,  // Gains
        umin: f32, umax: f32,       // Output limits
        tau_sec: f32,
    ) -> PIDController {


        info!("PID CONTROLLER SETUP {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}", t_sample_sec, kp, ki, kd, umin, umax, tau_sec);
        PIDController {
            t_sample_sec, 
            kp, ki, kd,
            umin, umax,
            tau_sec,
            prev_e: 0.0,
            prev_x: 0.0,
            i: 0.0,
            d: 0.0,
        }
    }

    pub fn update(&mut self, x: f32, x_sp: f32) -> f32 {

        // Error signal
        let e = x_sp - x;
        info!("PID INPUTS x:{:?}, xsp:{:?}, err:{:?}", x, x_sp, e);

        // Proportional term
        let p: f32 = self.kp * e;

        // Integral term.  Compute i, provisionally
        let i = self.i + self.ki/2.0*self.t_sample_sec*(e+self.prev_e);

        // Differential term
        let d: f32 = 
        -(2.0 * self.kd * (x - self.prev_x)	// Use derivative of measurement, not error, to prevent differentiating the setpoint changes
        + (2.0 * self.tau_sec - self.t_sample_sec) * self.d)
        / (2.0 * self.tau_sec + self.t_sample_sec);

        // Sum the PID terms to get output
        let mut u = p + i + d;

        info!("PID DATA p{:?}, i{:?}, d{:?}, u{:?}", p, i, d, u);

        // Check for saturation before clamping
        let is_sat = self.is_saturated(u);

        u = self.clamp_output(u);

        info!("PID CLAMPED u_clamped{:?}, is_sat{:?} ", u, is_sat);
        // Cache previous values
        self.prev_e = e;
        self.prev_x = x;

        // Anti-windup of integrator
        // If NOT saturated, update the stored value of the integrator
        // Otherwise, keep the integral constant (not accumulating)
        if !is_sat {self.i = i;}  

        // return u
        u     
    }

    // Output clamp function
    fn clamp_output(&self, u:f32) -> f32{
        if u > self.umax {return self.umax;}
        if u < self.umin {return self.umin;}
        u    // return x
    }

    // Returns true if the PID output is saturated
    // Could combine with the clamp_output function, above...
    fn is_saturated(&self, u:f32) -> bool{
        if u > self.umax {return true}
        if u < self.umin {return true}
        false    // Not saturated, return false
    }
}

