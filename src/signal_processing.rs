use core::fmt::Debug;
use num_traits::float::Float;
use num_traits::AsPrimitive;

pub enum SignalProcError {
    ZeroLengthArray,
    LengthExceedsCapacity,
}

pub struct RingArray<T: Float, const N: usize> {
    tail: usize,  // Index of the most recently added element
    data: [T; N], // Array containing the data.
}

impl<T: Float, const N: usize> RingArray<T, N> {
    /// Creates a new RingArray of with length `length` and initialized to `init_value`.
    pub fn new() -> Self {
        Self {
            tail: N - 1,           // raw index to the last element
            // Initialize the data array to 0.0
            data: [T::from(0.0).unwrap(); N],  //  <-- Compiler complains here about 0.0.  Expected type T found {float}
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

pub struct MovingAverageFilter<T: Float, const N: usize> {
    ring_array: RingArray<T, N>,
    sum: T,
}

impl <T: Float, const N: usize> MovingAverageFilter<T, N> {

    pub fn new() -> MovingAverageFilter<T, N>  {
        let ring_array = RingArray::<T, N>::new();
        Self {
            ring_array,
            sum: T::from(0.0).unwrap(),   //  <-- Compiler complains here about 0.0.  Expected type T found {float}
        }
    }

    pub fn push(&mut self, input: T) -> T {

        // Push the input and pop the head.
        let head = self.ring_array.push(input);

        // Add input to the sum and subtract the head
        self.sum = self.sum + input - head; 

        let length = T::from(self.ring_array.len()).unwrap();

        // Want to cast length to type T. How?
        self.sum/length  //  <-- Error. Expectded denom to be type T, found usize
    }

    /// Returns the length
    pub fn len(&self) -> usize {
        self.ring_array.len()
    }
}
