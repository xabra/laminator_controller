use defmt::*;

#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum VacuumSetpoint {
    Evacuated,
    Vented,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct SetPoint {
    pub t: u32,       // Elapsed time, seconds
    pub temp: f32,     // setpoint temp
    pub p_chamber: VacuumSetpoint,
    pub p_bladder: VacuumSetpoint,
}

pub struct Recipe<const N: usize> {
    pub setpoints:  [SetPoint; N],
    pub t_recipe: u32,
    pub is_running: bool,
}

impl<const N: usize> Recipe<N> {
    pub fn new(setpoints: [SetPoint; N]) -> Self {
        Self {
            setpoints,
            t_recipe: 0_u32,
            is_running: false,
        }
    }

    pub fn list_setpoints(&self) {
        for (i, sp)  in self.setpoints.iter().enumerate() {
            info!("SetPoint {:?}, sp_time = {:?}, sp_temp = {:?}", i, sp.t, sp.temp);
        }
    }
    pub fn recipe_current_time(&self) -> u32 {
        self.t_recipe
    }
    pub fn reset(&mut self) {
        self.t_recipe = 0;
    }

    // Start or stop the recipe
    pub fn run(&mut self, on: bool) {
        if self.is_running != on {  // If it changed state...
            if on == true {self.reset();}   // If new state is 'running' then reset the timer
            self.is_running = on;   // Update internal state
        }
        
    }
    pub fn is_running(&self) -> bool{
        self.is_running
    }
    pub fn recipe_end_time(&self) -> u32{
        let n = self.setpoints.len();
        self.setpoints[n-1].t   // return the time of the last recipe point.
    }
    pub fn update(&mut self) {
        if self.is_running() {  // If it is running...
            if self.recipe_current_time()>= self.recipe_end_time() {    // If we've run off the end of the recipe...
                self.run(false);        // Stop recipe
            } else {
                self.t_recipe += 1; // TODO this is not right. It should be related to the loop cycle time/elapsed time, not fixed at 1 sec.
            }   
        }
    }

    pub fn get_current_setpoint(&self) -> (SetPoint, usize) {
        let n = self.setpoints.len();
        let t = self.recipe_current_time();
        if t < self.setpoints[0].t {return (self.setpoints[0], 0);}      // Before the first setpoint? return first sp
        if t>= self.recipe_end_time() {return (self.setpoints[n-1], n-1);}      // After or equal to the last setpoint? return last sp

        // Otherwise, scan setpoints up to penultimate setpoint..
        let mut i_segment: usize = 0;
        for i in 0..n-1  {  
            if t >= self.setpoints[i].t &&  t < self.setpoints[i+1].t { // 
                i_segment = i;
                break;
            }
        }

        let t1 = self.setpoints[i_segment].t as f32;
        let t2: f32 = self.setpoints[i_segment+1].t as f32;
        let temp1 = self.setpoints[i_segment].temp;
        let temp2 = self.setpoints[i_segment+1].temp;
        
        // Interpolate the temperature between the endpoints of the segment
        let interp_temp:f32 = temp1 + ((t as f32)-t1)/(t2-t1) *  (temp2 - temp1);

        // Return the (new setpoint, i_segment) tuple
        return (
            SetPoint { 
                t, 
                temp:  interp_temp, //interp_sp_temp,
                // Return pressure SPs corresponding to the preceeding recipe SP
                p_chamber: self.setpoints[i_segment].p_chamber, 
                p_bladder: self.setpoints[i_segment].p_bladder,
            }, 
            i_segment,
        );

    }
}