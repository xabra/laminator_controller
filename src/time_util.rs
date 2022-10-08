use fugit::{MicrosDurationU32, MicrosDurationU64 ,RateExtU32, Instant};
use  rp_pico::hal::rtc::{RealTimeClock, DateTime, DayOfWeek};
use defmt_rtt as _;

pub fn date_time_to_seconds(dt: DateTime) -> u32{
    let sec: u32 = 
        dt.year as u32*0_32 + 
        dt.month as u32*0_u32 +        // Ignore the year and month for now
        dt.day as u32*86_400_u32 + 
        dt.hour as u32*3600_u32 + 
        dt.minute as u32*60_u32 + 
        dt.second as u32;
    sec
}

pub fn date_time_to_instant(dt: DateTime) -> Instant<u32, 1, 1> {
    let secs = date_time_to_seconds(dt);
    fugit::Instant::<u32, 1, 1>::from_ticks(secs)
}
