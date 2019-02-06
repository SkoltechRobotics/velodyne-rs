use crate::packet::{PacketSource, StatusBytes, get_status};
use std::net::Ipv4Addr;
use chrono::{DateTime, NaiveDate, Utc};
use std::time;
use std::io::{self, ErrorKind, Cursor};
use byteorder::{LittleEndian, ReadBytesExt};
use log::{debug, info, warn};

use super::calib::CalibDb;

use super::{Status, ReturnType, GpsStatus, PowerLevel};

const INIT_TIMEOUT: u64 = 5;

#[derive(Default)]
pub(super) struct StatusAccumulator {
    init: bool,

    dt: [u8; 6],
    gps_val: u8,
    temp_val: u8,
    version_val: u8,

    // Internall state for cycle accumulation
    cycle_ids: [u8; 7],
    cycle_values: [u8; 7],
    cycle_pos: usize,

    // Internall state for cycles processing
    cycle_state: CycleState,

    lasers: LasersCalib,
    sensor_state: [u8; 21],
}

// TODO: CRC check, check radians/degrees
impl StatusAccumulator {
    /// See `StatusListener::init(..)` method docs
    pub(super) fn init<T: PacketSource>(&mut self, packets: &mut T)
        -> io::Result<(Status, CalibDb)>
    {
        let mut sensor_status = default_sensor_status();
        let mut calib_db = CalibDb::default();

        let t = time::Instant::now();
        loop {
            if t.elapsed().as_secs() > INIT_TIMEOUT {
                return Err(io::Error::new(ErrorKind::TimedOut,
                    "Failed to initialize listener in 5 seconds"));
            }
            let status = packets.next_packet()?
                .map(|(_, packet)| get_status(packet))
                .ok_or(io::Error::new(ErrorKind::Other,
                    "Failed to get packet data from packet listener"))?;

            self.feed(status, &mut sensor_status, &mut calib_db);
            if self.init { return Ok((sensor_status, calib_db)); }
        }
    }

    fn process_warning(&mut self, b: u8, status: &mut Status) {
        /*
        status.lens_contamination = (b & 0b1000_000) != 0;
        status.hot = (b & 0b0100_0000) != 0;
        status.cold = (b & 0b0010_0000) != 0;
        status.pps = (b & 0b0000_0100) != 0;
        status.gps_time = (b & 0b0000_0010) != 0;
        */

        status.lens_contamination = (b & 0b0000_0001) != 0;
        status.hot = (b & 0b0000_0010) != 0;
        status.cold = (b & 0b0000_0100) != 0;
        status.pps = (b & 0b0010_0000) != 0;
        status.gps_time = (b & 0b0100_0000) != 0;
    }

    fn process_calib_db(&self, db: &mut CalibDb) {
        let iter = self.lasers.0.iter().zip(db.lasers.iter_mut()).enumerate();
        for (i, (data, dbl)) in iter {
            assert_eq!(data[0] as usize, i, "wrong data for calibration db");
            let mut rdr = Cursor::new(&data[1..19]);
            let vert_corr = read_i16(&mut rdr) as f32 / 100.;
            let rot_corr = read_i16(&mut rdr) as f32 / 100.;

            let (vert_corr_sin, vert_corr_cos) = vert_corr.to_radians().sin_cos();
            let (rot_corr_sin, rot_corr_cos) = rot_corr.to_radians().sin_cos();

            dbl.rot_corr_sin = rot_corr_sin;
            dbl.rot_corr_cos = rot_corr_cos;
            dbl.vert_corr_sin = vert_corr_sin;
            dbl.vert_corr_cos = vert_corr_cos;

            dbl.dist_correction = read_i16(&mut rdr) as f32/10.;
            dbl.dist_corr_x = read_i16(&mut rdr) as f32/10.;
            dbl.dist_corr_y = read_i16(&mut rdr) as f32/10.;
            dbl.vert_offset = read_i16(&mut rdr) as f32/10.;
            dbl.horiz_offset = read_i16(&mut rdr) as f32/10.;
            dbl.focal_dist = read_i16(&mut rdr) as f32/10.;
            dbl.focal_slope = read_i16(&mut rdr) as f32/10.;

            dbl.min_intensity = data[19];
            dbl.max_intensity = data[20];
        }
    }

    fn process_full_cycle(&mut self, status: &mut Status,
        calib_db: &mut CalibDb) -> Result<(), &'static str>
    {
        debug!("full cycle");
        if !self.init {
            info!("Initialization complete");
            self.init = true;
        }

        let d = self.sensor_state;
        let mut rdr = Cursor::new(&d[..]);
        status.rpm = read_u16(&mut rdr);
        status.fov_start = read_u16(&mut rdr);
        status.fov_end = read_u16(&mut rdr);
        status.real_life_time = read_u16(&mut rdr);
        status.ip_source = Ipv4Addr::new(d[8], d[9], d[10], d[11]);
        status.ip_dest = Ipv4Addr::new(d[12], d[13], d[14], d[15]);
        status.return_type = match d[16] {
            0 => ReturnType::Strongest,
            1 => ReturnType::Last,
            2 => ReturnType::Both,
            _ => return Err("invalid return type"),
        };
        status.power_level = match d[18] {
            0xA8 => PowerLevel::AutoNormalized,
            0xA0 => PowerLevel::AutoRaw,
            v if v & 0x0f == 8 && ((v & 0xf0) >> 4) < 8 => {
                PowerLevel::Manual((v & 0xf0) >> 4)
            },
            _ => return Err("invalid power level")
        };

        self.process_calib_db(calib_db);
        Ok(())
    }

    /// Consumes cycle data and checks its content
    ///
    /// Returns:
    ///     Ok(true) -- Cycle consumed without problems
    ///     Ok(false) -- Out of order cycle, caller must reset state
    ///     Err(desc) -- Bug in the code or bad data (e.g. incorect datetime),
    ///         reset state and raise warning
    fn consume_cycle(&mut self, status: &mut Status,
            calib_db: &mut CalibDb)
        -> Result<bool, &'static str>
    {
        let ids = self.cycle_ids;
        let vals = self.cycle_values;
        self.cycle_state = match self.cycle_state {
            CycleState::FirstCycle => {
                debug!("First cycle");
                if !(&ids[..5] == b"12345" && ids[5] == 0xf7 && ids[6] == 0xf6) {
                    return Ok(false);
                }
                if !(&vals[..5] == b"UNIT#") {
                    return Ok(false);
                }
                status.upper_threshold = vals[5];
                status.lower_threshold = vals[6];
                CycleState::Lasers{ laser: 0, part: 0 }
            },
            CycleState::Lasers{ laser, part } => {
                debug!("lasers {} {}", laser, part);
                match part {
                    0 | 1 | 2 => {
                        if &ids != b"1234567" { return Ok(false); }
                        if part == 0 && vals[0] != laser as u8 {
                            return Ok(false);
                        }
                        let s = 7*part;
                        if !self.init {
                            self.lasers.0[laser][s..s+7].copy_from_slice(&vals);
                        }
                        if laser == 63 && part == 2 {
                            CycleState::CalibrationDt
                        } else {
                            CycleState::Lasers{ laser: laser, part: part + 1 }
                        }
                    },
                    3 => {
                        if &ids != b"W234567" { return Ok(false); }
                        self.process_warning(vals[0], status);
                        CycleState::Lasers{ laser: laser + 1, part: 0 }
                    }
                    _ => return Err("Unreachable branch: laser cycles"),
                }
            },
            CycleState::CalibrationDt => {
                debug!("CalibrationDt");
                if &ids != b"1234567" { return Ok(false); }
                let dt = get_dt(vals[0], vals[1], vals[2],
                                vals[3], vals[4], vals[5])?;
                status.calib_dt = dt;
                status.humidity = vals[6];
                CycleState::SensorState{part: 0}
            },
            CycleState::SensorState{part} => {
                debug!("SensorState {}", part);
                match part {
                    0 => {
                        if ids != [0xfe, 0xff, 0xfc, 0xfd, 0xfa, 0xfb, 0x37] {
                            return Ok(false);
                        }
                        self.sensor_state[..7].copy_from_slice(&vals);
                        CycleState::SensorState{part: 1}
                    },
                    1 => {
                        if &ids != b"1234567" { return Ok(false); }
                        self.sensor_state[7..14].copy_from_slice(&vals);
                        CycleState::SensorState{part: 2}
                    },
                    2 => {
                        if ids != [0x31, 0x32, 0xf9, 0x34, 0xf8, 0x36, 0x37] {
                            return Ok(false);
                        }
                        self.sensor_state[14..21].copy_from_slice(&vals);
                        self.process_full_cycle(status, calib_db)?;
                        CycleState::FirstCycle
                    },
                    _ => return Err("Unreachable branch: state cycle"),
                }
            },
        };
        Ok(true)
    }

    fn update_status(&mut self, status: &mut Status)
        -> Result<(), &'static str>
    {
        let dt = self.dt;
        status.dt = get_dt(dt[0], dt[1], dt[2], dt[3], dt[4], dt[5])?;
        status.gps = match self.gps_val {
            0x41 => GpsStatus::SyncNmea,
            0x56 => GpsStatus::NmeaOnly,
            0x50 => GpsStatus::SyncOnly,
            0x00 => GpsStatus::NotConnected,
            _ => { return Err("Unknown GPS status code") },
        };
        status.temperature = self.temp_val;
        status.version = self.version_val;
        Ok(())
    }

    /// See `StatusListener.feed(..)` docs
    pub(super) fn feed(&mut self, status: StatusBytes,
        sensor_status: &mut Status, calib_db: &mut CalibDb)
    {
        let is_ok = match status.id as char {
            'H' => {
                self.dt[3] = status.value;
                self.cycle_pos == 0
            },
            'M' => {
                self.dt[4] = status.value;
                self.cycle_pos == 1
            },
            'S' => {
                self.dt[5] = status.value;
                self.cycle_pos == 2
            },
            'D' => {
                self.dt[2] = status.value;
                self.cycle_pos == 3
            },
            'N' => {
                self.dt[1] = status.value;
                self.cycle_pos == 4
            },
            'Y' => {
                self.dt[0] = status.value;
                self.cycle_pos == 5            },
            'G' => {
                self.gps_val = status.value;
                self.cycle_pos == 6
            },
            'T' => {
                self.temp_val = status.value;
                self.cycle_pos == 7
            },
            'V' => {
                self.version_val = status.value;
                self.cycle_pos == 8
            },
            _ => self.cycle_pos > 8 && self.cycle_pos < 16
        };

        if !is_ok {
            let msg = "Wrong cycle position detected. Reseting.";
            if self.init { warn!("{}", msg); } else { debug!("{}", msg); }
            self.cycle_pos = 0;
            return;
        }

        if self.cycle_pos == 8 {
            if let Err(s) = self.update_status(sensor_status) {
                warn!("{}", s);
                self.cycle_state = CycleState::FirstCycle;
            }
        }

        if self.cycle_pos <= 8 {
            self.cycle_pos += 1;
            return;
        }

        self.cycle_ids[self.cycle_pos - 9] = status.id;
        self.cycle_values[self.cycle_pos - 9] = status.value;

        if self.cycle_pos == 15 {
            match self.consume_cycle(sensor_status, calib_db) {
                Ok(is_ok) => {
                    if !is_ok {
                        let msg = "Wrong cycle state. Reseting.";
                        if self.init {
                            warn!("{}", msg);
                        } else {
                            debug!("{}", msg);
                        }
                        self.cycle_state = CycleState::FirstCycle;
                    }
                },
                Err(s) => {
                    warn!("{}", s);
                    self.cycle_state = CycleState::FirstCycle;
                }
            };
            self.cycle_pos = 0;
        } else {
            self.cycle_pos += 1;
        }
    }
}

fn default_sensor_status() -> Status {
    let dt = get_dt(0, 1, 1, 0, 0, 0).unwrap();
    Status {
        dt: dt,
        gps: GpsStatus::NotConnected,
        temperature: 0,
        version: 0,
        lens_contamination: false,
        hot: false,
        cold: false,
        pps: false,
        gps_time: false,
        rpm: 0,
        fov_start: 0,
        fov_end: 0,
        real_life_time: 0,
        ip_source: Ipv4Addr::new(0, 0 ,0 ,0),
        ip_dest: Ipv4Addr::new(0, 0 ,0 ,0),
        return_type: ReturnType::Strongest,
        power_level: PowerLevel::AutoNormalized,
        humidity: 0,
        upper_threshold: 0,
        lower_threshold: 0,
        calib_dt: dt,
    }
}

#[inline(always)]
fn read_u16(rdr: &mut Cursor<&[u8]>) -> u16 {
     rdr.read_u16::<LittleEndian>().unwrap()
}

#[inline(always)]
fn read_i16(rdr: &mut Cursor<&[u8]>)-> i16 {
    rdr.read_i16::<LittleEndian>().unwrap()
}

fn get_dt(year: u8, month: u8, day: u8, h: u8, m: u8, s: u8)
    -> Result<DateTime<Utc>, &'static str>
{
    let y = year as i32 + 2000;
    let res = NaiveDate::from_ymd_opt(y, month as u32, day as u32)
        .and_then(|d| d.and_hms_opt(h as u32, m as u32, s as u32));
    if let Some(dt) = res {
        Ok(DateTime::<Utc>::from_utc(dt, Utc))
    } else {
        Err("Incorrect datetime")
    }
}

#[derive(Copy, Clone)]
enum CycleState {
    FirstCycle,
    Lasers{ laser: usize, part: usize},
    CalibrationDt,
    SensorState{ part: usize },
}

impl Default for CycleState {
    fn default() -> Self { CycleState::FirstCycle }
}


struct LasersCalib([[u8; 21]; 64]);

impl Default for LasersCalib {
    fn default() -> Self { LasersCalib([[0u8; 21]; 64]) }
}
