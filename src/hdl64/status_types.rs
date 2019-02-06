use std::net::Ipv4Addr;
use chrono::{DateTime, Utc};

/// Possible statuses of external GPS sensor connection
#[derive(Copy, Clone, Debug)]
pub enum GpsStatus {
    /// NMEA messages and synchronization pulses are available
    SyncNmea,
    /// Only NMEA messages are available
    NmeaOnly,
    /// Only synchronization pulses are available
    SyncOnly,
    /// GPS sensor not connected
    NotConnected,
}

/// Multiple return modes
#[derive(Debug, Clone, Copy)]
pub enum ReturnType {
    /// Strongest return only (default)
    Strongest,
    /// Last return only
    Last,
    /// Both strongest and last returns. If the strongest return is equal to the
    /// last return, the next strongest return is reported.
    Both,
}

/// Power level status
#[derive(Debug, Clone, Copy)]
pub enum PowerLevel {
    /// Automatically selected laser power with normalized intensity returns.
    AutoNormalized,
    /// Automatically selected laser power with raw intensity returns.
    ///
    /// Last 3 bits in the least significant byte of measured distance will
    /// contain power value for given laser.
    AutoRaw,
    /// Manually selected power value, from 0 to 7
    Manual(u8),
}


/// HDL-64 Status Type Calibration and Unit Parameters
#[derive(Debug, Clone, Copy)]
pub struct Status {
    /// Current sensor datetime
    pub dt: DateTime<Utc>,
    /// Status of GPS sensor connection
    pub gps: GpsStatus,
    /// Inner sensor temperature in Celsius
    pub temperature: u8,
    /// Firmware version
    ///
    /// 4 most significant bits denote major and 4 least significant bits
    /// denote minor version number (e.g. 0x47 means version 4.07)
    pub version: u8,

    /// True if laser lenses must be cleaned
    pub lens_contamination: bool,
    /// True if sensor is too hot internally
    pub hot: bool,
    /// True if sensor is too cold internally
    pub cold: bool,
    /// True if PPS (Pulse-per-second) signal is available from GPS sensor
    pub pps: bool,
    /// True if unit synchronized internall time with GPS satellites
    pub gps_time: bool,

    /// Number of sensor rotations per minute
    pub rpm: u16,
    /// Start of field of view in degrees multiplied by 100
    pub fov_start: u16,
    /// End of field of view in degrees multiplied by 100
    pub fov_end: u16,
    /// Real life time (hours)
    pub real_life_time: u16,
    /// IP source address
    pub ip_source: Ipv4Addr,
    /// IP destination address
    pub ip_dest: Ipv4Addr,
    /// Multiple Return Status
    pub return_type: ReturnType,
    /// Lasers output power level
    pub power_level: PowerLevel,

    /// Humidity measured by the sensor
    /// (reserved, does not work on exisiting sensors)
    pub humidity: u8,
    /// Noise threshold for upper lasers block
    pub upper_threshold: u8,
    /// Noise threshold for lower lasers block
    pub lower_threshold: u8,

    /// Date and time when sensor calibration was performed
    pub calib_dt: DateTime<Utc>,
}
