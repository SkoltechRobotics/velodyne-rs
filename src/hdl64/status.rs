//! Utility module for acquiring sensor status data from HDL-64
//!
//! The main part of this module is [`StatusListener`](struct.StatusListener.html)
//! struct. It handles processing of status data stored in the last two bytes of
//! the data packets.
//!
//! To use it first create instance using either
//! [`new(..)`](struct.StatusListener.html#method.new) or
//! [`init(..)`](struct.StatusListener.html#method.init). After successful
//! struct creation you can use
//! [`get_status()`](struct.StatusListener.html#method.get_status) method
//! to get current status data and
//! [`get_calib_db()`](struct.StatusListener.html#method.get_calib_db) method
//! to get calibration data stored inside device, which is neccecary for
//! conversion of points into Cartesian coordinates, but note that this data is
//! less presice compared to data stored in the XML file. Do not forget to
//! continously update data using
//! [`feed(&packet.status)`](struct.StatusListener.html#method.feed)
//! method by passing packet's status into it.
use crate::packet::{PacketSource, StatusBytes};
use std::io;

use super::calib::CalibDb;

use super::Status;
use super::status_accum::StatusAccumulator;

/// HDL-64 status listener
///
/// Minimum number of statuses for successful initialization equals to 4160
/// (~ 1 s). This number can be significantly higher if some packets are lost,
/// as full sequence of 4160 status bytes is required for initialization.
pub struct StatusListener {
    status: Status,
    calib_db: CalibDb,

    accum: StatusAccumulator,
}

impl StatusListener {
    /// Get calibration data stored in the sensor
    pub fn get_calib_db(&self, dist_lsb: f32) -> CalibDb {
        let mut calib_db = self.calib_db.clone();
        calib_db.dist_lsb = dist_lsb;
        calib_db
    }
}

impl super::super::StatusListener for StatusListener {
    type Status = Status;

    fn init<T: PacketSource>(packet_source: &mut T) -> io::Result<Self> {
        let mut accum = StatusAccumulator::default();
        let (status, calib_db) = accum.init(packet_source)?;
        Ok(StatusListener { status: status, calib_db: calib_db, accum: accum })
    }

    fn feed(&mut self, status: StatusBytes) {
        let sensor_status = &mut self.status;
        let calib_db = &mut self.calib_db;
        self.accum.feed(status, sensor_status, calib_db);
    }

    fn get_status(&self) -> &Self::Status {
        &self.status
    }
}
