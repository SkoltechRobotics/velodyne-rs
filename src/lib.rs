pub mod packet;

pub mod hdl64;
pub mod hdl32;

use std::{io, fmt};
use std::cmp::max;
use std::marker::PhantomData;
use std::net::SocketAddrV4;

use crate::packet::{PacketSource, RawPacket, StatusBytes, PacketMeta};

/// 3D point with additionall data
#[derive(Default, Copy, Clone, Debug)]
pub struct FullPoint {
    /// XYZ coordinates of the point
    pub xyz: [f32; 3],
    /// Laser number which has measured the point
    pub laser_id: u8,
    /// Intensity value
    pub intensity: u8,
    /// Point measurment timestamp. This value represents microseconds from the
    /// top of the hour.
    pub timestamp: u32,
}

impl From<FullPoint> for [f32; 3] {
    fn from(p: FullPoint) -> Self { p.xyz }
}

/// Erros ehich indicates failed point conversion
///
/// Usually means that header bytes in a packet were invalid.
#[derive(Copy, Clone, Debug)]
pub struct ConversionError;

impl fmt::Display for ConversionError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl std::error::Error for ConversionError {}

/// Trait for converting raw lidar points fo `FullPoint`.
///
/// Implementors can use calibration tables under the hood.
pub trait Convertor {
    /// Converts `RawPoint`s from packet to `P` and for every resulting points
    /// calls `f` using it as an input argument.
    fn convert<F, P>(&self, raw_point: &RawPacket, f: F)
        -> Result<PacketMeta, ConversionError>
        where F: FnMut(P), P: From<FullPoint>;
}

/// Trait for tracking sensor status
///
/// In case if sensor does not emits meaningful status data or you don't need
/// status information, use `DummyStatusListener`.
pub trait StatusListener: Sized {
    /// Type of the status information
    type Status: Clone;

    /// Initialize listener from packet source
    fn init<T: PacketSource>(packet_source: &mut T) -> io::Result<Self>;

    /// Feed status from the parsed packet.
    ///
    /// This method will update internall state of the listener, updating sensor
    /// status (which is accessible through `get_status` method) when possible
    fn feed(&mut self, status: StatusBytes);

    /// Get current status state
    fn get_status(&self) -> &Self::Status;
}

/// Dummy status listener which does nothing
#[derive(Copy, Clone, Debug, Default)]
pub struct DummyStatusListener;

impl StatusListener for DummyStatusListener {
    type Status = ();

    fn init<T: PacketSource>(_source: &mut T) -> io::Result<Self> {
        Ok(DummyStatusListener)
    }

    fn feed(&mut self, _status: StatusBytes) { }
    fn get_status(&self) -> &Self::Status { &() }
}


/// This struct listens to the `packet_source` and converts packets data to
/// points
pub struct PointSource<T, C, S>
    where T: PacketSource, C: Convertor, S: StatusListener
{
    packet_source: T,
    status_lst: S,
    convertor: C,
}

impl<T, C, S> PointSource<T, C, S>
    where T: PacketSource, C: Convertor, S: StatusListener
{
    /// Create new `PointSource`
    pub fn new(mut packet_source: T, convertor: C) -> io::Result<Self> {
        let status_lst = S::init(&mut packet_source)?;
        Ok(Self { packet_source, status_lst, convertor })
    }

    /// Get current sensor status
    pub fn get_status(&self) -> &S::Status {
        self.status_lst.get_status()
    }

    /// Process points in the next recieved packet
    pub fn process_points<F, P>(&mut self, process_point: F)
        -> io::Result<Option<(SocketAddrV4, PacketMeta)>>
        where P: From<FullPoint>, F: FnMut(P)
    {
        let packets = &mut self.packet_source;
        let convertor = &self.convertor;

        let (addr, packet) = match packets.next_packet()? {
            Some(val) => val,
            None => return Ok(None),
        };

        let meta = convertor.convert(packet, process_point)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData,
                "invalid block header"))?;
        self.status_lst.feed(meta.status);

        Ok(Some((addr, meta)))
    }
}

impl<T: PacketSource> PointSource<T, hdl64::Hdl64Convertor, hdl64::StatusListener> {
    /// Initialize HDL-64 packet source
    pub fn hdl64_init(mut packet_source: T) -> io::Result<Self> {
        let status_lst = hdl64::StatusListener::init(&mut packet_source)?;
        let db = status_lst.get_calib_db(0.2);
        let convertor = hdl64::Hdl64Convertor::new(db);
        Ok(Self { packet_source, status_lst, convertor })
    }

    /// Update HDL-64 calibration table
    pub fn hdl64_set_calib_db(&mut self, calib_db: hdl64::CalibDb) {
        self.convertor = hdl64::Hdl64Convertor::new(calib_db);
    }

    pub fn get_calib_db(&self) -> hdl64::CalibDb {
        self.convertor.db.clone()
    }
}

impl<T: PacketSource> PointSource<T, hdl32::Hdl32Convertor, DummyStatusListener> {
    /// Initialize HDL-32E point source
    pub fn hdl32_init(packet_source: T) -> Self {
        Self {
            packet_source,
            status_lst: Default::default(),
            convertor: Default::default(),
        }
    }
}


/// Iterator which returns points for each sensor rotation
pub struct TurnIterator<T, C, S, P>
    where T: PacketSource, C: Convertor, S: StatusListener, P: From<FullPoint>
{
    point_source: PointSource<T, C, S>,
    cap: usize,
    prev_azimuth: u16,
    split_azimuth: u16,
    _p: PhantomData<P>,
}

impl<T, C, S, P> TurnIterator<T, C, S, P>
    where T: PacketSource, C: Convertor, S: StatusListener, P: From<FullPoint>
{
    /// Create new `TurnIterator`
    pub fn new(packet_source: T, convertor: C) -> io::Result<Self> {
        let point_source = PointSource::new(packet_source, convertor)?;
        Ok(Self {
            point_source, cap: 0, prev_azimuth: 0, split_azimuth: 0,
            _p: Default::default(),
        })
    }

    /// Set azimuth at which next turn will begin in `degrees*100`,
    pub fn set_split_azimuth(&mut self, val: u16) {
        self.split_azimuth = val % 36000;
    }
}

impl<T, P> TurnIterator<T, hdl64::Hdl64Convertor, hdl64::StatusListener, P>
    where T: PacketSource, P: From<FullPoint>
{
    /// Initialize `TurnIterator` for HDL-64
    pub fn hdl64_init(packet_source: T) -> io::Result<Self> {
        let point_source = PointSource::hdl64_init(packet_source)?;
        Ok(Self {
            point_source, cap: 0, prev_azimuth: 0, split_azimuth: 0,
            _p: Default::default(),
        })
    }

    /// Update HDL-64 calibration table
    pub fn hdl64_set_calib_db(&mut self, calib_db: hdl64::CalibDb) {
        self.point_source.hdl64_set_calib_db(calib_db);
    }
}

impl<T, P> TurnIterator<T, hdl32::Hdl32Convertor, DummyStatusListener, P>
    where T: PacketSource, P: From<FullPoint>
{
    /// Initialize `TurnIterator` for HDL-32E
    pub fn hdl32_init(packet_source: T) -> Self {
        let point_source = PointSource::hdl32_init(packet_source);
        Self {
            point_source, cap: 0, prev_azimuth: 0, split_azimuth: 0,
            _p: Default::default(),
        }
    }
}

impl<T, C, S, P> Iterator for TurnIterator<T, C, S, P>
   where T: PacketSource, C: Convertor, S: StatusListener, P: From<FullPoint>
{
    type Item = io::Result<(S::Status, Vec<P>)>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut buf = Vec::with_capacity(self.cap);
        loop {
            let res = self.point_source.process_points(|point| buf.push(point));
            let azimuth = match res {
                Ok(Some((_, meta))) => meta.azimuth,
                Ok(None) => return None,
                Err(err) => return Some(Err(err)),
            };
            let sa = self.split_azimuth;
            // assumes that `azimuth` is never equal to `self.prev_azimuth`
            let flag = if self.prev_azimuth > azimuth {
                !(self.prev_azimuth >= sa &&  sa > azimuth)
            } else {
                azimuth >= sa &&  sa > self.prev_azimuth
            };
            self.prev_azimuth = azimuth;
            if flag { break; }
        }
        self.cap = max(self.cap, (11*buf.len())/10);
        let status = self.point_source.get_status().clone();
        Some(Ok((status, buf)))
    }
}
