//! Packet sources module
//!
//! # Example
//! ```
//! // Listen for packets on default port with default timeout,
//! // alternatively pcap files can be used
//! # fn main() -> Result<(), std::io::Error> {
//! use velodyne::packet::{PacketSource, UdpSource};
//!
//! let mut listener = UdpSource::new()?;
//! loop {
//!     // Callback will be called for each point in the acquired packet,
//!     // each packet contains 12*32=384 points in total
//!     let res = listener.next_packet()?;
//!     match res {
//!         Some((src_addr, packet)) => {
//!             // process packet
//!         },
//!         // source is exhausted
//!         None => break,
//!     }
//! }
//! # Ok(()) }
//! ```
use std::io;
use byteorder::{ByteOrder, LE};
use std::net::SocketAddrV4;

mod udp;
pub use self::udp::UdpSource;
mod pcap;
pub use self::pcap::PcapSource;

/// Size in bytes of raw UDP packet data
const PACKET_SIZE: usize = 1206;
const BLOCKS: usize = 12;
const LASERS: usize = 32;
const POINT_SIZE: usize = 3;
const HEADER_SIZE: usize = 2;
const AZIMUTH_SIZE: usize = 2;
const BLOCK_SIZE: usize = HEADER_SIZE + AZIMUTH_SIZE + POINT_SIZE*LASERS; // 100
const BLOCKS_SIZE: usize = BLOCKS*BLOCK_SIZE; // 1200
const STATUS_ID: usize = 1204;
const STATUS_VALUE: usize = 1205;

/// Raw UDP packet data
pub type RawPacket = [u8; PACKET_SIZE];

/// Status id and value bytes incorporated into each packet
#[derive(Copy, Clone, Debug)]
pub struct StatusBytes {
    pub id: u8,
    pub value: u8,
}

/// Raw point data
///
/// Note that `laser` field contains laser position in the block, thus it always
/// ranges from 0 to 31, even for 16 and 64 laser sensors.
#[derive(Debug, Copy, Clone)]
pub struct RawPoint {
    pub distance: u16,
    pub intensity: u8,
    pub laser: u8,
}

/// Meta information associated with the recieved packet
#[derive(Debug, Copy, Clone)]
pub struct PacketMeta {
    pub azimuth: u16,
    pub timestamp: u32,
    pub status: StatusBytes,
}

/// Return status bytes from raw packet data
pub fn get_status(data: &RawPacket) -> StatusBytes {
    StatusBytes { id: data[STATUS_ID], value: data[STATUS_VALUE] }
}

/// Parse Velodyne UDP packet data
pub fn parse_packet<'a>(data: &'a RawPacket) -> (
    PacketMeta,
    impl Iterator<Item=([u8; 2], u16, impl Iterator<Item=RawPoint> + 'a)> + 'a,
) {
    let timestamp = LE::read_u32(&data[BLOCKS_SIZE..BLOCKS_SIZE + 4]);

    // initial azimuth of the packet
    let a0 = LE::read_u16(&data[HEADER_SIZE..HEADER_SIZE+AZIMUTH_SIZE]);

    let iter = data[..1200]
        .chunks_exact(100)
        .map(|block| {
            let header = [block[0], block[1]];
            let azimuth = LE::read_u16(&block[2..4]);

            let block_iter = block[4..100]
                .chunks_exact(3)
                .enumerate()
                .map(|(laser, chunk)| {
                    let distance = LE::read_u16(&chunk[..2]);
                    let intensity = chunk[2];
                    let laser = laser as u8;
                    RawPoint { distance, intensity, laser }
                })
                .filter(|point| point.distance != 0);
            (header, azimuth, block_iter)
        });

    let status = get_status(data);
    let meta = PacketMeta { azimuth: a0, timestamp, status };
    (meta, iter)
}

/// Source of raw sensor packets and basic parser.
pub trait PacketSource {
    /// Get next raw packet.
    ///
    /// Will return `Ok(None)` if source is exhausted.
    fn next_packet(&mut self) -> io::Result<Option<(SocketAddrV4, &RawPacket)>>;
}
