use std::io;
use std::net::{UdpSocket, ToSocketAddrs, SocketAddrV4, SocketAddr};
use std::time::Duration;

use super::{PacketSource, RawPacket, PACKET_SIZE};

const DEFAULT_ADDR: &'static str = "0.0.0.0:2368";

/// Acquires and processes packets from the network
pub struct UdpSource {
    socket: UdpSocket,
    buf: RawPacket,
}

impl UdpSource {
    /// Listen for inbound UDP packets on port 2368 with 1 second timeout
    pub fn new() -> io::Result<Self> {
        Self::new_custom(DEFAULT_ADDR, Some(Duration::from_secs(1)))
    }

    /// Listen for inbound UDP packets on specified address
    pub fn new_custom<A>(addr: A, timeout: Option<Duration>)
        -> io::Result<Self>
        where A: ToSocketAddrs
    {
        let socket = UdpSocket::bind(addr)?;
        socket.set_read_timeout(timeout)?;
        Ok(Self::new_custom_socket(socket))
    }

    /// Listen for inbound UDP packets on initialized socket
    pub fn new_custom_socket(socket: UdpSocket) -> Self {
        Self { socket: socket, buf: [0u8; PACKET_SIZE] }
    }
}

impl PacketSource for UdpSource {
    fn next_packet(&mut self)
        -> io::Result<Option<(SocketAddrV4, &RawPacket)>>
    {
        let socket = &self.socket;
        let buf = &mut self.buf;
        match socket.recv_from(buf) {
            Ok((n, addr)) => if n != PACKET_SIZE {
                    Err(io::Error::new(io::ErrorKind::InvalidData,
                        "Packet is smaller than 1206 bytes"))
                } else {
                    match addr {
                        SocketAddr::V4(addr) => Ok(Some((addr, &*buf))),
                        SocketAddr::V6(_) => panic!("IPv6 is not supported"),
                    }

                },
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                Ok(None)
            },
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                Ok(None)
            },
            Err(e) => Err(e),
        }
    }
}
