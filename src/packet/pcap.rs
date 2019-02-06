use byteorder::{ReadBytesExt, LE};
use std::time::{Instant, Duration};
use std::fs::File;
use std::path::Path;
use std::io;
use std::io::{SeekFrom, Seek, Read, ErrorKind, Cursor};
use std::thread::sleep;
use std::net::{SocketAddrV4, Ipv4Addr};
use log::warn;

use memmap::Mmap;

use super::{PacketSource, RawPacket, PACKET_SIZE};

const NS_IN_SEC: u32 = 1_000_000_000;

// tcpdump -s 1248 -i enp2s0 -w out.pcap port 2368

/// Acquires and processes packets from pre-recorded pcap file
pub struct PcapSource {
    file: Cursor<Mmap>,
    is_nano: bool,
    do_sync: bool,
    do_loop: bool,
    packet_t0: (u32, u32),
    t0: Instant,
}

impl PcapSource {
    /// Initialize source with the given `path`.
    ///
    /// If `do_sync` is `true` will emulate arrival of packets using recorded
    /// timings, otherwise it will emit packets as fast as it can.
    pub fn new<P: AsRef<Path>>(path: P, do_sync: bool, do_loop: bool)
        -> io::Result<Self>
    {
        let file = File::open(path)?;
        let mmap = unsafe { Mmap::map(&file)? };
        let mut f = Cursor::new(mmap);

        let (is_le, is_nano) = match f.read_u32::<LE>()? {
            0xa1b2c3d4 => (true, false),
            0xa1b23c4d => (true, true),
            0xd4c3b2a1 => (false, false),
            0x4d3cb2a1 => (false, true),
            _ => return Err(io::Error::new(ErrorKind::InvalidInput,
                "invalid pcap magic number")),
        };
        if !is_le {
            panic!("Big-endian pcap files currently not supported.")
        }
        Self::read_header(f, is_nano, do_sync, do_loop)
    }

    fn read_header(
            mut file: Cursor<Mmap>, is_nano: bool, do_sync: bool, do_loop: bool,
        ) -> io::Result<Self>
    {
        let version_major = file.read_u16::<LE>()?;
        let version_minor = file.read_u16::<LE>()?;
        // skip thiszone, sigfigs and snaplen
        file.seek(SeekFrom::Current(12))?;
        let network = file.read_u32::<LE>()?;
        assert_eq!(version_major, 2);
        assert_eq!(version_minor, 4);
        // Check LINKTYPE_ETHERNET
        assert_eq!(network, 1, "expected LINKTYPE_ETHERNET");

        // time from UNIX_EPOCH
        // note that this time is not Y2038 safe
        let packet_t0 = (
            file.read_u32::<LE>()?,
            file.read_u32::<LE>()? * if is_nano { 1 } else { 1000 },
        );
        // seek back from peeking into start time
        file.seek(SeekFrom::Current(-8))?;

        let t0 = Instant::now();
        Ok(Self { file, is_nano, do_sync, do_loop, packet_t0, t0 })
    }

    pub fn reset(&mut self) {
        self.file.set_position(24);
        self.t0 = Instant::now();
    }

    fn read_packet(&mut self) -> io::Result<(u64, SocketAddrV4)> {
        let mut meta = [0u32; 4];
        self.file.read_u32_into::<LE>(&mut meta)?;
        let [t_s, t_us, incl_len, orig_len] = meta;
        let eth_start = self.file.position();

        // 14 bytes for Ethernet header
        // 20 bytes for IP header (without options)
        // 8 bytes for UDP header
        if orig_len < PACKET_SIZE as u32 + 42 {
            // VeloView records unindentified short packets which we ignore
            warn!("unindentified short packet");
            self.file.set_position(eth_start + incl_len as u64);
            return self.read_packet();
        }
        if orig_len > incl_len {
            self.file.set_position(eth_start + incl_len as u64);
            Err(io::Error::new(io::ErrorKind::InvalidData,
                "UDP packet was truncated"))?;
        }

        let t = (t_s, t_us * if self.is_nano { 1 } else { 1000 } );

        let delta: i64 = orig_len as i64 - PACKET_SIZE as i64 - 16;

        // Skip Ethernet headers
        self.file.seek(SeekFrom::Current(delta))?;

        let mut h = [0u8; 16];
        self.file.read_exact(&mut h)?;
        let port = ((h[12] as u16) << 8) + (h[13] as u16);
        let addr = SocketAddrV4::new(Ipv4Addr::new(h[0], h[1], h[2], h[3]), port);

        let udp_pos = self.file.position();
        self.file.set_position(eth_start + incl_len as u64);

        if self.do_sync { self.time_sync(t); }

        Ok((udp_pos, addr))
    }

    fn time_sync(&self, t: (u32, u32)) {
        // realtime time difference
        let rt_dt = self.t0.elapsed();
        let (rt_s, rt_ns) = (rt_dt.as_secs(), rt_dt.subsec_nanos());
        // time difference between packets
        let t0 = self.packet_t0;
        let mut dt_s = (t.0 as i64) -  (t0.0 as i64);
        let mut dt_ns = (t.1 as i32) - (t0.1 as i32);
        if dt_ns < 0 {
            dt_s -= 1;
            dt_ns += NS_IN_SEC as i32;
        }
        if dt_s < 0 { return; }
        assert!(dt_ns >= 0 && dt_ns < NS_IN_SEC as i32,
            "nanoseconds out of range");
        let p_s = dt_s as u64;
        let p_ns = dt_ns as u32;

        sleep(if p_s >= rt_s && p_ns > rt_ns {
            Duration::new(p_s - rt_s, p_ns - rt_ns )
        } else if p_s > rt_s && p_ns <= rt_ns {
            Duration::new(p_s - rt_s - 1, (NS_IN_SEC + p_ns) - rt_ns )
        } else {
            return;
        })
    }
}

impl PacketSource for PcapSource {
    fn next_packet(&mut self)
        -> io::Result<Option<(SocketAddrV4, &RawPacket)>>
    {
        match self.read_packet() {
            Ok((pos, addr)) => {
                let buf = self.file.get_ref();
                // we rely on `read_packet` to return correct `pos`
                debug_assert!(buf.len() > (pos as usize) + PACKET_SIZE);
                let packet = unsafe {
                    &*(buf.as_ref().as_ptr().offset(pos as isize)
                        as *const [u8; PACKET_SIZE])
                };
                Ok(Some((addr, packet)))
            },
            Err(ref e) if e.kind() == ErrorKind::UnexpectedEof => {
                if self.do_loop {
                    self.reset();
                    self.next_packet()
                } else {
                    Ok(None)
                }
            },
            Err(e) => Err(e),
        }
    }
}
