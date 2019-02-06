extern crate velodyne;

use std::error;

use velodyne::packet::{PcapSource, PacketSource, parse_packet};

fn main() -> Result<(), Box<error::Error>>{
    let path = std::env::args().nth(1).expect("provide path to pcap file");
    let mut source = PcapSource::new(path, false, false)?;
    loop {
        let (meta, _) = match source.next_packet()? {
            Some((_, raw_packet)) => parse_packet(raw_packet),
            None => break,
        };
        let status = meta.status;
        println!("{}\t{}\t{}", status.id, status.value, status.id as char);
    }
    Ok(())
}
