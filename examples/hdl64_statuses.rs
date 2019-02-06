use velodyne::{packet, PointSource, FullPoint};
use velodyne::packet::PacketSource;
use velodyne::hdl64::Status;
use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;
use structopt::StructOpt;

#[derive(StructOpt)]
#[structopt(
    name = "hdl64 status listener",
    about = "Simple example of listening to HDL-64 sensor status")]
enum Cli {
    #[structopt(name = "pcap")]
    /// Replay packets from pcap file
    Pcap {
        /// Path to pcap file
        path: String,
        /// Loop playback
        #[structopt(long = "loop", short = "l")]
        loop_play: bool,
        /// Synchronize time
        #[structopt(long = "sync", short = "s")]
        sync: bool,
        /// Comma-separated list of fields to display
        ///
        /// Supported values: gps, temperature, version, dirty, hot, cold, pps,
        /// gps_time, rpm, fov_start, fov_end, rl_time, ip_source, ip_dest,
        /// return_type, power_level, humidity, noise_upper, noise_lower,
        /// calib_dt
        #[structopt(long = "fields", short = "f",
            default_value = "gps,hot,cold,dirty,temperature")]
        fields: String,
    },
    #[structopt(name = "udp")]
    /// Read packets from UDP port in real time
    Udp {
        /// UDP port to listen
        #[structopt(long = "port", short = "p", default_value = "2368")]
        port: u16,
        /// Capture timeout, seconds
        #[structopt(long = "timeout", short = "t")]
        timeout: Option<u64>,
        /// Comma-separated list of fields to display
        ///
        /// Supported values: gps, temperature, version, dirty, hot, cold, pps,
        /// gps_time, rpm, fov_start, fov_end, rl_time, ip_source, ip_dest,
        /// return_type, power_level, humidity, noise_upper, noise_lower,
        /// calib_dt
        #[structopt(long = "fields", short = "f",
            default_value = "gps,hot,cold,dirty,temperature")]
        fields: String,
    },
}

fn print_fields(status: &Status, fields: &str) {
    print!("[{:?}]", status.dt);
    for field in fields.split(',') {
        match field {
            "gps" => print!("\tgps: {:?}", status.gps),
            "temperature" => print!("\tt: {:?}", status.temperature),
            "version" => print!("\tver: {:?}", status.version),
            "dirty" => print!("\tdirty: {:?}", status.lens_contamination),
            "hot" => print!("\thot: {:?}", status.hot),
            "cold" => print!("\tcold: {:?}", status.cold),
            "pps" => print!("\tpps: {:?}", status.pps),
            "gps_time" => print!("\tgps_time: {:?}", status.gps_time),
            "rpm" => print!("\trpm: {:?}", status.rpm),
            "fov_start" => print!("\tfov_start: {:?}", status.fov_start),
            "fov_end" => print!("\tfov_end: {:?}", status.fov_end),
            "rl_time" => print!("\trl_time: {:?}", status.real_life_time),
            "ip_source" => print!("\tip_source: {:?}", status.ip_source),
            "ip_dest" => print!("\tip_dest: {:?}", status.ip_dest),
            "return_type" => print!("\treturn: {:?}", status.return_type),
            "power_level" => print!("\tpower_level: {:?}", status.power_level),
            "humidity" => print!("\thumidity: {:?}", status.humidity),
            "noise_upper" => print!("\tnoise_upper: {:?}", status.upper_threshold),
            "noise_lower" => print!("\tnoise_lower: {:?}", status.lower_threshold),
            "calib_dt" => print!("\tcalib_dt: {:?}", status.calib_dt),
            _ => print!("\t{}: ????", field),
        }
    }
    println!();
}

fn run<T: PacketSource>(packet_source: T, fields: &str) -> io::Result<()> {
    print!("Listener initialization... ");
    let mut point_source = PointSource::hdl64_init(packet_source)?;
    println!("Done.");

    let mut prev_dt = point_source.get_status().dt;
    loop {
        let (_, _meta) = match point_source.process_points(|_: FullPoint| {})? {
            Some(meta) => meta,
            None => break,
        };

        let status = point_source.get_status();
        if status.dt != prev_dt {
            print_fields(status, fields);
            prev_dt = status.dt;
        }
    }

    //println!("{:?}", point_source.get_calib_db());
    println!("Listener does not provide packets anymore.");
    Ok(())
}

fn main() -> io::Result<()> {
    let args = Cli::from_args();
    match args {
        Cli::Pcap{ path, loop_play, sync, fields } => {
            let source = packet::PcapSource::new(path, sync, loop_play)
                .expect("Failed to initialize pcap source");
            run(source, &fields)
        },
        Cli::Udp{ port, timeout, fields } => {
            let ip_addr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));
            let addr = SocketAddr::new(ip_addr, port);
            let timeout = timeout.map(|t| Duration::from_secs(t));
            let source = packet::UdpSource::new_custom(addr, timeout)
                .expect("Failed to initialize pcap source");
            run(source, &fields)
        },
    }
}
