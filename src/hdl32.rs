//! HDL-32E sensor types
use super::{FullPoint, ConversionError, Convertor};
use crate::packet::{RawPacket, PacketMeta, parse_packet};

const HDL_32_TABLE: [f32; 32] = [
    -30.67, -9.33, -29.33, -8.00, -28.00, -6.67, -26.67, -5.33,
    -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33, 0.00,
    -20.00,  1.33, -18.67,  2.67, -17.33,  4.00, -16.00, 5.33,
    -14.67,  6.67, -13.33,  8.00, -12.00,  9.33, -10.67, 10.67,
];

#[derive(Copy, Clone, Debug, Default)]
/// Default HDL-32E convertor from `RawPoint` to `FullPoint`
pub struct Hdl32Convertor;

impl Convertor for Hdl32Convertor {
    fn convert<F, P>(&self, raw_packet: &RawPacket, mut f: F)
        -> Result<PacketMeta, ConversionError>
        where F: FnMut(P), P: From<FullPoint>
    {
        let (meta, iter) = parse_packet(raw_packet);
        let timestamp = meta.timestamp;
        let mut cache = [0u16; 32];
        let mut prev_azimuth = std::u16::MAX;

        for (header, azimuth, block_iter) in iter {
            let azim_sin_cos = (azimuth as f32/100.).to_radians().sin_cos();
            if &header != b"\xFF\xEE" { Err(ConversionError)? }
            for raw_point in block_iter {
                let laser_id = raw_point.laser;

                // filter points for double-return mode
                let cached = &mut cache[laser_id as usize];
                if azimuth == prev_azimuth && *cached == raw_point.distance {
                    *cached = 0;
                    continue
                }
                *cached = raw_point.distance;

                let distance = (raw_point.distance as f32)/500.;
                let hor_angle = HDL_32_TABLE[laser_id as usize].to_radians();

                let xyz = compute_xyz(distance, azim_sin_cos, hor_angle);

                let intensity = raw_point.intensity;

                //  TODO: add timestamp deltas
                let point = FullPoint { xyz, intensity, laser_id, timestamp };
                f(point.into());
            }
            prev_azimuth = azimuth;
        }
        Ok(meta)
    }
}

fn compute_xyz(dist: f32, (a_sin, a_cos): (f32, f32), w: f32) -> [f32; 3] {
    // TODO: use precomputed table
    let (w_sin, w_cos) = w.sin_cos();
    let t = dist*w_cos;
    [
        t*a_sin,
        t*a_cos,
        dist*w_sin,
    ]
}
