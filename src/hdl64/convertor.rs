use super::super::{FullPoint, ConversionError, Convertor};
use super::{CalibDb, LaserCalib};
use crate::packet::{RawPacket, PacketMeta, parse_packet};

#[inline(always)]
fn calib_intensity(intens: u8, raw_distance: u16, calib: &LaserCalib) -> u8 {
    let t1 = 1. - calib.focal_dist/13_100.;
    let t2 = 1. - (raw_distance as f32)/65_535.;
    let t3 = t1*t1 - t2*t2;
    let intens = intens.saturating_sub(calib.min_intensity) as f32;
    let res = intens + 256.*calib.focal_slope*t3.abs();
    match res {
        r if r > 255. => 255,
        r if r < 0. => 0,
        r => r as u8,
    }
}

// azimuth in radians
// distance is already multiplied by lsb
#[inline(always)]
fn compute_xyz(distance: f32, azim_sin_cos: (f32, f32), calib: &LaserCalib)
    -> [f32; 3]
{
    let cal_distance = distance + calib.dist_correction;

    let (sin, cos) = azim_sin_cos;
    let cos = cos*calib.rot_corr_cos + sin*calib.rot_corr_sin;
    let sin = sin*calib.rot_corr_cos - cos*calib.rot_corr_sin;

    // correction computation
    let xy_dist = cal_distance * calib.vert_corr_cos -
        calib.vert_offset * calib.vert_corr_sin;
    let xx = (xy_dist * sin - calib.horiz_offset * cos).abs();
    let yy = (xy_dist * cos + calib.horiz_offset * sin).abs();
    let (d_corr_x, d_corr_y) = if cal_distance > 2500. {
        (calib.dist_correction, calib.dist_correction)
    } else {
        let dx = calib.dist_correction - calib.dist_corr_x;
        let dy = calib.dist_correction - calib.dist_corr_y;
        (
            dx*(xx - 240.)/(2504. - 240.) + calib.dist_corr_x,
            dy*(yy - 193.)/(2504. - 193.) + calib.dist_corr_y,
        )
    };

    let xy_dist = (distance + d_corr_x) * calib.vert_corr_cos -
        calib.vert_offset * calib.vert_corr_sin;
    let x = xy_dist * sin - calib.horiz_offset * cos;

    let xy_dist = (distance + d_corr_y) * calib.vert_corr_cos -
        calib.vert_offset * calib.vert_corr_sin;
    let y = xy_dist * cos + calib.horiz_offset * sin;

    let z = cal_distance * calib.vert_corr_sin +
        calib.vert_offset * calib.vert_corr_cos;

    [x/100., y/100., z/100.]
}

/// HDL-64 convertor from `RawPoint` to `FullPoint`
pub struct Hdl64Convertor {
    pub(crate) db: CalibDb,
}

impl Hdl64Convertor {
    pub fn new(db: CalibDb) -> Self { Self { db } }
}


impl<'a> Convertor for Hdl64Convertor {
    fn convert<F, P>(&self, raw_packet: &RawPacket, mut f: F)
        -> Result<PacketMeta, ConversionError>
        where F: FnMut(P), P: From<FullPoint>
    {
        let (meta, iter) = parse_packet(raw_packet);
        let timestamp = meta.timestamp;

        let mut cache = [0u16; 64];
        let mut prev_azimuth = std::u16::MAX;

        for (header, azimuth, block_iter) in iter {
            let azim_sin_cos = (azimuth as f32/100.).to_radians().sin_cos();
            let laser_delta = match &header {
                b"\xFF\xEE" => 0,
                b"\xFF\xDD" => 32,
                _ => return Err(ConversionError),
            };
            for raw_point in block_iter {
                let laser_id = raw_point.laser + laser_delta;

                // filter points for double-return mode
                let cached = &mut cache[laser_id as usize];
                if azimuth == prev_azimuth && *cached == raw_point.distance {
                    *cached = 0;
                    continue
                }
                *cached = raw_point.distance;

                let distance = raw_point.distance as f32 * self.db.dist_lsb;
                let calib = &self.db.lasers[laser_id as usize];

                let xyz = compute_xyz(distance, azim_sin_cos, calib);

                let intensity = calib_intensity(
                    raw_point.intensity,
                    raw_point.distance,
                    calib,
                );

                //  TODO: add timestamp deltas
                let point = FullPoint { xyz, intensity, laser_id, timestamp };
                f(point.into());
            }
            prev_azimuth = azimuth;
        }
        Ok(meta)
    }
}
