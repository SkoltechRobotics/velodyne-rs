use std::{fmt, mem};

/// Laser calibration data
#[derive(Default, Clone, Debug)]
pub struct LaserCalib {
    pub min_intensity: u8,
    pub max_intensity: u8,

    pub rot_corr_sin: f32,
    pub rot_corr_cos: f32,
    pub vert_corr_sin: f32,
    pub vert_corr_cos: f32,

    pub dist_correction: f32,
    pub dist_corr_x: f32,
    pub dist_corr_y: f32,
    pub vert_offset: f32,
    pub horiz_offset: f32,
    pub focal_dist: f32,
    pub focal_slope: f32,

    //pub color: (f32, f32, f32),
}

/// Sensor calibration data
#[derive(Clone)]
pub struct CalibDb {
    pub dist_lsb: f32,
    pub lasers: [LaserCalib; 64]
}

impl Default for CalibDb {
    fn default() -> Self {
        let mut lasers: [LaserCalib; 64] = unsafe { mem::uninitialized() };
        for l in lasers.iter_mut() { *l = Default::default(); }
        CalibDb {dist_lsb: 0., lasers }
    }
}

impl fmt::Debug for CalibDb {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "CalibDb")?;
        write!(f, "\tI_max\tI_min\ta_rot\ta_vert\tdist\t")?;
        writeln!(f, "d_x\td_y\tv_off\th_off\tf_dist\tf_slope")?;
        for (i, l) in self.lasers.iter().enumerate() {
            let a_rot = l.rot_corr_sin.asin().to_degrees();
            let a_vert = l.vert_corr_sin.asin().to_degrees();
            writeln!(f,
                "{}\t{}\t{}\t{:.2}\t{:.2}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                i, l.min_intensity, l.max_intensity, a_rot, a_vert,
                l.dist_correction,
                l.dist_corr_x, l.dist_corr_y, l.vert_offset, l.horiz_offset,
                l.focal_dist, l.focal_slope
            )?;

        }
        Ok(())
    }
}
