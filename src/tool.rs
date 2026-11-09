//! Rotational tool envelopes. z=0 is the tool tip; +z points toward the spindle.
//! Validate/prepare once, then query analytic radii without allocation.
use serde::{Deserialize, Serialize};

pub(crate) fn valid_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-.:".contains(&b))
}

fn positive(value: f64) -> bool {
    value.is_finite() && (1e-6..=1e6).contains(&value)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum CutterShape {
    Flat,
    Ball,
    Bull {
        corner_radius_mm: f64,
    },
    Drill {
        point_angle_rad: f64,
    },
    Chamfer {
        included_angle_rad: f64,
        tip_diameter_mm: f64,
    },
    /// A plasma arc or laser beam: the diameter is the kerf width. It cuts
    /// through the whole sheet below the nozzle (the tip) wherever it moves
    /// while on; it never cuts above the tip.
    Beam,
    /// A touch probe: the diameter is the stylus ball's (its calibrated
    /// effective size), the shank is the stylus stem and the holder sections
    /// are the probe body. Its controlled point is the bottom of the ball.
    /// It never cuts: only probing operations use it, and its moves never
    /// change the stock. The cutting length is the whole ball (the envelope
    /// is the ball's lower half and a cylinder of its radius above), and
    /// the stem must be thinner than the ball.
    Probe,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Neck {
    pub diameter_mm: f64,
    pub length_mm: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Cutter {
    pub id: String,
    #[serde(default)]
    pub name: String,
    pub shape: CutterShape,
    pub diameter_mm: f64,
    pub cutting_length_mm: f64,
    pub overall_length_mm: f64,
    pub shank_diameter_mm: f64,
    pub flutes: u16,
    pub center_cutting: bool,
    #[serde(default)]
    pub material: String,
    #[serde(default)]
    pub neck: Option<Neck>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ProfileSlice {
    pub radius_mm: f64,
    pub cutting: bool,
}

pub struct PreparedCutter {
    definition: Cutter,
    radius: f64,
    tip_height: f64,
    slope: f64,
    shoulder: f64,
}

impl Cutter {
    pub fn prepare(&self) -> Result<PreparedCutter, String> {
        if !valid_id(&self.id) {
            return Err(
                "cutter.id must be a stable identifier (ASCII letters/digits/_-.:, 1..128 bytes)"
                    .into(),
            );
        }
        for (field, value) in [
            ("diameter_mm", self.diameter_mm),
            ("cutting_length_mm", self.cutting_length_mm),
            ("overall_length_mm", self.overall_length_mm),
            ("shank_diameter_mm", self.shank_diameter_mm),
        ] {
            if !positive(value) {
                return Err(format!(
                    "cutter.{}: {field} must be finite, in [1e-6, 1e6] mm",
                    self.id
                ));
            }
        }
        if self.flutes == 0 || self.flutes > 1000 {
            return Err("cutter.flutes must be in 1..=1000".into());
        }
        let neck_length = if let Some(neck) = &self.neck {
            if !positive(neck.diameter_mm)
                || !positive(neck.length_mm)
                || neck.diameter_mm > self.diameter_mm
                || neck.diameter_mm > self.shank_diameter_mm
            {
                return Err("neck must have positive finite dimensions and fit within cutter and shank diameters".into());
            }
            neck.length_mm
        } else {
            0.0
        };
        let shoulder = self.cutting_length_mm + neck_length;
        if shoulder >= self.overall_length_mm {
            return Err("cutting length plus neck must leave a positive shank length".into());
        }
        let radius = self.diameter_mm / 2.;
        let angle_slope = |angle: f64| -> Result<f64, String> {
            if !angle.is_finite() || !(0.001..std::f64::consts::PI - 0.001).contains(&angle) {
                return Err("tool included angle must be finite and strictly between 0.001 and pi-0.001 radians".into());
            }
            Ok((angle / 2.).tan())
        };
        if self.shape == CutterShape::Probe
            && (self.neck.is_some()
                || self.center_cutting
                || self.shank_diameter_mm >= self.diameter_mm
                || self.cutting_length_mm != self.diameter_mm)
        {
            return Err(format!(
                "probe {}: the stylus is a ball (cutting length = ball diameter) on a stem (shank) thinner than the ball, with no neck, not center cutting",
                self.id
            ));
        }
        let (tip_height, slope) = match self.shape {
            CutterShape::Flat | CutterShape::Beam => (0., 0.),
            CutterShape::Ball | CutterShape::Probe => (radius, 0.),
            CutterShape::Bull { corner_radius_mm } => {
                if !positive(corner_radius_mm) || corner_radius_mm > radius {
                    return Err(
                        "corner radius must be positive and no larger than cutter radius".into(),
                    );
                }
                (corner_radius_mm, 0.)
            }
            CutterShape::Drill { point_angle_rad } => {
                let slope = angle_slope(point_angle_rad)?;
                (radius / slope, slope)
            }
            CutterShape::Chamfer {
                included_angle_rad,
                tip_diameter_mm,
            } => {
                if !tip_diameter_mm.is_finite()
                    || !(0.0..self.diameter_mm).contains(&tip_diameter_mm)
                {
                    return Err("chamfer tip diameter must be finite, nonnegative and smaller than cutter diameter".into());
                }
                let slope = angle_slope(included_angle_rad)?;
                ((radius - tip_diameter_mm / 2.) / slope, slope)
            }
        };
        if tip_height > self.cutting_length_mm {
            return Err("cutting length does not contain the complete tool tip geometry".into());
        }
        Ok(PreparedCutter {
            definition: self.clone(),
            radius,
            tip_height,
            slope,
            shoulder,
        })
    }
}

impl PreparedCutter {
    pub fn definition(&self) -> &Cutter {
        &self.definition
    }
    /// Analytic cutting lower envelope at radial distance from the tool axis.
    /// This excludes shank/holder geometry and does not establish tool access.
    pub fn tip_height_at_radius(&self, radius_mm: f64) -> Option<f64> {
        if !radius_mm.is_finite() || !(0.0..=self.radius).contains(&radius_mm) {
            return None;
        }
        Some(match self.definition.shape {
            CutterShape::Flat | CutterShape::Beam => 0.,
            CutterShape::Ball | CutterShape::Probe => {
                self.radius
                    - ((self.radius - radius_mm) * (self.radius + radius_mm))
                        .max(0.)
                        .sqrt()
            }
            CutterShape::Bull {
                corner_radius_mm: c,
            } => {
                let r = (radius_mm - (self.radius - c)).max(0.);
                c - ((c - r) * (c + r)).max(0.).sqrt()
            }
            CutterShape::Drill { .. } => radius_mm / self.slope,
            CutterShape::Chamfer {
                tip_diameter_mm, ..
            } => (radius_mm - tip_diameter_mm / 2.).max(0.) / self.slope,
        })
    }
    pub fn radius_at(&self, z_mm: f64) -> Option<ProfileSlice> {
        if !z_mm.is_finite() || z_mm < 0. || z_mm > self.definition.overall_length_mm {
            return None;
        }
        if z_mm <= self.definition.cutting_length_mm {
            let radius_mm = if z_mm >= self.tip_height {
                self.radius
            } else {
                match self.definition.shape {
                    CutterShape::Flat | CutterShape::Beam => self.radius,
                    CutterShape::Ball | CutterShape::Probe => {
                        (z_mm * (2. * self.radius - z_mm)).max(0.).sqrt()
                    }
                    CutterShape::Bull {
                        corner_radius_mm: c,
                    } => self.radius - c + (z_mm * (2. * c - z_mm)).max(0.).sqrt(),
                    CutterShape::Drill { .. } => z_mm * self.slope,
                    CutterShape::Chamfer {
                        tip_diameter_mm, ..
                    } => tip_diameter_mm / 2. + z_mm * self.slope,
                }
            };
            Some(ProfileSlice {
                radius_mm,
                cutting: true,
            })
        } else {
            let diameter = if z_mm <= self.shoulder {
                self.definition
                    .neck
                    .as_ref()
                    .map_or(self.definition.shank_diameter_mm, |n| n.diameter_mm)
            } else {
                self.definition.shank_diameter_mm
            };
            Some(ProfileSlice {
                radius_mm: diameter / 2.,
                cutting: false,
            })
        }
    }
}

/// Tip-relative conical/cylindrical outer-envelope section, ordered tip to gauge.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HolderSection {
    pub z_min_mm: f64,
    pub z_max_mm: f64,
    pub radius_min_mm: f64,
    pub radius_max_mm: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolAssembly {
    pub id: String,
    pub cutter: Cutter,
    pub stickout_mm: f64,
    pub gauge_length_mm: f64,
    pub holders: Vec<HolderSection>,
}

pub struct PreparedAssembly {
    definition: ToolAssembly,
    cutter: PreparedCutter,
}

impl ToolAssembly {
    pub fn prepare(&self) -> Result<PreparedAssembly, String> {
        if !valid_id(&self.id) {
            return Err("tool assembly needs a stable id".into());
        }
        let cutter = self.cutter.prepare()?;
        if !positive(self.stickout_mm)
            || !positive(self.gauge_length_mm)
            || self.stickout_mm < cutter.shoulder
            || self.stickout_mm >= self.cutter.overall_length_mm
            || self.stickout_mm >= self.gauge_length_mm
        {
            return Err("stickout must expose cutting/neck regions, leave shank clamping length, and be shorter than gauge length".into());
        }
        if self.holders.is_empty() {
            return Err(
                "holder envelope required; missing holder geometry cannot be treated as clear"
                    .into(),
            );
        }
        let mut end = self.stickout_mm;
        for section in &self.holders {
            if section.z_min_mm != end
                || !positive(section.z_max_mm)
                || section.z_max_mm <= section.z_min_mm
                || !positive(section.radius_min_mm)
                || !positive(section.radius_max_mm)
            {
                return Err(
                    "holder sections must be finite, positive, contiguous and ordered tip to gauge"
                        .into(),
                );
            }
            end = section.z_max_mm;
        }
        if end < self.gauge_length_mm {
            return Err("holder envelope must reach the gauge plane".into());
        }
        // Every holder interval occupied by the inserted shank must enclose it.
        for s in &self.holders {
            if s.z_min_mm < self.cutter.overall_length_mm {
                let t = ((self.cutter.overall_length_mm.min(s.z_max_mm) - s.z_min_mm)
                    / (s.z_max_mm - s.z_min_mm))
                    .clamp(0., 1.);
                let r_end = s.radius_min_mm + t * (s.radius_max_mm - s.radius_min_mm);
                if s.radius_min_mm.min(r_end) < self.cutter.shank_diameter_mm / 2. {
                    return Err(
                        "holder outer envelope cannot be smaller than the inserted shank".into(),
                    );
                }
            }
        }
        Ok(PreparedAssembly {
            definition: self.clone(),
            cutter,
        })
    }
}

impl PreparedAssembly {
    pub fn cutter(&self) -> &PreparedCutter {
        &self.cutter
    }
    pub fn definition(&self) -> &ToolAssembly {
        &self.definition
    }
    pub fn radius_at(&self, z_mm: f64) -> Option<ProfileSlice> {
        if z_mm < self.definition.stickout_mm {
            return self.cutter.radius_at(z_mm);
        }
        if !z_mm.is_finite() {
            return None;
        }
        let radius = self
            .definition
            .holders
            .iter()
            .filter(|s| s.z_min_mm <= z_mm && z_mm <= s.z_max_mm)
            .map(|s| {
                s.radius_min_mm
                    + (z_mm - s.z_min_mm) / (s.z_max_mm - s.z_min_mm)
                        * (s.radius_max_mm - s.radius_min_mm)
            })
            .reduce(f64::max)?;
        Some(ProfileSlice {
            radius_mm: radius,
            cutting: false,
        })
    }
}
