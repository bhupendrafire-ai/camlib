//! Lower envelopes from source-backed trimmed planar face interiors.
//! Region boundary errors and plane placement error are eroded analytically at
//! query time. The CAD/source inclusion is explicit, not inferred from a mesh.
use super::*;
use crate::motion::WorkFrame;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InnerPlanarPatch {
    pub region: PlanarRegion,
    /// The nominal face is local Z=0. Axes/origin are in target coordinates.
    pub frame: WorkFrame,
    /// Orthogonal plane-placement error, converted conservatively to Z error.
    pub normal_error_mm: f64,
}
struct Patch {
    region: PreparedRegion,
    frame: WorkFrame,
    normal_error: f64,
    boundary_guard: f64,
    segments: u64,
}
pub(super) struct PreparedPatches {
    patches: Vec<Patch>,
}
impl PreparedPatches {
    /// The boundary segments of all patches (the most one cell charges).
    pub(super) fn segments(&self) -> u64 {
        self.patches.iter().map(|p| p.segments).sum()
    }
    pub(super) fn new(
        definitions: &[InnerPlanarPatch],
        options: Options,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let mut patches = Vec::with_capacity(definitions.len());
        let mut segments = 0;
        for definition in definitions {
            cancelled(cancel)?;
            definition.frame.validate()?;
            if !definition.normal_error_mm.is_finite()
                || !(0. ..=0.1).contains(&definition.normal_error_mm)
                || definition.frame.z[2].abs() <= 1e-12
            {
                return Err(
                    "inner patch requires a nonvertical plane and finite normal error in[0,.1]"
                        .into(),
                );
            }
            let region = definition.region.prepare(ConstructionOptions {
                tolerance_mm: options.geometry_tolerance_mm,
            })?;
            let summary = region.summary();
            segments += summary.segments;
            if segments > 262144 {
                return Err("inner patches exceed 262144 total boundary segments".into());
            }
            patches.push(Patch {
                region,
                frame: definition.frame.clone(),
                normal_error: definition.normal_error_mm,
                boundary_guard: summary.boundary_error_mm
                    + summary.numerical_guard_mm
                    + options.geometry_tolerance_mm,
                segments: summary.segments as u64,
            });
        }
        Ok(Self { patches })
    }

    /// Each candidate lower height is valid over the ENTIRE XY rectangle.
    /// Taking their max retains validity even for overlapping source faces.
    /// A mixed/near-boundary footprint contributes no invented inner height.
    pub(super) fn lower_height(
        &self,
        xy: [f64; 4],
        mut lower: f64,
        arithmetic: f64,
        budget: (&mut u64, u64),
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        let (work, limit) = budget;
        for patch in &self.patches {
            cancelled(cancel)?;
            let f = &patch.frame;
            // WorkFrame permits orthogonality residuals up to 1e-12. Include
            // those and translated-coordinate arithmetic in plane projection,
            // not only the stock query's local arithmetic guard.
            let frame_guard = arithmetic
                + 16e-12
                    * (1.
                        + f.origin_mm.iter().map(|v| v.abs()).sum::<f64>()
                        + xy.iter().map(|v| v.abs()).sum::<f64>());
            let height_error = (patch.normal_error + frame_guard) / f.z[2].abs();
            let mut uv = [[0.; 2]; 4];
            let mut zmin = f64::INFINITY;
            let mut zmax = f64::NEG_INFINITY;
            for (index, (x, y)) in [
                (xy[0], xy[1]),
                (xy[0], xy[3]),
                (xy[2], xy[1]),
                (xy[2], xy[3]),
            ]
            .into_iter()
            .enumerate()
            {
                let dx = x - f.origin_mm[0];
                let dy = y - f.origin_mm[1];
                let dz = -(f.z[0] * dx + f.z[1] * dy) / f.z[2];
                let z = f.origin_mm[2] + dz;
                zmin = zmin.min(z);
                zmax = zmax.max(z);
                uv[index] = [
                    f.x[0] * dx + f.x[1] * dy + f.x[2] * dz,
                    f.y[0] * dx + f.y[1] * dy + f.y[2] * dz,
                ];
            }
            if zmax - height_error <= lower || patch.segments == 0 {
                continue;
            }
            // Affine projection of the complete XY box is a parallelogram.
            // Its circumdisk encloses every projected point, including edges.
            let center = [(uv[0][0] + uv[3][0]) * 0.5, (uv[0][1] + uv[3][1]) * 0.5];
            if center.iter().any(|v| !v.is_finite() || v.abs() > 1e6) {
                continue;
            }
            let radius = uv
                .iter()
                .map(|p| (p[0] - center[0]).hypot(p[1] - center[1]))
                .fold(0., f64::max);
            let guard = patch.boundary_guard
                + frame_guard
                + 16e-12 * zmin.abs().max(zmax.abs())
                + height_error * f.x[2].hypot(f.y[2]);
            if patch.segments > limit.saturating_sub(*work) {
                return Ok(None);
            }
            *work += patch.segments;
            if patch
                .region
                .query(center)?
                .signed_clearance_mm
                .is_some_and(|d| d > radius + guard)
            {
                lower = lower.max(zmin - height_error - arithmetic);
            }
        }
        Ok(Some(lower))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn exhausted_work_never_queries_a_patch_and_cancellation_precedes_geometry() {
        let patch = InnerPlanarPatch {
            region: serde_json::from_value(serde_json::json!({
                "version":1,"units":"mm","name":"face","boundary_error_mm":0,
                "components":[{"outer":{"vertices":[[-5,-5,0],[5,-5,0],[5,5,0],[-5,5,0]]}}]
            }))
            .unwrap(),
            frame: WorkFrame {
                origin_mm: [0., 0., 2.],
                ..WorkFrame::identity()
            },
            normal_error_mm: 0.,
        };
        let prepared =
            PreparedPatches::new(&[patch], Options::default(), &AtomicBool::new(false)).unwrap();
        let mut work = 0;
        assert_eq!(
            prepared
                .lower_height(
                    [0., 0., 1., 1.],
                    0.,
                    1e-9,
                    (&mut work, 3),
                    &AtomicBool::new(false)
                )
                .unwrap(),
            None
        );
        assert_eq!(work, 0);
        let height = prepared
            .lower_height(
                [0., 0., 1., 1.],
                0.,
                1e-9,
                (&mut work, 4),
                &AtomicBool::new(false),
            )
            .unwrap()
            .unwrap();
        assert!((1.9999..2.).contains(&height));
        assert_eq!(work, 4);
        assert!(
            prepared
                .lower_height(
                    [0., 0., 1., 1.],
                    0.,
                    1e-9,
                    (&mut work, 100),
                    &AtomicBool::new(true)
                )
                .is_err()
        );
        assert_eq!(work, 4);
    }
}
