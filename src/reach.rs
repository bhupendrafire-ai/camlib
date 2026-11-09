//! Typed reach refusals: why a tool assembly cannot reach the depth an
//! operation needs, as data rather than prose (issue #129).
//!
//! Planning errors travel as text (through `Result<_, String>` and the
//! stream's stderr). A reach refusal keeps its human message unchanged and
//! appends one line, [`MARKER`] followed by the refusal as JSON, so callers
//! can recover it with [`ReachRefusal::parse`] (Rust) or as
//! `camlib.ReachRefused.refusal` (Python) instead of parsing the prose.
use serde::{Deserialize, Serialize};

/// Starts the machine-readable line a reach refusal appends to its message.
pub const MARKER: &str = "camlib-reach-refusal: ";

/// The part of the tool assembly whose reach was exceeded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReachLimit {
    /// The flutes: material would stand above the cutting length.
    CuttingLength,
    /// The reduced neck above the flutes.
    Neck,
    /// The shank above the flutes (or the neck).
    Shank,
    /// A holder section (`holder_section` gives which).
    Holder,
}

/// One reach refusal. Depths are measured down from `top_z_mm`, the top of
/// the material the limit must clear (the operation's top for the cutting
/// length; the obstruction's top for a neck, shank or holder section), to
/// `z_mm`, the lowest tip Z the operation needs: `depth_needed_mm` is
/// `top_z_mm - z_mm`. `depth_available_mm` is how far below that material
/// top the tip can go before the limiting part meets it (the cutting length,
/// or the height of the part's bottom above the tip).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReachRefusal {
    pub operation: String,
    pub tool: String,
    pub limit: ReachLimit,
    pub holder_section: Option<usize>,
    pub depth_needed_mm: f64,
    pub depth_available_mm: f64,
    pub top_z_mm: f64,
    pub z_mm: f64,
}

impl ReachRefusal {
    pub fn cutting_length(
        operation: &str,
        tool: &str,
        top_z_mm: f64,
        z_mm: f64,
        cutting_length_mm: f64,
    ) -> Self {
        Self {
            operation: operation.into(),
            tool: tool.into(),
            limit: ReachLimit::CuttingLength,
            holder_section: None,
            depth_needed_mm: top_z_mm - z_mm,
            depth_available_mm: cutting_length_mm,
            top_z_mm,
            z_mm,
        }
    }
    /// The error text: `message`, then the marker line with this refusal.
    /// A refusal that cannot be encoded (a non-finite value) keeps the
    /// message alone rather than inventing values.
    pub fn error(&self, message: impl std::fmt::Display) -> String {
        let finite = [
            self.depth_needed_mm,
            self.depth_available_mm,
            self.top_z_mm,
            self.z_mm,
        ]
        .iter()
        .all(|v| v.is_finite());
        match serde_json::to_string(self) {
            Ok(json) if finite => format!("{message}\n{MARKER}{json}"),
            _ => message.to_string(),
        }
    }
    /// The refusal carried by an error text, if any (the last marker wins).
    pub fn parse(error: &str) -> Option<Self> {
        let at = error.rfind(MARKER)? + MARKER.len();
        let mut values = serde_json::Deserializer::from_str(&error[at..]).into_iter::<Self>();
        values.next()?.ok()
    }
}

/// A tool part above the flutes (neck, shank or a holder section).
#[derive(Clone, Copy)]
struct Section {
    limit: ReachLimit,
    holder: Option<usize>,
    /// Its bottom and top, height above the tip.
    bottom: f64,
    top: f64,
    radius: crate::verification::Radius,
}
impl Section {
    /// Its largest radius from its bottom up to `height` above the tip
    /// (profiles are constant or linear, so the ends bound it).
    fn radius_to(&self, height: f64) -> f64 {
        self.radius
            .at(self.bottom)
            .max(self.radius.at(height.clamp(self.bottom, self.top)))
    }
    fn name(&self) -> String {
        match self.holder {
            Some(h) => format!("holder section {h}"),
            None => format!("{:?}", self.limit).to_lowercase(),
        }
    }
}

/// The depth a feed move actually cuts (issue #161, as #88 for parallel
/// finishing): where the remaining stock stands higher above an operation's
/// lowest level than the flutes reach, an operation is admitted move by move
/// instead of by the stock top. Every section above the flutes must stay
/// above the committed stock under it over the whole move, judged at the
/// move's lowest tip; otherwise the move is refused with a typed
/// [`ReachRefusal`] naming the section, the obstruction's top and the tip Z.
///
/// Whole-cell bounds keep it conservative: a cell counts when its upper
/// bound stands above a section's bottom and some point of it may lie within
/// the section's radius of the tool axis anywhere on the move (the distance
/// from the cell centre to the path, exact for lines and XY arcs and helices,
/// less the cell's half diagonal). The flutes' own reach stays with the stock
/// sweep, which refuses stock above the cutting length within the cutter.
pub(crate) struct MoveDepth {
    operation: String,
    tool: String,
    cutting_length: f64,
    sections: Vec<Section>,
    tolerance: f64,
}
impl MoveDepth {
    pub(crate) fn new(
        operation: &str,
        tool: &crate::tool::ToolAssembly,
        tolerance: f64,
    ) -> Result<Self, String> {
        use crate::verification::ToolPart;
        let prepared = tool.prepare()?;
        let sections = crate::verification::parts(&prepared)?
            .into_iter()
            .filter_map(|p| {
                let limit = match p.kind {
                    ToolPart::Cutter => return None,
                    ToolPart::Neck => ReachLimit::Neck,
                    ToolPart::Shank => ReachLimit::Shank,
                    ToolPart::Holder => ReachLimit::Holder,
                };
                Some(Section {
                    limit,
                    holder: p.holder,
                    bottom: p.min,
                    top: p.max,
                    radius: p.radius,
                })
            })
            .collect();
        Ok(Self {
            operation: operation.into(),
            tool: tool.id.clone(),
            cutting_length: tool.cutter.cutting_length_mm,
            sections,
            tolerance,
        })
    }
    /// Refuse `motion` if a section above the flutes would meet the stock
    /// committed before it. Rapids are left to the clearance heights and
    /// the stream's own checks.
    pub(crate) fn check(
        &self,
        motion: &crate::motion::Motion,
        stock: &crate::stock::Stock,
        cancel: &impl crate::stock::CancelSignal,
    ) -> Result<(), String> {
        if motion.kind.is_rapid() || self.sections.is_empty() {
            return Ok(());
        }
        let path = motion.curve.prepare(self.tolerance)?;
        let bounds = path.bounds();
        let tip = bounds.min[2];
        let guard = 16. * self.tolerance + crate::verification::path_error(motion, self.tolerance);
        let axis = crate::verification::AxisSpan::new(motion, &path, [0., 1.]);
        // A lower bound on the XY distance from `c` to the path.
        let distance = |c: [f64; 2]| match &axis {
            Some(axis) => axis.distance(c),
            None => ((bounds.min[0] - c[0]).max(c[0] - bounds.max[0]).max(0.))
                .hypot((bounds.min[1] - c[1]).max(c[1] - bounds.max[1]).max(0.)),
        };
        let spec = stock.spec();
        let [dx, dy] = stock.cell_widths_mm();
        let [nx, ny] = stock.dimensions();
        let half = dx.hypot(dy) / 2.;
        let reach = self
            .sections
            .iter()
            .map(|s| s.radius.at(s.bottom).max(s.radius.at(s.top)))
            .fold(0., f64::max);
        let lowest = self
            .sections
            .iter()
            .map(|s| s.bottom)
            .fold(f64::INFINITY, f64::min);
        let pad = reach + half + 2. * guard;
        let index = |v: f64, k: usize, n: usize, w: f64| {
            (((v - spec.min_mm[k]) / w).floor().max(0.) as usize).min(n)
        };
        let x0 = index(bounds.min[0] - pad, 0, nx, dx);
        let y0 = index(bounds.min[1] - pad, 1, ny, dy);
        let x1 = (index(bounds.max[0] + pad, 0, nx, dx) + 1).min(nx);
        let y1 = (index(bounds.max[1] + pad, 1, ny, dy) + 1).min(ny);
        let mut blocked: Vec<Option<f64>> = vec![None; self.sections.len()];
        for y in y0..y1 {
            if cancel.is_cancelled() {
                return Err(format!("{}: reach check cancelled", self.operation));
            }
            for x in x0..x1 {
                let upper = stock.cells()[y * nx + x].upper_mm;
                let above = upper - tip;
                if upper <= spec.min_mm[2] || above <= lowest - guard {
                    continue;
                }
                let c = [
                    spec.min_mm[0] + (x as f64 + 0.5) * dx,
                    spec.min_mm[1] + (y as f64 + 0.5) * dy,
                ];
                let near = distance(c) - half - guard;
                for (k, s) in self.sections.iter().enumerate() {
                    if above > s.bottom - guard && near < s.radius_to(above) + guard {
                        blocked[k] = Some(blocked[k].map_or(upper, |h: f64| h.max(upper)));
                    }
                }
            }
        }
        let Some((s, top)) = self
            .sections
            .iter()
            .zip(&blocked)
            .find_map(|(s, h)| h.map(|h| (s, h)))
        else {
            return Ok(());
        };
        let refusal = ReachRefusal {
            operation: self.operation.clone(),
            tool: self.tool.clone(),
            limit: s.limit,
            holder_section: s.holder,
            depth_needed_mm: top - tip,
            depth_available_mm: s.bottom,
            top_z_mm: top,
            z_mm: tip,
        };
        let at = motion.curve.end();
        let part = s.name();
        Err(refusal.error(format_args!(
            "{}: the pass at X{:.3} Y{:.3} Z{:.3} cuts deeper than the cutting length {}: the tool {} {part} (from {} mm above the tip) would meet remaining stock up to Z{top:.3}, {:.3} mm above the tip; remove that stock first (roughing), or use a tool whose flutes reach deeper or whose {part} stands higher or is slimmer",
            self.operation,
            at[0],
            at[1],
            tip,
            self.cutting_length,
            self.tool,
            s.bottom,
            top - tip
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refusal_round_trips_through_wrapped_error_text() {
        let r = ReachRefusal::cutting_length("pocket-1", "em6", 8., 4., 3.);
        let text = format!(
            "invalid pocket parameters: {}",
            r.error("pocket depth 4 exceeds")
        );
        assert!(text.starts_with("invalid pocket parameters: pocket depth 4 exceeds\n"));
        let parsed = ReachRefusal::parse(&text).unwrap();
        assert_eq!(parsed, r);
        assert_eq!(parsed.depth_needed_mm, 4.);
        // Text appended after the JSON does not hide it.
        assert_eq!(ReachRefusal::parse(&format!("{text}; then more")), Some(r));
        assert_eq!(ReachRefusal::parse("pocket depth 4 exceeds"), None);
        let bad = ReachRefusal::cutting_length("p", "t", f64::NAN, 0., 1.);
        assert_eq!(bad.error("plain"), "plain");
    }
}
