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
