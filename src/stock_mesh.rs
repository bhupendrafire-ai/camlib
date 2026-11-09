//! Stock given as a closed triangle mesh, and stock returned as one (#121).
//!
//! A milling or thermal run's stock is a height field (`crate::stock::Stock`).
//! A mesh stock is converted to it conservatively: over each cell, the upper
//! bound is the highest point of the mesh above the cell, and the lower bound
//! (material certainly there, from the stock's bottom up) is only claimed
//! where the whole cell column is solid from the grid's bottom plane. Material
//! the height field cannot represent (an undercut, a void, a hole seen from
//! below) is counted as possibly there: it is never dropped. That overstates
//! material only where a three-axis tool coming from above cannot tell the
//! difference without first cutting what lies over it.
//!
//! The finished stock goes back out as a closed mesh of each cell's upper
//! bound ([`Stock::finished_mesh`]). That mesh contains all the material that
//! may remain, and is stated as approximate. The 3D stock model (#1) can return
//! an exact mesh through the same interface.
use crate::{
    motion::Vec3,
    resources::check_memory,
    stock::{CancelSignal, HeightBounds, Stock, StockSpec},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{collections::HashMap, sync::Arc};

/// What a height-field stock's mesh is, stated wherever the mesh is produced.
pub const HEIGHT_FIELD_APPROXIMATION: &str = "approximate: the height-field stock model \
    returns each cell as a solid column from the stock's bottom up to the highest material \
    that may remain over the cell. The mesh contains all the material that may remain, but \
    also everything below it in each column (undercuts and voids are filled) and up to one \
    cell of removed material beside walls. It is conservative stock for a later setup, not \
    a measured or verified part.";

/// A stock solid as a closed, outward-oriented triangle mesh (mm), in the
/// frame of the job it is given to. Clones share the geometry.
#[derive(Clone, Debug)]
pub struct StockMesh(Arc<MeshData>);
#[derive(Debug)]
struct MeshData {
    vertices: Vec<Vec3>,
    indices: Vec<[u32; 3]>,
    bounds: [Vec3; 2],
    volume_mm3: f64,
    digest: [u8; 32],
}
impl PartialEq for StockMesh {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.0.digest == other.0.digest
    }
}

impl StockMesh {
    /// Check and keep a mesh given as vertices and triangles (vertex indices,
    /// counter-clockwise seen from outside). Refuses a mesh that is not a
    /// closed, consistently and outwardly oriented surface: every edge must be
    /// matched by an edge running the other way (vertices are matched by exact
    /// coordinates), and the enclosed volume must be positive.
    pub fn new(vertices: Vec<Vec3>, indices: Vec<[u32; 3]>) -> Result<Self, String> {
        if vertices.is_empty() || indices.is_empty() {
            return Err("a stock mesh needs vertices and at least one triangle".into());
        }
        if vertices.len() > u32::MAX as usize {
            return Err("a stock mesh has at most 2^32 - 1 vertices".into());
        }
        check_memory(
            vertices.len() as u128 * 48 + indices.len() as u128 * 64,
            "stock mesh",
        )?;
        if let Some(v) = vertices
            .iter()
            .find(|v| !v.iter().all(|c| c.is_finite() && c.abs() <= 1e6))
        {
            return Err(format!(
                "stock mesh vertex {v:?} is not a finite coordinate within ±1e6 mm"
            ));
        }
        if indices
            .iter()
            .flatten()
            .any(|&i| i as usize >= vertices.len())
        {
            return Err("stock mesh triangle refers to a vertex that does not exist".into());
        }
        // Match vertices by exact coordinates (signed zeros are one point).
        let mut weld = HashMap::with_capacity(vertices.len());
        let welded: Vec<u32> = vertices
            .iter()
            .map(|v| {
                let key = v.map(|c| (c + 0.).to_bits());
                let next = weld.len() as u32;
                *weld.entry(key).or_insert(next)
            })
            .collect();
        let mut edges: HashMap<(u32, u32), i64> = HashMap::with_capacity(indices.len() * 2);
        for t in &indices {
            for k in 0..3 {
                let (a, b) = (welded[t[k] as usize], welded[t[(k + 1) % 3] as usize]);
                if a != b {
                    *edges.entry((a.min(b), a.max(b))).or_default() += if a < b { 1 } else { -1 };
                }
            }
        }
        let open = edges.values().filter(|n| **n != 0).count();
        if open > 0 {
            return Err(format!(
                "stock mesh is not closed: {open} edge(s) are not matched by an edge running the \
                 other way (an open boundary, a gap, or triangles facing different ways); give a \
                 closed solid with every triangle counter-clockwise seen from outside"
            ));
        }
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for v in &vertices {
            for k in 0..3 {
                bounds[0][k] = bounds[0][k].min(v[k]);
                bounds[1][k] = bounds[1][k].max(v[k]);
            }
        }
        let c: Vec3 = std::array::from_fn(|k| (bounds[0][k] + bounds[1][k]) / 2.);
        let volume_mm3 = indices
            .iter()
            .map(|t| {
                let [a, b, d] = t.map(|i| sub(vertices[i as usize], c));
                dot(a, cross(b, d))
            })
            .sum::<f64>()
            / 6.;
        if volume_mm3.is_nan() || volume_mm3 <= 0. {
            return Err(format!(
                "stock mesh encloses no positive volume ({volume_mm3} mm³): it is inside out \
                 (triangles facing inward) or flat; give triangles counter-clockwise seen from \
                 outside"
            ));
        }
        let mut hash = Sha256::new();
        hash.update(b"camlib-stock-mesh-v1");
        hash.update((vertices.len() as u64).to_le_bytes());
        for v in &vertices {
            for c in v {
                hash.update(c.to_le_bytes());
            }
        }
        for t in &indices {
            for i in t {
                hash.update(i.to_le_bytes());
            }
        }
        Ok(Self(Arc::new(MeshData {
            vertices,
            indices,
            bounds,
            volume_mm3,
            digest: hash.finalize().into(),
        })))
    }
    /// A mesh given as separate triangles (vertices matched by exact
    /// coordinates).
    pub fn from_triangles(triangles: &[[Vec3; 3]]) -> Result<Self, String> {
        let mut lookup = HashMap::new();
        let mut vertices = Vec::new();
        let indices = triangles
            .iter()
            .map(|t| {
                t.map(|p| {
                    *lookup.entry(p.map(f64::to_bits)).or_insert_with(|| {
                        vertices.push(p);
                        (vertices.len() - 1) as u32
                    })
                })
            })
            .collect();
        Self::new(vertices, indices)
    }
    pub fn vertices(&self) -> &[Vec3] {
        &self.0.vertices
    }
    pub fn indices(&self) -> &[[u32; 3]] {
        &self.0.indices
    }
    /// Smallest and largest coordinates of the vertices.
    pub fn bounds(&self) -> [Vec3; 2] {
        self.0.bounds
    }
    /// The enclosed volume.
    pub fn volume_mm3(&self) -> f64 {
        self.0.volume_mm3
    }
    fn triangle(&self, t: usize) -> [Vec3; 3] {
        self.0.indices[t].map(|i| self.0.vertices[i as usize])
    }

    /// Conservative height bounds of this solid on `spec`'s grid (see the
    /// module documentation). The mesh must lie inside the grid's box.
    pub(crate) fn height_cells(
        &self,
        spec: &StockSpec,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<HeightBounds>, String> {
        let (nx, ny, step) = spec.grid()?;
        let count = nx * ny;
        check_memory(count as u128 * 40, "mesh stock conversion")?;
        let base = spec.min_mm[2];
        let top = spec.max_mm[2];
        let edge = |k: usize, i: usize| cell_edge(spec, [nx, ny], step, k, i);
        // Interpolated heights carry rounding of a few units in the last
        // place: bounds are widened by that much (horizontal facets are exact).
        let margin = 8. * f64::EPSILON * base.abs().max(top.abs()).max(1.);
        let mut upper = vec![f64::NEG_INFINITY; count];
        let mut lowest = vec![f64::INFINITY; count];
        let mut bottom_area = vec![0.; count];
        let range = |k: usize, lo: f64, hi: f64, n: usize| {
            let a = ((lo - spec.min_mm[k]) / step[k]).floor() - 1.;
            let b = ((hi - spec.min_mm[k]) / step[k]).floor() + 1.;
            (a.max(0.) as usize, b.clamp(0., (n - 1) as f64) as usize)
        };
        for t in 0..self.0.indices.len() {
            if t % 1024 == 0 && cancel.is_cancelled() {
                return Err("mesh stock conversion cancelled".into());
            }
            let p = self.triangle(t);
            let flat = p[0][2] == p[1][2] && p[1][2] == p[2][2];
            let on_bottom = flat && p[0][2] == base;
            let pad = if flat { 0. } else { margin };
            let lo = [0, 1].map(|k| p.iter().map(|v| v[k]).fold(f64::INFINITY, f64::min));
            let hi = [0, 1].map(|k| p.iter().map(|v| v[k]).fold(f64::NEG_INFINITY, f64::max));
            let (i0, i1) = range(0, lo[0], hi[0], nx);
            let (j0, j1) = range(1, lo[1], hi[1], ny);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let rect = [edge(0, i), edge(0, i + 1), edge(1, j), edge(1, j + 1)];
                    let polygon = clip(p, rect);
                    if polygon.is_empty() {
                        continue;
                    }
                    let c = j * nx + i;
                    if on_bottom {
                        // A facet on the bottom plane facing down has a
                        // negative counter-clockwise area seen from above.
                        bottom_area[c] -= signed_area(&polygon);
                        continue;
                    }
                    // Facets meeting the cell only along one of its sides
                    // belong to the neighbouring cell.
                    if (0..4).any(|s| polygon.iter().all(|v| v[s / 2] == rect[s])) {
                        continue;
                    }
                    for v in &polygon {
                        upper[c] = upper[c].max(v[2] + pad);
                        lowest[c] = lowest[c].min(v[2] - pad);
                    }
                }
            }
        }
        let mut cells = Vec::with_capacity(count);
        for c in 0..count {
            if c % 65536 == 0 && cancel.is_cancelled() {
                return Err("mesh stock conversion cancelled".into());
            }
            if upper[c] == f64::NEG_INFINITY {
                cells.push(HeightBounds {
                    lower_mm: base,
                    upper_mm: base,
                });
                continue;
            }
            let (i, j) = (c % nx, c / nx);
            let area = (edge(0, i + 1) - edge(0, i)) * (edge(1, j + 1) - edge(1, j));
            let upper_mm = upper[c].clamp(base, top);
            // Certain material from the bottom up: no surface inside the
            // column between the bottom plane and `lowest`, so that open box
            // is all solid or all empty; it is solid when facets on the
            // bottom plane close it from below. Inside one such column the
            // bottom covers all of it or none, so half the area decides.
            let lower_mm = if lowest[c] > base && bottom_area[c] >= area / 2. {
                lowest[c].min(upper_mm)
            } else {
                base
            };
            cells.push(HeightBounds { lower_mm, upper_mm });
        }
        Ok(cells)
    }
}

impl Serialize for StockMesh {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("StockMesh", 2)?;
        state.serialize_field("vertices", &self.0.vertices)?;
        state.serialize_field("indices", &self.0.indices)?;
        state.end()
    }
}
impl<'de> Deserialize<'de> for StockMesh {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Wire {
            triangles: Option<Vec<[Vec3; 3]>>,
            vertices: Option<Vec<Vec3>>,
            indices: Option<Vec<[u32; 3]>>,
        }
        use serde::de::Error;
        match Wire::deserialize(deserializer)? {
            Wire {
                triangles: Some(triangles),
                vertices: None,
                indices: None,
            } => Self::from_triangles(&triangles),
            Wire {
                triangles: None,
                vertices: Some(vertices),
                indices: Some(indices),
            } => Self::new(vertices, indices),
            _ => Err("a stock mesh needs triangles, or vertices and indices".into()),
        }
        .map_err(D::Error::custom)
    }
}

/// A run's stock given as a mesh: the height-field grid it is sampled on
/// (a `StockSpec` whose box contains the mesh) and the solid itself.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshStockSpec {
    grid: StockSpec,
    mesh: StockMesh,
}
impl MeshStockSpec {
    pub fn new(grid: StockSpec, mesh: StockMesh) -> Result<Self, String> {
        grid.grid()?;
        let [lo, hi] = mesh.bounds();
        if (0..3).any(|k| lo[k] < grid.min_mm[k] || hi[k] > grid.max_mm[k]) {
            return Err(format!(
                "stock mesh (from {lo:?} to {hi:?}) extends outside the stock box (from {:?} to \
                 {:?}); make the box contain the mesh, e.g. its bounds",
                grid.min_mm, grid.max_mm
            ));
        }
        Ok(Self { grid, mesh })
    }
    /// A grid on the mesh's own bounds.
    pub fn from_mesh(mesh: StockMesh, cell_size_mm: f64, max_cells: usize) -> Result<Self, String> {
        let [min_mm, max_mm] = mesh.bounds();
        Self::new(
            StockSpec {
                version: 1,
                units: "mm".into(),
                min_mm,
                max_mm,
                cell_size_mm,
                max_cells,
            },
            mesh,
        )
    }
    pub fn grid(&self) -> &StockSpec {
        &self.grid
    }
    pub(crate) fn grid_mut(&mut self) -> &mut StockSpec {
        &mut self.grid
    }
    pub fn mesh(&self) -> &StockMesh {
        &self.mesh
    }
    /// The run's starting stock: the mesh converted to the height field.
    pub fn initial_stock(&self, cancel: &impl CancelSignal) -> Result<Stock, String> {
        let cells = self.mesh.height_cells(&self.grid, cancel)?;
        Stock::from_initial_cells(self.grid.clone(), cells)
    }
}
impl Serialize for MeshStockSpec {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let g = &self.grid;
        let mut state = serializer.serialize_struct("MeshStockSpec", 7)?;
        state.serialize_field("version", &g.version)?;
        state.serialize_field("units", &g.units)?;
        state.serialize_field("min_mm", &g.min_mm)?;
        state.serialize_field("max_mm", &g.max_mm)?;
        state.serialize_field("cell_size_mm", &g.cell_size_mm)?;
        state.serialize_field("max_cells", &g.max_cells)?;
        state.serialize_field("mesh", &self.mesh)?;
        state.end()
    }
}
impl<'de> Deserialize<'de> for MeshStockSpec {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Wire {
            version: u32,
            units: String,
            min_mm: Vec3,
            max_mm: Vec3,
            cell_size_mm: f64,
            max_cells: usize,
            mesh: StockMesh,
        }
        let w = Wire::deserialize(deserializer)?;
        Self::new(
            StockSpec {
                version: w.version,
                units: w.units,
                min_mm: w.min_mm,
                max_mm: w.max_mm,
                cell_size_mm: w.cell_size_mm,
                max_cells: w.max_cells,
            },
            w.mesh,
        )
        .map_err(serde::de::Error::custom)
    }
}

/// A stock returned as a closed mesh, with what it is.
#[derive(Clone, Debug)]
pub struct FinishedStock {
    pub mesh: StockMesh,
    /// None when the mesh is the stock exactly; otherwise how it differs.
    pub approximation: Option<&'static str>,
}

impl Stock {
    /// The stock as a closed mesh of each cell's upper bound (no material
    /// above it): a column from the stock's bottom up to that bound. It
    /// contains all the material that may remain and is approximate
    /// ([`HEIGHT_FIELD_APPROXIMATION`]). Refused when no material is left.
    pub fn finished_mesh(&self, cancel: &impl CancelSignal) -> Result<FinishedStock, String> {
        Ok(FinishedStock {
            mesh: height_field_mesh(self.spec(), self.cells(), cancel)?,
            approximation: Some(HEIGHT_FIELD_APPROXIMATION),
        })
    }
}

/// The closed mesh of a height field's upper bounds (see
/// [`Stock::finished_mesh`]). Neighbouring cells share their vertices exactly
/// (no T-junctions), so the mesh passes [`StockMesh::new`]'s checks, and read
/// back on the same grid it gives the same upper bounds.
pub fn height_field_mesh(
    spec: &StockSpec,
    cells: &[HeightBounds],
    cancel: &impl CancelSignal,
) -> Result<StockMesh, String> {
    let (nx, ny, step) = spec.grid()?;
    if cells.len() != nx * ny {
        return Err("stock cells do not match the stock grid".into());
    }
    let base = spec.min_mm[2];
    let height = |i: isize, j: isize| -> f64 {
        if i < 0 || j < 0 || i >= nx as isize || j >= ny as isize {
            base
        } else {
            cells[j as usize * nx + i as usize].upper_mm.max(base)
        }
    };
    let solid = cells.iter().filter(|c| c.upper_mm > base).count();
    if solid == 0 {
        return Err("the stock holds no material, so it has no mesh".into());
    }
    check_memory(solid as u128 * 600, "stock mesh")?;
    let mut lookup: HashMap<(u32, u32, u64), u32> = HashMap::new();
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut vertex = |gi: usize, gj: usize, z: f64| -> u32 {
        *lookup
            .entry((gi as u32, gj as u32, z.to_bits()))
            .or_insert_with(|| {
                vertices.push([
                    cell_edge(spec, [nx, ny], step, 0, gi),
                    cell_edge(spec, [nx, ny], step, 1, gj),
                    z,
                ]);
                (vertices.len() - 1) as u32
            })
    };
    // Heights of the (up to) four cells around grid vertex (gi, gj).
    let around = |gi: usize, gj: usize| {
        let (gi, gj) = (gi as isize, gj as isize);
        [
            height(gi - 1, gj - 1),
            height(gi, gj - 1),
            height(gi - 1, gj),
            height(gi, gj),
        ]
    };
    // A wall's vertical side at a grid vertex: every height met there, so
    // the walls meeting at the vertex share their vertices.
    let chain = |gi: usize, gj: usize, lo: f64, hi: f64| {
        let mut zs: Vec<f64> = around(gi, gj)
            .into_iter()
            .filter(|z| *z > lo && *z < hi)
            .collect();
        zs.push(lo);
        zs.push(hi);
        zs.sort_by(f64::total_cmp);
        zs.dedup();
        zs
    };
    for j in 0..ny {
        if cancel.is_cancelled() {
            return Err("stock mesh cancelled".into());
        }
        for i in 0..nx {
            let z = height(i as isize, j as isize);
            if z <= base {
                continue;
            }
            let t = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)].map(|(a, b)| vertex(a, b, z));
            indices.extend([[t[0], t[1], t[2]], [t[0], t[2], t[3]]]);
            let b =
                [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)].map(|(a, b)| vertex(a, b, base));
            indices.extend([[b[0], b[2], b[1]], [b[0], b[3], b[2]]]);
            // Sides: neighbour offset, then the bottom-left and bottom-right
            // grid vertices seen from outside (so the wall faces outward).
            let sides = [
                ((1, 0), (i + 1, j), (i + 1, j + 1)),
                ((-1, 0), (i, j + 1), (i, j)),
                ((0, 1), (i + 1, j + 1), (i, j + 1)),
                ((0, -1), (i, j), (i + 1, j)),
            ];
            for ((di, dj), left, right) in sides {
                let low = height(i as isize + di, j as isize + dj);
                if low >= z {
                    continue;
                }
                let lz = chain(left.0, left.1, low, z);
                let rz = chain(right.0, right.1, low, z);
                let l: Vec<u32> = lz.iter().map(|h| vertex(left.0, left.1, *h)).collect();
                let r: Vec<u32> = rz.iter().map(|h| vertex(right.0, right.1, *h)).collect();
                // Zip the two vertical chains (each triangle has vertices on
                // both sides, so none is degenerate).
                let (mut a, mut c) = (0, 0);
                while a + 1 < l.len() || c + 1 < r.len() {
                    if c + 1 < r.len() && (a + 1 == l.len() || rz[c + 1] <= lz[a + 1]) {
                        indices.push([l[a], r[c], r[c + 1]]);
                        c += 1;
                    } else {
                        indices.push([l[a], r[c], l[a + 1]]);
                        a += 1;
                    }
                }
            }
        }
    }
    StockMesh::new(vertices, indices)
}

/// The coordinate of grid line `i` along axis `k`: the stock's own cell
/// spacing, with the last line exactly on the box.
fn cell_edge(spec: &StockSpec, n: [usize; 2], step: [f64; 2], k: usize, i: usize) -> f64 {
    if i == n[k] {
        spec.max_mm[k]
    } else {
        spec.min_mm[k] + i as f64 * step[k]
    }
}

/// A triangle clipped to an axis-aligned rectangle `[x0, x1, y0, y1]` (its
/// projection; heights interpolated along edges). Points on a clipping line
/// take its exact coordinate.
fn clip(triangle: [Vec3; 3], rect: [f64; 4]) -> Vec<Vec3> {
    let mut polygon = triangle.to_vec();
    for (s, &value) in rect.iter().enumerate() {
        let axis = s / 2;
        let keep = |p: &Vec3| {
            if s % 2 == 0 {
                p[axis] >= value
            } else {
                p[axis] <= value
            }
        };
        let input = std::mem::take(&mut polygon);
        for (k, &q) in input.iter().enumerate() {
            let p = input[(k + input.len() - 1) % input.len()];
            let cross = |a: Vec3, b: Vec3| -> Vec3 {
                let t = ((value - a[axis]) / (b[axis] - a[axis])).clamp(0., 1.);
                let mut point = [0.; 3];
                for m in 0..3 {
                    point[m] = a[m] + t * (b[m] - a[m]);
                }
                point[axis] = value;
                point
            };
            match (keep(&p), keep(&q)) {
                (true, true) => polygon.push(q),
                (true, false) => polygon.push(cross(p, q)),
                (false, true) => {
                    polygon.push(cross(q, p));
                    polygon.push(q);
                }
                (false, false) => {}
            }
        }
        if polygon.is_empty() {
            break;
        }
    }
    polygon
}

fn signed_area(polygon: &[Vec3]) -> f64 {
    let o = polygon[0];
    (1..polygon.len().saturating_sub(1))
        .map(|k| {
            let (a, b) = (sub(polygon[k], o), sub(polygon[k + 1], o));
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        / 2.
}
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Fill a fresh height buffer (lower, upper pairs) with the starting stock
/// of a run whose stock is given as JSON (a box or a mesh stock): what a
/// stream receiver starts from.
pub fn initialise_stock(
    buffer: &mut impl crate::buffers::HeightWrite,
    stock_json: &str,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    let spec: crate::stream::RunStockSpec =
        serde_json::from_str(stock_json).map_err(|e| e.to_string())?;
    let stock = spec.initial_stock(cancel)?;
    let cells = stock.height()?.cells();
    if buffer.scalar_len() != cells.len() * 2 {
        return Err("height buffer does not match the stock grid".into());
    }
    for (i, c) in cells.iter().enumerate() {
        buffer.write(2 * i, c.lower_mm);
        buffer.write(2 * i + 1, c.upper_mm);
    }
    Ok(())
}

/// [`height_field_mesh`] of a height buffer (lower, upper pairs) on the grid
/// given as JSON.
pub fn buffer_mesh(
    buffer: &impl crate::buffers::HeightRead,
    grid_json: &str,
    cancel: &impl CancelSignal,
) -> Result<StockMesh, String> {
    let spec: StockSpec = serde_json::from_str(grid_json).map_err(|e| e.to_string())?;
    let (nx, ny, _) = spec.grid()?;
    if buffer.scalar_len() != nx * ny * 2 {
        return Err("height buffer does not match the stock grid".into());
    }
    let cells: Vec<HeightBounds> = (0..nx * ny)
        .map(|i| HeightBounds {
            lower_mm: buffer.read(2 * i),
            upper_mm: buffer.read(2 * i + 1),
        })
        .collect();
    if cells.iter().any(|c| {
        !(c.lower_mm.is_finite()
            && c.upper_mm.is_finite()
            && spec.min_mm[2] <= c.lower_mm
            && c.lower_mm <= c.upper_mm
            && c.upper_mm <= spec.max_mm[2])
    }) {
        return Err("height buffer holds invalid stock bounds".into());
    }
    height_field_mesh(&spec, &cells, cancel)
}

/// A mesh from little-endian f64 vertex coordinates (x, y, z per vertex)
/// and u32 triangle indices (three per triangle), checked by
/// [`StockMesh::new`].
pub fn mesh_from_bytes(vertices: &[u8], indices: &[u8]) -> Result<StockMesh, String> {
    if !vertices.len().is_multiple_of(24) || !indices.len().is_multiple_of(12) {
        return Err("stock mesh buffers need 3 f64 per vertex and 3 u32 per triangle".into());
    }
    let f = |b: &[u8]| f64::from_le_bytes(b.try_into().unwrap());
    let u = |b: &[u8]| u32::from_le_bytes(b.try_into().unwrap());
    StockMesh::new(
        vertices
            .as_chunks::<24>()
            .0
            .iter()
            .map(|v| [f(&v[..8]), f(&v[8..16]), f(&v[16..])])
            .collect(),
        indices
            .as_chunks::<12>()
            .0
            .iter()
            .map(|t| [u(&t[..4]), u(&t[4..8]), u(&t[8..])])
            .collect(),
    )
}
/// Little-endian bytes of a mesh's vertices and indices.
pub fn mesh_to_bytes(mesh: &StockMesh) -> (Vec<u8>, Vec<u8>) {
    (
        mesh.vertices()
            .iter()
            .flatten()
            .flat_map(|c| c.to_le_bytes())
            .collect(),
        mesh.indices()
            .iter()
            .flatten()
            .flat_map(|i| i.to_le_bytes())
            .collect(),
    )
}
