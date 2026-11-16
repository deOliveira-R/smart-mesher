//! The format-neutral CSG deck: surfaces, cells with region expressions, universes, rectangular
//! lattices and materials. Format readers fill it; [`crate::ImportedCsg::instantiate`] flattens it.
//!
//! # Conventions of the IR (the readers convert to these)
//! * A surface's negative side is `f < 0` (`Sign::Negative`), the side the carrier's function is
//!   negative on: inside a sphere or cylinder, below a plane along its normal.
//! * A [`Fill::Universe`] transform maps a point `u` of the filling universe to `xf(u)` in the
//!   frame of the cell that carries the fill.
//! * A [`Lattice`] lives in its own frame: element `[i, j, k]` is the box starting at
//!   `lower_left + [i, j, k] * pitch`, `i` along +x, `j` along +y, `k` along +z, and the
//!   universe placed in it has its origin at the element centre. `universes` is ordered `i`
//!   fastest, then `j`, then `k` (so a reader must reverse any top-to-bottom row order).
//! * A lattice axis with one element may have an infinite pitch (no clip along that axis, element
//!   centre coordinate 0).
//! * The `outer` universe of a lattice fills the part of the container outside the array box, with
//!   its origin at the lattice frame origin.
//! * A [`HexLattice`] lives in `CsgIr::hex_lattices`; a cell reaches it through the same
//!   [`Fill::Lattice`] (lattice and hex lattice ids share one id space, as in OpenMC). Its
//!   frame, ring order and index map are documented on [`HexLattice`] and in `DESIGN.md`.

use crate::ImportError;
use sm_core::{Point3, Transform, Vector3};
use sm_geom_api::{Cylinder, Frame, Plane, Sphere, SurfaceDef};
use std::collections::BTreeMap;

pub use sm_geom_api::Sign;

/// Boundary condition a surface carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BoundaryFlag {
    /// An ordinary interior surface.
    #[default]
    None,
    /// Particles leave the problem.
    Vacuum,
    /// Specular reflection (exact on planes only).
    Reflective,
    /// White (cosine-law) re-emission.
    White,
}

/// A surface with its boundary flag. The definition is already in the deck's global frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Surf {
    /// The geometry.
    pub def: SurfaceDef,
    /// The boundary condition.
    pub flag: BoundaryFlag,
}

fn axis_vec(axis: usize) -> Vector3 {
    let mut v = [0.0; 3];
    v[axis] = 1.0;
    Vector3::new(v[0], v[1], v[2])
}

fn geom_err(e: impl core::fmt::Display) -> ImportError {
    ImportError::Invalid(format!("invalid surface: {e}"))
}

impl Surf {
    /// A surface without a boundary flag.
    pub fn new(def: SurfaceDef) -> Self {
        Self { def, flag: BoundaryFlag::None }
    }
    /// The same surface with a boundary flag.
    pub fn with_flag(mut self, flag: BoundaryFlag) -> Self {
        self.flag = flag;
        self
    }
    /// The plane `axis = c` (axis 0, 1, 2 for x, y, z); the positive side is `axis > c`.
    pub fn axis_plane(axis: usize, c: f64) -> Self {
        Self::new(axis_plane_def(axis % 3, c))
    }
    /// The plane through `point` with normal `normal`; the positive side is along the normal.
    ///
    /// # Errors
    /// [`ImportError::Invalid`] for a zero normal.
    pub fn plane(point: [f64; 3], normal: [f64; 3]) -> Result<Self, ImportError> {
        let frame = Frame::from_z(Point3::new(point[0], point[1], point[2]), Vector3::new(normal[0], normal[1], normal[2])).map_err(geom_err)?;
        Ok(Self::new(SurfaceDef::Plane(Plane { frame })))
    }
    /// The sphere of radius `r` about `c`; the negative side is the inside.
    ///
    /// # Errors
    /// [`ImportError::Invalid`] for a non-positive or non-finite radius.
    pub fn sphere(c: [f64; 3], r: f64) -> Result<Self, ImportError> {
        let s = Sphere::new(Frame::at(Point3::new(c[0], c[1], c[2])), r).map_err(geom_err)?;
        Ok(Self::new(SurfaceDef::Sphere(s)))
    }
    /// The infinite cylinder of radius `r` about the line through `c` along `axis` (0, 1, 2).
    ///
    /// # Errors
    /// [`ImportError::Invalid`] for a non-positive or non-finite radius.
    pub fn cylinder(axis: usize, c: [f64; 3], r: f64) -> Result<Self, ImportError> {
        let frame = Frame::from_z(Point3::new(c[0], c[1], c[2]), axis_vec(axis % 3)).map_err(geom_err)?;
        Ok(Self::new(SurfaceDef::Cylinder(Cylinder::new(frame, r).map_err(geom_err)?)))
    }
}

/// The plane `axis = c` as a definition.
pub(crate) fn axis_plane_def(axis: usize, c: f64) -> SurfaceDef {
    let mut o = [0.0; 3];
    o[axis] = c;
    let frame = Frame { origin: Point3::new(o[0], o[1], o[2]), ..Frame::at(Point3::new(0.0, 0.0, 0.0)) };
    let z = axis_vec(axis);
    // x and y complete a right-handed frame with z along the axis.
    let (x, y) = match axis {
        0 => (Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        1 => (Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)),
        _ => (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
    };
    SurfaceDef::Plane(Plane { frame: Frame { x, y, z, ..frame } })
}

/// A region expression over deck surface ids.
#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    /// The `sign` side of a surface.
    Half(u32, Sign),
    /// Intersection (all of space when empty).
    And(Vec<Expr>),
    /// Union (nothing when empty).
    Or(Vec<Expr>),
    /// Complement.
    Not(Box<Expr>),
}

impl Expr {
    /// The negative side (`f < 0`) of surface `s`.
    pub fn neg(s: u32) -> Self {
        Self::Half(s, Sign::Negative)
    }
    /// The positive side of surface `s`.
    pub fn pos(s: u32) -> Self {
        Self::Half(s, Sign::Positive)
    }
    /// Intersection of signed surfaces: `+s` for a positive id, `-s` for a negative one (the
    /// signed-integer form of the region strings).
    pub fn signed(ids: &[i64]) -> Self {
        Self::And(ids.iter().map(|&i| Self::Half(i.unsigned_abs() as u32, if i < 0 { Sign::Negative } else { Sign::Positive })).collect())
    }
}

/// What fills a cell.
#[derive(Clone, Debug, PartialEq, Default)]
pub enum Fill {
    /// The cell holds its material.
    #[default]
    None,
    /// A universe, placed by `xf` (universe frame to cell frame).
    Universe {
        /// The universe id.
        id: u32,
        /// Placement.
        xf: Transform,
    },
    /// A lattice, its frame placed by `xf` (lattice frame to cell frame).
    Lattice {
        /// The lattice id.
        id: u32,
        /// Placement.
        xf: Transform,
    },
}

/// A cell: a region of its universe, either a material or a fill.
#[derive(Clone, Debug, PartialEq)]
pub struct Cell {
    /// The region, over deck surface ids.
    pub region: Expr,
    /// The material id; `None` is void.
    pub material: Option<u32>,
    /// The universe the cell belongs to.
    pub universe: u32,
    /// What fills it.
    pub fill: Fill,
}

impl Cell {
    /// A cell of `universe` holding `material` (`None` is void).
    pub fn new(universe: u32, region: Expr, material: Option<u32>) -> Self {
        Self { region, material, universe, fill: Fill::None }
    }
    /// A cell of `universe` filled by `fill`.
    pub fn filled(universe: u32, region: Expr, fill: Fill) -> Self {
        Self { region, material: None, universe, fill }
    }
}

/// A rectangular lattice (see the module docs for the frame and order).
#[derive(Clone, Debug, PartialEq)]
pub struct Lattice {
    /// Lower-left corner of element `[0, 0, 0]` in the lattice frame.
    pub lower_left: [f64; 3],
    /// Element size. An axis with one element may be `f64::INFINITY`.
    pub pitch: [f64; 3],
    /// Elements per axis.
    pub dims: [u32; 3],
    /// The universe of each element, `i` fastest, then `j`, then `k`.
    pub universes: Vec<u32>,
    /// Universe outside the array, if any.
    pub outer: Option<u32>,
}

/// Orientation of a [`HexLattice`], named as OpenMC's `orientation` attribute.
///
/// `X` is rows along x (pointy-top hexagons, flats at `x = +-p/2` of the element centre); it is
/// `LatticeKind::Hex` in `sm-rt-api` and `HexOrient::XRows` in `sm-model-api`. `Y` is rows along y
/// (flat-top, flats at `y = +-p/2`), `LatticeKind::HexYRows`; it is OpenMC's default.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum HexOrientation {
    /// OpenMC `orientation="x"`.
    X,
    /// OpenMC `orientation="y"` (the OpenMC default).
    Y,
}

/// A hexagonal lattice of `n_rings` rings and `n_axial` layers (OpenMC `<hex_lattice>`).
///
/// # Frame
/// `center` is the lattice centre in the deck frame. `p = pitch` is the flat-to-flat pitch, which
/// is also the distance between neighbouring element centres. With centred axial indices `(q, r)`
/// (`max(|q|, |r|, |q + r|) < n_rings`) and layer `k` (0 is the lowest z):
/// * `X`: centre `= center + p (q + r/2, sqrt(3)/2 r)`;
/// * `Y`: centre `= center + p (sqrt(3)/2 q, r + q/2)`;
/// * z of layer `k`: `center[2] + z_pitch (k + 1/2 - n_axial/2)` ([`Self::element_center`]).
///
/// The universe placed in an element has its origin at the element centre. A lattice without z
/// extent (`n_axial == 1` and no z pitch in the deck) has `z_pitch == f64::INFINITY` and
/// `center[2] == 0`: the universes keep the cell's z, as for a 2-D rectangular lattice.
///
/// # Order of `universes`
/// OpenMC order (the Python API order, not the row picture of `geometry.xml`): layer `k` from the
/// lowest z up; within a layer ring `0` (the outermost, radius `n_rings - 1`) to ring `n_rings - 1`
/// (the single centre element); within a ring from OpenMC's start position, clockwise
/// ([`Self::ring_axial`]). A layer holds `3 n (n - 1) + 1` universes for `n` rings
/// ([`Self::elements_per_layer`]).
///
/// `outer` is homogeneous (the reader refuses one whose cells depend on position, as for the
/// rectangular lattice): it fills the container outside the hexagonal array.
#[derive(Clone, Debug, PartialEq)]
pub struct HexLattice {
    /// Lattice centre (the z entry is 0 when the lattice has no z extent).
    pub center: [f64; 3],
    /// Flat-to-flat pitch in the xy plane.
    pub pitch: f64,
    /// Layer pitch in z; `f64::INFINITY` for a lattice without z extent (`n_axial == 1`).
    pub z_pitch: f64,
    /// Orientation.
    pub orientation: HexOrientation,
    /// Number of rings, at least 1 (1 is a single element).
    pub n_rings: u32,
    /// Number of axial layers, at least 1.
    pub n_axial: u32,
    /// The universe of each element in OpenMC order (see the type docs).
    pub universes: Vec<u32>,
    /// Universe outside the array, if any (homogeneous).
    pub outer: Option<u32>,
}

impl HexLattice {
    /// Elements in one layer of `n_rings` rings, `3 n (n - 1) + 1` (0 for no rings; saturates
    /// at `usize::MAX`, which no consistent lattice reaches).
    pub fn elements_per_layer(n_rings: u32) -> usize {
        let n = n_rings as usize;
        if n == 0 { 0 } else { n.saturating_mul(n - 1).saturating_mul(3).saturating_add(1) }
    }

    /// Elements in ring `ring` (0 is the outermost) of `n_rings` rings; 0 when `ring >= n_rings`
    /// (there is no such ring).
    pub fn ring_len(n_rings: u32, ring: u32) -> usize {
        if ring >= n_rings {
            return 0;
        }
        let radius = (n_rings - 1 - ring) as usize;
        if radius == 0 { 1 } else { 6 * radius }
    }

    /// Elements in the rings before `ring` (the outermost `ring` rings): the offset of ring `ring`
    /// within a layer, in closed form. `None` when `ring > n_rings`.
    pub fn ring_offset(n_rings: u32, ring: u32) -> Option<usize> {
        (ring <= n_rings).then(|| Self::elements_per_layer(n_rings) - Self::elements_per_layer(n_rings - ring))
    }

    /// Start corner and the six side directions of the ring of radius `radius` (see
    /// [`Self::ring_axial`]).
    fn ring_walk(orientation: HexOrientation, radius: i64) -> ((i64, i64), [(i64, i64); 6]) {
        match orientation {
            HexOrientation::X => ((radius, 0), [(0, -1), (-1, 0), (-1, 1), (0, 1), (1, 0), (1, -1)]),
            HexOrientation::Y => ((0, radius), [(1, -1), (0, -1), (-1, 0), (-1, 1), (0, 1), (1, 0)]),
        }
    }

    /// Centred axial indices `(q, r)` of position `pos` of ring `ring` (0 is the outermost), in
    /// OpenMC's order. With radius `R = n_rings - 1 - ring` the ring starts at `(R, 0)` for `X` and
    /// `(0, R)` for `Y`, then makes `R` steps along each of six directions, clockwise:
    /// `(0,-1) (-1,0) (-1,1) (0,1) (1,0) (1,-1)` for `X`, `(1,-1) (0,-1) (-1,0) (-1,1) (0,1) (1,0)`
    /// for `Y`. `None` when `ring` or `pos` is out of range.
    pub fn ring_axial(orientation: HexOrientation, n_rings: u32, ring: u32, pos: u32) -> Option<(i32, i32)> {
        if ring >= n_rings || pos as usize >= Self::ring_len(n_rings, ring) {
            return None;
        }
        let radius = i64::from(n_rings - 1 - ring);
        if radius == 0 {
            return Some((0, 0));
        }
        let (start, dirs) = Self::ring_walk(orientation, radius);
        let side = (i64::from(pos) / radius) as usize;
        let step = i64::from(pos) % radius;
        // The corner at the start of side `side`, then `step` steps along it.
        let (mut q, mut r) = start;
        for d in &dirs[..side] {
            q += d.0 * radius;
            r += d.1 * radius;
        }
        Some((i32::try_from(q + dirs[side].0 * step).ok()?, i32::try_from(r + dirs[side].1 * step).ok()?))
    }

    /// The inverse of [`Self::ring_axial`]: `(ring, position)` of the element at `(q, r)`, or `None`
    /// outside the `n_rings` rings.
    ///
    /// Closed form: the six sides of the ring of the element's radius are tried (a constant
    /// number of steps), the step along the side follows from one coordinate.
    pub fn ring_position(orientation: HexOrientation, n_rings: u32, q: i32, r: i32) -> Option<(u32, u32)> {
        let radius = i64::from(q).abs().max(i64::from(r).abs()).max((i64::from(q) + i64::from(r)).abs());
        if radius >= i64::from(n_rings) {
            return None;
        }
        let ring = n_rings - 1 - radius as u32;
        if radius == 0 {
            return Some((ring, 0));
        }
        let (q, r) = (i64::from(q), i64::from(r));
        let (start, dirs) = Self::ring_walk(orientation, radius);
        let (mut cq, mut cr) = start;
        for (side, d) in dirs.iter().enumerate() {
            // `(q, r) = corner + step * d` with 0 <= step < radius.
            let step = if d.0 != 0 { (q - cq) / d.0 } else { (r - cr) / d.1 };
            if (0..radius).contains(&step) && cq + d.0 * step == q && cr + d.1 * step == r {
                return u32::try_from(side as i64 * radius + step).ok().map(|pos| (ring, pos));
            }
            cq += d.0 * radius;
            cr += d.1 * radius;
        }
        None
    }

    /// Index into [`Self::universes`] of layer `k`, ring `ring`, position `pos`; `None` for a
    /// layer, ring or position out of range.
    pub fn flat_index(&self, k: u32, ring: u32, pos: u32) -> Option<usize> {
        if k >= self.n_axial || pos as usize >= Self::ring_len(self.n_rings, ring) {
            return None;
        }
        let before = Self::ring_offset(self.n_rings, ring)?;
        (k as usize).checked_mul(Self::elements_per_layer(self.n_rings))?.checked_add(before + pos as usize)
    }

    /// The universe of the element at centred axial `(q, r)` and layer `k`; `None` outside the
    /// rings or layers.
    pub fn universe_at(&self, q: i32, r: i32, k: u32) -> Option<u32> {
        if k >= self.n_axial {
            return None;
        }
        let (ring, pos) = Self::ring_position(self.orientation, self.n_rings, q, r)?;
        self.universes.get(self.flat_index(k, ring, pos)?).copied()
    }

    /// Centre of the element at centred axial `(q, r)` and layer `k` in the deck frame (formulas in
    /// the type docs). The z entry is `center[2]` for a lattice without z extent.
    pub fn element_center(&self, q: i32, r: i32, k: u32) -> [f64; 3] {
        let s = 3f64.sqrt() / 2.0;
        let (q, r) = (f64::from(q), f64::from(r));
        let (x, y) = match self.orientation {
            HexOrientation::X => (self.pitch * (q + r / 2.0), self.pitch * s * r),
            HexOrientation::Y => (self.pitch * s * q, self.pitch * (r + q / 2.0)),
        };
        let z = if self.z_pitch.is_finite() { self.center[2] + self.z_pitch * (f64::from(k) + 0.5 - f64::from(self.n_axial) / 2.0) } else { self.center[2] };
        [self.center[0] + x, self.center[1] + y, z]
    }

    /// Structural check: at least one ring and layer, `universes` count, finite positive pitch, a
    /// positive `z_pitch` that is infinite only with one layer, finite centre.
    ///
    /// # Errors
    /// A description of the first problem.
    pub fn check(&self) -> Result<(), String> {
        if self.n_rings == 0 || self.n_axial == 0 {
            return Err(format!("{} rings and {} layers", self.n_rings, self.n_axial));
        }
        let n = Self::elements_per_layer(self.n_rings).checked_mul(self.n_axial as usize).ok_or("too many elements")?;
        if self.universes.len() != n {
            return Err(format!("lists {} universes for {n} elements", self.universes.len()));
        }
        if !(self.pitch.is_finite() && self.pitch > 0.0) {
            return Err(format!("pitch {} is not finite and positive", self.pitch));
        }
        if !(self.z_pitch > 0.0 && (self.z_pitch.is_finite() || self.n_axial == 1)) {
            return Err(format!("z pitch {} with {} layers", self.z_pitch, self.n_axial));
        }
        if !self.center.iter().all(|c| c.is_finite()) {
            return Err(format!("centre {:?} is not finite", self.center));
        }
        Ok(())
    }
}

/// A material: its id and optional name.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Material {
    /// The material's name when the deck has one (may be empty or repeated; see
    /// [`crate::resolve_region_names`]).
    pub name: Option<String>,
}

/// A whole CSG deck.
#[derive(Clone, Debug, PartialEq)]
pub struct CsgIr {
    /// Surfaces by deck id.
    pub surfaces: BTreeMap<u32, Surf>,
    /// Cells by deck id.
    pub cells: BTreeMap<u32, Cell>,
    /// Lattices by deck id.
    pub lattices: BTreeMap<u32, Lattice>,
    /// Hexagonal lattices by deck id (the id space of `lattices`; a cell reaches either through
    /// [`Fill::Lattice`]). Empty for every rectangular deck. [`crate::ImportedCsg::instantiate`]
    /// places each element as a hexagonal prism (path element `[q, r, k]`).
    pub hex_lattices: BTreeMap<u32, HexLattice>,
    /// Materials by deck id.
    pub materials: BTreeMap<u32, Material>,
    /// The universe the model is rooted at.
    pub root_universe: u32,
    /// Data the reader read but the IR does not keep (densities, temperatures, ...), for reports.
    pub dropped: Vec<String>,
}

impl CsgIr {
    /// An empty deck rooted at universe `root_universe`.
    pub fn new(root_universe: u32) -> Self {
        Self { surfaces: BTreeMap::new(), cells: BTreeMap::new(), lattices: BTreeMap::new(), hex_lattices: BTreeMap::new(), materials: BTreeMap::new(), root_universe, dropped: Vec::new() }
    }

    /// Hash of the hexagonal-lattice section: `None` for a deck without hex lattices (the section
    /// is never emitted for rectangular decks, so no existing hash can depend on it), else a hash
    /// of every hex lattice in id order (tag, id, orientation, rings, layers, pitches, centre,
    /// `outer`, universes in OpenMC order). The ids are deck ids: a consumer that hashes the flat
    /// model appends this section only when it is `Some`.
    pub fn hex_section_hash(&self) -> Option<sm_rt_api::ContentHash> {
        if self.hex_lattices.is_empty() {
            return None;
        }
        let mut h = sm_rt_api::ContentHasher::new();
        h.write_bytes(b"csgio.hex.v1");
        h.write_u64(self.hex_lattices.len() as u64);
        for (id, l) in &self.hex_lattices {
            h.write_u64(u64::from(*id));
            h.write_u64(match l.orientation {
                HexOrientation::X => 0,
                HexOrientation::Y => 1,
            });
            h.write_u64(u64::from(l.n_rings));
            h.write_u64(u64::from(l.n_axial));
            h.write_f64(l.pitch);
            h.write_f64(l.z_pitch);
            l.center.iter().for_each(|c| h.write_f64(*c));
            h.write_u64(l.outer.map_or(0, |u| u64::from(u) + 1));
            h.write_u64(l.universes.len() as u64);
            l.universes.iter().for_each(|u| h.write_u64(u64::from(*u)));
        }
        Some(h.finish())
    }
}
