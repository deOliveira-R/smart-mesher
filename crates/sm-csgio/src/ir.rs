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
        Self { surfaces: BTreeMap::new(), cells: BTreeMap::new(), lattices: BTreeMap::new(), materials: BTreeMap::new(), root_universe, dropped: Vec::new() }
    }
}
