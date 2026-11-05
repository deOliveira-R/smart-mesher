//! CSG leaf input: cells that are unions of intersections of signed half-spaces.
//!
//! A [`CsgRegion`] is a union of [`CsgCell`]s (each an intersection of half-spaces over
//! surfaces of a `GeomStore`). A leaf is a list of [`CsgCellDef`]s over one store.
//!
//! # Cells must be disjoint
//! There is no priority and no "first cell wins" rule: a point is never resolved by cell order.
//! Overlap cannot be proven absent for general CSG, so it is reported where it is found: at
//! build time for what a builder can see (`csg.overlap`, `world.overlap`), by a Check-stage
//! sampling op, and at track time as [`crate::LostCause::Overlap`].

use crate::cache::{ContentHash, ContentHasher};
use crate::input::ComplementPolicy;
use sm_core::Aabb;
use sm_geom_api::{CsgCell, GeomStore, ShapeValue, Sign};

/// Length-prefixed string, so concatenated fields are unambiguous.
fn write_str(h: &mut ContentHasher, s: &str) {
    h.write_u64(s.len() as u64);
    h.write_bytes(s.as_bytes());
}

/// A region of space as a union of intersections of half-spaces (a DNF over signed surfaces).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct CsgRegion {
    terms: Vec<CsgCell>,
}

impl CsgRegion {
    /// The union of `terms`. No terms is the empty region.
    pub fn new(terms: Vec<CsgCell>) -> Self {
        Self { terms }
    }
    /// All of space: one term with no half-space.
    pub fn everything() -> Self {
        Self { terms: vec![CsgCell::default()] }
    }
    /// A single intersection.
    pub fn single(cell: CsgCell) -> Self {
        Self { terms: vec![cell] }
    }
    /// The terms, in input order.
    pub fn terms(&self) -> &[CsgCell] {
        &self.terms
    }
    /// Hash by *value*: term order and half-space order as given, each surface by its
    /// definition in `geom` (not by its handle), so renumbering handles does not move it.
    /// A handle the store does not know hashes as a distinct marker. (The def hash is the
    /// def's own value stream at exact bits; it is a new contract hash, with length prefixes.)
    pub fn content_hash(&self, geom: &dyn GeomStore) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.terms.len() as u64);
        for t in &self.terms {
            h.write_u64(t.halfspaces.len() as u64);
            for hs in &t.halfspaces {
                h.write_u64(match hs.sign {
                    Sign::Negative => 0,
                    Sign::Positive => 1,
                });
                match geom.surface_def(hs.surface) {
                    Some(def) => {
                        h.write_u64(1);
                        // Explicit fields (the def's own value stream, exact bits), not `Debug`.
                        def.for_each_value(|v| match v {
                            ShapeValue::Int(x) => {
                                h.write_u64(1);
                                h.write_u64(x);
                            }
                            ShapeValue::Length(x) => {
                                h.write_u64(2);
                                h.write_f64(x);
                            }
                            ShapeValue::Angle(x) => {
                                h.write_u64(3);
                                h.write_f64(x);
                            }
                            ShapeValue::Unitless(x) => {
                                h.write_u64(4);
                                h.write_f64(x);
                            }
                            ShapeValue::Param(x) => {
                                h.write_u64(5);
                                h.write_f64(x);
                            }
                            // A future kind of value: hash its text rather than drop it.
                            other => {
                                h.write_u64(0);
                                write_str(&mut h, &format!("{other:?}"));
                            }
                        });
                        // Terminator, so one def's values cannot run into the next half-space.
                        h.write_u64(0xFFFF_FFFF);
                    }
                    None => h.write_u64(0),
                }
            }
        }
        h.finish()
    }
}

/// One CSG cell of a leaf.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct CsgCellDef {
    /// The cell's region. Cells of a leaf must be pairwise disjoint.
    pub region: CsgRegion,
    /// Region (material) name the caller pairs with its cross sections.
    pub name: String,
    /// Exact volume when the producer knows it; `None` otherwise (never estimated).
    pub volume: Option<f64>,
}

impl CsgCellDef {
    /// A named cell without a known volume.
    pub fn new(region: CsgRegion, name: impl Into<String>) -> Self {
        Self { region, name: name.into(), volume: None }
    }
    /// Attach the exact volume.
    pub fn with_volume(mut self, volume: f64) -> Self {
        self.volume = Some(volume);
        self
    }
}

/// A leaf of CSG cells over one geometry store.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct CsgLeafInput<'a> {
    /// The store holding the surfaces the half-spaces name.
    pub geom: &'a dyn GeomStore,
    /// The cells, dense by local cell id.
    pub cells: &'a [CsgCellDef],
    /// Bounds of the leaf (required when some cell is unbounded).
    pub bounds: Option<Aabb>,
    /// What lies outside the cells.
    pub complement: &'a ComplementPolicy,
}

impl<'a> CsgLeafInput<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(geom: &'a dyn GeomStore, cells: &'a [CsgCellDef], bounds: Option<Aabb>, complement: &'a ComplementPolicy) -> Self {
        Self { geom, cells, bounds, complement }
    }
    /// Hash of the leaf by value: every cell's region hash, name, volume bits and the bounds.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.cells.len() as u64);
        for c in self.cells {
            h.write_hash(c.region.content_hash(self.geom));
            write_str(&mut h, &c.name);
            h.write_u64(u64::from(c.volume.is_some()));
            h.write_f64(c.volume.unwrap_or(0.0));
        }
        match self.bounds {
            Some(b) => {
                h.write_u64(1);
                for p in [b.lo, b.hi] {
                    for v in [p.x, p.y, p.z] {
                        h.write_f64(v);
                    }
                }
            }
            None => h.write_u64(0),
        }
        match self.complement {
            ComplementPolicy::Region(name) => {
                h.write_u64(1);
                write_str(&mut h, name);
            }
            // Exhaustive in this crate: a new policy fails to compile here until it is hashed.
            ComplementPolicy::Outside => h.write_u64(0),
        }
        h.finish()
    }
}
