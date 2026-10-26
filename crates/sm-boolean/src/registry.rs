//! A deterministic spatial index of the vertices taking part in an imprint.

use std::collections::BTreeMap;

use sm_core::{EntityId, Point3};

use crate::geo;

/// Vertices by position, for "is there already a vertex here?" lookups within a tolerance.
///
/// Cells are `cell`-sized cubes keyed in a `BTreeMap`, and a lookup scans the 27 neighbouring
/// cells, so the answer never depends on hash order: the nearest vertex within the tolerance
/// wins, ties go to the lowest id.
#[derive(Clone, Debug)]
pub(crate) struct VertexIndex {
    tol: f64,
    cell: f64,
    cells: BTreeMap<(i64, i64, i64), Vec<(EntityId, Point3)>>,
    all: Vec<(EntityId, Point3)>,
}

impl VertexIndex {
    pub(crate) fn new(tol: f64) -> Self {
        Self { tol, cell: (tol * 8.0).max(1e-12), cells: BTreeMap::new(), all: Vec::new() }
    }

    fn key(&self, p: Point3) -> (i64, i64, i64) {
        let q = |x: f64| (x / self.cell).floor() as i64;
        (q(p.x), q(p.y), q(p.z))
    }

    pub(crate) fn insert(&mut self, id: EntityId, p: Point3) {
        let k = self.key(p);
        self.cells.entry(k).or_default().push((id, p));
        self.all.push((id, p));
    }

    /// The vertex nearest to `p` within the tolerance.
    pub(crate) fn find(&self, p: Point3) -> Option<EntityId> {
        let (cx, cy, cz) = self.key(p);
        let mut best: Option<(f64, EntityId)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(list) = self.cells.get(&(cx + dx, cy + dy, cz + dz)) else { continue };
                    for &(id, q) in list {
                        let d = geo::dist(p, q);
                        if d <= self.tol && best.is_none_or(|b| d < b.0 || (d == b.0 && id < b.1)) {
                            best = Some((d, id));
                        }
                    }
                }
            }
        }
        best.map(|b| b.1)
    }

    /// Every vertex, in insertion order.
    pub(crate) fn all(&self) -> &[(EntityId, Point3)] {
        &self.all
    }
}
