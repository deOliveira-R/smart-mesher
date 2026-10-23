//! Radial rings: the faces around an edge in canonical angular order.

use super::CellComplex;
use crate::view::TopologyView;
use sm_geom_api::GeomStore;
use crate::types::{Carrier, CellFlags, Dim, Sense};
use crate::view::{EdgeUse, RADIAL_ANGLE_STEP};
use core::f64::consts::TAU;
use sm_core::{EntityId, Vector3};
use smallvec::SmallVec;

impl CellComplex {
    /// Angles (radians, counter-clockwise about the edge direction) of the face uses relative to
    /// the first use. `None` if some carrier is missing, so the caller falls back to id order.
    fn radial_angles(&self, edge: EntityId, uses: &[EdgeUse]) -> Option<Vec<f64>> {
        let Carrier::Curve(cc) = self.carrier(edge) else { return None };
        let curve = self.shapes.curve(cc.curve)?;
        let tm = 0.5 * (cc.range.lo + cc.range.hi);
        let jet = curve.jet(tm);
        let len = jet.d1.norm();
        if !(len > 0.0 && len.is_finite()) {
            return None;
        }
        let t = jet.d1 * (cc.sense.sign() / len);
        let mut dirs: Vec<Vector3> = Vec::with_capacity(uses.len());
        for u in uses {
            let Carrier::Surface(sc) = self.carrier(u.face) else { return None };
            let surf = self.shapes.surface(sc.surface)?;
            let proj = surf.project(jet.p, None).ok()?;
            let n = surf.normal(proj.uv).ok()? * sc.sense.sign();
            let d = n.cross(t) * u.sense.sign();
            let l = d.norm();
            if !(l > 0.0) {
                return None;
            }
            dirs.push(d * (1.0 / l));
        }
        let r = dirs[0];
        Some(dirs.iter().map(|d| (r.cross(*d).dot(t)).atan2(r.dot(*d))).collect())
    }

    /// Compute the radial ring of one edge (see [`crate::TopologyView::radial_uses`]).
    pub(super) fn compute_radial(&self, edge: EntityId) -> SmallVec<[EdgeUse; 4]> {
        if self.flags(edge).contains(CellFlags::DEGENERATE) {
            return SmallVec::new();
        }
        // The coboundary is ascending by (face, side), i.e. by (face, Forward-first).
        let mut uses: SmallVec<[EdgeUse; 4]> = self
            .up(edge)
            .iter()
            .filter(|e| Dim::of_real_kind(e.cell.kind()) == Some(Dim::Face))
            .map(|e| EdgeUse { face: e.cell, sense: if e.side == crate::types::Side::Right { Sense::Reversed } else { Sense::Forward } })
            .collect();
        if uses.len() < 2 {
            return uses;
        }
        let Some(angles) = self.radial_angles(edge, &uses) else { return uses };
        let full = (TAU / RADIAL_ANGLE_STEP).round() as i64;
        let mut keyed: Vec<(i64, EntityId, Sense, usize)> = uses
            .iter()
            .zip(angles)
            .enumerate()
            .map(|(i, (u, a))| {
                let a = if a < 0.0 { a + TAU } else { a };
                let mut k = (a / RADIAL_ANGLE_STEP).round() as i64;
                if k >= full {
                    k -= full;
                }
                (k, u.face, u.sense, i)
            })
            .collect();
        keyed.sort_by_key(|k| (k.0, k.1, k.2 == Sense::Reversed));
        uses = keyed.iter().map(|k| EdgeUse { face: k.1, sense: k.2 }).collect();
        uses
    }
}
