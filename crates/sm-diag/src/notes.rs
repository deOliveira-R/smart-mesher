//! Scheme notes (ruling INT-7) as located findings of `mesh.quality`.
//!
//! A scheme reports quality-floor violations as [`SchemeNote`]s (a position, nearby model
//! entities, a metric and a suggestion). The mesh knows nothing of them, so the note is placed
//! on the mesh the way every finding is: the cell whose centroid is nearest to the note's
//! position, then that cell's own classification (`owner`) and the lower-dimensional model
//! entities around it (`near`).

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_mesh_api::{MeshEntityKind, MeshRead};
use sm_scheme_api::SchemeNote;

use crate::locate::{EntityRef, location, locate};

/// Most scheme notes listed in a report (the count of all is `n_scheme_notes`).
pub const MAX_SCHEME_NOTES: usize = 100;

/// A scheme's located diagnostic, placed on the mesh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SchemeFinding {
    /// The measure that is out of bounds, as the scheme names it (`min_dihedral`, ...).
    pub metric: String,
    /// Measured value.
    pub value: f64,
    /// The bound it violates.
    pub threshold: f64,
    /// What would probably cure it.
    pub suggestion: String,
    /// The position the scheme reported.
    pub position: [f64; 3],
    /// Model entities the scheme named (for example the nearby faces).
    pub entities: Vec<EntityId>,
    /// The mesh cell nearest to `position`; absent if the mesh has no cell.
    pub cell: Option<EntityRef>,
    /// That cell's own classification.
    pub owner: Option<EntityId>,
    /// Lower-dimensional model entities around that cell.
    pub near: Vec<EntityId>,
}

/// Place the first [`MAX_SCHEME_NOTES`] of `notes` on `m`. Returns the total number of notes and
/// the placed findings, in the order of `notes`.
///
/// The nearest cell is the one with the smallest squared distance from its location to the
/// position, ties to the lowest index, so the result does not depend on thread count.
pub fn scheme_findings(m: &dyn MeshRead, notes: &[SchemeNote]) -> (usize, Vec<SchemeFinding>) {
    let total = notes.len();
    if total == 0 {
        return (0, Vec::new());
    }
    let n_cells = m.count(MeshEntityKind::Cell);
    let centres: Vec<Option<[f64; 3]>> = (0..n_cells as u32).into_par_iter().map(|c| location(m, MeshEntityKind::Cell, c)).collect();
    let out = notes
        .iter()
        .take(MAX_SCHEME_NOTES)
        .map(|n| {
            let p = [n.position.x, n.position.y, n.position.z];
            let nearest = centres
                .par_iter()
                .enumerate()
                .filter_map(|(i, c)| c.map(|c| (dist2(c, p), i as u32)))
                .filter(|(d, _)| d.is_finite())
                .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let (cell, owner, near) = match nearest {
                Some((_, ix)) => {
                    let l = locate(m, MeshEntityKind::Cell, ix);
                    (Some(l.entity), l.owner, l.near)
                }
                None => (None, None, Vec::new()),
            };
            SchemeFinding {
                metric: n.metric.clone(),
                value: n.value,
                threshold: n.threshold,
                suggestion: n.suggestion.clone(),
                position: p,
                entities: n.entities.clone(),
                cell,
                owner,
                near,
            }
        })
        .collect();
    (total, out)
}

fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]) * (a[k] - b[k])).sum()
}
