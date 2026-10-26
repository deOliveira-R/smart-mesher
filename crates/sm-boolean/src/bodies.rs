//! Resolving op operands (bodies or volumes) and the manifold guard.

use std::collections::BTreeSet;

use sm_core::EntityId;
use sm_topo_api::{Dim, Incidence, ShellKind, Side, TopologyView, TopologyViewExt};

use crate::error::BooleanError;

/// The live volumes of `entries` (a body gives its volumes, a volume itself), ascending and
/// distinct.
pub(crate) fn volumes_of<V: TopologyView + ?Sized>(view: &V, entries: &[EntityId]) -> Result<Vec<EntityId>, BooleanError> {
    let mut out: BTreeSet<EntityId> = BTreeSet::new();
    for &e in entries {
        if !view.is_alive(e) {
            return Err(BooleanError::Invalid(format!("{e} is not a live entity")));
        }
        match view.dim_of(e) {
            Some(Dim::Volume) => {
                out.insert(e);
            }
            Some(Dim::Body) => {
                let mut any = false;
                for inc in view.boundary(e) {
                    if view.dim_of(inc.cell) == Some(Dim::Volume) {
                        out.insert(inc.cell);
                        any = true;
                    }
                }
                if !any {
                    return Err(BooleanError::Invalid(format!("{e} has no volume: boolean operations need solid bodies")));
                }
            }
            _ => return Err(BooleanError::Invalid(format!("{e} is neither a body nor a volume"))),
        }
    }
    Ok(out.into_iter().collect())
}

/// Refuse baffles, fins, free faces and internal shells.
pub(crate) fn check_manifold<V: TopologyView + ?Sized>(view: &V, volume: EntityId) -> Result<(), BooleanError> {
    for inc in view.boundary(volume) {
        if inc.ring == Incidence::EMBEDDED || inc.side == Side::Both {
            return Err(BooleanError::NonManifoldInput(volume));
        }
    }
    let rings: BTreeSet<u16> = view.boundary(volume).iter().map(|e| e.ring).collect();
    for r in rings {
        if view.shell_kind(volume, r) == Some(ShellKind::Internal) {
            return Err(BooleanError::NonManifoldInput(volume));
        }
    }
    if view.boundary(volume).is_empty() {
        return Err(BooleanError::Invalid(format!("{volume} has no boundary faces")));
    }
    Ok(())
}

/// Faces of the volumes, ascending and distinct.
pub(crate) fn faces_of_volumes<V: TopologyView + ?Sized>(view: &V, vols: &[EntityId]) -> Vec<EntityId> {
    let mut s: BTreeSet<EntityId> = BTreeSet::new();
    for &v in vols {
        s.extend(view.volume_faces(v).iter().copied());
    }
    s.into_iter().collect()
}
