//! Export selection on a lowered mesh: see [`select_lowered`].

use std::sync::Arc;

use sm_mesh_api::MeshRead;
use sm_meshio::report::LossReport;
use sm_meshio::select::{Roles, Select, apply, check};

use crate::error::FemError;
use crate::lower::{Lowered, SetOrigin};

/// The notes (`code: detail xcount`) of a selection's loss report.
fn notes_of(loss: &LossReport) -> Vec<String> {
    loss.items.iter().map(|i| format!("{}: {} x{}", i.code, i.detail, i.count)).collect()
}

/// Roles for a lowering: the lowered blocks and sidesets are the selection's blocks and sidesets;
/// region sets and deck faces follow the cells (companions); any other set of the mesh is a block,
/// sideset or nodeset by its members.
fn roles_of(lowered: &Lowered) -> Roles {
    lowered
        .sets
        .iter()
        .filter(|s| matches!(s.origin, SetOrigin::Region | SetOrigin::DeckFace))
        .fold(Roles::by_kind(), |r, s| r.companion(s.name.clone()))
}

/// Check `sel` against the lowered mesh (names and dimension) without cutting anything.
///
/// # Errors
/// [`FemError::Select`].
pub fn check_select(lowered: &Lowered, sel: &Select) -> Result<(), FemError> {
    Ok(check((*lowered.mesh).as_read(), sel, &roles_of(lowered))?)
}

/// Cut a lowered mesh down to `sel` ([`sm_meshio::select`]).
///
/// The selection keeps the cells of the selected blocks, the nodes they use, and the sidesets and
/// nodesets asked for; region sets and deck face sets follow the kept cells and keep their names
/// (a set with nothing left is dropped). Deck records whose sets are gone, declared element types
/// of dropped blocks and regions that lost all cells are dropped with them. A selection that
/// keeps every cell and set ([`Select::keeps_all_cells_and_sets`]) is only checked and returns `lowered` unchanged.
///
/// Returns the new lowering and the notes of the selection (`export.empty_selection` for a named
/// sideset or nodeset with nothing in it).
///
/// # Errors
/// [`FemError::Select`] for an unknown set name, a dimension that does not match, no cell left,
/// high-order cells or a cut the classification rules refuse.
pub fn select_lowered(lowered: Lowered, sel: &Select) -> Result<(Lowered, Vec<String>), FemError> {
    let roles = roles_of(&lowered);
    if sel.keeps_all_cells_and_sets() {
        check((*lowered.mesh).as_read(), sel, &roles)?;
        return Ok((lowered, Vec::new()));
    }
    let out = apply((*lowered.mesh).as_read(), sel, &roles)?;
    let notes = notes_of(&out.report);
    let mesh = out.mesh;
    let has = |name: &str| mesh.set_by_name(name).is_some();
    let sets = lowered
        .sets
        .iter()
        .filter_map(|s| {
            let m = mesh.set_by_name(&s.name)?;
            let mut s = s.clone();
            s.len = m.members.len();
            Some(s)
        })
        .collect();
    let decks = lowered
        .decks
        .iter()
        .filter(|d| d.face_set.as_deref().is_none_or(has) && d.mate_set.as_deref().is_none_or(has))
        .cloned()
        .collect();
    let block_types = lowered.block_types.iter().filter(|(n, _)| has(n)).map(|(n, t)| (n.clone(), *t)).collect();
    let regions = lowered
        .regions
        .iter()
        .filter_map(|r| {
            let m = mesh.set_by_name(&r.set)?;
            let mut r = r.clone();
            r.len = m.members.len();
            Some(r)
        })
        .collect();
    Ok((Lowered { mesh: Arc::new(mesh), decks, sets, block_types, regions }, notes))
}
