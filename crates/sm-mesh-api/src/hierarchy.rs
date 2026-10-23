//! Several meshes over one model: independent meshes (levels, coarse CMFD meshes, instance
//! prototypes) linked by fine->coarse [`LevelMap`]s.
//!
//! A coarse mesh is an independent mesh over the same model; its cells need not be classified
//! like the fine cells they contain (a CMFD node can span several volumes and may sit on a
//! `PartitionCell`). The map says which coarse cell contains each fine cell; an optional face
//! map says which coarse face carries each fine face (CMFD surface currents).

use crate::error::{MeshError, Result};
use crate::index::{CellIx, FaceIx, MeshEntityKind, NO_CELL};
use crate::instance::InstancedView;
use crate::read::MeshRead;
use crate::validate::connectivity_hash;
use sm_core::EntityId;

/// Total map from the cells of a fine mesh to the cells of a coarse mesh, with its inverse as
/// CSR. It remembers which meshes it was built for (ids and connectivity hashes) and refuses to
/// be used with a stale or different pair.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LevelMap {
    fine: EntityId,
    coarse: EntityId,
    fine_hash: u64,
    coarse_hash: u64,
    coarse_of: Vec<CellIx>,
    off: Vec<u32>,
    fines: Vec<CellIx>,
    face_map: Option<Vec<u32>>,
}

impl LevelMap {
    /// Build from the coarse parent of every fine cell. Every coarse cell must have a child.
    ///
    /// `face_map`, when given, has one entry per fine face: the coarse face that carries it, or
    /// [`NO_CELL`] if the fine face lies inside a coarse cell. It is validated against both
    /// meshes: a fine face between two different coarse cells must map to the coarse face whose
    /// cells are exactly those two; a face between fine cells of the same coarse cell must not
    /// map; a one-cell fine face may map only to a one-cell coarse face of its coarse cell.
    pub fn new(
        fine: &dyn MeshRead,
        coarse: &dyn MeshRead,
        coarse_of: Vec<CellIx>,
        face_map: Option<Vec<u32>>,
    ) -> Result<Self> {
        let (nfc, ncc) = (
            fine.count(MeshEntityKind::Cell),
            coarse.count(MeshEntityKind::Cell),
        );
        if coarse_of.len() != nfc {
            return Err(MeshError::Hierarchy(format!(
                "map has {} entries for {nfc} fine cells",
                coarse_of.len()
            )));
        }
        let mut off = vec![0u32; ncc + 1];
        for &c in &coarse_of {
            if c.get() >= ncc {
                return Err(MeshError::Hierarchy(format!(
                    "coarse cell {c:?} out of range"
                )));
            }
            off[c.get() + 1] += 1;
        }
        if off[1..].contains(&0) {
            return Err(MeshError::Hierarchy(
                "a coarse cell has no fine children".into(),
            ));
        }
        for i in 0..ncc {
            off[i + 1] += off[i];
        }
        let mut cursor = off.clone();
        let mut fines = vec![CellIx(0); nfc];
        for (f, &c) in coarse_of.iter().enumerate() {
            fines[cursor[c.get()] as usize] = CellIx::new(f);
            cursor[c.get()] += 1;
        }
        if let Some(fm) = &face_map {
            check_face_map(fine, coarse, &coarse_of, fm)?;
        }
        Ok(Self {
            fine: fine.mesh_id(),
            coarse: coarse.mesh_id(),
            fine_hash: connectivity_hash(fine),
            coarse_hash: connectivity_hash(coarse),
            coarse_of,
            off,
            fines,
            face_map,
        })
    }

    /// Id of the fine mesh.
    pub fn fine_id(&self) -> EntityId {
        self.fine
    }
    /// Id of the coarse mesh.
    pub fn coarse_id(&self) -> EntityId {
        self.coarse
    }
    /// Connectivity hash of the fine mesh at construction time.
    pub fn fine_hash(&self) -> u64 {
        self.fine_hash
    }
    /// Connectivity hash of the coarse mesh at construction time.
    pub fn coarse_hash(&self) -> u64 {
        self.coarse_hash
    }
    /// Number of fine cells.
    pub fn fine_len(&self) -> usize {
        self.coarse_of.len()
    }
    /// Number of coarse cells.
    pub fn coarse_len(&self) -> usize {
        self.off.len() - 1
    }
    /// Coarse parent of a fine cell. Panics if `fine` is out of range.
    pub fn coarse_of(&self, fine: CellIx) -> CellIx {
        self.coarse_of[fine.get()]
    }
    /// Fine children of a coarse cell, ascending. Panics if `coarse` is out of range.
    pub fn fines_of(&self, coarse: CellIx) -> &[CellIx] {
        &self.fines[self.off[coarse.get()] as usize..self.off[coarse.get() + 1] as usize]
    }
    /// The coarse face carrying fine face `f` ([`NO_CELL`] entries mean none), if a face map was given.
    pub fn face_map(&self) -> Option<&[u32]> {
        self.face_map.as_deref()
    }

    /// Fail unless `fine` and `coarse` are the meshes (same ids, same connectivity) this map
    /// was built for.
    pub fn check_meshes(&self, fine: &dyn MeshRead, coarse: &dyn MeshRead) -> Result<()> {
        if fine.mesh_id() != self.fine || coarse.mesh_id() != self.coarse {
            return Err(MeshError::Hierarchy(
                "map was built for different meshes".into(),
            ));
        }
        if connectivity_hash(fine) != self.fine_hash
            || connectivity_hash(coarse) != self.coarse_hash
        {
            return Err(MeshError::Hierarchy(
                "stale map: a mesh changed since the map was built".into(),
            ));
        }
        Ok(())
    }

    /// Sum fine-cell values into coarse cells (extensive quantities: volumes, sources, reaction rates).
    pub fn restrict_sum(&self, fine_values: &[f64]) -> Result<Vec<f64>> {
        if fine_values.len() != self.fine_len() {
            return Err(MeshError::Hierarchy(format!(
                "expected {} fine values, got {}",
                self.fine_len(),
                fine_values.len()
            )));
        }
        let mut out = vec![0.0; self.coarse_len()];
        for (c, o) in out.iter_mut().enumerate() {
            // Fixed (ascending) order: bit-reproducible.
            *o = self.fines[self.off[c] as usize..self.off[c + 1] as usize]
                .iter()
                .map(|f| fine_values[f.get()])
                .sum();
        }
        Ok(out)
    }

    /// Copy each coarse value to all its fine children (intensive quantities: fluxes, correction factors).
    pub fn prolong_copy(&self, coarse_values: &[f64]) -> Result<Vec<f64>> {
        if coarse_values.len() != self.coarse_len() {
            return Err(MeshError::Hierarchy(format!(
                "expected {} coarse values, got {}",
                self.coarse_len(),
                coarse_values.len()
            )));
        }
        Ok(self
            .coarse_of
            .iter()
            .map(|c| coarse_values[c.get()])
            .collect())
    }

    /// Compose with the map from this map's coarse mesh to a still coarser mesh.
    pub fn compose(&self, next: &LevelMap) -> Result<LevelMap> {
        if next.fine != self.coarse || next.fine_hash != self.coarse_hash {
            return Err(MeshError::Hierarchy(
                "maps do not chain: next.fine is not this map's coarse mesh".into(),
            ));
        }
        let coarse_of: Vec<CellIx> = self.coarse_of.iter().map(|&c| next.coarse_of(c)).collect();
        let face_map = match (&self.face_map, &next.face_map) {
            (Some(a), Some(b)) => Some(
                a.iter()
                    .map(|&g| if g == NO_CELL { NO_CELL } else { b[g as usize] })
                    .collect(),
            ),
            _ => None,
        };
        let mut off = vec![0u32; next.coarse_len() + 1];
        for c in &coarse_of {
            off[c.get() + 1] += 1;
        }
        for i in 0..next.coarse_len() {
            off[i + 1] += off[i];
        }
        let mut cursor = off.clone();
        let mut fines = vec![CellIx(0); coarse_of.len()];
        for (f, c) in coarse_of.iter().enumerate() {
            fines[cursor[c.get()] as usize] = CellIx::new(f);
            cursor[c.get()] += 1;
        }
        Ok(LevelMap {
            fine: self.fine,
            coarse: next.coarse,
            fine_hash: self.fine_hash,
            coarse_hash: next.coarse_hash,
            coarse_of,
            off,
            fines,
            face_map,
        })
    }
}

fn check_face_map(
    fine: &dyn MeshRead,
    coarse: &dyn MeshRead,
    coarse_of: &[CellIx],
    fm: &[u32],
) -> Result<()> {
    let (nff, ncf) = (
        fine.count(MeshEntityKind::Face),
        coarse.count(MeshEntityKind::Face),
    );
    if fm.len() != nff {
        return Err(MeshError::Hierarchy(format!(
            "face map has {} entries for {nff} fine faces",
            fm.len()
        )));
    }
    for (f, &g) in fm.iter().enumerate() {
        let fi = FaceIx::new(f);
        let cl = coarse_of[fine.face_left(fi).get()];
        let cr = fine.face_right(fi).map(|r| coarse_of[r.get()]);
        let bad =
            |why: &str| MeshError::Hierarchy(format!("face map entry for fine face {f}: {why}"));
        if g != NO_CELL && g as usize >= ncf {
            return Err(bad("coarse face out of range"));
        }
        match (cr, g) {
            (Some(r), g) if r != cl => {
                if g == NO_CELL {
                    return Err(bad(
                        "face between two coarse cells must map to a coarse face",
                    ));
                }
                let gi = FaceIx(g);
                let (lo, hi) = if cl < r { (cl, r) } else { (r, cl) };
                if coarse.face_left(gi) != lo || coarse.face_right(gi) != Some(hi) {
                    return Err(bad("coarse face does not separate the two coarse cells"));
                }
            }
            (Some(_), g) => {
                if g != NO_CELL {
                    return Err(bad("face inside a coarse cell must not map"));
                }
            }
            (None, g) => {
                if g == NO_CELL {
                    if fine.face_kind(fi) == crate::index::FaceKind::Boundary {
                        return Err(bad(
                            "a boundary fine face must map to a boundary coarse face",
                        ));
                    }
                } else {
                    let gi = FaceIx(g);
                    if coarse.face_left(gi) != cl || coarse.face_right(gi).is_some() {
                        return Err(bad(
                            "one-cell fine face must map to a one-cell coarse face of its coarse cell",
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

/// Read access to the meshes of one model.
pub trait MeshCatalog {
    /// Ids of all meshes, ascending.
    fn mesh_ids(&self) -> &[EntityId];
    /// A mesh by id.
    fn mesh(&self, id: EntityId) -> Option<&dyn MeshRead>;
    /// All maps whose fine mesh is `fine`, as `(coarse id, map)` in ascending coarse id.
    fn maps_from(&self, fine: EntityId) -> Vec<(EntityId, &LevelMap)>;
    /// All maps whose coarse mesh is `coarse`, as `(fine id, map)` in ascending fine id.
    fn maps_to(&self, coarse: EntityId) -> Vec<(EntityId, &LevelMap)>;
    /// Instanced views stored in the catalog, ascending by id.
    fn views(&self) -> &[InstancedView];
}

/// Write access to a catalog.
pub trait CatalogEdit: MeshCatalog {
    /// Insert a mesh. Its id must be new.
    fn insert_mesh(&mut self, mesh: Box<dyn MeshRead>) -> Result<()>;
    /// Link two meshes already in the catalog with a map. Rejects unknown meshes, a map built for
    /// meshes whose connectivity has since changed, and a second map for the same pair.
    fn link(&mut self, map: LevelMap) -> Result<()>;
    /// Add an instanced view; every prototype must already be in the catalog.
    fn insert_view(&mut self, view: InstancedView) -> Result<()>;
}
