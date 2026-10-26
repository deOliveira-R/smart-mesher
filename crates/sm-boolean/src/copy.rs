//! Deep copies of volumes: for `keep = true` the kernel works on a copy of the operands and
//! leaves the originals untouched.

use std::collections::BTreeMap;

use sm_core::EntityId;
use sm_topo_api::{Carrier, CoedgeSpec, Dim, FaceUseSpec, Incidence, Sense, ShellKind, ShellSpec, Side, TopologyEdit, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;

/// Copy `volume` with every cell it is built from (vertices, edges, faces, shells, pcurves,
/// flags, tolerance overrides) and give the copy its own body. The copies share the curve and
/// surface handles with the originals: a later `replace_surface` on one affects both. Cells are
/// created in ascending order of their original ids (vertices, edges, faces, volume, body).
/// Returns the new volume.
pub(crate) fn copy_volume<E: TopologyEdit + ?Sized>(topo: &mut E, volume: EntityId) -> Result<EntityId, BooleanError> {
    let cells = topo.closure(volume);
    let of_dim = |t: &E, d: Dim| -> Vec<EntityId> { cells.iter().copied().filter(|c| t.dim_of(*c) == Some(d)).collect() };
    let (verts, edges, faces) = (of_dim(&*topo, Dim::Vertex), of_dim(&*topo, Dim::Edge), of_dim(&*topo, Dim::Face));
    let mut vmap: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    for v in verts {
        let p = geo::vertex_point(&*topo, v).ok_or(BooleanError::Unsupported("a vertex has no point"))?;
        let n = topo.make_vertex(p)?;
        let fl = topo.flags(v);
        if fl != sm_topo_api::CellFlags::NONE {
            topo.set_flags(n, fl)?;
        }
        if let Some(t) = topo.tolerance_override(v) {
            topo.raise_tolerance(n, t)?;
        }
        vmap.insert(v, n);
    }
    let mut emap: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    for e in edges {
        let (s, t) = topo.edge_ends(e).ok_or(BooleanError::Unsupported("an edge has no ends"))?;
        let carrier = match topo.carrier(e) {
            Carrier::Curve(c) => Some(*c),
            _ => None,
        };
        let n = topo.make_edge(vmap[&s], vmap[&t], carrier)?;
        let fl = topo.flags(e);
        if fl != sm_topo_api::CellFlags::NONE {
            topo.set_flags(n, fl)?;
        }
        if let Some(tol) = topo.tolerance_override(e) {
            topo.raise_tolerance(n, tol)?;
        }
        emap.insert(e, n);
    }
    let mut fmap: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    for f in faces {
        let rings: Vec<Vec<CoedgeSpec>> = topo
            .loops(f)
            .map(|l| l.entries.iter().map(|e| CoedgeSpec { edge: emap[&e.cell], sense: if e.side == Side::Right { Sense::Reversed } else { Sense::Forward } }).collect())
            .collect();
        let refs: Vec<&[CoedgeSpec]> = rings.iter().map(Vec::as_slice).collect();
        let carrier = match topo.carrier(f) {
            Carrier::Surface(s) => Some(*s),
            _ => None,
        };
        let n = topo.make_face(&refs, carrier)?;
        // Pcurves follow their coedges.
        let keys: Vec<sm_topo_api::CoedgeKey> = topo.coedges(f).map(|c| c.key).collect();
        for k in keys {
            if let Some(pc) = topo.pcurve(k).cloned() {
                topo.set_pcurve(sm_topo_api::CoedgeKey { face: n, edge: emap[&k.edge], side: k.side }, Some(pc))?;
            }
        }
        let fl = topo.flags(f);
        if fl != sm_topo_api::CellFlags::NONE {
            topo.set_flags(n, fl)?;
        }
        if let Some(tol) = topo.tolerance_override(f) {
            topo.raise_tolerance(n, tol)?;
        }
        fmap.insert(f, n);
    }
    // Shells.
    let mut rings: BTreeMap<u16, Vec<FaceUseSpec>> = BTreeMap::new();
    for inc in topo.boundary(volume) {
        if inc.ring == Incidence::EMBEDDED {
            return Err(BooleanError::NonManifoldInput(volume));
        }
        rings.entry(inc.ring).or_default().push(FaceUseSpec { face: fmap[&inc.cell], side: inc.side });
    }
    let kinds: Vec<(u16, ShellKind)> = rings.keys().map(|&r| (r, topo.shell_kind(volume, r).unwrap_or(if r == 0 { ShellKind::Outer } else { ShellKind::Void }))).collect();
    let specs: Vec<ShellSpec<'_>> = kinds.iter().map(|(r, k)| ShellSpec { kind: *k, uses: &rings[r] }).collect();
    let nv = topo.make_volume(&specs)?;
    topo.make_body(&[nv])?;
    Ok(nv)
}
