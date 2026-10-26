//! Block and set layout: which cells go to which block, which sides to which side set. No I/O.

use crate::error::{ExodusError, Result};
use crate::options::{BlockSelection, ExodusOptions};
use crate::tables::{exodus_element_type, exodus_side};
use rayon::prelude::*;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, LoopRelation, MeshDim, MeshError, MeshRead, SetMembers, Side, VertexIx, loop_relation,
};
use std::collections::BTreeMap;

/// Largest name Exodus allows.
pub(crate) const MAX_NAME: usize = 255;

/// One element block: cells of one shape, ascending by cell index.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockPlan {
    /// Block id.
    pub id: i64,
    /// Block name.
    pub name: String,
    /// Shape of every cell in the block.
    pub shape: CellShape,
    /// Dense cell indices, ascending.
    pub cells: Vec<u32>,
    /// 0-based number of the block's first element in the file.
    pub first_elem: usize,
}

/// One side set: parallel arrays of 1-based element numbers and Exodus sides.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SideSetPlan {
    /// Side set id.
    pub id: i64,
    /// Name.
    pub name: String,
    /// 1-based element numbers.
    pub elems: Vec<u32>,
    /// Exodus side numbers (1-based).
    pub sides: Vec<u8>,
}

/// One node set.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeSetPlan {
    /// Node set id.
    pub id: i64,
    /// Name.
    pub name: String,
    /// 0-based vertex indices, ascending.
    pub nodes: Vec<u32>,
}

/// Block and set layout of an export.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExodusPlan {
    /// 2 or 3.
    pub num_dim: u8,
    /// Number of nodes (all vertices).
    pub num_nodes: usize,
    /// Number of elements (active cells).
    pub num_elem: usize,
    /// Effective `maximum_name_length`.
    pub max_name_length: u32,
    /// Element blocks, ascending by id.
    pub blocks: Vec<BlockPlan>,
    /// Side sets (empty ones omitted).
    pub sidesets: Vec<SideSetPlan>,
    /// Node sets (empty ones omitted).
    pub nodesets: Vec<NodeSetPlan>,
    /// Side-set entries dropped because their cell is inactive.
    pub skipped_sides: usize,
    /// Cell index of element `e` (0-based), block by block; `elem_num_map[e] = elem_order[e] + 1`.
    pub elem_order: Vec<u32>,
}

struct Candidate {
    set: usize,
    id: Option<i64>,
}

/// Compute the block and set layout (see the crate docs for the rules).
///
/// Errors: `EmptyMesh`, `Unsupported` (high-order cells, polyhedra with `polyhedra = false`,
/// dimension-2 meshes off the z = 0 plane), `OverlappingBlocks`, `BadLabel`, `NameTooLong`,
/// `Mesh(InconsistentOrientation)` for a side that matches no face of its cell.
pub fn plan(mesh: &dyn MeshRead, opts: &ExodusOptions) -> Result<ExodusPlan> {
    let nc = mesh.cell_count();
    let nv = mesh.vertex_count();
    let active: Vec<bool> = (0..nc).map(|c| mesh.cell_active(CellIx::new(c))).collect();
    let n_active = active.iter().filter(|a| **a).count();
    if nv == 0 || n_active == 0 {
        return Err(ExodusError::EmptyMesh);
    }
    for c in 0..nc {
        if mesh.high_order_cell(CellIx::new(c)).is_some() {
            return Err(ExodusError::Unsupported("high-order cells".into()));
        }
    }
    let num_dim = if mesh.dim() == MeshDim::Two {
        if mesh.coords().z[..nv].iter().any(|z| *z != 0.0) {
            return Err(ExodusError::Unsupported("dimension-2 mesh off the z = 0 plane (shell surfaces)".into()));
        }
        2
    } else {
        3
    };
    let sets = mesh.sets();

    // ---- candidate block sets -----------------------------------------------------------------
    let mut cands: Vec<Candidate> = Vec::new();
    match &opts.blocks {
        BlockSelection::Labelled => {
            for (i, s) in sets.iter().enumerate() {
                if matches!(s.members, SetMembers::Cells(_)) && s.label.is_some() {
                    cands.push(Candidate { set: i, id: s.label });
                }
            }
        }
        BlockSelection::Names(names) => {
            for n in names {
                match sets.iter().find(|s| &s.name == n) {
                    Some(s) if matches!(s.members, SetMembers::Cells(_)) => {}
                    _ => return Err(ExodusError::Mesh(MeshError::State(format!("no cell set '{n}'")))),
                }
            }
            for (i, s) in sets.iter().enumerate() {
                if matches!(s.members, SetMembers::Cells(_)) && names.contains(&s.name) {
                    cands.push(Candidate { set: i, id: s.label });
                }
            }
        }
        BlockSelection::None => {}
    }
    let mut max_label = 0i64;
    let mut used = std::collections::BTreeSet::new();
    for c in &cands {
        if let Some(l) = c.id {
            if l <= 0 || !used.insert(l) {
                return Err(ExodusError::BadLabel { set: sets[c.set].name.clone(), label: l });
            }
            max_label = max_label.max(l);
        }
    }

    // ---- disjointness and grouping by shape ---------------------------------------------------
    let mut owner = vec![u32::MAX; nc];
    let mut groups: Vec<BTreeMap<u8, Vec<u32>>> = Vec::with_capacity(cands.len());
    for (ci, c) in cands.iter().enumerate() {
        let SetMembers::Cells(cells) = &sets[c.set].members else { continue };
        let mut g: BTreeMap<u8, Vec<u32>> = BTreeMap::new();
        for cell in cells {
            let k = cell.get();
            if !active[k] {
                continue;
            }
            if owner[k] != u32::MAX {
                return Err(ExodusError::OverlappingBlocks {
                    cell: cell.0,
                    a: sets[cands[owner[k] as usize].set].name.clone(),
                    b: sets[c.set].name.clone(),
                });
            }
            owner[k] = ci as u32;
            g.entry(mesh.cell_shape(*cell).code()).or_default().push(cell.0);
        }
        groups.push(g);
    }

    // ---- blocks ------------------------------------------------------------------------------
    let mut next_id = max_label + 1;
    let mut fresh = || {
        let v = next_id;
        next_id += 1;
        v
    };
    let shape_of = |code: u8| CellShape::from_code(code).unwrap_or(CellShape::Polyhedron);
    let mut blocks: Vec<BlockPlan> = Vec::new();
    let mut base_ids: Vec<i64> = Vec::new();
    for c in &cands {
        base_ids.push(c.id.unwrap_or_else(&mut fresh));
    }
    for (ci, g) in groups.iter().enumerate() {
        let name = &sets[cands[ci].set].name;
        for (n, (code, cells)) in g.iter().enumerate() {
            let shape = shape_of(*code);
            let (id, bname) = if n == 0 {
                (base_ids[ci], name.clone())
            } else {
                (fresh(), format!("{name}_{}", exodus_element_type(shape)))
            };
            blocks.push(BlockPlan { id, name: bname, shape, cells: cells.clone(), first_elem: 0 });
        }
    }
    let mut rest: BTreeMap<u8, Vec<u32>> = BTreeMap::new();
    for c in 0..nc {
        if active[c] && owner[c] == u32::MAX {
            rest.entry(mesh.cell_shape(CellIx::new(c)).code()).or_default().push(c as u32);
        }
    }
    for (code, cells) in rest {
        let shape = shape_of(code);
        blocks.push(BlockPlan {
            id: fresh(),
            name: format!("unassigned_{}", exodus_element_type(shape)),
            shape,
            cells,
            first_elem: 0,
        });
    }
    if !opts.polyhedra && blocks.iter().any(|b| b.shape == CellShape::Polyhedron) {
        return Err(ExodusError::Unsupported("polyhedral cells need ExodusOptions::polyhedra = true (NFACED)".into()));
    }
    blocks.sort_by_key(|b| b.id);
    let mut elem_order = Vec::with_capacity(n_active);
    let mut cell_elem = vec![0u32; nc];
    for b in &mut blocks {
        b.first_elem = elem_order.len();
        for &c in &b.cells {
            elem_order.push(c);
            cell_elem[c as usize] = elem_order.len() as u32;
        }
    }

    // ---- node sets ---------------------------------------------------------------------------
    let node_cands: Vec<usize> = (0..sets.len())
        .filter(|&i| matches!(sets[i].members, SetMembers::Vertices(_) | SetMembers::Points(_)))
        .collect();
    let node_ids = assign_ids(&node_cands, sets)?;
    let mut nodesets = Vec::new();
    for (&i, &id) in node_cands.iter().zip(&node_ids) {
        let nodes: Vec<u32> = match &sets[i].members {
            SetMembers::Vertices(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Points(v) => v.iter().map(|x| x.0).filter(|x| (*x as usize) < nv).collect(),
            _ => continue,
        };
        if !nodes.is_empty() {
            nodesets.push(NodeSetPlan { id, name: sets[i].name.clone(), nodes });
        }
    }

    // ---- side sets ---------------------------------------------------------------------------
    let side_cands: Vec<usize> = (0..sets.len())
        .filter(|&i| match sets[i].members {
            SetMembers::Sides(_) => true,
            SetMembers::Faces(_) => opts.faces_as_sidesets,
            _ => false,
        })
        .collect();
    let side_ids = assign_ids(&side_cands, sets)?;
    let mut sidesets = Vec::new();
    let mut skipped_sides = 0usize;
    for (&i, &id) in side_cands.iter().zip(&side_ids) {
        let entries: Vec<(FaceIx, Side)> = match &sets[i].members {
            SetMembers::Sides(v) => v.iter().map(|sf| (sf.face(), sf.side())).collect(),
            SetMembers::Faces(v) => v.iter().map(|f| (*f, Side::Left)).collect(),
            _ => continue,
        };
        let results: Vec<Result<Option<(u32, u8)>>> = entries
            .par_iter()
            .with_min_len(1024)
            .map(|&(f, side)| {
                let cell = match side {
                    Side::Left => mesh.face_left(f),
                    Side::Right => mesh.face_right(f).ok_or(MeshError::InconsistentOrientation(f))?,
                };
                if !active[cell.get()] {
                    return Ok(None);
                }
                let s = side_of(mesh, cell, f, side)?;
                Ok(Some((cell_elem[cell.get()], s)))
            })
            .collect();
        let mut set = SideSetPlan { id, name: sets[i].name.clone(), elems: Vec::new(), sides: Vec::new() };
        for r in results {
            match r? {
                Some((e, s)) => {
                    set.elems.push(e);
                    set.sides.push(s);
                }
                None => skipped_sides += 1,
            }
        }
        if !set.elems.is_empty() {
            sidesets.push(set);
        }
    }

    // ---- names -------------------------------------------------------------------------------
    let longest = blocks
        .iter()
        .map(|b| &b.name)
        .chain(sidesets.iter().map(|s| &s.name))
        .chain(nodesets.iter().map(|s| &s.name))
        .max_by_key(|n| n.len());
    if let Some(n) = longest
        && n.len() > MAX_NAME
    {
        return Err(ExodusError::NameTooLong { name: n.clone() });
    }
    if opts.max_name_length as usize > MAX_NAME {
        return Err(ExodusError::TooLarge { what: "max_name_length".into(), value: u64::from(opts.max_name_length), limit: MAX_NAME as u64 });
    }
    let max_name_length = opts.max_name_length.max(longest.map_or(0, |n| n.len() as u32)).max(1);

    Ok(ExodusPlan { num_dim, num_nodes: nv, num_elem: n_active, max_name_length, blocks, sidesets, nodesets, skipped_sides, elem_order })
}

/// Ids of a family of sets: the label, else the next unused positive id above the maximum label.
fn assign_ids(idx: &[usize], sets: &[sm_mesh_api::MeshSet]) -> Result<Vec<i64>> {
    let mut used = std::collections::BTreeSet::new();
    let mut max = 0i64;
    for &i in idx {
        if let Some(l) = sets[i].label {
            if l <= 0 || !used.insert(l) {
                return Err(ExodusError::BadLabel { set: sets[i].name.clone(), label: l });
            }
            max = max.max(l);
        }
    }
    let mut next = max;
    Ok(idx
        .iter()
        .map(|&i| {
            sets[i].label.unwrap_or_else(|| {
                next += 1;
                next
            })
        })
        .collect())
}

/// Exodus side of `cell` for the face entry `(f, side)`.
fn side_of(mesh: &dyn MeshRead, cell: CellIx, f: FaceIx, side: Side) -> Result<u8> {
    let want_rel = if side == Side::Left { LoopRelation::Same } else { LoopRelation::Reversed };
    let shape = mesh.cell_shape(cell);
    let fv = mesh.face_vertices(f);
    let bad = || ExodusError::Mesh(MeshError::InconsistentOrientation(f));
    match shape {
        CellShape::Polyhedron => {
            let cf = mesh.cell_faces(cell);
            let pos = cf.binary_search_by_key(&f, |sf| sf.face()).map_err(|_| ExodusError::Mesh(MeshError::NotIncident(f, cell)))?;
            if cf[pos].side() != side {
                return Err(bad());
            }
            u8::try_from(pos + 1).map_err(|_| ExodusError::TooLarge { what: "polyhedron faces".into(), value: pos as u64 + 1, limit: 255 })
        }
        CellShape::Polygon => {
            let corners = mesh.cell_corners(cell).ok_or_else(bad)?;
            let n = corners.len();
            for i in 0..n {
                let want = [corners[i], corners[(i + 1) % n]];
                if loop_relation(fv, &want) == Some(want_rel) {
                    return u8::try_from(i + 1).map_err(|_| ExodusError::TooLarge { what: "polygon edges".into(), value: i as u64 + 1, limit: 255 });
                }
            }
            Err(bad())
        }
        _ => {
            let corners = mesh.cell_corners(cell).ok_or_else(bad)?;
            for (k, lf) in shape.local_faces().iter().enumerate() {
                let mut want = [VertexIx(0); 4];
                for (w, &l) in want.iter_mut().zip(lf.iter()) {
                    *w = *corners.get(l as usize).ok_or_else(bad)?;
                }
                if loop_relation(fv, &want[..lf.len()]) == Some(want_rel) {
                    return exodus_side(shape, k).ok_or_else(bad);
                }
            }
            Err(bad())
        }
    }
}
