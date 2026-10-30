//! Row 10 (and the side half of row 9): an exported Exodus file read back with
//! `sm_exodus::read::read_exodus` and compared with the in-process mesh. Side numbering is the
//! Exodus II manual's, typed here independently of `sm_exodus::tables`.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::EntityId;
use sm_mesh_api::{CellIx, FaceIx, MeshRead};

use super::meshchk::cell_vertices;

/// Exodus side -> 1-based local nodes, per element type (Exodus II manual).
fn side_nodes(elem_type: &str, side: i64) -> Option<&'static [usize]> {
    const HEX: [&[usize]; 6] = [&[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 4, 8, 7], &[1, 5, 8, 4], &[1, 4, 3, 2], &[5, 6, 7, 8]];
    const TET: [&[usize]; 4] = [&[1, 2, 4], &[2, 3, 4], &[1, 4, 3], &[1, 3, 2]];
    const WEDGE: [&[usize]; 5] = [&[1, 2, 5, 4], &[2, 3, 6, 5], &[1, 4, 6, 3], &[1, 3, 2], &[4, 5, 6]];
    let t: &[&[usize]] = match elem_type.to_ascii_uppercase().as_str() {
        "HEX8" | "HEX" => &HEX,
        "TET4" | "TETRA" | "TETRA4" => &TET,
        "WEDGE6" | "WEDGE" => &WEDGE,
        _ => return None,
    };
    usize::try_from(side - 1).ok().and_then(|i| t.get(i).copied())
}

/// What row 10 found, as failure messages (empty = pass) plus facts for the report.
#[derive(Debug, Default)]
pub struct ExoCheck {
    pub problems: Vec<String>,
    pub blocks: Vec<(i64, String, usize)>,
}

/// Check `out.exo` against the hybrid mesh `m`. `vols[k]` is the volume of block `k + 1` and
/// `side_faces[k]` the model faces of side set `k + 1` (all faces of that volume).
pub fn check_hybrid(bytes: &[u8], m: &dyn MeshRead, vols: &[EntityId; 3], side_faces: &[BTreeSet<EntityId>; 3], expect_types: [&str; 3]) -> ExoCheck {
    let mut r = ExoCheck::default();
    let d = match sm_exodus::read::read_exodus(bytes) {
        Ok(d) => d,
        Err(x) => {
            r.problems.push(format!("read_exodus: {x}"));
            return r;
        }
    };
    // Nodes = mesh vertices, coordinates bit-equal.
    if d.num_nodes != m.vertex_count() {
        r.problems.push(format!("num_nodes {} != mesh vertices {}", d.num_nodes, m.vertex_count()));
    } else if let Some(map) = &d.node_num_map
        && map.iter().enumerate().any(|(i, &n)| n != i as i64 + 1)
    {
        r.problems.push("node_num_map is not the identity".into());
    } else {
        let mut bad = 0;
        for i in 0..d.num_nodes {
            let q = m.point(i);
            let c = [q.x, q.y, q.z];
            for (ax, col) in d.coords.iter().enumerate() {
                if col.get(i).map(|x| x.to_bits()) != Some(c[ax].to_bits()) {
                    bad += 1;
                }
            }
        }
        if bad > 0 {
            r.problems.push(format!("{bad} node coordinates differ from the in-process mesh (bitwise)"));
        }
    }
    // Blocks: element types, counts and node sets per volume.
    let mut cells_of: BTreeMap<EntityId, Vec<Vec<u32>>> = BTreeMap::new();
    for c in 0..m.cell_count() {
        let c = CellIx::new(c);
        cells_of.entry(m.cell_class(c).entity).or_default().push(cell_vertices(m, c));
    }
    let mut elem_block: Vec<(i64, String, Vec<i64>)> = Vec::new(); // per element: (block id, type, nodes)
    for b in &d.blocks {
        r.blocks.push((b.id, b.elem_type.clone(), b.num_elem));
        let npe = if b.num_elem == 0 { 0 } else { b.connectivity.len() / b.num_elem };
        for k in 0..b.num_elem {
            elem_block.push((b.id, b.elem_type.clone(), b.connectivity.get(k * npe..(k + 1) * npe).map(<[i64]>::to_vec).unwrap_or_default()));
        }
    }
    for (k, vol) in vols.iter().enumerate() {
        let id = k as i64 + 1;
        let want = expect_types[k];
        let blocks: Vec<_> = d.blocks.iter().filter(|b| b.id == id).collect();
        if blocks.len() != 1 || !blocks[0].elem_type.eq_ignore_ascii_case(want) {
            r.problems.push(format!("block {id}: want one {want} block, found {:?}", blocks.iter().map(|b| (&b.elem_type, b.num_elem)).collect::<Vec<_>>()));
            continue;
        }
        let mut want_sets: Vec<Vec<u32>> = cells_of.get(vol).cloned().unwrap_or_default();
        want_sets.sort();
        let mut got: Vec<Vec<u32>> = elem_block
            .iter()
            .filter(|e| e.0 == id)
            .map(|e| {
                let mut v: Vec<u32> = e.2.iter().map(|&n| (n - 1) as u32).collect();
                v.sort_unstable();
                v
            })
            .collect();
        got.sort();
        if got != want_sets {
            r.problems.push(format!("block {id}: {} elements vs {} cells on {vol}, node sets differ", got.len(), want_sets.len()));
        }
    }
    // Side sets: reconstruct face node sets under the Exodus numbering.
    for (k, faces) in side_faces.iter().enumerate() {
        let id = k as i64 + 1;
        let Some(ss) = d.sidesets.iter().find(|s| s.id == id) else {
            r.problems.push(format!("no side set {id}"));
            continue;
        };
        let mut got = Vec::new();
        let mut wrong_block = 0;
        for (&el, &side) in ss.elems.iter().zip(&ss.sides) {
            let Some((bid, ty, nodes)) = usize::try_from(el - 1).ok().and_then(|i| elem_block.get(i)) else {
                r.problems.push(format!("side set {id}: element {el} out of range"));
                break;
            };
            if *bid != id {
                wrong_block += 1;
            }
            let Some(local) = side_nodes(ty, side) else {
                r.problems.push(format!("side set {id}: side {side} of {ty} has no table entry"));
                break;
            };
            let mut v: Vec<u32> = local.iter().filter_map(|&l| nodes.get(l - 1)).map(|&n| (n - 1) as u32).collect();
            v.sort_unstable();
            got.push(v);
        }
        got.sort();
        let mut want: Vec<Vec<u32>> = (0..m.face_count())
            .map(FaceIx::new)
            .filter(|&f| faces.contains(&m.face_class(f).entity))
            .map(|f| {
                let mut v: Vec<u32> = m.face_vertices(f).iter().map(|x| x.get() as u32).collect();
                v.sort_unstable();
                v
            })
            .collect();
        want.sort();
        if got != want {
            r.problems.push(format!("side set {id}: {} sides reconstruct a different face set than the {} mesh faces of the sideset", got.len(), want.len()));
        }
        if wrong_block > 0 {
            r.problems.push(format!("side set {id}: {wrong_block} sides are on an element outside block {id}"));
        }
    }
    r
}

/// `poly.exo`: every block is `NFACED` and the element count per block id equals the poly cells
/// classified on that block's volume.
pub fn check_poly(bytes: &[u8], pm: &dyn MeshRead, vols: &[EntityId; 3]) -> ExoCheck {
    let mut r = ExoCheck::default();
    let d = match sm_exodus::read::read_exodus(bytes) {
        Ok(d) => d,
        Err(x) => {
            r.problems.push(format!("read_exodus: {x}"));
            return r;
        }
    };
    let mut per: BTreeMap<EntityId, usize> = BTreeMap::new();
    for c in 0..pm.cell_count() {
        *per.entry(pm.cell_class(CellIx::new(c)).entity).or_insert(0) += 1;
    }
    for b in &d.blocks {
        r.blocks.push((b.id, b.elem_type.clone(), b.num_elem));
        if !b.elem_type.eq_ignore_ascii_case("NFACED") {
            r.problems.push(format!("block {} ({}) is {}, not NFACED", b.id, b.name, b.elem_type));
        }
    }
    for (k, vol) in vols.iter().enumerate() {
        let id = k as i64 + 1;
        let n: usize = d.blocks.iter().filter(|b| b.id == id).map(|b| b.num_elem).sum();
        let want = per.get(vol).copied().unwrap_or(0);
        if n != want {
            r.problems.push(format!("poly block {id}: {n} elements, {want} poly cells on {vol}"));
        }
    }
    r
}
