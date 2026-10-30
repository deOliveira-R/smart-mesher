//! Repair of dual cells that are not star-shaped about their volume centroid (the
//! `GeomSideTet` rule of `validate_geometry`), as at reflex interface or boundary edges.
//!
//! A failing cell `(v, R)` is rebuilt from its *corners*: for each tet `t` of `R` at `v` the
//! region `{lambda_v >= lambda_w for all w}` of `t` (barycentric coordinates). A corner is a
//! convex hexahedron with planar quads: three edge kites `(m_vw, fc, c_t, fc')` in the planes
//! `lambda_v = lambda_w` and three quads `(v, m_vw, fc_f, m_vx)` in the faces of `t`. The corners
//! tile the cell exactly once its edge faces are refined (`c_1, fc_12, c_2, ...`, one kite per
//! tet), so the corners are grouped into wedges, each grown greedily while it stays star-shaped
//! about its own centroid (with a margin) and edge-manifold. Wedges are output cells; the quads
//! between wedges are new interior faces. Volume is unchanged: the faces only subdivide space.

use crate::agglom;
use crate::geom::{self, V3};
use crate::polydata::PolyData;
use crate::prep::{NONE, Prep};
use rayon::prelude::*;
use sm_mesh_api::{CellIx, Side};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Side tetrahedra of an accepted wedge exceed this fraction of its volume, so the strict
/// `> 0` rule of `validate_geometry` holds despite rounding differences.
const MARGIN: f64 = 1e-9;

/// Side tetrahedra at or below this fraction of the cell volume count as failing in
/// [`bad_cells`] (rounding level; far below any side tetrahedron of a regular cell).
const DETECT_MARGIN: f64 = 1e-12;

/// Split decisions: per split dual cell (ckey index), the wedge of each of its tets.
#[derive(Default)]
pub(crate) struct Split {
    /// `cell -> [(tet, wedge)]` sorted by tet.
    pub cells: BTreeMap<u32, Vec<(u32, u32)>>,
}

impl Split {
    pub fn is_split(&self, c: u32) -> bool {
        self.cells.contains_key(&c)
    }
    /// Wedge of tet `t` in cell `c` (0 for an unsplit cell).
    pub fn wedge(&self, c: u32, t: u32) -> u32 {
        match self.cells.get(&c) {
            None => 0,
            Some(v) => v.binary_search_by_key(&t, |e| e.0).map_or(0, |i| v[i].1),
        }
    }
    /// Number of output cells of cell `c`.
    pub fn n_wedges(&self, c: u32) -> u32 {
        self.cells.get(&c).map_or(1, |v| v.iter().map(|e| e.1).max().map_or(1, |m| m + 1))
    }
}

/// The six oriented quads of the corner of tet `t` at `v` (outward from the corner, as virtual
/// point ids) and, for the last three (in the faces of `t`), the neighbouring tet of the same
/// region across that face (`NONE` at a region boundary).
pub(crate) struct Corner {
    pub quads: [[u32; 4]; 6],
    pub nb: [u32; 3],
}

/// Corner of tet `t` at vertex `v` in `region`.
pub(crate) fn corner(p: &Prep<'_>, v: u32, region: u32, t: u32) -> Option<Corner> {
    let tv = p.tv[t as usize];
    let mut quads = [[0u32; 4]; 6];
    let mut k = 0;
    for &w in tv.iter().filter(|&&w| w != v) {
        let s = p.slot_for(v, w, region)?;
        let e = p.s_edge[s];
        let (en, ex) = p.entry_exit(t, e)?;
        let kite = [p.edge_mid(e), p.p_tri(en), t, p.p_tri(ex)];
        // kites wind about a -> b (outward from a's cell)
        quads[k] = if p.edge_ends(e).0 == v { kite } else { [kite[0], kite[3], kite[2], kite[1]] };
        k += 1;
    }
    let mut nb = [NONE; 3];
    let mut j = 0;
    for &sf in p.src.cell_faces(CellIx(t)) {
        let l = p.src.face_vertices(sf.face());
        if !l.iter().any(|x| x.0 == v) {
            continue;
        }
        let mut o = [l[0].0, l[1].0, l[2].0];
        if sf.side() == Side::Right {
            o.swap(0, 2);
        }
        let r = o.iter().position(|&x| x == v)?;
        let (a, b) = (o[(r + 1) % 3], o[(r + 2) % 3]);
        let ea = p.ekeys.binary_search(&crate::prep::pair_key(v, a)).ok()? as u32;
        let eb = p.ekeys.binary_search(&crate::prep::pair_key(v, b)).ok()? as u32;
        quads[3 + j] = [p.p_vert(v), p.edge_mid(ea), p.p_tri(sf.face().0), p.edge_mid(eb)];
        nb[j] = p.src.across(sf).map(|c| c.0).filter(|&c| p.treg[c as usize] == region).unwrap_or(NONE);
        j += 1;
    }
    (k == 3 && j == 3).then_some(Corner { quads, nb })
}

/// Volume and centroid of a closed set of oriented loops (fan rule with vertex-mean apex).
fn vol_centroid(loops: &[[V3; 4]]) -> (f64, V3) {
    let o = loops[0][0];
    let (mut vol, mut mom) = (0.0, [0.0; 3]);
    for l in loops {
        let ap = geom::mean(l);
        for i in 0..4 {
            let (a, b) = (l[i], l[(i + 1) % 4]);
            let v = geom::dot(geom::sub(ap, o), geom::cross(geom::sub(a, o), geom::sub(b, o))) / 6.0;
            vol += v;
            for k in 0..3 {
                mom[k] += (o[k] + ap[k] + a[k] + b[k]) * (v / 4.0);
            }
        }
    }
    if vol == 0.0 {
        return (0.0, o);
    }
    (vol, [mom[0] / vol, mom[1] / vol, mom[2] / vol])
}

/// The wedge made of `members` (corner indices) is edge-manifold and star-shaped about its
/// centroid with margin.
fn wedge_ok(p: &Prep<'_>, corners: &[Corner], tets: &[u32], members: &[usize]) -> bool {
    let mut ids: SmallVec<[[u32; 4]; 64]> = SmallVec::new();
    for &m in members {
        let c = &corners[m];
        ids.extend_from_slice(&c.quads[..3]);
        for j in 0..3 {
            let inside = c.nb[j] != NONE
                && tets.binary_search(&c.nb[j]).is_ok_and(|i| members.contains(&i));
            if !inside {
                ids.push(c.quads[3 + j]);
            }
        }
    }
    // every edge used exactly twice
    let mut edges: SmallVec<[(u32, u32); 256]> = SmallVec::new();
    for q in &ids {
        for i in 0..4 {
            let (x, y) = (q[i], q[(i + 1) % 4]);
            edges.push((x.min(y), x.max(y)));
        }
    }
    edges.sort_unstable();
    let mut i = 0;
    while i < edges.len() {
        let mut j = i;
        while j < edges.len() && edges[j] == edges[i] {
            j += 1;
        }
        if j - i != 2 {
            return false;
        }
        i = j;
    }
    let loops: SmallVec<[[V3; 4]; 64]> = ids.iter().map(|q| q.map(|x| p.coord(x))).collect();
    let (vol, g) = vol_centroid(&loops);
    if vol <= 0.0 {
        return false;
    }
    let min = MARGIN * vol;
    loops.iter().all(|l| {
        let ap = geom::mean(l);
        (0..4).all(|i| {
            let (a, b) = (l[i], l[(i + 1) % 4]);
            geom::dot(geom::sub(ap, g), geom::cross(geom::sub(a, g), geom::sub(b, g))) > min
        })
    })
}

/// The tets of `region` at `v`, ascending (the union of the rings of the cell's slots).
pub(crate) fn cell_tets(p: &Prep<'_>, cell: usize) -> Vec<u32> {
    let mut tets: Vec<u32> = p
        .slots_of_cell(cell)
        .iter()
        .flat_map(|&s| p.ring[p.s_off[s as usize] as usize..p.s_off[s as usize + 1] as usize].iter().copied())
        .collect();
    tets.sort_unstable();
    tets.dedup();
    tets
}

/// Group the corners of cell `cell` into wedges: seeds by ascending tet, each wedge grown over
/// shared faces (candidates by ascending tet, repeated passes) while [`wedge_ok`] holds.
/// Returns `(tet, wedge)` sorted by tet, or `None` if a corner cannot be formed.
pub(crate) fn plan(p: &Prep<'_>, cell: usize) -> Option<Vec<(u32, u32)>> {
    let key = p.ckeys[cell];
    let (region, v) = ((key >> 32) as u32, key as u32);
    let tets = cell_tets(p, cell);
    let corners: Vec<Corner> = tets.iter().map(|&t| corner(p, v, region, t)).collect::<Option<_>>()?;
    let k = tets.len();
    let mut wedge = vec![NONE; k];
    let mut next = 0u32;
    let mut members: Vec<usize> = Vec::with_capacity(k);
    for seed in 0..k {
        if wedge[seed] != NONE {
            continue;
        }
        members.clear();
        members.push(seed);
        wedge[seed] = next;
        loop {
            let mut grew = false;
            for j in 0..k {
                if wedge[j] != NONE {
                    continue;
                }
                let adjacent = corners[j]
                    .nb
                    .iter()
                    .any(|&n| n != NONE && tets.binary_search(&n).is_ok_and(|i| wedge[i] == next));
                if !adjacent {
                    continue;
                }
                members.push(j);
                if wedge_ok(p, &corners, &tets, &members) {
                    wedge[j] = next;
                    grew = true;
                } else {
                    members.pop();
                }
            }
            if !grew {
                break;
            }
        }
        next += 1;
    }
    Some(tets.into_iter().zip(wedge).collect())
}

/// Output cells that fail the `validate_geometry` volume or side-tetrahedron rule, ascending.
/// Same rule as the validator (fan rule, vertex-mean apex). The finished mesh orders a cell's
/// faces and rotates face loops differently, which changes rounding only, so a cell is also
/// flagged when a side tetrahedron is within [`DETECT_MARGIN`] of zero.
pub(crate) fn bad_cells(d: &PolyData) -> Vec<u32> {
    (0..d.n_cells() as u32)
        .into_par_iter()
        .filter(|&c| !cell_ok(d, c as usize))
        .collect()
}

fn cell_ok(d: &PolyData, c: usize) -> bool {
    let faces = d.cell_faces(c);
    let (vol, g) = agglom::vol_centroid(d, faces);
    if vol <= 0.0 {
        return false;
    }
    let min = DETECT_MARGIN * vol;
    faces.iter().all(|sf| {
        let l = d.face_loop(sf.face().get());
        let ap = agglom::apex(d, l);
        let n = l.len();
        (0..n).all(|i| {
            let (mut a, mut b) = (d.pts[l[i] as usize], d.pts[l[(i + 1) % n] as usize]);
            if sf.side() == Side::Right {
                std::mem::swap(&mut a, &mut b);
            }
            geom::dot(geom::sub(ap, g), geom::cross(geom::sub(a, g), geom::sub(b, g))) > min
        })
    })
}
