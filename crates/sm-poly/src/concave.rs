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
use sm_mesh_api::{CellIx, Side, SignedFace};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Side tetrahedra of an accepted wedge exceed this fraction of its volume, so the strict
/// `> 0` rule of `validate_geometry` holds despite rounding differences.
const MARGIN: f64 = 1e-9;

/// Side tetrahedra at or below this fraction of the cell volume count as failing in
/// [`bad_cells`] (rounding level; far below any side tetrahedron of a regular cell).
const DETECT_MARGIN: f64 = 1e-12;

/// A face of a cell is "visible" from the vertex mean (sm-quality's `Starness` metric) when
/// `cos(area vector, apex - mean) >` this. Wedges are accepted above [`STAR_MARGIN`]; a cell is
/// flagged by [`bad_cells`] at or below [`STAR_DETECT`] (rounding level).
const STAR_MARGIN: f64 = 1e-6;
const STAR_DETECT: f64 = 1e-9;

/// Smallest `cos(area vector, apex - c)` over the loops (same rule as sm-quality's `Starness`:
/// fan area vector about the loop mean), `c` the mean of the cell's distinct vertices.
fn min_starness<'a>(c: V3, loops: impl Iterator<Item = &'a [V3]>) -> f64 {
    let mut m = f64::INFINITY;
    for l in loops {
        let n = l.len();
        let ap = geom::mean(l);
        let mut a = [0.0; 3];
        for i in 0..n {
            let x = geom::cross(geom::sub(l[i], ap), geom::sub(l[(i + 1) % n], ap));
            a = [a[0] + 0.5 * x[0], a[1] + 0.5 * x[1], a[2] + 0.5 * x[2]];
        }
        let d = geom::sub(ap, c);
        let den = geom::dot(a, a).sqrt() * geom::dot(d, d).sqrt();
        m = m.min(if den <= 1e-300 { 0.0 } else { geom::dot(a, d) / den });
    }
    m
}

/// Mean of the distinct points among `ids` (virtual point ids).
fn distinct_mean(p: &Prep<'_>, ids: &[[u32; 4]]) -> V3 {
    let mut u: SmallVec<[u32; 64]> = ids.iter().flatten().copied().collect();
    u.sort_unstable();
    u.dedup();
    let mut s = [0.0; 3];
    for &x in &u {
        let q = p.coord(x);
        s = [s[0] + q[0], s[1] + q[1], s[2] + q[2]];
    }
    let n = u.len().max(1) as f64;
    [s[0] / n, s[1] / n, s[2] / n]
}

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
    let centroid_ok = loops.iter().all(|l| {
        let ap = geom::mean(l);
        (0..4).all(|i| {
            let (a, b) = (l[i], l[(i + 1) % 4]);
            geom::dot(geom::sub(ap, g), geom::cross(geom::sub(a, g), geom::sub(b, g))) > min
        })
    });
    // also star-shaped about the vertex mean (the sm-quality `Starness` rule)
    centroid_ok && min_starness(distinct_mean(p, &ids), loops.iter().map(|l| &l[..])) > STAR_MARGIN
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

/// Sizes of the append-only arrays of [`PolyData`], to undo a tentative [`subdivide`].
struct Mark {
    pts: usize,
    faces: usize,
    f_pts: usize,
}

impl Mark {
    fn of(d: &PolyData) -> Self {
        Self { pts: d.pts.len(), faces: d.n_faces(), f_pts: d.f_pts.len() }
    }
    fn rollback(self, d: &mut PolyData) {
        d.pts.truncate(self.pts);
        d.pcls.truncate(self.pts);
        d.f_pts.truncate(self.f_pts);
        d.f_off.truncate(self.faces + 1);
        d.f_cls.truncate(self.faces);
        d.f_cells.truncate(self.faces);
        d.f_alive.truncate(self.faces);
    }
}

/// Push the pieces of face `f` (a quad: two triangles along a diagonal; a triangle: three
/// triangles about a new centroid point classified like the face, or like cell `c` for an
/// interior face). The pieces inherit the owners of `f`; `f` itself is left alone.
fn subdivide(d: &mut PolyData, f: usize, c: usize) -> SmallVec<[u32; 3]> {

    let l: SmallVec<[u32; 4]> = d.face_loop(f).iter().copied().collect();
    let cls = d.f_cls[f];
    let parts: SmallVec<[u32; 3]> = if l.len() == 4 {
        [d.push_face(&[l[0], l[1], l[2]], cls), d.push_face(&[l[0], l[2], l[3]], cls)].into_iter().collect()
    } else {
        let g = geom::mean(&[d.pts[l[0] as usize], d.pts[l[1] as usize], d.pts[l[2] as usize]]);
        let gc = match cls {
            Some(e) => sm_mesh_api::Classification::on(e),
            None => sm_mesh_api::Classification::on(d.regions[d.c_region[c] as usize]),
        };
        let gi = d.pts.len() as u32;
        d.pts.push(g);
        d.pcls.push(gc);
        (0..3).map(|i| d.push_face(&[l[i], l[(i + 1) % 3], gi], cls)).collect()
    };
    let owners = d.f_cells[f];
    for &g in &parts {
        d.f_cells[g as usize] = owners;
    }
    parts
}

/// Face list of cell `o` (edited or original) with `f` replaced by `parts` on `side_ix` (0 left).
fn spliced(
    d: &PolyData,
    edits: &[Option<Vec<SignedFace>>],
    o: usize,
    f: usize,
    parts: &[u32],
    side_ix: usize,
) -> Vec<SignedFace> {
    let side = if side_ix == 0 { Side::Left } else { Side::Right };
    let mut list = edits[o].clone().unwrap_or_else(|| d.cell_faces(o).to_vec());
    if let Some(i) = list.iter().position(|sf| sf.face().get() == f) {
        list.splice(i..=i, parts.iter().map(|&g| SignedFace::new(sm_mesh_api::FaceIx(g), side)));
    }
    list
}

/// Every neighbour of `c` across `f` still passes [`cell_ok_faces`] with `f` replaced by `parts`.
fn neighbours_ok(d: &PolyData, edits: &[Option<Vec<SignedFace>>], f: usize, c: usize, parts: &[u32]) -> bool {
    let owners = d.f_cells[f];
    owners.iter().enumerate().all(|(k, &o)| {
        o == NONE || o as usize == c || cell_ok_faces(d, &spliced(d, edits, o as usize, f, parts, k))
    })
}

/// A cell whose face sizes and vertex count match a fast-path shape (`Tet`, `Hex`, `Wedge`,
/// `Pyramid`) is tagged with that shape by the mesh, but a polyhedral mesh must keep every cell
/// `Polyhedron` (exporters declare one `POLYHEDRON` block). Such cells occur as a single-corner
/// wedge (6 quads, 8 points) and as the dual of a vertex with four neighbours (4 triangles).
/// One face of each is subdivided, in the neighbouring cell too, so the shape stays
/// `Polyhedron` and every face stays planar: a quad becomes two triangles along a diagonal, a
/// triangle becomes three triangles about its centroid (a new point on the face).
///
/// With `all` false only single-corner wedges (`wedged[c]`, cells that came from split dual
/// cells) are broken; the library default leaves other cells, e.g. the four hexahedra of a
/// single tet, as the mesh infers them.
pub(crate) fn break_primitive_cells(d: &mut PolyData, wedged: &[bool], all: bool) {
    let nc = d.n_cells();
    let mut edits: Vec<Option<Vec<SignedFace>>> = vec![None; nc];
    let mut any = false;
    for c in 0..nc {
        if !d.c_alive[c] {
            continue;
        }
        let faces: Vec<SignedFace> = edits[c].clone().unwrap_or_else(|| d.cell_faces(c).to_vec());
        if faces.len() < 4 || faces.len() > 6 {
            continue;
        }
        let (mut tris, mut quads, mut other) = (0usize, 0usize, 0usize);
        let mut pts: SmallVec<[u32; 24]> = SmallVec::new();
        for sf in &faces {
            let l = d.face_loop(sf.face().get());
            match l.len() {
                3 => tris += 1,
                4 => quads += 1,
                _ => other += 1,
            }
            pts.extend_from_slice(l);
        }
        pts.sort_unstable();
        pts.dedup();
        let shape = (tris, quads, pts.len());
        let primitive = other == 0
            && if all {
                matches!(shape, (4, 0, 4) | (0, 6, 8) | (2, 3, 6) | (4, 1, 5))
            } else {
                wedged[c] && shape == (0, 6, 8)
            };
        if !primitive {
            continue;
        }
        // A quad is split along a diagonal (no new point). A tetrahedron has only triangles: one
        // is split about its centroid, which moves the vertex mean of the neighbour, so take
        // the first face whose neighbour stays valid (sm-quality `Starness` and the side tets).
        let quad = faces.iter().map(|sf| sf.face().get()).find(|&f| d.face_loop(f).len() == 4);
        let cands: SmallVec<[usize; 6]> = match quad {
            Some(f) => SmallVec::from_slice(&[f]),
            None => faces.iter().map(|sf| sf.face().get()).collect(),
        };
        let mut pick = None;
        for &f in &cands {
            let mark = Mark::of(d);
            let parts = subdivide(d, f, c);
            if neighbours_ok(d, &edits, f, c, &parts) {
                pick = Some((f, parts));
                break;
            }
            mark.rollback(d);
        }
        let (f, parts) = pick.unwrap_or_else(|| (cands[0], subdivide(d, cands[0], c)));
        d.f_alive[f] = false;
        let owners = d.f_cells[f];
        for (k, &o) in owners.iter().enumerate() {
            if o == NONE {
                continue;
            }
            edits[o as usize] = Some(spliced(d, &edits, o as usize, f, &parts, k));
        }
        any = true;
    }
    if !any {
        return;
    }
    let (mut off, mut cf) = (vec![0u32], Vec::new());
    for (c, e) in edits.iter().enumerate() {
        match e {
            Some(l) => cf.extend_from_slice(l),
            None if d.c_alive[c] => cf.extend_from_slice(d.cell_faces(c)),
            None => {}
        }
        off.push(cf.len() as u32);
    }
    d.c_off = off;
    d.c_faces = cf;
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

/// Points of the loop of `sf` as seen from its cell (outward winding).
fn oriented(d: &PolyData, sf: &SignedFace) -> SmallVec<[V3; 8]> {
    let mut v: SmallVec<[V3; 8]> = d.face_loop(sf.face().get()).iter().map(|&x| d.pts[x as usize]).collect();
    if sf.side() == Side::Right {
        v.reverse();
    }
    v
}

fn cell_ok(d: &PolyData, c: usize) -> bool {
    cell_ok_faces(d, d.cell_faces(c))
}

/// [`cell_ok`] for an explicit face list.
fn cell_ok_faces(d: &PolyData, faces: &[SignedFace]) -> bool {
    let (vol, g) = agglom::vol_centroid(d, faces);
    if vol <= 0.0 {
        return false;
    }
    let min = DETECT_MARGIN * vol;
    // sm-quality `Starness`: star-shaped about the mean of the cell's distinct vertices
    let mut vs: SmallVec<[u32; 64]> =
        faces.iter().flat_map(|sf| d.face_loop(sf.face().get()).iter().copied()).collect();
    vs.sort_unstable();
    vs.dedup();
    let mut vm = [0.0; 3];
    for &x in &vs {
        let q = d.pts[x as usize];
        vm = [vm[0] + q[0], vm[1] + q[1], vm[2] + q[2]];
    }
    let nv = vs.len().max(1) as f64;
    let vm = [vm[0] / nv, vm[1] / nv, vm[2] / nv];
    let loops: SmallVec<[SmallVec<[V3; 8]>; 16]> = faces.iter().map(|sf| oriented(d, sf)).collect();
    if min_starness(vm, loops.iter().map(|v| &v[..])) <= STAR_DETECT {
        return false;
    }
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
