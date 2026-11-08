//! Facet twins: the triangle models a faceted tracker (DAGMC, XDG, sm-rt facet input) sees, derived
//! from the B-rep with `sm_tess` at an explicit tolerance.
//!
//! `sm_tess` stores `f32` positions and uses platform `sin`/`cos`, so the twin's bytes are
//! blessed per `os-arch` (see `cases`), never claimed identical across platforms.

use crate::error::{BenchError, Result};
use crate::formats::Facets;
use sm_core::{EntityId, Point3};
use sm_tess::{TessOptions, Tessellation, tessellate};
use sm_topo_api::{CellComplex, Region, TopologyViewExt};

fn geom<E: core::fmt::Display>(e: E) -> BenchError {
    BenchError::Geometry(e.to_string())
}

fn tess_opts(sag: f64) -> TessOptions {
    // Only the sagitta controls the sampling: a loose angle tolerance leaves the chord tolerance
    // as the single recorded parameter.
    TessOptions { chord_tol: sag, angle_tol: core::f64::consts::PI, max_edge: f64::INFINITY }
}

fn pos(t: &Tessellation, i: u32) -> [f64; 3] {
    let p = t.position(i);
    [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])]
}

/// One face of a tessellation as facets: triangles keep the face's orientation, sides come from
/// `front`/`back`. Vertices are appended through `map` (tessellation index to facet index), so
/// faces of one tessellation share vertices.
fn add_face(f: &mut Facets, t: &Tessellation, face: EntityId, front: i32, back: i32, map: &mut std::collections::BTreeMap<u32, u32>) -> Result<()> {
    let idx = t.face_indices(face).ok_or_else(|| BenchError::Geometry("face missing from tessellation".into()))?;
    for tri in idx.as_chunks::<3>().0 {
        let mut out = [0u32; 3];
        for (k, &i) in tri.iter().enumerate() {
            out[k] = match map.get(&i) {
                Some(&v) => v,
                None => {
                    let v = u32::try_from(f.verts.len()).map_err(|_| BenchError::Geometry("too many vertices".into()))?;
                    f.verts.push(pos(t, i));
                    map.insert(i, v);
                    v
                }
            };
        }
        f.tris.push(out);
        f.sides.push([front, back]);
    }
    Ok(())
}

fn region_of(r: Region, vol_region: &dyn Fn(EntityId) -> i32) -> i32 {
    match r {
        Region::Volume(v) => vol_region(v),
        Region::Exterior => -1,
    }
}

/// Tessellate every face of `view` at `sag` into facets whose sides follow the topology
/// (`front = left`, `back = right`, `Exterior = -1`); `vol_region` numbers the volumes.
pub fn facets_of_complex(view: &CellComplex, sag: f64, vol_region: &dyn Fn(EntityId) -> i32) -> Result<Facets> {
    let faces: Vec<EntityId> = view.faces().to_vec();
    let t = tessellate(view, &faces, &tess_opts(sag)).map_err(geom)?;
    let mut f = Facets::default();
    let mut map = std::collections::BTreeMap::new();
    for face in faces {
        let s = view.face_sides(face).ok_or_else(|| BenchError::Geometry("face without sides".into()))?;
        add_face(&mut f, &t, face, region_of(s.left, vol_region), region_of(s.right, vol_region), &mut map)?;
    }
    Ok(f)
}

/// The facet twin of one pin cell (fuel `0`, cladding `1`, complement `-1`) plus the vertex
/// indices of the cladding's outer rings at `z = 0` and `z = h`.
pub struct PinTwin {
    /// Facets of the base pin.
    pub facets: Facets,
    /// Ring vertices of the outer cladding circle at `z = 0`, ascending angle.
    pub ring_bot: Vec<u32>,
    /// Ring vertices at `z = h`, ascending angle.
    pub ring_top: Vec<u32>,
    /// Triangle indices that belong to the outer cladding cylinder.
    pub clad_lateral: Vec<usize>,
}

/// Base pin of `sm_testkit::topo::pin_assembly` tessellated at `sag`.
pub fn pin_twin(fix: &sm_testkit::topo::ModelFixture, sag: f64, r_clad: f64, h: f64) -> Result<PinTwin> {
    use sm_model_api::{Domain, flatten};
    let m = &fix.model;
    let fuel_c = m.color_by_label(Domain::Volume, 1).ok_or_else(|| BenchError::Geometry("no fuel color".into()))?;
    let flat = flatten(m).map_err(geom)?;
    let first = flat.paths.first().map(|_| 0u32).ok_or_else(|| BenchError::Geometry("no instances".into()))?;
    let vol_color: Vec<(EntityId, EntityId)> = flat.entries.iter().filter(|e| e.path == first).map(|e| (e.volume, e.color)).collect();
    let vol_region = |v: EntityId| -> i32 { i32::from(vol_color.iter().find(|(x, _)| *x == v).is_none_or(|(_, c)| *c != fuel_c)) };
    let mut facets = facets_of_complex(&fix.complex, sag, &vol_region)?;
    // Outer cladding cylinder: faces between clad (1) and the exterior whose vertices span z.
    let mut clad_lateral = Vec::new();
    for (i, s) in facets.sides.iter().enumerate() {
        if *s == [1, -1] {
            let c = facets.corners(i);
            let on_circle = c.iter().all(|p| ((p[0] * p[0] + p[1] * p[1]).sqrt() - r_clad).abs() < 1e-5);
            let spans = c.iter().any(|p| p[2] < 0.5 * h) && c.iter().any(|p| p[2] > 0.5 * h);
            if on_circle && spans {
                clad_lateral.push(i);
            }
        }
    }
    let ring = |z: f64| -> Vec<u32> {
        let mut ids: Vec<u32> = clad_lateral.iter().flat_map(|&i| facets.tris[i]).filter(|&v| (facets.verts[v as usize][2] - z).abs() < 1e-6).collect();
        ids.sort_unstable();
        ids.dedup();
        ids.sort_by(|&a, &b| {
            let (pa, pb) = (facets.verts[a as usize], facets.verts[b as usize]);
            // Angle in [0, 2 pi) measured from +x; ties impossible on a ring of distinct points.
            let ang = |p: [f64; 3]| {
                let t = p[1].atan2(p[0]);
                if t < 0.0 { t + core::f64::consts::TAU } else { t }
            };
            ang(pa).total_cmp(&ang(pb))
        });
        ids
    };
    let (ring_bot, ring_top) = (ring(0.0), ring(h));
    if ring_bot.len() < 3 || ring_bot.len() != ring_top.len() {
        return Err(BenchError::Geometry(format!("cladding ring has {} / {} vertices", ring_bot.len(), ring_top.len())));
    }
    // Outer cladding now borders the moderator (region 2), not the complement.
    for &i in &clad_lateral {
        facets.sides[i] = [1, 2];
    }
    Ok(PinTwin { facets, ring_bot, ring_top, clad_lateral })
}

/// Add the moderator of a square cell of half width `a` around a pin twin: the square caps with
/// a circular hole (strips from the cladding rings), four side walls, all with sides
/// `(region, -1)`.
pub fn add_moderator(tw: &mut PinTwin, a: f64, h: f64, region: i32) -> Result<()> {
    let n = tw.ring_bot.len();
    let ring_pt = |tw: &PinTwin, ring: &[u32], i: usize| tw.facets.verts[ring[i % n] as usize];
    // Square points: the ring point pushed radially onto the square, plus corners between.
    let side_of = |p: [f64; 2]| -> u8 {
        if p[0].abs() >= p[1].abs() {
            if p[0] > 0.0 { 0 } else { 2 }
        } else if p[1] > 0.0 {
            1
        } else {
            3
        }
    };
    let corner = |s0: u8, s1: u8| -> Option<[f64; 2]> {
        // Corner between consecutive sides (ascending angle): 0->1 (+,+), 1->2 (-,+), 2->3 (-,-), 3->0 (+,-).
        if (s0 + 1) % 4 != s1 {
            return None;
        }
        Some(match s0 {
            0 => [a, a],
            1 => [-a, a],
            2 => [-a, -a],
            _ => [a, -a],
        })
    };
    let mut sq_pts: Vec<[f64; 2]> = Vec::with_capacity(n);
    let mut between: Vec<Option<[f64; 2]>> = Vec::with_capacity(n);
    for i in 0..n {
        let p = ring_pt(tw, &tw.ring_bot, i);
        let m = p[0].abs().max(p[1].abs());
        sq_pts.push([p[0] * (a / m), p[1] * (a / m)]);
    }
    for i in 0..n {
        let (s0, s1) = (side_of(sq_pts[i]), side_of(sq_pts[(i + 1) % n]));
        if s0 == s1 {
            between.push(None);
        } else {
            let k = corner(s0, s1).ok_or_else(|| BenchError::Geometry("ring angular gap exceeds 90 degrees".into()))?;
            // A ring point exactly on the diagonal already is the corner: a second vertex there
            // would give zero-area triangles (and the tracker refuses them), so the corner is
            // not added; the polyline is the same.
            let on = |p: [f64; 2]| (p[0] - k[0]).abs() <= 1e-12 * a && (p[1] - k[1]).abs() <= 1e-12 * a;
            between.push(if on(sq_pts[i]) || on(sq_pts[(i + 1) % n]) { None } else { Some(k) });
        }
    }
    let f = &mut tw.facets;
    let mut add_pt = |p: [f64; 3]| -> Result<u32> {
        let v = u32::try_from(f.verts.len()).map_err(|_| BenchError::Geometry("too many vertices".into()))?;
        f.verts.push(p);
        Ok(v)
    };
    // Square polyline vertex ids per level (ring point, then optional corner), ascending angle.
    let mut poly: Vec<[u32; 2]> = Vec::new(); // [bottom id, top id]
    let mut strips: Vec<(usize, usize, Option<usize>)> = Vec::new(); // (ring i, poly index of s_i, poly index of corner)
    for i in 0..n {
        let s = sq_pts[i];
        let ids = [add_pt([s[0], s[1], 0.0])?, add_pt([s[0], s[1], h])?];
        let at = poly.len();
        poly.push(ids);
        let mut corner_ix = None;
        if let Some(k) = between[i] {
            corner_ix = Some(poly.len());
            poly.push([add_pt([k[0], k[1], 0.0])?, add_pt([k[0], k[1], h])?]);
        }
        strips.push((i, at, corner_ix));
    }
    let (ring_bot, ring_top) = (tw.ring_bot.clone(), tw.ring_top.clone());
    let m = poly.len();
    let tri = |f: &mut Facets, t: [u32; 3], flip: bool| {
        f.tris.push(if flip { [t[0], t[2], t[1]] } else { t });
        f.sides.push([region, -1]);
    };
    for &(i, at, corner_ix) in &strips {
        let j = (i + 1) % n;
        let s_next = if j == 0 { 0 } else { strips[j].1 };
        for (level, ring, flip) in [(1usize, &ring_top, false), (0usize, &ring_bot, true)] {
            let (ci, cj) = (ring[i], ring[j]);
            let si = poly[at][level];
            let sj = poly[s_next][level];
            match corner_ix {
                None => {
                    tri(&mut tw.facets, [ci, si, sj], flip);
                    tri(&mut tw.facets, [ci, sj, cj], flip);
                }
                Some(k) => {
                    let kk = poly[k][level];
                    tri(&mut tw.facets, [ci, si, kk], flip);
                    tri(&mut tw.facets, [ci, kk, sj], flip);
                    tri(&mut tw.facets, [ci, sj, cj], flip);
                }
            }
        }
    }
    for k in 0..m {
        let l = (k + 1) % m;
        let (b0, t0, b1, t1) = (poly[k][0], poly[k][1], poly[l][0], poly[l][1]);
        tri(&mut tw.facets, [b0, b1, t1], false);
        tri(&mut tw.facets, [b0, t1, t0], false);
    }
    Ok(())
}

/// Replicate `base` at every placement of a flattened model (`copies` placements, each applied
/// to the vertices; reversed placements flip the triangles).
pub fn replicate(base: &Facets, placements: &[sm_model_api::Placement]) -> Result<Facets> {
    let mut f = Facets::default();
    for pl in placements {
        let verts: Vec<[f64; 3]> = base
            .verts
            .iter()
            .map(|v| {
                let p = pl.transform.apply_point(Point3::new(v[0], v[1], v[2]));
                [p.x, p.y, p.z]
            })
            .collect();
        for (i, t) in base.tris.iter().enumerate() {
            let b = u32::try_from(f.verts.len()).map_err(|_| BenchError::Geometry("too many vertices".into()))?;
            let t = if pl.orientation_reversed { [b + t[0], b + t[2], b + t[1]] } else { [b + t[0], b + t[1], b + t[2]] };
            f.tris.push(t);
            f.sides.push(base.sides[i]);
        }
        f.verts.extend(verts);
    }
    Ok(f)
}

/// Nested torus shells about the z axis (major radius `major`): shell `k` has minor radius
/// `minors[k]` (ascending) and encloses region `k`; the last borders the complement.
pub fn nested_tori(major: f64, minors: &[f64], sag: f64) -> Result<Facets> {
    let mut out = Facets::default();
    for (k, &minor) in minors.iter().enumerate() {
        let fx = sm_testkit::topo::torus(major, minor).map_err(geom)?;
        let faces: Vec<EntityId> = fx.complex.faces().to_vec();
        let t = tessellate(&fx.complex, &faces, &tess_opts(sag)).map_err(geom)?;
        let mut one = Facets::default();
        let mut map = std::collections::BTreeMap::new();
        for face in faces {
            add_face(&mut one, &t, face, 0, 0, &mut map)?;
        }
        // Orient outward: the normal must point away from the tube centre circle.
        let outward = |f: &Facets, i: usize| -> f64 {
            let c = f.corners(i);
            let cen = [(c[0][0] + c[1][0] + c[2][0]) / 3.0, (c[0][1] + c[1][1] + c[2][1]) / 3.0, (c[0][2] + c[1][2] + c[2][2]) / 3.0];
            let rho = (cen[0] * cen[0] + cen[1] * cen[1]).sqrt();
            let q = [cen[0] * major / rho, cen[1] * major / rho, 0.0];
            let n = tri_normal(&c);
            n[0] * (cen[0] - q[0]) + n[1] * (cen[1] - q[1]) + n[2] * cen[2]
        };
        let votes: i64 = (0..one.tris.len()).map(|i| if outward(&one, i) >= 0.0 { 1 } else { -1 }).sum();
        let flip = votes < 0;
        let back = if k + 1 < minors.len() { k as i32 + 1 } else { -1 };
        out.add(&one.verts, &one.tris, k as i32, back, flip)?;
    }
    Ok(out)
}

/// Unnormalised normal of a triangle.
pub fn tri_normal(c: &[[f64; 3]; 3]) -> [f64; 3] {
    let u = [c[1][0] - c[0][0], c[1][1] - c[0][1], c[1][2] - c[0][2]];
    let v = [c[2][0] - c[0][0], c[2][1] - c[0][1], c[2][2] - c[0][2]];
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

/// Signed volume of region `r` by the divergence theorem: triangles with `r` in front count
/// positive, with `r` behind negative.
pub fn region_volume(f: &Facets, r: i32) -> f64 {
    let mut v = 0.0;
    for i in 0..f.tris.len() {
        let c = f.corners(i);
        let triple = c[0][0] * (c[1][1] * c[2][2] - c[1][2] * c[2][1]) - c[0][1] * (c[1][0] * c[2][2] - c[1][2] * c[2][0]) + c[0][2] * (c[1][0] * c[2][1] - c[1][1] * c[2][0]);
        if f.sides[i][0] == r {
            v += triple / 6.0;
        }
        if f.sides[i][1] == r {
            v -= triple / 6.0;
        }
    }
    v
}

/// Largest radial deviation of the triangles of the vertical cylinder of radius `r` about the
/// z axis from the true cylinder, measured from the triangle planes (apothem) and the vertices.
/// Only triangles whose three vertices lie within `1e-5` of the cylinder are considered.
pub fn max_cylinder_deviation(f: &Facets, r: f64) -> f64 {
    let mut worst = 0.0f64;
    for i in 0..f.tris.len() {
        let c = f.corners(i);
        if !c.iter().all(|p| ((p[0] * p[0] + p[1] * p[1]).sqrt() - r).abs() < 1e-5) {
            continue;
        }
        let n = tri_normal(&c);
        let nxy = (n[0] * n[0] + n[1] * n[1]).sqrt();
        if nxy < 1e-12 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() {
            continue; // a cap triangle, not part of the lateral surface
        }
        let d = (n[0] * c[0][0] + n[1] * c[0][1]).abs() / nxy; // distance of the plane from the axis
        worst = worst.max((r - d).abs());
        for p in c {
            worst = worst.max(((p[0] * p[0] + p[1] * p[1]).sqrt() - r).abs());
        }
    }
    worst
}

/// Facet twin of an imported STEP model: one region per solid (ordinal), complement `-1`.
pub fn facets_of_step(bytes: &[u8], sag: f64) -> Result<Facets> {
    use sm_topo_api::TopologyEdit;
    let opts = sm_cadio::ReadOptions::default();
    let model = sm_cadio::read_step(bytes, &opts).map_err(geom)?;
    let mut cc = CellComplex::new();
    cc.begin_op(sm_topo_api::OpId(1)).map_err(geom)?;
    let realized = match sm_cadio::realize(&model, &mut cc, &sm_cadio::RealizeOptions::default()) {
        Ok(r) => r,
        Err(e) => {
            cc.abort_op();
            return Err(geom(e));
        }
    };
    cc.end_op().map_err(geom)?;
    let vols = realized.volumes;
    let vol_region = move |v: EntityId| -> i32 { vols.iter().position(|x| *x == v).map_or(-1, |i| i as i32) };
    facets_of_complex(&cc, sag, &vol_region)
}
