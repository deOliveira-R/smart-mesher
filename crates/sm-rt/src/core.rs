//! The flat structure every query runs on (struct-of-arrays, CSR connectivity).
//!
//! One `Core` serves both inputs: a polyhedral mesh (faces are mesh faces) and a facet soup
//! (every triangle is a face, every region a polyhedral cell). Faces carry the loop normal's
//! left -> right orientation; a cell's sign for a face is `+` when it is the face's left cell.
//! Face geometry is the `FAN_VERTEX_MEAN_V1` triangle fan (3-vertex faces are one triangle), so
//! chords agree with `sm_mesh_api::cell_volume`.

use crate::bvh::{Box6, Bvh, EMPTY_BOX, union};
use crate::vec3::{add, cross, dot, norm, scale, sub};
use rayon::prelude::*;
use sm_core::Point3;
use sm_math::sos::{Tagged, orient3d_sos};
use sm_math::{Sign, orient3d};
use sm_rt_api::FaceAction;

/// "No cell / no face / no surface" in the `u32` columns.
pub(crate) const NONE: u32 = u32::MAX;
/// Face flag: all loop vertices lie on the face plane within the planarity tolerance.
pub(crate) const F_PLANAR: u8 = 1;
/// Cell flag: convex with only planar faces (Cyrus-Beck applies).
pub(crate) const C_CONVEX: u8 = 1;
/// Cell flag: walk the cell through the face BVH (large cell whose faces are all in the BVH).
pub(crate) const C_BVH: u8 = 2;
/// Cells with more faces than this and all faces in the face BVH use the BVH walk.
pub(crate) const BIG_CELL: usize = 24;
/// Cells with more faces than this are never classified convex (the check is quadratic).
pub(crate) const CONVEX_CHECK_MAX: usize = 64;

const ID_O: u64 = u64::MAX;
const ID_O2: u64 = u64::MAX - 1;

/// Flat tracking structure.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Core {
    pub pos: Vec<[f64; 3]>,
    pub fv_off: Vec<u32>,
    pub fv: Vec<u32>,
    pub f_left: Vec<u32>,
    pub f_right: Vec<u32>,
    /// Unit normal (left -> right) and offset: `n . x = d`.
    pub f_plane: Vec<[f64; 4]>,
    pub f_apex: Vec<[f64; 3]>,
    pub f_flags: Vec<u8>,
    pub f_surface: Vec<u32>,
    /// Index into `actions` for model-boundary faces, else `NONE`.
    pub f_action: Vec<u32>,
    pub actions: Vec<FaceAction>,
    pub c_off: Vec<u32>,
    pub c_face: Vec<u32>,
    pub c_region: Vec<u32>,
    pub c_flags: Vec<u8>,
    pub c_bbox: Vec<Box6>,
    pub c_volume: Vec<f64>,
    pub face_bvh: Bvh,
    pub cell_bvh: Bvh,
}

/// A finding of the derive pass.
pub(crate) struct DeriveIssues {
    /// Faces whose loop is not planar within the tolerance (tracked through their fan).
    pub nonplanar: Vec<u32>,
    /// Faces with zero area (the build refuses these).
    pub degenerate: Vec<u32>,
}

impl Core {
    #[inline]
    pub fn n_faces(&self) -> usize {
        self.f_left.len()
    }
    #[inline]
    pub fn n_cells(&self) -> usize {
        self.c_region.len()
    }
    #[inline]
    pub fn cell_faces(&self, c: usize) -> &[u32] {
        &self.c_face[self.c_off[c] as usize..self.c_off[c + 1] as usize]
    }
    #[inline]
    pub fn face_verts(&self, f: usize) -> &[u32] {
        &self.fv[self.fv_off[f] as usize..self.fv_off[f + 1] as usize]
    }
    /// `+1` if `c` is the left cell of `f`, else `-1`.
    #[inline]
    pub fn sign(&self, c: usize, f: usize) -> f64 {
        if self.f_left[f] as usize == c { 1.0 } else { -1.0 }
    }
    /// Number of fan triangles of face `f`.
    #[inline]
    pub fn n_tris(&self, f: usize) -> usize {
        let n = self.face_verts(f).len();
        if n == 3 { 1 } else { n }
    }
    /// Fan triangle `i` of face `f`: SoS ids and corner positions.
    #[inline]
    pub fn tri(&self, f: usize, i: usize) -> ([u64; 3], [[f64; 3]; 3]) {
        let l = self.face_verts(f);
        if l.len() == 3 {
            return ([u64::from(l[0]), u64::from(l[1]), u64::from(l[2])], [self.pos[l[0] as usize], self.pos[l[1] as usize], self.pos[l[2] as usize]]);
        }
        let n = l.len();
        let (a, b) = (l[i], l[(i + 1) % n]);
        ([u64::from(a), u64::from(b), self.pos.len() as u64 + f as u64], [self.pos[a as usize], self.pos[b as usize], self.f_apex[f]])
    }

    /// Heap bytes of the built structure (lengths, not capacities).
    pub fn memory_bytes(&self) -> usize {
        self.pos.len() * 24
            + (self.fv_off.len() + self.fv.len() + self.f_left.len() + self.f_right.len() + self.f_surface.len() + self.f_action.len()) * 4
            + self.f_plane.len() * 32
            + self.f_apex.len() * 24
            + self.f_flags.len()
            + self.actions.len() * core::mem::size_of::<FaceAction>()
            + (self.c_off.len() + self.c_face.len() + self.c_region.len()) * 4
            + self.c_flags.len()
            + self.c_bbox.len() * 48
            + self.c_volume.len() * 8
            + self.face_bvh.bytes()
            + self.cell_bvh.bytes()
    }

    /// Compute apexes, planes, flags, boxes, volumes and the two BVHs from the raw columns
    /// (`pos`, face loops, `f_left`/`f_right`, cell CSR, `c_region`, `f_surface`).
    /// `volumes` overrides the polyhedron volume (the mesh path passes `sm_mesh_api::cell_volume`).
    pub fn derive(&mut self, planarity_tol: f64, leaf_size: usize, volumes: Option<Vec<f64>>) -> DeriveIssues {
        let nf = self.n_faces();
        let nc = self.n_cells();
        struct FaceOut {
            apex: [f64; 3],
            plane: [f64; 4],
            flags: u8,
            bb: Box6,
            degenerate: bool,
        }
        let outs: Vec<FaceOut> = (0..nf)
            .into_par_iter()
            .map(|f| {
                let l = self.face_verts(f);
                let n = l.len();
                let mut s = [0.0; 3];
                let mut bb = EMPTY_BOX;
                for &v in l {
                    let p = self.pos[v as usize];
                    s = add(s, p);
                    bb = union(&bb, &[p[0], p[1], p[2], p[0], p[1], p[2]]);
                }
                let apex = scale(s, 1.0 / n as f64);
                let mut a = [0.0; 3];
                for i in 0..n {
                    let (p, q) = (self.pos[l[i] as usize], self.pos[l[(i + 1) % n] as usize]);
                    a = add(a, scale(cross(sub(p, apex), sub(q, apex)), 0.5));
                }
                let len = norm(a);
                if len == 0.0 || !len.is_finite() {
                    return FaceOut { apex, plane: [0.0; 4], flags: 0, bb, degenerate: true };
                }
                let nrm = scale(a, 1.0 / len);
                let d = dot(nrm, apex);
                let diam = norm([bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]);
                let dev = l.iter().map(|&v| (dot(nrm, self.pos[v as usize]) - d).abs()).fold(0.0, f64::max);
                let planar = n == 3 || dev <= planarity_tol * diam;
                FaceOut { apex, plane: [nrm[0], nrm[1], nrm[2], d], flags: u8::from(planar) * F_PLANAR, bb, degenerate: false }
            })
            .collect();
        let mut issues = DeriveIssues { nonplanar: Vec::new(), degenerate: Vec::new() };
        self.f_apex = Vec::with_capacity(nf);
        self.f_plane = Vec::with_capacity(nf);
        self.f_flags = Vec::with_capacity(nf);
        let mut fbox: Vec<Box6> = Vec::with_capacity(nf);
        for (f, o) in outs.into_iter().enumerate() {
            if o.degenerate {
                issues.degenerate.push(f as u32);
            } else if o.flags & F_PLANAR == 0 {
                issues.nonplanar.push(f as u32);
            }
            self.f_apex.push(o.apex);
            self.f_plane.push(o.plane);
            self.f_flags.push(o.flags);
            fbox.push(o.bb);
        }

        // Face BVH over surface faces; cell flags need to know which faces are in it.
        let surf: Vec<u32> = (0..nf as u32).filter(|&f| self.f_surface[f as usize] != NONE).collect();
        self.face_bvh = Bvh::build(&fbox, surf, leaf_size);

        struct CellOut {
            flags: u8,
            bb: Box6,
        }
        let couts: Vec<CellOut> = (0..nc)
            .into_par_iter()
            .map(|c| {
                let fs = self.cell_faces(c);
                let mut bb = EMPTY_BOX;
                for &f in fs {
                    bb = union(&bb, &fbox[f as usize]);
                }
                let all_planar = fs.iter().all(|&f| self.f_flags[f as usize] & F_PLANAR != 0);
                let mut flags = 0;
                if all_planar && fs.len() <= CONVEX_CHECK_MAX {
                    let diam = norm([bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]);
                    let ctol = planarity_tol * diam;
                    let convex = fs.iter().all(|&f| {
                        let (pl, s) = (self.f_plane[f as usize], self.sign(c, f as usize));
                        fs.iter().all(|&g| self.face_verts(g as usize).iter().all(|&v| s * (dot([pl[0], pl[1], pl[2]], self.pos[v as usize]) - pl[3]) <= ctol))
                    });
                    if convex {
                        flags |= C_CONVEX;
                    }
                }
                if flags & C_CONVEX == 0 && fs.len() > BIG_CELL && fs.iter().all(|&f| self.f_surface[f as usize] != NONE) {
                    flags |= C_BVH;
                }
                CellOut { flags, bb }
            })
            .collect();
        self.c_flags = couts.iter().map(|c| c.flags).collect();
        self.c_bbox = couts.iter().map(|c| c.bb).collect();
        self.c_volume = match volumes {
            Some(v) => v,
            None => (0..nc).into_par_iter().map(|c| self.volume_from_arrays(c)).collect(),
        };
        self.cell_bvh = Bvh::build(&self.c_bbox, (0..nc as u32).collect(), leaf_size);
        issues
    }

    /// Polyhedron volume of cell `c` by the `FAN_VERTEX_MEAN_V1` rule (sum of fan tetrahedra).
    fn volume_from_arrays(&self, c: usize) -> f64 {
        let fs = self.cell_faces(c);
        let Some(&f0) = fs.first() else { return 0.0 };
        let o = self.pos[self.face_verts(f0 as usize)[0] as usize];
        let mut vol = 0.0;
        for &f in fs {
            let f = f as usize;
            let right = self.f_left[f] as usize != c;
            for i in 0..self.n_tris(f) {
                let (_, [mut a, mut b, apex]) = self.tri(f, i);
                if right {
                    core::mem::swap(&mut a, &mut b);
                }
                vol += dot(sub(apex, o), cross(sub(a, o), sub(b, o))) / 6.0;
            }
        }
        vol
    }

    /// Sign of the edge function of the line `(o, o2)` against the oriented edge `a -> b`, with
    /// exact zeros resolved by simulation of simplicity (ids: vertex index, apex `n_vertices +
    /// face`, line points the two largest ids). Antisymmetric in `(a, b)`; an exactly degenerate
    /// edge allocates inside `sm-math`.
    #[inline]
    fn edge_sign(o: Point3, o2: Point3, ia: u64, pa: [f64; 3], ib: u64, pb: [f64; 3]) -> i8 {
        let (lo, hi, flip) = if ia < ib { ((ia, pa), (ib, pb), false) } else { ((ib, pb), (ia, pa), true) };
        let (plo, phi) = (Point3::new(lo.1[0], lo.1[1], lo.1[2]), Point3::new(hi.1[0], hi.1[1], hi.1[2]));
        let s = match orient3d(o, o2, plo, phi) {
            Sign::Zero => {
                orient3d_sos(Tagged::new(o, ID_O), Tagged::new(o2, ID_O2), Tagged::new(plo, lo.0), Tagged::new(phi, hi.0)).unwrap_or(Sign::Positive)
            }
            s => s,
        };
        let v = match s {
            Sign::Negative => -1,
            _ => 1,
        };
        if flip { -v } else { v }
    }

    /// Crossings of the line `(o, o + dir)` with the fan of face `f`: calls `cb(t, along)` once
    /// per crossed fan triangle, `along` meaning the line runs with the loop normal. `t` is the
    /// parameter along `dir` from `o` (plane intersection).
    pub fn face_crossings(&self, f: usize, o: [f64; 3], dir: [f64; 3], mut cb: impl FnMut(f64, bool)) {
        let o2 = add(o, dir);
        let (po, po2) = (Point3::new(o[0], o[1], o[2]), Point3::new(o2[0], o2[1], o2[2]));
        let l = self.face_verts(f);
        let n = l.len();
        let planar = self.f_flags[f] & F_PLANAR != 0;
        let pl = self.f_plane[f];
        let t_plane = |o: [f64; 3]| -> f64 {
            let nd = dot([pl[0], pl[1], pl[2]], dir);
            if nd == 0.0 { 0.0 } else { (pl[3] - dot([pl[0], pl[1], pl[2]], o)) / nd }
        };
        let tri_t = |pts: &[[f64; 3]; 3]| -> f64 {
            if planar {
                return t_plane(o);
            }
            let nt = cross(sub(pts[1], pts[0]), sub(pts[2], pts[0]));
            let nd = dot(nt, dir);
            if nd == 0.0 {
                let c = scale(add(add(pts[0], pts[1]), pts[2]), 1.0 / 3.0);
                dot(sub(c, o), dir)
            } else {
                dot(nt, sub(pts[0], o)) / nd
            }
        };
        if n == 3 {
            let (ids, pts) = self.tri(f, 0);
            let s0 = Self::edge_sign(po, po2, ids[0], pts[0], ids[1], pts[1]);
            let s1 = Self::edge_sign(po, po2, ids[1], pts[1], ids[2], pts[2]);
            if s0 != s1 {
                return;
            }
            let s2 = Self::edge_sign(po, po2, ids[2], pts[2], ids[0], pts[0]);
            if s1 == s2 {
                cb(tri_t(&pts), s0 < 0);
            }
            return;
        }
        let apex = self.f_apex[f];
        let ia = self.pos.len() as u64 + f as u64;
        let spoke = |i: usize| Self::edge_sign(po, po2, u64::from(l[i]), self.pos[l[i] as usize], ia, apex);
        let sp0 = spoke(0);
        let mut sp_i = sp0;
        for i in 0..n {
            let j = (i + 1) % n;
            let sp_j = if j == 0 { sp0 } else { spoke(j) };
            // Edges of fan triangle (a_i, a_j, apex): ring, a_j -> apex, apex -> a_i.
            if sp_j == -sp_i {
                let ring = Self::edge_sign(po, po2, u64::from(l[i]), self.pos[l[i] as usize], u64::from(l[j]), self.pos[l[j] as usize]);
                if ring == sp_j {
                    let pts = [self.pos[l[i] as usize], self.pos[l[j] as usize], apex];
                    cb(tri_t(&pts), ring < 0);
                }
            }
            sp_i = sp_j;
        }
    }
}
