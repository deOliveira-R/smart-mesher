//! Stub implementor: a brute-force analytic world of nested spheres and axis-aligned boxes.
//!
//! Each solid is one volume; a solid's parent is the volume directly around it (the implicit
//! complement for top-level solids). The volume of a solid is its shape minus its children.
//! Every ray query tests every surface of the volume, so it is exact to rounding and O(surfaces),
//! which makes it the reference the real backends are tested against.
//!
//! Limits (stub only): children must lie inside the parent and not touch each other; a ray that
//! passes exactly through a box edge hits one face first and the other at distance 0, which the
//! history then excludes.

use crate::boundary::Boundary;
use crate::build::{Backend, BuildInput, BuildParams, BuildReport, Precision};
use crate::cache::{CacheBlob, ContentHash, ContentHasher, hash_params};
use crate::error::{Result, RtError};
use crate::handle::{FacetId, SurfHandle, SurfSense, VolHandle};
use crate::query::{Closest, FireOptions, RayHistory, RayHit, RayOrientation};
use crate::world::{BackendInfo, SurfaceInfo, SurfaceVolumes, VolumeInfo, World, check_dir, check_point};
use sm_core::{Aabb, Point3, Vector3};
use std::f64::consts::PI;

/// A solid shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalyticShape {
    /// Sphere.
    Sphere {
        /// Center.
        center: Point3,
        /// Radius (> 0).
        radius: f64,
    },
    /// Axis-aligned box.
    Box {
        /// Lower corner.
        lo: Point3,
        /// Upper corner.
        hi: Point3,
    },
}

/// One solid of an [`AnalyticSpec`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolidSpec {
    /// Shape.
    pub shape: AnalyticShape,
    /// Index of the enclosing solid in [`AnalyticSpec::solids`] (must be smaller than this
    /// solid's own index), or `None` for the implicit complement.
    pub parent: Option<u32>,
    /// Material or group tag.
    pub tag: u32,
    /// Condition on this solid's outer surface(s).
    pub boundary: Boundary,
}

/// Input of the analytic stub.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AnalyticSpec {
    /// Solids; solid `i` becomes `VolHandle(i)`, the complement `VolHandle(n)`.
    pub solids: Vec<SolidSpec>,
}

impl AnalyticSpec {
    pub(crate) fn hash_into(&self, h: &mut ContentHasher) {
        for s in &self.solids {
            h.write(format!("{s:?}").as_bytes());
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Kind {
    Sphere { c: Point3, r: f64 },
    /// Box face: `axis` 0..3, `side` -1 (low) or +1 (high).
    Face { axis: usize, side: f64, lo: [f64; 3], hi: [f64; 3] },
}

#[derive(Clone, Copy, Debug)]
struct Surf {
    kind: Kind,
    fwd: VolHandle,
    rev: VolHandle,
    boundary: Boundary,
}

/// The stub world. See the module docs.
#[derive(Debug)]
pub struct AnalyticWorld {
    solids: Vec<SolidSpec>,
    surfs: Vec<Surf>,
    vol_surfs: Vec<Vec<SurfHandle>>,
    hash: ContentHash,
    eps: f64,
}

fn arr(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn comp(v: Vector3, a: usize) -> f64 {
    [v.x, v.y, v.z][a]
}

impl AnalyticWorld {
    /// Build. Fails with `BadInput` on non-finite or degenerate shapes or a bad parent index.
    pub fn new(spec: &AnalyticSpec, params: &BuildParams) -> Result<Self> {
        let n = spec.solids.len() as u32;
        let ic = VolHandle(n);
        let mut surfs = Vec::new();
        for (i, s) in spec.solids.iter().enumerate() {
            let i = i as u32;
            let parent = match s.parent {
                None => ic,
                Some(p) if p < i => VolHandle(p),
                Some(p) => return Err(RtError::BadInput(format!("solid {i}: parent {p} must precede it"))),
            };
            let mut push = |kind| surfs.push(Surf { kind, fwd: VolHandle(i), rev: parent, boundary: s.boundary });
            match s.shape {
                AnalyticShape::Sphere { center, radius } => {
                    if !(radius > 0.0 && radius.is_finite()) {
                        return Err(RtError::BadInput(format!("solid {i}: bad radius")));
                    }
                    check_point(center)?;
                    push(Kind::Sphere { c: center, r: radius });
                }
                AnalyticShape::Box { lo, hi } => {
                    check_point(lo)?;
                    check_point(hi)?;
                    let (l, h) = (arr(lo), arr(hi));
                    if (0..3).any(|a| h[a] <= l[a]) {
                        return Err(RtError::BadInput(format!("solid {i}: empty box")));
                    }
                    for axis in 0..3 {
                        for side in [-1.0, 1.0] {
                            push(Kind::Face { axis, side, lo: l, hi: h });
                        }
                    }
                }
            }
        }
        let mut vol_surfs = vec![Vec::new(); n as usize + 1];
        for (k, s) in surfs.iter().enumerate() {
            let h = SurfHandle(k as u32);
            vol_surfs[s.fwd.index()].push(h);
            vol_surfs[s.rev.index()].push(h);
        }
        let mut hs = ContentHasher::new();
        spec.hash_into(&mut hs);
        hs.write_u64(hash_params(params).0 as u64);
        Ok(Self { solids: spec.solids.clone(), surfs, vol_surfs, hash: hs.finish(), eps: params.surface_eps })
    }

    fn vol(&self, v: VolHandle) -> Result<()> {
        if v.index() <= self.solids.len() { Ok(()) } else { Err(RtError::BadVolume(v)) }
    }

    fn surf(&self, s: SurfHandle) -> Result<&Surf> {
        self.surfs.get(s.index()).ok_or(RtError::BadSurface(s))
    }

    /// Outward normal at `p` (out of the forward volume).
    fn normal_at(k: &Kind, p: Point3) -> Vector3 {
        match *k {
            Kind::Sphere { c, r } => (p - c) * (1.0 / r),
            Kind::Face { axis, side, .. } => {
                let mut n = [0.0; 3];
                n[axis] = side;
                Vector3::new(n[0], n[1], n[2])
            }
        }
    }

    /// Hit distances of the ray with a surface, ascending, `t > 0` only.
    fn hits(k: &Kind, o: Point3, d: Vector3, out: &mut [f64; 2]) -> usize {
        match *k {
            Kind::Sphere { c, r } => {
                let oc = o - c;
                let b = oc.dot(d);
                let perp = oc - d * b;
                let disc = r * r - perp.dot(perp);
                if disc < 0.0 {
                    return 0;
                }
                let s = disc.sqrt();
                let mut n = 0;
                for t in [-b - s, -b + s] {
                    if t > 0.0 {
                        out[n] = t;
                        n += 1;
                    }
                }
                n
            }
            Kind::Face { axis, side, lo, hi } => {
                let da = comp(d, axis);
                if da == 0.0 {
                    return 0;
                }
                let plane = if side < 0.0 { lo[axis] } else { hi[axis] };
                let t = (plane - arr(o)[axis]) / da;
                if t <= 0.0 {
                    return 0;
                }
                let p = arr(o + d * t);
                for a in 0..3 {
                    if a != axis && (p[a] < lo[a] || p[a] > hi[a]) {
                        return 0;
                    }
                }
                out[0] = t;
                1
            }
        }
    }

    fn dist_to(k: &Kind, p: Point3) -> f64 {
        match *k {
            Kind::Sphere { c, r } => ((p - c).norm() - r).abs(),
            Kind::Face { axis, side, lo, hi } => {
                let q = arr(p);
                let mut d2 = 0.0;
                for a in 0..3 {
                    let t = if a == axis { if side < 0.0 { lo[a] } else { hi[a] } } else { q[a].clamp(lo[a], hi[a]) };
                    d2 += (q[a] - t) * (q[a] - t);
                }
                d2.sqrt()
            }
        }
    }

    /// Is `p` inside solid `i`? Boundary points decided by `dir`, else counted inside.
    fn in_solid(&self, i: usize, p: Point3, dir: Option<Vector3>) -> bool {
        let (sd, n) = match self.solids[i].shape {
            AnalyticShape::Sphere { center, radius } => {
                let v = p - center;
                let len = v.norm();
                let n = if len > 0.0 { v * (1.0 / len) } else { Vector3::new(1.0, 0.0, 0.0) };
                (len - radius, n)
            }
            AnalyticShape::Box { lo, hi } => {
                let (l, h, q) = (arr(lo), arr(hi), arr(p));
                let mut best = (f64::NEG_INFINITY, Vector3::default());
                for a in 0..3 {
                    for (side, d) in [(-1.0, l[a] - q[a]), (1.0, q[a] - h[a])] {
                        if d > best.0 {
                            let mut nn = [0.0; 3];
                            nn[a] = side;
                            best = (d, Vector3::new(nn[0], nn[1], nn[2]));
                        }
                    }
                }
                best
            }
        };
        let tol = self.eps * 1.0f64.max(sd.abs());
        if sd.abs() <= tol.max(self.eps) {
            return match dir {
                Some(d) => d.dot(n) < 0.0,
                None => true,
            };
        }
        sd < 0.0
    }

    fn children(&self, v: usize) -> impl Iterator<Item = usize> + '_ {
        self.solids.iter().enumerate().filter(move |(_, s)| s.parent.map(|p| p as usize) == Some(v) || (v == self.solids.len() && s.parent.is_none())).map(|(i, _)| i)
    }

    fn solid_volume(shape: AnalyticShape) -> f64 {
        match shape {
            AnalyticShape::Sphere { radius, .. } => 4.0 / 3.0 * PI * radius * radius * radius,
            AnalyticShape::Box { lo, hi } => (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z),
        }
    }
}

impl World for AnalyticWorld {
    fn backend_info(&self) -> BackendInfo {
        BackendInfo { name: "analytic", precision: Precision::F64, exact_surfaces: true, needs_watertight: false }
    }
    fn content_hash(&self) -> ContentHash {
        self.hash
    }
    fn bounds(&self) -> Aabb {
        let mut b = Aabb::empty();
        for s in &self.solids {
            match s.shape {
                AnalyticShape::Sphere { center: c, radius: r } => {
                    b.extend(Point3::new(c.x - r, c.y - r, c.z - r));
                    b.extend(Point3::new(c.x + r, c.y + r, c.z + r));
                }
                AnalyticShape::Box { lo, hi } => {
                    b.extend(lo);
                    b.extend(hi);
                }
            }
        }
        b
    }
    fn volume_count(&self) -> u32 {
        self.solids.len() as u32 + 1
    }
    fn surface_count(&self) -> u32 {
        self.surfs.len() as u32
    }
    fn implicit_complement(&self) -> VolHandle {
        VolHandle(self.solids.len() as u32)
    }
    fn volume_info(&self, v: VolHandle) -> Result<VolumeInfo> {
        self.vol(v)?;
        let tag = self.solids.get(v.index()).map_or(0, |s| s.tag);
        Ok(VolumeInfo { entity: None, tag, is_complement: v == self.implicit_complement(), instance: None })
    }
    fn surface_info(&self, s: SurfHandle) -> Result<SurfaceInfo> {
        self.surf(s)?;
        Ok(SurfaceInfo { entity: None, facet_count: 1 })
    }
    fn volume_surfaces(&self, v: VolHandle, out: &mut Vec<(SurfHandle, SurfSense)>) -> Result<()> {
        self.vol(v)?;
        out.clear();
        for &s in &self.vol_surfs[v.index()] {
            out.push((s, self.surface_sense(v, s)?));
        }
        Ok(())
    }
    fn surface_volumes(&self, s: SurfHandle) -> Result<SurfaceVolumes> {
        let s = self.surf(s)?;
        Ok(SurfaceVolumes { forward: s.fwd, reverse: s.rev })
    }
    fn boundary(&self, s: SurfHandle) -> Result<Boundary> {
        Ok(self.surf(s)?.boundary)
    }

    fn ray_fire(
        &self,
        volume: VolHandle,
        origin: Point3,
        dir: Vector3,
        opts: FireOptions,
        history: Option<&mut RayHistory>,
    ) -> Result<Option<RayHit>> {
        self.vol(volume)?;
        check_point(origin)?;
        check_dir(dir)?;
        let limit = opts.dist_limit.unwrap_or(f64::INFINITY);
        let mut best: Option<(f64, SurfHandle)> = None;
        let mut ts = [0.0; 2];
        for &h in &self.vol_surfs[volume.index()] {
            let s = &self.surfs[h.index()];
            let facet = FacetId { surface: h, index: 0 };
            let sign = if s.fwd == volume { 1.0 } else { -1.0 };
            let excluded = history.as_deref().is_some_and(|hist| hist.contains(facet));
            let n = Self::hits(&s.kind, origin, dir, &mut ts);
            for &t in ts.iter().take(n) {
                // An excluded facet cannot be hit again at the start point (a root within
                // `eps` of the origin is that same point), but its far root is legal.
                if t > limit || (excluded && t <= self.eps) {
                    continue;
                }
                let nrm = Self::normal_at(&s.kind, origin + dir * t);
                let c = dir.dot(nrm) * sign;
                let ok = match opts.orientation {
                    RayOrientation::Exiting => c > 0.0,
                    RayOrientation::Entering => c < 0.0,
                    RayOrientation::Any => c != 0.0,
                };
                if ok {
                    if best.is_none_or(|(bt, _)| t < bt) {
                        best = Some((t, h));
                    }
                    break;
                }
            }
        }
        Ok(best.map(|(dist, surface)| {
            let facet = FacetId { surface, index: 0 };
            if let Some(h) = history {
                h.add(facet);
            }
            RayHit { surface, facet, dist }
        }))
    }

    fn point_in_volume(
        &self,
        volume: VolHandle,
        p: Point3,
        dir: Option<Vector3>,
        _history: Option<&RayHistory>,
    ) -> Result<bool> {
        self.vol(volume)?;
        check_point(p)?;
        if let Some(d) = dir {
            check_dir(d)?;
        }
        let v = volume.index();
        let own = v == self.solids.len() || self.in_solid(v, p, dir);
        // With no direction a boundary point belongs to the solid, hence not to its parent.
        Ok(own && !self.children(v).any(|c| self.in_solid(c, p, dir)))
    }

    fn closest_to_location(&self, volume: VolHandle, p: Point3) -> Result<Closest> {
        self.vol(volume)?;
        check_point(p)?;
        let mut best: Option<Closest> = None;
        for &h in &self.vol_surfs[volume.index()] {
            let dist = Self::dist_to(&self.surfs[h.index()].kind, p);
            if best.is_none_or(|b| dist < b.dist) {
                best = Some(Closest { dist, surface: h });
            }
        }
        best.ok_or(RtError::Unbounded(volume))
    }

    fn get_angle(&self, surface: SurfHandle, p: Point3, _history: Option<&RayHistory>) -> Result<Vector3> {
        check_point(p)?;
        Ok(Self::normal_at(&self.surf(surface)?.kind, p))
    }

    fn measure_volume(&self, v: VolHandle) -> Result<f64> {
        self.vol(v)?;
        if v == self.implicit_complement() {
            return Err(RtError::Unbounded(v));
        }
        let own = Self::solid_volume(self.solids[v.index()].shape);
        Ok(own - self.children(v.index()).map(|c| Self::solid_volume(self.solids[c].shape)).sum::<f64>())
    }

    fn measure_area(&self, s: SurfHandle) -> Result<f64> {
        Ok(match self.surf(s)?.kind {
            Kind::Sphere { r, .. } => 4.0 * PI * r * r,
            Kind::Face { axis, lo, hi, .. } => {
                let e: Vec<f64> = (0..3).filter(|&a| a != axis).map(|a| hi[a] - lo[a]).collect();
                e[0] * e[1]
            }
        })
    }

    fn export_cache(&self) -> Option<CacheBlob> {
        None // nothing worth caching: brute force has no acceleration structure
    }
}

/// [`Backend`] for the stub: builds only [`BuildInput::Analytic`].
#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyticBackend;

impl Backend for AnalyticBackend {
    fn name(&self) -> &'static str {
        "analytic"
    }
    fn build(&self, input: &BuildInput<'_>, params: &BuildParams) -> Result<(Box<dyn World>, BuildReport)> {
        match input {
            BuildInput::Analytic(spec) => Ok((Box::new(AnalyticWorld::new(spec, params)?), BuildReport::default())),
            _ => Err(RtError::Unsupported("analytic backend builds only AnalyticSpec")),
        }
    }
    fn restore(&self, input: &BuildInput<'_>, _blob: &CacheBlob) -> Result<Box<dyn World>> {
        self.build(input, &BuildParams::default()).map(|(w, _)| w)
    }
}
