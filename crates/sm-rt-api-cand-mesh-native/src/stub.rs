//! Reference implementor: a brute-force analytic world of spheres and boxes in a box domain.
//!
//! It exists to pin down the *semantics* of [`Tracker`] (step events, tie rules, entry-face
//! exclusion, boundaries, determinism) with exact answers, not to be fast. Every query tests
//! every primitive. Cell `i < n` is primitive `i` (the first primitive containing a point wins,
//! so list inner shapes first); cell `n` is the background (an implicit complement inside the
//! domain box). Region id = cell id. Surface `i < n` is primitive `i`'s surface, surface
//! `n + 1 + k` is domain face `k` (`-x,+x,-y,+y,-z,+z`); `FaceId` equals `SurfaceId` (one face
//! per surface). Curved primitives have no constant normal, so `face_normal` supports domain
//! faces only.

use crate::cache::{ContentHash, ContentHasher};
use crate::error::{BuildIssue, LostCause, Result, RtError};
use crate::ids::{CellId, FaceId, RegionId, SurfaceId};
use crate::state::{
    BoundaryKind, Closest, Located, RegionInfo, RegionTable, Step, StepEnd, SurfaceInfo, SurfaceTable, TrackState,
    TrackerCaps,
};
use crate::tracker::{Tracker, check_direction, lost};
use sm_core::{Aabb, Point3, Vector3};

/// An analytic primitive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Sphere.
    Sphere {
        /// Centre.
        center: Point3,
        /// Radius (positive).
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

fn comp(p: Point3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}
fn compv(p: Vector3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}

impl Shape {
    /// Signed distance-like value: negative inside, positive outside (exact for spheres, the
    /// box version is the exact Euclidean distance outside and the nearest-face distance inside).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        match *self {
            Shape::Sphere { center, radius } => (p - center).norm() - radius,
            Shape::Box { lo, hi } => {
                let mut out2 = 0.0;
                let mut inside = f64::INFINITY;
                for a in 0..3 {
                    let (l, h, x) = (comp(lo, a), comp(hi, a), comp(p, a));
                    let e = (l - x).max(x - h).max(0.0);
                    out2 += e * e;
                    inside = inside.min((x - l).min(h - x));
                }
                if out2 > 0.0 { out2.sqrt() } else { -inside }
            }
        }
    }

    /// Volume.
    pub fn volume(&self) -> f64 {
        match *self {
            Shape::Sphere { radius, .. } => 4.0 / 3.0 * core::f64::consts::PI * radius.powi(3),
            Shape::Box { lo, hi } => (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z),
        }
    }

    /// Parameters `(t_in, t_out)` where the ray `o + t d` is inside, or `None` if it misses.
    pub fn interval(&self, o: Point3, d: Vector3) -> Option<(f64, f64)> {
        match *self {
            Shape::Sphere { center, radius } => {
                // Closest-approach form: the perpendicular offset m is formed directly, so a
                // grazing chord keeps full relative accuracy even when the origin is far away.
                let q = o - center;
                let tc = -q.dot(d);
                let m = q + d * tc;
                let m2 = m.dot(m);
                if m2 > radius * radius {
                    return None;
                }
                let mn = m2.sqrt();
                let half = ((radius - mn) * (radius + mn)).sqrt();
                Some((tc - half, tc + half))
            }
            Shape::Box { lo, hi } => {
                let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                for a in 0..3 {
                    let (l, h, x, v) = (comp(lo, a), comp(hi, a), comp(o, a), compv(d, a));
                    if v == 0.0 {
                        if x < l || x > h {
                            return None;
                        }
                    } else {
                        let (a0, a1) = ((l - x) / v, (h - x) / v);
                        t0 = t0.max(a0.min(a1));
                        t1 = t1.min(a0.max(a1));
                    }
                }
                (t0 <= t1).then_some((t0, t1))
            }
        }
    }

    fn closest_point(&self, p: Point3) -> Point3 {
        match *self {
            Shape::Sphere { center, radius } => {
                let v = p - center;
                let n = v.norm();
                let u = if n > 0.0 { v * (1.0 / n) } else { Vector3::new(1.0, 0.0, 0.0) };
                center + u * radius
            }
            Shape::Box { lo, hi } => {
                let mut q = [comp(p, 0), comp(p, 1), comp(p, 2)];
                let inside = (0..3).all(|a| q[a] >= comp(lo, a) && q[a] <= comp(hi, a));
                if inside {
                    // Project onto the nearest face.
                    let (mut best, mut ba, mut bv) = (f64::INFINITY, 0, 0.0);
                    for (a, &qa) in q.iter().enumerate() {
                        for v in [comp(lo, a), comp(hi, a)] {
                            if (qa - v).abs() < best {
                                best = (qa - v).abs();
                                ba = a;
                                bv = v;
                            }
                        }
                    }
                    q[ba] = bv;
                } else {
                    for (a, qa) in q.iter_mut().enumerate() {
                        *qa = qa.clamp(comp(lo, a), comp(hi, a));
                    }
                }
                Point3::new(q[0], q[1], q[2])
            }
        }
    }
}

/// Brute-force analytic tracker.
#[derive(Clone, Debug)]
pub struct AnalyticWorld {
    shapes: Vec<Shape>,
    domain: Aabb,
    boundary: BoundaryKind,
    regions: RegionTable,
    surfaces: SurfaceTable,
    tol: f64,
}

impl AnalyticWorld {
    /// Build a world. `shapes` are in priority order (inner first when nested).
    pub fn new(shapes: Vec<Shape>, domain: Aabb, boundary: BoundaryKind) -> Result<Self> {
        let mut issues = Vec::new();
        if domain.is_empty() {
            issues.push(BuildIssue::new("analytic.domain", "domain box is empty"));
        }
        for (i, s) in shapes.iter().enumerate() {
            let ok = match *s {
                Shape::Sphere { center, radius } => radius > 0.0 && radius.is_finite() && center.x.is_finite() && center.y.is_finite() && center.z.is_finite(),
                Shape::Box { lo, hi } => (0..3).all(|a| comp(lo, a) < comp(hi, a)),
            };
            if !ok {
                issues.push(BuildIssue::new("analytic.shape", format!("shape {i} is degenerate or non-finite")));
            }
        }
        if !issues.is_empty() {
            return Err(RtError::Build(issues));
        }
        if matches!(boundary, BoundaryKind::Periodic { .. }) {
            return Err(RtError::Unsupported("periodic boundary in the analytic world"));
        }
        let n = shapes.len();
        let mut regions: Vec<RegionInfo> =
            (0..n).map(|i| RegionInfo { entity: None, name: format!("shape{i}"), implicit_complement: false }).collect();
        regions.push(RegionInfo { entity: None, name: "background".into(), implicit_complement: true });
        let bg = RegionId::new(n);
        let mut surfaces: Vec<SurfaceInfo> =
            (0..n).map(|i| SurfaceInfo { entity: None, front: bg, back: RegionId::new(i), boundary: None }).collect();
        surfaces.push(SurfaceInfo { entity: None, front: RegionId::NONE, back: RegionId::NONE, boundary: None });
        for _ in 0..6 {
            surfaces.push(SurfaceInfo { entity: None, front: RegionId::NONE, back: bg, boundary: Some(boundary) });
        }
        let diag = (domain.hi - domain.lo).norm();
        Ok(Self { shapes, domain, boundary, regions: RegionTable(regions), surfaces: SurfaceTable(surfaces), tol: 1e-12 * diag })
    }

    fn bg(&self) -> usize {
        self.shapes.len()
    }
    fn dom_surface(&self, k: usize) -> SurfaceId {
        SurfaceId::new(self.bg() + 1 + k)
    }

    fn classify(&self, p: Point3) -> usize {
        self.shapes.iter().position(|s| s.signed_distance(p) <= 0.0).unwrap_or(self.bg())
    }

    /// `(t, face index k)` where the ray leaves the domain box.
    fn domain_exit(&self, o: Point3, d: Vector3) -> (f64, usize) {
        let mut best = (f64::INFINITY, 0);
        for a in 0..3 {
            let v = compv(d, a);
            if v == 0.0 {
                continue;
            }
            let (t, k) = if v > 0.0 { ((comp(self.domain.hi, a) - comp(o, a)) / v, 2 * a + 1) } else { ((comp(self.domain.lo, a) - comp(o, a)) / v, 2 * a) };
            if t < best.0 {
                best = (t, k);
            }
        }
        best
    }
}

impl Tracker for AnalyticWorld {
    fn caps(&self) -> TrackerCaps {
        TrackerCaps::ANALYTIC.union(TrackerCaps::EXACT_PLANAR)
    }
    fn bounds(&self) -> Aabb {
        self.domain
    }
    fn regions(&self) -> &RegionTable {
        &self.regions
    }
    fn surfaces(&self) -> &SurfaceTable {
        &self.surfaces
    }
    fn cell_count(&self) -> usize {
        self.shapes.len() + 1
    }
    fn cell_region(&self, c: CellId) -> Option<RegionId> {
        (c.get() < self.cell_count()).then_some(RegionId(c.0))
    }
    /// Full primitive volume (nesting is not subtracted); `None` for the background.
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        self.shapes.get(c.get()).map(Shape::volume)
    }
    fn structure_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        for s in &self.shapes {
            match *s {
                Shape::Sphere { center, radius } => {
                    h.write_u64(1);
                    for v in [center.x, center.y, center.z, radius] {
                        h.write_f64(v);
                    }
                }
                Shape::Box { lo, hi } => {
                    h.write_u64(2);
                    for v in [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z] {
                        h.write_f64(v);
                    }
                }
            }
        }
        for p in [self.domain.lo, self.domain.hi] {
            for v in [p.x, p.y, p.z] {
                h.write_f64(v);
            }
        }
        h.finish()
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, _hint: Option<CellId>) -> Result<Located> {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(RtError::NonFinite("position"));
        }
        if !self.domain.contains(p) {
            return Ok(Located { cell: CellId::NONE, region: RegionId::NONE, on_face: None });
        }
        let on_face = self.shapes.iter().position(|s| s.signed_distance(p).abs() <= self.tol).map(FaceId::new);
        let q = match (on_face, dir) {
            (Some(_), Some(d)) => p + d * (self.tol * 10.0),
            _ => p,
        };
        let c = self.classify(q);
        Ok(Located { cell: CellId::new(c), region: RegionId::new(c), on_face })
    }

    fn step(&self, st: &mut TrackState, max_dist: f64) -> Result<Step> {
        check_direction(st.dir)?;
        let from = st.cell;
        let region = st.region;
        if from.is_none() || from.get() > self.bg() {
            return Ok(Step { dist: 0.0, cell: from, region, end: StepEnd::Lost(lost(st, LostCause::NoCellContainsPoint)) });
        }
        let skip = st.entered_via.map(|f| f.get());
        let (t_dom, k_dom) = self.domain_exit(st.pos, st.dir);
        // Breakpoints (t, surface), only strictly ahead; the entry surface is ignored near t = 0.
        let mut bps: Vec<(f64, usize)> = Vec::new();
        for (i, s) in self.shapes.iter().enumerate() {
            if let Some((a, b)) = s.interval(st.pos, st.dir) {
                for t in [a, b] {
                    let ignore = skip == Some(i) && t <= self.tol;
                    if t > 0.0 && t < t_dom && !ignore {
                        bps.push((t, i));
                    }
                }
            }
        }
        bps.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        // Walk intervals; the first one whose cell differs ends the step.
        let mut events = bps.iter().peekable();
        let mut hit: Option<(f64, usize, usize)> = None;
        while let Some(&&(t, surf)) = events.peek() {
            events.next();
            // Interval (t, next distinct t or t_dom).
            let next = events.peek().map_or(t_dom, |e| e.0);
            if next > t {
                let mid = st.pos + st.dir * (0.5 * (t + next));
                let c = self.classify(mid);
                if c != from.get() {
                    hit = Some((t, surf, c));
                    break;
                }
            }
        }
        let limit_first = |t: f64| t > max_dist;
        match hit {
            Some((t, _, _)) if limit_first(t) => {}
            Some((t, surf, c)) => {
                super::tracker::move_along(st, t);
                st.cell = CellId::new(c);
                st.region = RegionId::new(c);
                st.entered_via = Some(FaceId::new(surf));
                let face = FaceId::new(surf);
                let end = StepEnd::RegionBoundary { surface: SurfaceId::new(surf), face, to_region: st.region, to_cell: st.cell };
                return Ok(Step { dist: t, cell: from, region, end });
            }
            None if !limit_first(t_dom) => {
                super::tracker::move_along(st, t_dom);
                st.entered_via = None;
                let surface = self.dom_surface(k_dom);
                let end = StepEnd::DomainBoundary { surface, face: FaceId(surface.0), kind: self.boundary };
                return Ok(Step { dist: t_dom, cell: from, region, end });
            }
            None => {}
        }
        let d = max_dist.min(t_dom);
        if !d.is_finite() {
            return Ok(Step { dist: 0.0, cell: from, region, end: StepEnd::Lost(lost(st, LostCause::NonFinite)) });
        }
        super::tracker::move_along(st, d);
        st.entered_via = None;
        Ok(Step { dist: d, cell: from, region, end: StepEnd::Limit })
    }

    fn closest(&self, p: Point3, region: Option<RegionId>) -> Result<Option<Closest>> {
        let mut best: Option<Closest> = None;
        for (i, s) in self.shapes.iter().enumerate() {
            if region.is_some_and(|r| r.get() != i) {
                continue;
            }
            let d = s.signed_distance(p).abs();
            if best.is_none_or(|b| d < b.dist) {
                best = Some(Closest { dist: d, surface: SurfaceId::new(i), point: s.closest_point(p) });
            }
        }
        Ok(best)
    }

    fn face_normal(&self, f: FaceId) -> Result<Vector3> {
        let k = f.get().checked_sub(self.bg() + 1).filter(|&k| k < 6).ok_or(RtError::Unsupported("normal of a curved primitive"))?;
        let mut n = [0.0; 3];
        n[k / 2] = if k % 2 == 0 { -1.0 } else { 1.0 };
        Ok(Vector3::new(n[0], n[1], n[2]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tally::{EndCode, Flow, Segment, TraceEnd, TraceOptions, TrackBundle, TrackLengthTally};
    use crate::tally::{TallySpec};
    use crate::tracker::BoundaryAction;
    use sm_geom_api::Ray;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }
    fn domain() -> Aabb {
        Aabb { lo: p(-5.0, -5.0, -5.0), hi: p(5.0, 5.0, 5.0) }
    }
    fn world(shapes: Vec<Shape>, bc: BoundaryKind) -> AnalyticWorld {
        AnalyticWorld::new(shapes, domain(), bc).unwrap()
    }
    fn sphere(r: f64) -> Shape {
        Shape::Sphere { center: p(0.0, 0.0, 0.0), radius: r }
    }

    fn chords(w: &AnalyticWorld, o: Point3, d: Vector3) -> (Vec<Segment>, TraceEnd) {
        let mut st = w.init_state(o, d, None).unwrap();
        let mut v = Vec::new();
        let end = w
            .trace(&mut st, &TraceOptions::default(), &mut |s: &Segment| {
                v.push(*s);
                Flow::Continue
            })
            .unwrap();
        (v, end)
    }

    #[test]
    fn sphere_chord_is_exact_for_many_impact_parameters() {
        let r = 2.0;
        let w = world(vec![sphere(r)], BoundaryKind::Vacuum);
        for b in [0.0, 0.3, 1.0, 1.7, 1.999, 1.999999] {
            let (segs, end) = chords(&w, p(-5.0, b, 0.0), Vector3::new(1.0, 0.0, 0.0));
            assert!(matches!(end, TraceEnd::Domain { kind: BoundaryKind::Vacuum, .. }));
            assert_eq!(segs.len(), 3, "b={b}");
            let want = 2.0 * ((r - b) * (r + b)).sqrt();
            let got = segs[1].length;
            assert!((got - want).abs() <= 1e-12 * want.max(1e-3), "b={b} got={got} want={want}");
            assert_eq!(segs[1].cell, CellId(0));
            let total: f64 = segs.iter().map(|s| s.length).sum();
            assert!((total - 10.0).abs() < 1e-12);
            assert!((segs[1].t0 - (5.0 - want / 2.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn oblique_chord_through_sphere_and_miss() {
        let w = world(vec![sphere(2.0)], BoundaryKind::Vacuum);
        // Through the centre along a body diagonal: chord is the diameter.
        let s3 = 3f64.sqrt();
        let d = Vector3::new(1.0 / s3, 1.0 / s3, 1.0 / s3);
        let (segs, _) = chords(&w, p(-3.0, -3.0, -3.0), d);
        assert!((segs.iter().find(|s| s.cell == CellId(0)).unwrap().length - 4.0).abs() < 1e-12);
        // Offset parallel ray that misses.
        let (segs, _) = chords(&w, p(-5.0, 2.5, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(segs.len(), 1);
        assert!((segs[0].length - 10.0).abs() < 1e-12);
    }

    #[test]
    fn nested_shells_partition_the_outer_chord() {
        let (ri, ro) = (1.0, 2.0);
        let w = world(vec![sphere(ri), sphere(ro)], BoundaryKind::Vacuum);
        let b = 0.6;
        let (segs, _) = chords(&w, p(-5.0, b, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let cells: Vec<u32> = segs.iter().map(|s| s.cell.0).collect();
        assert_eq!(cells, vec![2, 1, 0, 1, 2]);
        let inner = 2.0 * (ri * ri - b * b).sqrt();
        let outer = 2.0 * (ro * ro - b * b).sqrt();
        assert!((segs[2].length - inner).abs() < 1e-12);
        assert!((segs[1].length + segs[2].length + segs[3].length - outer).abs() < 1e-12);
    }

    #[test]
    fn box_chord_and_ray_starting_inside() {
        let w = world(vec![Shape::Box { lo: p(-1.0, -1.0, -1.0), hi: p(1.0, 1.0, 1.0) }], BoundaryKind::Vacuum);
        let (segs, _) = chords(&w, p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(segs.len(), 2);
        assert!((segs[0].length - 1.0).abs() < 1e-14 && (segs[1].length - 4.0).abs() < 1e-14);
        assert_eq!(segs[0].face_in, None);
        assert_eq!(segs[1].face_in, Some(FaceId(0)));
    }

    #[test]
    fn fire_stops_at_region_boundary_and_reports_next_region() {
        let w = world(vec![sphere(2.0)], BoundaryKind::Vacuum);
        let mut st = w.init_state(p(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 100).unwrap();
        assert!((r.dist - 3.0).abs() < 1e-12);
        let StepEnd::RegionBoundary { surface, to_region, .. } = r.end else { panic!("{:?}", r.end) };
        assert_eq!(to_region, RegionId(0));
        assert_eq!(w.surfaces().next_region(surface, RegionId(1)), Some(RegionId(0)));
        // Limit shorter than the boundary.
        let mut st = w.init_state(p(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None).unwrap();
        let r = w.fire(&mut st, 1.0, 100).unwrap();
        assert!(matches!(r.end, StepEnd::Limit) && (r.dist - 1.0).abs() < 1e-15);
        assert!(w.point_in_region(RegionId(0), p(0.5, 0.0, 0.0), None).unwrap());
        assert!(!w.point_in_region(RegionId(0), p(3.0, 0.0, 0.0), None).unwrap());
    }

    #[test]
    fn locate_on_surface_uses_direction() {
        let w = world(vec![sphere(2.0)], BoundaryKind::Vacuum);
        let inward = w.locate(p(-2.0, 0.0, 0.0), Some(Vector3::new(1.0, 0.0, 0.0)), None).unwrap();
        let outward = w.locate(p(-2.0, 0.0, 0.0), Some(Vector3::new(-1.0, 0.0, 0.0)), None).unwrap();
        assert_eq!((inward.cell, outward.cell), (CellId(0), CellId(1)));
        assert_eq!(inward.on_face, Some(FaceId(0)));
        assert_eq!(w.locate(p(9.0, 0.0, 0.0), None, None).unwrap().cell, CellId::NONE);
    }

    #[test]
    fn closest_to_sphere() {
        let w = world(vec![sphere(2.0)], BoundaryKind::Vacuum);
        let c = w.closest(p(0.0, 0.0, 3.5), None).unwrap().unwrap();
        assert!((c.dist - 1.5).abs() < 1e-15 && (c.point.z - 2.0).abs() < 1e-15);
    }

    #[test]
    fn reflective_box_conserves_path_length() {
        let w = world(vec![], BoundaryKind::Reflective);
        let mut st = w.init_state(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None).unwrap();
        let mut travelled = 0.0;
        for _ in 0..5 {
            let r = w.fire(&mut st, f64::INFINITY, 10).unwrap();
            travelled += r.dist;
            let StepEnd::DomainBoundary { face, kind, .. } = r.end else { panic!() };
            assert_eq!(w.resolve_boundary(&mut st, face, kind, [0.0, 0.0]).unwrap(), BoundaryAction::Continue);
        }
        // 5 + 10 + 10 + 10 + 10 and back to x = 5, 3 reflections: ends at x = -5 after 4 legs, then 5th leg.
        assert!((travelled - 45.0).abs() < 1e-12, "{travelled}");
        assert_eq!(st.dir.x, -1.0);
    }

    #[test]
    fn white_boundary_reemits_inward_with_unit_direction() {
        let w = world(vec![], BoundaryKind::White);
        let mut st = w.init_state(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10).unwrap();
        let StepEnd::DomainBoundary { face, kind, .. } = r.end else { panic!() };
        w.resolve_boundary(&mut st, face, kind, [0.37, 0.81]).unwrap();
        assert!((st.dir.norm() - 1.0).abs() < 1e-14 && st.dir.x < 0.0);
        assert!((st.dir.x * st.dir.x - (1.0 - 0.37)).abs() < 1e-12, "cos^2 = 1-u0");
    }

    #[test]
    fn vacuum_terminates_and_periodic_is_unsupported() {
        let w = world(vec![], BoundaryKind::Vacuum);
        let mut st = w.init_state(p(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), None).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10).unwrap();
        let StepEnd::DomainBoundary { face, kind, .. } = r.end else { panic!() };
        assert_eq!(w.resolve_boundary(&mut st, face, kind, [0.0; 2]).unwrap(), BoundaryAction::Terminate);
        assert!(AnalyticWorld::new(vec![], domain(), BoundaryKind::Periodic { partner: SurfaceId(0) }).is_err());
    }

    #[test]
    fn bad_input_is_a_typed_error() {
        let w = world(vec![sphere(1.0)], BoundaryKind::Vacuum);
        assert!(matches!(w.init_state(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), None), Err(RtError::BadDirection { .. })));
        assert!(matches!(w.init_state(p(f64::NAN, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None), Err(RtError::NonFinite(_))));
        assert!(AnalyticWorld::new(vec![sphere(-1.0)], domain(), BoundaryKind::Vacuum).is_err());
    }

    fn fan(n: usize) -> Vec<Ray> {
        // Deterministic fan of rays across the domain, no RNG needed.
        (0..n)
            .map(|i| {
                let a = 0.1 + 2.9 * (i as f64) / (n as f64);
                Ray { origin: p(-4.9, -3.0 + 6.0 * (i as f64) / (n as f64), 0.2), dir: Vector3::new(a.cos(), a.sin() * 0.3, 0.1) }
            })
            .collect()
    }

    #[test]
    fn bundle_is_identical_for_one_and_many_threads_and_tally_sums_to_path() {
        let w = world(vec![sphere(1.0), sphere(2.0), Shape::Box { lo: p(2.5, -1.0, -1.0), hi: p(3.5, 1.0, 1.0) }], BoundaryKind::Vacuum);
        let rays = fan(400);
        let opts = TraceOptions::default();
        let serial = w.trace_bundle(&rays, &opts).unwrap();
        // Fixed block partition (independent of thread count), blocks merged in order.
        let threaded = {
            let blocks: Vec<&[Ray]> = rays.chunks(64).collect();
            let parts: Vec<TrackBundle> = std::thread::scope(|sc| {
                let hs: Vec<_> = blocks.iter().map(|b| sc.spawn(|| w.trace_bundle(b, &opts).unwrap())).collect();
                hs.into_iter().map(|h| h.join().unwrap()).collect()
            });
            let mut all = TrackBundle::new();
            for part in &parts {
                all.append(part);
            }
            all
        };
        assert_eq!(serial.content_hash(), threaded.content_hash());
        assert_eq!(serial, threaded);
        assert_eq!(serial.ray_count(), 400);
        assert!(serial.end.iter().all(|e| *e == EndCode::Vacuum));
        // Tally by cell: per-ray scored length equals the geometric path to the domain wall.
        let mut tally = TrackLengthTally::new(w.bin_map(&TallySpec::Cells).unwrap());
        for r in &rays {
            let n = r.dir.norm();
            let mut st = w.init_state(r.origin, r.dir * (1.0 / n), None).unwrap();
            w.trace(&mut st, &TraceOptions::default(), &mut tally).unwrap();
        }
        let sum: f64 = tally.acc.iter().sum();
        let bundle_sum: f64 = serial.length.iter().sum();
        assert!((sum - bundle_sum).abs() < 1e-9 * bundle_sum);
        assert_eq!(w.bin_map(&TallySpec::Regions).unwrap().n_bins, 4);
    }

    #[test]
    fn structure_hash_is_stable_and_sensitive() {
        let a = world(vec![sphere(1.0)], BoundaryKind::Vacuum);
        let b = world(vec![sphere(1.0)], BoundaryKind::Vacuum);
        let c = world(vec![sphere(1.0 + 1e-9)], BoundaryKind::Vacuum);
        assert_eq!(a.structure_hash(), b.structure_hash());
        assert_ne!(a.structure_hash(), c.structure_hash());
        assert_eq!(a.info().cells, 2);
    }
}
