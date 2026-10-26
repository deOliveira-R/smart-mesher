//! The [`World`] trait: the DAGMC query surface plus tracking helpers.

use crate::boundary::{Boundary, BoundaryAction, reflect};
use crate::cache::{CacheBlob, ContentHash};
use crate::error::{Result, RtError};
use crate::handle::{SurfHandle, SurfSense, VolHandle};
use crate::query::{
    Closest, FireOptions, RayHistory, RayHit, RayIn, Segment, TraceEnd, TraceOptions, TraceOutcome,
};
use sm_core::{Aabb, EntityId, Point3, Vector3};
use std::ops::ControlFlow;

/// Static facts about a backend, so callers can pick or report it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BackendInfo {
    /// Short name, e.g. `"analytic"`, `"obb"`, `"bvh8"`.
    pub name: &'static str,
    /// Arithmetic of the hit test. Results are always reported in `f64`.
    pub precision: crate::build::Precision,
    /// True if surfaces are exact (analytic) rather than faceted.
    pub exact_surfaces: bool,
    /// True if the geometry must be watertight to be trusted (faceted backends).
    pub needs_watertight: bool,
}

/// Static description of a volume.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VolumeInfo {
    /// Source entity (`None` for the implicit complement).
    pub entity: Option<EntityId>,
    /// Material or group tag chosen by the deck (`0` = none).
    pub tag: u32,
    /// True for the implicit complement.
    pub is_complement: bool,
    /// Instance this volume belongs to (hierarchy), if the world is instanced.
    pub instance: Option<u32>,
}

/// Static description of a surface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SurfaceInfo {
    /// Source entity (a B-rep face or mesh classification entity).
    pub entity: Option<EntityId>,
    /// Number of facets (1 for analytic surfaces).
    pub facet_count: u32,
}

/// The two volumes of a surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SurfaceVolumes {
    /// Volume the normal points out of.
    pub forward: VolHandle,
    /// Volume the normal points into.
    pub reverse: VolHandle,
}

/// A built, immutable, thread-safe geometry for particle tracking. Cheap to share by `&` or
/// `Arc`; all queries take `&self` and allocate nothing in the common path.
///
/// Conventions: directions are **unit** vectors (checked, `RtError::NotUnit`); distances are in
/// model length units; surface normals point out of the forward volume.
pub trait World: Send + Sync {
    // ---------------------------------------------------------------- identity and structure

    /// Backend facts.
    fn backend_info(&self) -> BackendInfo;
    /// Hash of the build input and parameters; keys the `.smsh` cache. Equal inputs give equal
    /// hashes on any thread count or machine.
    fn content_hash(&self) -> ContentHash;
    /// Bounds of all surfaces.
    fn bounds(&self) -> Aabb;
    /// Number of volumes, complement included. Handles are `0..volume_count()`.
    fn volume_count(&self) -> u32;
    /// Number of surfaces. Handles are `0..surface_count()`.
    fn surface_count(&self) -> u32;
    /// The implicit complement: everything outside every modelled volume (DAGMC
    /// `setup_impl_compl`). Always exists, is unbounded, and its boundary is the outside of the
    /// top-level volumes.
    fn implicit_complement(&self) -> VolHandle;
    /// Volume description.
    fn volume_info(&self, v: VolHandle) -> Result<VolumeInfo>;
    /// Surface description.
    fn surface_info(&self, s: SurfHandle) -> Result<SurfaceInfo>;
    /// Surfaces bounding `v` with their senses, ascending by handle. Clears `out` first.
    fn volume_surfaces(&self, v: VolHandle, out: &mut Vec<(SurfHandle, SurfSense)>) -> Result<()>;
    /// The two volumes of `s`.
    fn surface_volumes(&self, s: SurfHandle) -> Result<SurfaceVolumes>;
    /// Boundary condition of `s`.
    fn boundary(&self, s: SurfHandle) -> Result<Boundary>;

    // ---------------------------------------------------------------- DAGMC queries

    /// Fire a ray from `origin` along unit `dir` inside `volume` (DAGMC `ray_fire`). Returns the
    /// nearest hit that is not in `history`, passes `opts`, and has `dist > 0`; `None` if there is
    /// none. The hit facet is appended to `history`.
    ///
    /// A `None` from a bounded volume without `dist_limit` means a gap in the geometry; use
    /// [`World::trace`] to get that classified.
    fn ray_fire(
        &self,
        volume: VolHandle,
        origin: Point3,
        dir: Vector3,
        opts: FireOptions,
        history: Option<&mut RayHistory>,
    ) -> Result<Option<RayHit>>;

    /// Is `p` inside `volume` (DAGMC `point_in_volume`)? A point on a surface is decided by `dir`
    /// (inside if the direction enters the volume); with `dir = None` a boundary point belongs
    /// to the forward volume of that surface, so the volumes partition space.
    /// `history` makes the test robust at a point just crossed.
    fn point_in_volume(
        &self,
        volume: VolHandle,
        p: Point3,
        dir: Option<Vector3>,
        history: Option<&RayHistory>,
    ) -> Result<bool>;

    /// Ray-free point containment (DAGMC `point_in_volume_slow`: solid-angle / winding number).
    /// Slower, used to cross-check the ray method.
    fn point_in_volume_exact(&self, volume: VolHandle, p: Point3) -> Result<bool> {
        self.point_in_volume(volume, p, None, None)
    }

    /// Distance from `p` to the nearest point of `volume`'s boundary and the surface holding it
    /// (DAGMC `closest_to_location`).
    fn closest_to_location(&self, volume: VolHandle, p: Point3) -> Result<Closest>;

    /// Unit normal of `surface` at `p`, pointing out of its forward volume (DAGMC `get_angle`).
    /// `p` should lie on the surface; `history` names the facet when `p` is on a facet edge.
    fn get_angle(&self, surface: SurfHandle, p: Point3, history: Option<&RayHistory>) -> Result<Vector3>;

    /// Alias of [`World::get_angle`] with a name that says what it returns.
    fn normal(&self, surface: SurfHandle, p: Point3, history: Option<&RayHistory>) -> Result<Vector3> {
        self.get_angle(surface, p, history)
    }

    /// Volume of `v` (DAGMC `measure_volume`). `Unbounded` for the implicit complement.
    fn measure_volume(&self, v: VolHandle) -> Result<f64>;
    /// Area of `s` (DAGMC `measure_area`).
    fn measure_area(&self, s: SurfHandle) -> Result<f64>;

    /// Sense of `s` with respect to `v` (DAGMC `surface_sense`). If `s` has `v` on both sides
    /// (a baffle), `Forward` is returned.
    fn surface_sense(&self, v: VolHandle, s: SurfHandle) -> Result<SurfSense> {
        let sv = self.surface_volumes(s)?;
        if sv.forward == v {
            Ok(SurfSense::Forward)
        } else if sv.reverse == v {
            Ok(SurfSense::Reverse)
        } else {
            Err(RtError::NotIncident { surface: s, volume: v })
        }
    }

    /// The volume on the other side of `s` from `old` (DAGMC `next_vol`).
    fn next_vol(&self, s: SurfHandle, old: VolHandle) -> Result<VolHandle> {
        let sv = self.surface_volumes(s)?;
        if sv.forward == old {
            Ok(sv.reverse)
        } else if sv.reverse == old {
            Ok(sv.forward)
        } else {
            Err(RtError::NotIncident { surface: s, volume: old })
        }
    }

    /// Does a ray at `p` on `s` with direction `dir` enter `volume`? (DAGMC
    /// `test_volume_boundary`; `true` = entering.)
    fn test_volume_boundary(
        &self,
        volume: VolHandle,
        s: SurfHandle,
        p: Point3,
        dir: Vector3,
        history: Option<&RayHistory>,
    ) -> Result<bool> {
        let n = self.get_angle(s, p, history)?;
        let sense = self.surface_sense(volume, s)?;
        Ok(dir.dot(n) * sense.sign() < 0.0)
    }

    /// The volume containing `p` (DAGMC `find_volume`). Tests volumes in ascending handle order
    /// (backends should override with an accelerated search). `Ambiguous` if not exactly one
    /// non-complement volume claims `p` and the complement does not hold it either.
    fn find_volume(&self, p: Point3, dir: Option<Vector3>) -> Result<VolHandle> {
        let ic = self.implicit_complement();
        let mut found: Option<VolHandle> = None;
        let mut count = 0u32;
        for i in 0..self.volume_count() {
            let v = VolHandle(i);
            if v == ic {
                continue;
            }
            if self.point_in_volume(v, p, dir, None)? {
                count += 1;
                found.get_or_insert(v);
            }
        }
        match (count, found) {
            (1, Some(v)) => Ok(v),
            (0, _) => Ok(ic),
            _ => Err(RtError::Ambiguous { point: p, count }),
        }
    }

    // ---------------------------------------------------------------- beyond DAGMC

    /// Walk a ray through consecutive volumes, calling `sink` once per [`Segment`] in order.
    /// Stops at `opts.max_dist`, at a non-interior boundary, on escape, on a lost particle, or
    /// when `sink` returns `Break`. This is the call MOC/Sn ray sweeps use; it replaces a loop of
    /// `ray_fire` + `next_vol`. `history` is updated as in `ray_fire` and may be reused for the
    /// next call.
    fn trace(
        &self,
        start: VolHandle,
        origin: Point3,
        dir: Vector3,
        opts: TraceOptions,
        history: &mut RayHistory,
        sink: &mut dyn FnMut(&Segment) -> ControlFlow<()>,
    ) -> Result<TraceOutcome> {
        let ic = self.implicit_complement();
        let mut vol = start;
        let mut t = 0.0f64;
        let mut crossings = 0u32;
        let done = |end, t, vol, crossings| Ok(TraceOutcome { end, dist: t, volume: vol, crossings });
        loop {
            let hit = self.ray_fire(vol, origin + dir * t, dir, FireOptions::default(), Some(history))?;
            let Some(hit) = hit else {
                if vol == ic {
                    if opts.max_dist.is_finite() && opts.max_dist > t {
                        let seg = Segment { volume: vol, t0: t, t1: opts.max_dist, exit: None };
                        let _ = sink(&seg);
                        return done(TraceEnd::Distance, opts.max_dist, vol, crossings);
                    }
                    return done(TraceEnd::Escaped, t, vol, crossings);
                }
                return done(TraceEnd::Lost { volume: vol, t }, t, vol, crossings);
            };
            let t_hit = t + hit.dist;
            if t_hit >= opts.max_dist {
                history.rollback_last_intersection();
                let seg = Segment { volume: vol, t0: t, t1: opts.max_dist, exit: None };
                let _ = sink(&seg);
                return done(TraceEnd::Distance, opts.max_dist, vol, crossings);
            }
            let seg = Segment { volume: vol, t0: t, t1: t_hit, exit: Some(hit.surface) };
            if sink(&seg).is_break() {
                return done(TraceEnd::Stopped, t_hit, vol, crossings);
            }
            t = t_hit;
            crossings += 1;
            if crossings > opts.max_crossings {
                return Err(RtError::TooManyCrossings(opts.max_crossings));
            }
            match self.boundary(hit.surface)? {
                Boundary::Interior => vol = self.next_vol(hit.surface, vol)?,
                kind => return done(TraceEnd::Boundary { surface: hit.surface, kind, t }, t, vol, crossings),
            }
        }
    }

    /// Resolve a boundary after [`World::trace`] returned `TraceEnd::Boundary`. `pos` and `dir`
    /// are the particle state at the surface, `vol` the volume it is leaving.
    fn apply_boundary(
        &self,
        surface: SurfHandle,
        vol: VolHandle,
        pos: Point3,
        dir: Vector3,
    ) -> Result<BoundaryAction> {
        let kind = self.boundary(surface)?;
        let sense = self.surface_sense(vol, surface)?;
        match kind {
            Boundary::Interior => Ok(BoundaryAction::Continue { volume: self.next_vol(surface, vol)? }),
            Boundary::Vacuum => Ok(BoundaryAction::Leak),
            Boundary::Reflecting => {
                let n = self.get_angle(surface, pos, None)?;
                Ok(BoundaryAction::Reflect { dir: reflect(dir, n) })
            }
            Boundary::White => {
                let n = self.get_angle(surface, pos, None)?;
                // Outward from `vol` is n * sense; inward is the opposite.
                Ok(BoundaryAction::White { inward_normal: n * -sense.sign(), volume: vol })
            }
            Boundary::Periodic { partner, map } => {
                let pos2 = map.apply_point(pos);
                let dir2 = map.apply_vector(dir);
                let n2 = self.get_angle(partner, pos2, None)?;
                let sv = self.surface_volumes(partner)?;
                // Moving against the partner normal enters its forward volume.
                let volume = if dir2.dot(n2) < 0.0 { sv.forward } else { sv.reverse };
                Ok(BoundaryAction::Periodic { pos: pos2, dir: dir2, surface: partner, volume })
            }
        }
    }

    /// Fire many rays. `out[i]` is the result for `rays[i]`; each ray has its own history slot.
    /// The default is sequential; backends may use any parallelism that keeps `out` identical
    /// (each ray is independent, so order cannot matter).
    fn ray_fire_batch(
        &self,
        rays: &[RayIn],
        opts: FireOptions,
        histories: &mut [RayHistory],
        out: &mut Vec<Result<Option<RayHit>>>,
    ) -> Result<()> {
        if histories.len() != rays.len() {
            return Err(RtError::BadInput("histories.len() != rays.len()".into()));
        }
        out.clear();
        for (r, h) in rays.iter().zip(histories.iter_mut()) {
            out.push(self.ray_fire(r.volume, r.origin, r.dir, opts, Some(h)));
        }
        Ok(())
    }

    /// The serialized acceleration structure for the `.smsh` cache, if the backend has one.
    fn export_cache(&self) -> Option<CacheBlob> {
        None
    }
}

/// Validate a direction: finite and unit within 1e-9. Backends call this first.
pub fn check_dir(d: Vector3) -> Result<()> {
    if !(d.x.is_finite() && d.y.is_finite() && d.z.is_finite()) {
        return Err(RtError::NonFinite);
    }
    let n = d.norm();
    if (n - 1.0).abs() > 1e-9 {
        return Err(RtError::NotUnit(n));
    }
    Ok(())
}

/// Validate a point: finite.
pub fn check_point(p: Point3) -> Result<()> {
    if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() { Ok(()) } else { Err(RtError::NonFinite) }
}

/// Cell-level tracking over a volume mesh (for Sn sweeps, MOC on meshes, and Monte Carlo on
/// unstructured meshes). A separate trait because it exposes cells, not volumes; implemented by
/// the mesh-backed backend.
pub trait MeshTrack: Send + Sync {
    /// The cell containing `p`, or `None` outside the mesh.
    fn locate_cell(&self, p: Point3, hint: Option<sm_mesh_api::CellIx>) -> Option<sm_mesh_api::CellIx>;
    /// Exit of a ray from inside `cell`: the face crossed, distance and the cell entered
    /// (`None` = domain boundary).
    fn next_cell(
        &self,
        cell: sm_mesh_api::CellIx,
        origin: Point3,
        dir: Vector3,
        skip: Option<sm_mesh_api::FaceIx>,
    ) -> Result<Option<CellExit>>;
}

/// Result of [`MeshTrack::next_cell`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellExit {
    /// Face crossed.
    pub face: sm_mesh_api::FaceIx,
    /// Distance to it.
    pub dist: f64,
    /// Cell entered, `None` at the domain boundary.
    pub next: Option<sm_mesh_api::CellIx>,
}
