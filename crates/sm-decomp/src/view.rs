//! The adapter from a topology view to a [`VolumeView`].

use crate::plan::{CutStep, DecompError, FaceKind, FaceView, VolumeView, plan_cuts_view};
use sm_core::EntityId;
use sm_geom_api::SurfaceDef;
use sm_topo_api::{Carrier, Dim, Sense, Side, TopologyView};

/// Read one volume of `topo` into a [`VolumeView`].
///
/// A cylinder face is a cavity wall (`outward: false`) when the volume's outward normal on it
/// points toward the axis. A baffle face (the volume on both sides) and every surface other than
/// plane and cylinder become [`FaceKind::Other`].
///
/// # Errors
/// [`DecompError::NotAVolume`] when `volume` is not a live volume of `topo`.
pub fn volume_view(topo: &dyn TopologyView, volume: EntityId) -> Result<VolumeView, DecompError> {
    if !topo.is_alive(volume) || topo.dim_of(volume) != Some(Dim::Volume) {
        return Err(DecompError::NotAVolume(volume));
    }
    let faces = topo
        .volume_faces(volume)
        .iter()
        .map(|&f| {
            let side = topo
                .boundary(volume)
                .iter()
                .find(|i| i.cell == f)
                .map(|i| i.side);
            let kind = match (topo.carrier(f), side) {
                (Carrier::Surface(sc), Some(side)) if side != Side::Both => {
                    match topo.shapes().surface_def(sc.surface) {
                        Some(SurfaceDef::Plane(p)) => FaceKind::Plane {
                            origin: [p.frame.origin.x, p.frame.origin.y, p.frame.origin.z],
                            normal: [p.frame.z.x, p.frame.z.y, p.frame.z.z],
                        },
                        Some(SurfaceDef::Cylinder(c)) => {
                            // The surface normal points away from the axis; flip by the carrier sense and the volume side.
                            let s = if sc.sense == Sense::Forward {
                                1.0
                            } else {
                                -1.0
                            } * if side == Side::Left { 1.0 } else { -1.0 };
                            FaceKind::Cylinder {
                                origin: [c.frame.origin.x, c.frame.origin.y, c.frame.origin.z],
                                axis: [c.frame.z.x, c.frame.z.y, c.frame.z.z],
                                radius: c.radius,
                                outward: s > 0.0,
                            }
                        }
                        _ => FaceKind::Other,
                    }
                }
                _ => FaceKind::Other,
            };
            let mut points = Vec::new();
            for &e in topo.face_edges(f) {
                for inc in topo.boundary(e) {
                    if let Carrier::Point(p) = topo.carrier(inc.cell) {
                        points.push([p.x, p.y, p.z]);
                    }
                }
            }
            let area = topo.face_moments(f).ok().map(|m| m.area);
            FaceView {
                id: f,
                kind,
                points,
                area,
            }
        })
        .collect();
    Ok(VolumeView { id: volume, faces })
}

/// Plan the cuts for `volume` of `topo` (see [`plan_cuts_view`]); the tolerance is the model's.
///
/// # Errors
/// [`DecompError::NotAVolume`], or [`DecompError::NoDecomposition`] with the reason.
pub fn plan_cuts(topo: &dyn TopologyView, volume: EntityId) -> Result<Vec<CutStep>, DecompError> {
    plan_cuts_view(
        &volume_view(topo, volume)?,
        topo.tolerance().linear.max(1e-9),
    )
}
