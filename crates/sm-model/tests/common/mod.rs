//! Shared test rig: engines over both topology hosts, an op runner, a query runner and a
//! test-only op that inserts a mesh on a volume (to exercise the meshed-volume rule).
#![allow(dead_code)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{CoreLayers, MeshEntry, MeshLayer, ModelLayer, TopoHost, insert_mesh};
use sm_model::ModelPlugin;
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Hash256, Meta, Op, OpError, QuietControl, Registry, Services,
    Session, TimelineError, TypedOp,
};
use sm_topo_api::CellComplex;

pub type Factory = fn() -> Box<dyn TopoHost>;

pub fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

pub fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

const UNIT_CUBE: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// Inserts the unit hex mesh with `selection = [volume]` (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FakeMesh {
    pub volume: EntityId,
}

impl TypedOp for FakeMesh {
    const NAME: &'static str = "test.fake_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let q = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let fx = sm_testkit::meshes::build_mesh(
            &UNIT_CUBE,
            &[sm_testkit::meshes::CellSpec::Template(sm_mesh_api::CellShape::Hex, (0..8).collect())],
            &mut alloc,
        )
        .map_err(|e| OpError::Failed(e.to_string()))?;
        insert_mesh(ctx, MeshEntry::new(Box::new(fx), vec![self.volume], None, q), &before, alloc, "mesh")
    }
}

pub fn registry(factory: Factory) -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).expect("core layers");
    reg.install(&ModelPlugin).expect("model plugin");
    reg.register::<FakeMesh>().expect("fake mesh");
    reg.register::<MirrorDeck>().expect("mirror deck");
    reg.register::<MergedBricks>().expect("merged bricks");
    reg.register::<WebcutBrick>().expect("webcut brick");
    reg
}

pub fn engine(factory: Factory, cache: DerivedCache) -> Engine {
    Engine::new(Arc::new(registry(factory)), Services::new(), Arc::new(cache)).expect("engine")
}

pub fn session(factory: Factory) -> Session {
    Session::new(engine(factory, DerivedCache::default()))
}

pub fn meta() -> Meta {
    Meta::new("test", "", "")
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn range(kind: EntityKind, n: u64) -> Vec<EntityId> {
    (1..=n).map(|s| id(kind, s)).collect()
}

pub fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, sm_op_api::Delta), TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

/// Run a registered query by name with a JSON input.
pub fn query(s: &Session, name: &str, input: serde_json::Value) -> Result<serde_json::Value, sm_op_api::RegistryError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), name, &input, &q.control())
}

pub fn model(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().expect("model layer")
}

/// All vertex points of the model layer, in ascending id order.
pub fn vertex_points(s: &Session) -> Vec<[f64; 3]> {
    use sm_topo_api::{Carrier, Dim};
    let t = model(s).topo();
    t.cells(Dim::Vertex)
        .iter()
        .map(|&v| match t.carrier(v) {
            Carrier::Point(p) => [p.x, p.y, p.z],
            other => panic!("vertex carrier {other:?}"),
        })
        .collect()
}

/// Face orientation oracle: for every volume use, the loop-derived vector area of the face must
/// point away from the volume centroid. Returns the worst `out * n . (c_face - c_vol) / |n|`
/// (positive means every face points outward).
pub fn worst_outwardness(s: &Session) -> f64 {
    use sm_topo_api::{Dim, Side};
    let t = model(s).topo();
    let mut worst = f64::INFINITY;
    for &v in t.cells(Dim::Volume) {
        let m = t.volume_moments(v).expect("volume moments");
        let vc = m.centroid().expect("centroid");
        for inc in t.boundary(v) {
            let fm = t.face_moments(inc.cell).expect("face moments");
            let fc = fm.centroid().expect("face centroid");
            let out = if inc.side == Side::Left { 1.0 } else { -1.0 };
            assert!(fm.area > 0.0, "face {} has non-positive area {}", inc.cell, fm.area);
            let n = fm.vector_area;
            if n.norm() < 1e-9 * fm.area {
                // A closed curved face (a cylinder side) has no net vector area.
                continue;
            }
            let d = sm_core::Vector3::new(fc.x - vc.x, fc.y - vc.y, fc.z - vc.z);
            let len = n.norm().max(1e-300);
            worst = worst.min(out * n.dot(d) / len);
        }
    }
    worst
}

/// `validate` and `validate_geometry` issues of the model layer's topology.
pub fn issues(s: &Session) -> Vec<String> {
    let t = model(s).topo();
    sm_topo_api::validate(t).into_iter().chain(sm_topo_api::validate_geometry(t)).map(|i| format!("{i:?}")).collect()
}

/// Declares a self-paired mirror deck on `face` (plane `normal . x = offset`) with a
/// model-only transaction (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MirrorDeck {
    pub face: EntityId,
    pub normal: [f64; 3],
    pub offset: f64,
}

impl TypedOp for MirrorDeck {
    const NAME: &'static str = "test.mirror_deck";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, model_txn};
        use sm_model_api::{BoundaryKind, Deck, Plane};
        let plane = Plane::new(sm_core::Vector3::new(self.normal[0], self.normal[1], self.normal[2]), self.offset).or_op()?;
        model_txn(
            ctx,
            |m, t| {
                use sm_model_api::Domain;
                let vols: Vec<EntityId> = sm_topo_api::TopologyView::cells(t, sm_topo_api::Dim::Volume).to_vec();
                let mat = m.add_color(1, "mat", Domain::Volume).or_op()?;
                m.color_volumes(mat, &vols).or_op()?;
                let cap = m.add_color(9, "cap", Domain::Face).or_op()?;
                m.color_faces(cap, &[self.face]).or_op()?;
                let b = m.add_boundary("mirror", BoundaryKind::Deck(Deck::Mirror { plane })).or_op()?;
                m.assign_self_paired(b, &[self.face]).or_op()?;
                sm_testkit::topo::finish_boundaries(m, t, 7).or_op()?;
                Ok(b)
            },
            |b, _| vec![("boundary", vec![*b])],
        )?;
        Ok(())
    }
}

/// Two touching bricks, merged along their shared face (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergedBricks {}

impl TypedOp for MergedBricks {
    const NAME: &'static str = "test.merged_bricks";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, commit_topology, topo_txn};
        use sm_topo_api::fixtures;
        let ((a, b), txn) = topo_txn(ctx, |topo, _| {
            let (a, b) = fixtures::two_bricks(topo).or_op()?;
            fixtures::merge_touching_bricks(topo, &a, &b).or_op()?;
            Ok((a, b))
        })?;
        commit_topology(ctx, &txn, &[("volumes", &[a.volume, b.volume])])
    }
}

/// A 1 x 1 x 2 brick webcut at mid-height: one body with two volumes sharing the cap (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WebcutBrick {}

impl TypedOp for WebcutBrick {
    const NAME: &'static str = "test.webcut_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, commit_topology, topo_txn};
        use sm_topo_api::fixtures;
        let (vols, txn) = topo_txn(ctx, |topo, _| {
            let b = fixtures::brick(topo, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 2.0]).or_op()?;
            let w = fixtures::webcut_brick(topo, &b, 0.5).or_op()?;
            Ok(w.volumes)
        })?;
        commit_topology(ctx, &txn, &[("volumes", &vols)])
    }
}

/// `model.measure` of one entity, decoded.
pub fn measure(s: &Session, e: EntityId) -> sm_model::Measurement {
    serde_json::from_value(query(s, "model.measure", serde_json::json!({"entity": e.to_string()})).expect("measure")).expect("decode")
}

/// Independent checks of the (only) sphere face of the model, which is checked from the geometry
/// alone (closed form, independent of the numeric `face_moments`). Verifies, from the stored geometry alone:
/// the surface is a sphere of the expected centre and radius; the face normal
/// (`sense * du x dv`) points radially outward; a numerical divergence integral
/// `V = 1/3 oint (p - c) . n dA` over the parameter domain gives `4/3 pi r^3`; every pcurve lies
/// on the surface and on its edge; and the (u, v) loop runs the way the carrier sense expects,
/// so the loop-derived normal is outward too.
pub fn check_sphere(s: &Session, center: [f64; 3], r: f64) {
    use sm_core::{Point3, Vector3};
    use sm_geom_api::{SurfaceDef, Uv};
    use sm_topo_api::{Carrier, Dim, Sense, Side, TopologyViewExt};
    let t = model(s).topo();
    let face = t.cells(Dim::Face)[0];
    assert_eq!(t.cells(Dim::Face).len(), 1);
    let Carrier::Surface(sc) = t.carrier(face).clone() else { panic!("sphere face has no surface") };
    assert_eq!(sc.sense, Sense::Forward, "carrier sense stays Forward");
    let def = t.shapes().surface_def(sc.surface).expect("surface def");
    let SurfaceDef::Sphere(sp) = def else { panic!("not a sphere: {def:?}") };
    let c = Point3::new(center[0], center[1], center[2]);
    assert!((sp.radius - r).abs() < 1e-12 * r, "radius {} vs {r}", sp.radius);
    assert!((sp.frame.origin - c).norm() < 1e-12 * (1.0 + r), "centre {:?} vs {center:?}", sp.frame.origin);
    let surf = t.shapes().surface(sc.surface).expect("surface");
    let sign = sc.sense.sign();
    // Face normal vs the radial direction, and the divergence integral.
    let (nu, nv) = (96usize, 48usize);
    let (du, dv) = (core::f64::consts::TAU / nu as f64, core::f64::consts::PI / nv as f64);
    let mut vol = 0.0;
    for i in 0..nu {
        for j in 0..nv {
            let uv = Uv::new((i as f64 + 0.5) * du, -core::f64::consts::FRAC_PI_2 + (j as f64 + 0.5) * dv);
            let jet = surf.jet(uv);
            let radial: Vector3 = (jet.p - c) * (1.0 / r);
            let n = jet.du.cross(jet.dv) * sign; // area-weighted face normal
            assert!(n.dot(radial) > 0.0, "face normal points inward at {uv:?}");
            vol += (jet.p - c).dot(n) * du * dv / 3.0;
        }
    }
    let want = 4.0 / 3.0 * core::f64::consts::PI * r * r * r;
    assert!((vol - want).abs() < 2e-3 * want, "divergence volume {vol} vs {want}");
    // The pcurves lie on the surface and the edge curves; the loop's (u, v) winding matches
    // the carrier sense: CCW in (u, v) with `du x dv` outward (and sense Forward) is outward.
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for co in t.coedges(face) {
        let pc = t.pcurve(co.key).expect("pcurve");
        let rg = pc.range();
        let mut seg: Vec<(f64, f64)> = (0..=16).map(|k| rg.lo + (rg.hi - rg.lo) * k as f64 / 16.0).map(|x| (pc.eval(x).u, pc.eval(x).v)).collect();
        if let Carrier::Curve(cc) = t.carrier(co.key.edge) {
            let curve = t.shapes().curve(cc.curve).expect("curve");
            for k in 0..=4 {
                let x = rg.lo + (rg.hi - rg.lo) * k as f64 / 4.0;
                assert!((surf.eval(pc.eval(x)) - curve.eval(x)).norm() < 1e-9 * (1.0 + r), "pcurve off its edge");
            }
        }
        if co.key.side == Side::Right {
            seg.reverse();
        }
        pts.extend(seg);
    }
    let shoelace: f64 = pts.iter().zip(pts.iter().cycle().skip(1)).map(|(a, b)| a.0 * b.1 - b.0 * a.1).sum::<f64>() / 2.0;
    let jet = surf.jet(Uv::new(1.0, 0.3));
    let outward_uv = jet.du.cross(jet.dv).dot(jet.p - c).signum() * sign;
    assert!(shoelace * outward_uv > 0.0, "loop runs against the face normal (shoelace {shoelace})");
}

/// Model validation errors against the model layer's topology (planarity tolerance 1e-6: the
/// second-moment planarity test loses digits at coordinates of order 10).
///
/// TODO(CR-2 follow-up): the loosened tolerance is a workaround for the `FaceMoments::plane`
/// cancellation. sm-topo-api now offers the stable `TopologyView::face_plane`, but `validate` and
/// `symmetry` (frozen sm-model-api) still call `FaceMoments::plane`; once they switch (separately
/// approved), use `Tolerance::default()` here so the mirror-deck acceptance runs at the default.
pub fn model_errors(s: &Session) -> Vec<String> {
    let l = model(s);
    sm_model_api::validate(l.model(), l.topo(), sm_model_api::Tolerance::default()).errors().map(|i| format!("{:?}: {}", i.code, i.detail)).collect()
}
