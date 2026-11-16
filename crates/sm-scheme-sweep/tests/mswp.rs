//! Many-to-many sweeps over imprinted faces (W4-HEX-MSWEEP): one volume whose source and target
//! faces are each split in two by an imprinted line (no internal face) is swept like the same
//! slab split into two volumes and swept 1-to-1.

mod common;

use common::{mesh_with, min_scaled_jacobian, registry, total_volume};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellIx, CellShape, MeshEntityKind, MeshRead, validate};
use sm_model_api::Model;
use sm_scheme_api::{
    Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, SchemeError, Sequential,
};
use sm_scheme_sweep::discover_column_sets;
use sm_topo_api::fixtures::{BrickIds, WebcutIds, brick, webcut_brick, with_op};
use sm_topo_api::{CellComplex, FaceUseSpec, OpId, Side, TopologyEdit, TopologyView};

const NX: u32 = 3;
const NY: u32 = 2;
const NZ_LOW: u32 = 2;
const NZ_HIGH: u32 = 3;

/// A `2 x 1 x 1` brick cut at `z = 0.5`: with the cap kept (two volumes) or removed (one
/// volume with its `-x` and `+x` faces each split in two).
struct Slab {
    topo: CellComplex,
    brick: BrickIds,
    cut: WebcutIds,
}

fn slab(merged: bool) -> Slab {
    let mut topo = CellComplex::new();
    let (b, w) = with_op(&mut topo, OpId(1), |t| {
        let b = brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])?;
        let w = webcut_brick(t, &b, 0.5)?;
        Ok((b, w))
    })
    .expect("fixture");
    if merged {
        let [v0, v1] = w.volumes;
        with_op(&mut topo, OpId(2), |t| {
            let uses = sm_scheme_api::geom::volume_face_uses(t, v1);
            for (f, side) in uses {
                t.remove_face_use(v1, f, Side::Both)?;
                if f != w.cap {
                    t.add_face_use(v0, 0, FaceUseSpec { face: f, side })?;
                }
            }
            t.remove_face_use(v0, w.cap, Side::Both)?;
            t.kill(v1)?;
            t.kill(w.cap)?;
            Ok(())
        })
        .expect("merge");
    }
    Slab {
        topo,
        brick: b,
        cut: w,
    }
}

/// Interval counts: `NX` along the sweep, `NY` across, `NZ_LOW`/`NZ_HIGH` below/above the cut.
fn intent(s: &Slab) -> MeshIntent {
    let mut i = MeshIntent::default();
    let mut set = |e: EntityId, n: u32| i.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    for &e in &s.brick.edges[0..4] {
        set(e, NX);
    }
    for &e in &s.brick.edges[4..8] {
        set(e, NY);
    }
    for &e in &[s.cut.chords[0], s.cut.chords[1]] {
        set(e, NY);
    }
    for &e in &[s.cut.chords[2], s.cut.chords[3]] {
        set(e, NX);
    }
    for k in 0..4 {
        set(s.cut.split_edges[k][0], NZ_LOW);
        set(s.cut.split_edges[k][1], NZ_HIGH);
    }
    i
}

fn sweep(source: &[EntityId], target: &[EntityId]) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": source, "target": target }),
        Firmness::Hard,
    )
}

fn coords(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let c = m.coords();
    let mut v: Vec<[f64; 3]> = (0..c.x.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect();
    sort_rounded(&mut v);
    v
}

/// Sort by coordinates rounded to 1e-9, so rounding noise cannot reorder two equal points.
fn sort_rounded(v: &mut [[f64; 3]]) {
    let key = |p: &[f64; 3]| p.map(|x| (x * 1e9).round() as i64);
    v.sort_by_key(key);
}

/// Cell centroids, sorted (a cell is identified by where it sits).
fn centroids(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let mut v = Vec::new();
    for c in 0..m.count(MeshEntityKind::Cell) {
        let nodes = m.cell_corners(CellIx(c as u32)).expect("corners");
        let co = m.coords();
        let mut s = [0.0; 3];
        for &n in nodes {
            let n = n.0 as usize;
            s[0] += co.x[n];
            s[1] += co.y[n];
            s[2] += co.z[n];
        }
        let k = nodes.len() as f64;
        v.push([s[0] / k, s[1] / k, s[2] / k]);
    }
    sort_rounded(&mut v);
    v
}

fn assert_same(a: &[[f64; 3]], b: &[[f64; 3]], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: count");
    for (p, q) in a.iter().zip(b) {
        for d in 0..3 {
            assert!((p[d] - q[d]).abs() < 1e-12, "{what}: {p:?} vs {q:?}");
        }
    }
}

/// Acceptance 1: the imprinted slab (one volume, `-x` and `+x` each split in two) against the
/// same slab as two volumes swept 1-to-1.
#[test]
fn imprinted_slab_equals_two_volumes_swept_one_to_one() {
    // One volume, set sweep.
    let one = slab(true);
    assert!(sm_topo_api::validate(&one.topo).is_empty());
    let (src, dst) = (one.cut.split_faces[0], one.cut.split_faces[1]);
    let mut it = intent(&one);
    it.set_scheme(one.cut.volumes[0], sweep(&src, &dst));
    let a = mesh_with(
        &registry(),
        &one.topo,
        &Model::new(),
        &[one.cut.volumes[0]],
        &it,
        &Sequential,
    )
    .expect("set sweep");
    let m1 = &a.finished.mesh;
    assert!(validate(m1).is_empty(), "{:?}", validate(m1));

    // Two volumes, 1-to-1 each.
    let two = slab(false);
    let mut it2 = intent(&two);
    for &v in &two.cut.volumes {
        let faces = two.topo.volume_faces(v);
        let k = usize::from(!faces.contains(&two.cut.split_faces[0][0]));
        it2.set_scheme(
            v,
            sweep(&[two.cut.split_faces[0][k]], &[two.cut.split_faces[1][k]]),
        );
    }
    let b = mesh_with(
        &registry(),
        &two.topo,
        &Model::new(),
        &two.cut.volumes,
        &it2,
        &Sequential,
    )
    .expect("1-to-1 sweeps");
    let m2 = &b.finished.mesh;

    // Analytic counts: NX * NY * (NZ_LOW + NZ_HIGH) cells, one more node in each direction.
    let cells = (NX * NY * (NZ_LOW + NZ_HIGH)) as usize;
    let nodes = ((NX + 1) * (NY + 1) * (NZ_LOW + NZ_HIGH + 1)) as usize;
    for (m, what) in [(m1, "set sweep"), (m2, "two volumes")] {
        assert_eq!(m.count(MeshEntityKind::Cell), cells, "{what}");
        assert_eq!(m.count(MeshEntityKind::Vertex), nodes, "{what}");
        assert!((total_volume(m) - 2.0).abs() < 1e-12, "{what}");
        for c in 0..cells {
            assert_eq!(m.cell_shape(CellIx(c as u32)), CellShape::Hex, "{what}");
        }
        assert!((min_scaled_jacobian(m) - 1.0).abs() < 1e-12, "{what}");
    }
    assert_same(&coords(m1), &coords(m2), "node set");
    assert_same(&centroids(m1), &centroids(m2), "cell centroids");
    eprintln!(
        "W4-HEX-MSWEEP: {cells} hex cells, {nodes} nodes, volume {:.15}, min scaled Jacobian {:.15}",
        total_volume(m1),
        min_scaled_jacobian(m1)
    );
}

fn members(t: &CellComplex, v: EntityId) -> Vec<(EntityId, Vec<EntityId>)> {
    vec![(v, t.volume_faces(v).to_vec())]
}

#[test]
fn discovery_pairs_faces_by_their_columns() {
    let s = slab(true);
    let v = s.cut.volumes[0];
    let m = members(&s.topo, v);
    // Sources given in the opposite order to the targets: pairing is by columns, not by order.
    let [sa, sb] = s.cut.split_faces[0];
    let [ta, tb] = s.cut.split_faces[1];
    let c = discover_column_sets(&s.topo, &m, &[sb, sa], &[ta, tb]).expect("columns");
    let mut want = vec![[sa, ta], [sb, tb]];
    want.sort_unstable();
    assert_eq!(c.pairs(), want);
    // 6 vertices on each side; 4 + 2 shared columns... the six linking faces.
    assert_eq!(c.columns.len(), 6);
    assert_eq!(c.linking.len(), 6);
}

/// Acceptance 4: an unsupported set names the face ids and the missing correspondence.
#[test]
fn unsupported_sets_name_the_faces_and_the_correspondence() {
    let s = slab(true);
    let v = s.cut.volumes[0];
    let m = members(&s.topo, v);
    let [sa, sb] = s.cut.split_faces[0];
    let [ta, _] = s.cut.split_faces[1];
    // Many-to-one: two source faces, one target face.
    let e = discover_column_sets(&s.topo, &m, &[sa, sb], &[ta])
        .expect_err("many-to-one")
        .to_string();
    for id in [sa, sb, ta] {
        assert!(e.contains(&id.to_string()), "{e}");
    }
    assert!(e.contains("no face-to-face correspondence"), "{e}");
    // Equal sizes but the second target is not the image of the second source.
    let side = s.cut.split_faces[2][0];
    let e = discover_column_sets(&s.topo, &m, &[sa, sb], &[ta, side])
        .expect_err("no correspondence")
        .to_string();
    for id in [sa, sb, ta, side] {
        assert!(e.contains(&id.to_string()), "{e}");
    }
    assert!(e.contains("no face-to-face correspondence"), "{e}");
    // Through the driver: the parameter check says the same for sets of different sizes, and
    // the plan says it for a pair that has no correspondence.
    let mut it = intent(&s);
    it.set_scheme(v, sweep(&[sa, sb], &[ta]));
    match mesh_with(&registry(), &s.topo, &Model::new(), &[v], &it, &Sequential) {
        Err(SchemeError::BadParams { reason, .. }) => {
            assert!(reason.contains(&sb.to_string()), "{reason}");
            assert!(
                reason.contains("no face-to-face correspondence"),
                "{reason}"
            );
        }
        other => panic!("expected BadParams, got {:?}", other.map(|_| ())),
    }
    let mut it = intent(&s);
    it.set_scheme(v, sweep(&[sa, sb], &[ta, side]));
    match mesh_with(&registry(), &s.topo, &Model::new(), &[v], &it, &Sequential) {
        Err(SchemeError::NotApplicable { reason, .. }) => {
            assert!(reason.contains(&side.to_string()), "{reason}");
            assert!(
                reason.contains("no face-to-face correspondence"),
                "{reason}"
            );
        }
        other => panic!("expected NotApplicable, got {:?}", other.map(|_| ())),
    }
    // Only one of the two imprinted source faces: the other one becomes a linking face that is
    // not a strip, so the pair is refused at plan time.
    let mut it = intent(&s);
    it.set_scheme(v, sweep(&[sa], &[ta]));
    assert!(mesh_with(&registry(), &s.topo, &Model::new(), &[v], &it, &Sequential).is_err());
}

#[test]
fn a_bare_id_and_a_list_are_the_same_params() {
    let s = slab(false);
    let f = s.brick.faces[0];
    let g = s.brick.faces[1];
    let a = sm_scheme_api::Params(serde_json::json!({ "source": f, "target": g }));
    let b = sm_scheme_api::Params(serde_json::json!({ "source": [f], "target": [g] }));
    let (pa, pb) = (
        sm_scheme_sweep::SweepParams::from_params(&a).expect("bare"),
        sm_scheme_sweep::SweepParams::from_params(&b).expect("list"),
    );
    assert_eq!(pa, pb);
    assert_eq!(
        serde_json::to_value(&pb).expect("json")["source"],
        serde_json::json!([f])
    );
    let none = sm_scheme_api::Params(serde_json::json!({ "source": [], "target": [] }));
    assert!(sm_scheme_sweep::SweepParams::from_params(&none).is_err());
}

#[test]
fn set_sweep_is_deterministic() {
    let one = slab(true);
    let (src, dst) = (one.cut.split_faces[0], one.cut.split_faces[1]);
    let mut it = intent(&one);
    it.set_scheme(one.cut.volumes[0], sweep(&src, &dst));
    let run = |exec: &dyn sm_scheme_api::Executor| {
        let r = mesh_with(
            &registry(),
            &one.topo,
            &Model::new(),
            &[one.cut.volumes[0]],
            &it,
            exec,
        )
        .expect("mesh");
        let m = &r.finished.mesh;
        (coords(m), centroids(m))
    };
    let a = run(&Sequential);
    let b = run(&sm_scheme_api::ScopedThreads(4));
    assert_eq!(a, b);
}
