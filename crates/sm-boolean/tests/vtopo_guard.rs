//! W4-BOOL-TIDY (3): virtual-topology invariant audit. Every op of sm-boolean that edits host
//! geometry or a face loop while KEEPING the cell ids (regularize joins, merge and heal weld
//! survivors, heal orientation, webcut and imprint splits) is run on a model that carries a
//! `partition_face` chord or a composite on the affected face. The outcome is either a typed
//! refusal (state untouched) or an accepted edit after which the overlay view validates and the
//! partition children still tile the host face. Never a corrupt overlay.

mod common;

use common::rig::*;
use sm_boolean::ops::{
    AxisPlane, Heal, Imprint, Merge, PlaneAxis, PlaneSpec, Regularize, Unite, WebcutPlane,
};
use sm_core::{EntityId, Point3};
use sm_model::{CompositeFaces, PartitionEdge, PartitionFace};
use sm_topo_api::{Carrier, Dim, TopologyView, TopologyViewExt};
use std::collections::BTreeSet;

fn vertex_point(t: &dyn TopologyView, v: EntityId) -> Point3 {
    match t.carrier(v) {
        Carrier::Point(p) => *p,
        c => panic!("vertex carrier {c:?}"),
    }
}

fn face_vertices(t: &dyn TopologyView, f: EntityId) -> BTreeSet<EntityId> {
    let mut found = BTreeSet::new();
    for c in t.boundary(f) {
        for ec in t.boundary(c.cell) {
            found.insert(ec.cell);
        }
    }
    found
}

/// The first face with a vertex at each of `pts`, and those vertices.
fn face_at(r: &Rig, pts: &[[f64; 3]]) -> (EntityId, Vec<EntityId>) {
    let t = r.layer().host();
    for &f in t.cells(Dim::Face) {
        let vs = face_vertices(t, f);
        let pick: Vec<Option<EntityId>> = pts
            .iter()
            .map(|p| {
                let q = Point3::new(p[0], p[1], p[2]);
                vs.iter()
                    .copied()
                    .find(|&v| (vertex_point(t, v) - q).norm() < 1e-9)
            })
            .collect();
        if pick.iter().all(Option::is_some) {
            return (f, pick.into_iter().flatten().collect());
        }
    }
    panic!("no face with vertices at {pts:?}");
}

/// What an edit did to a model with virtual topology.
#[derive(Debug)]
enum Outcome {
    /// Refused with this message; the state hash is unchanged.
    Refused(String),
    /// Accepted; the issues of `validate` and `validate_geometry` on the overlay view, and the
    /// worst gap between the area of each partitioned host face and its children.
    Accepted { issues: usize, tiling_gap: f64 },
}

/// Run `op`, then judge the overlay view.
fn judge(r: &mut Rig, op: &dyn sm_op_api::Op) -> Outcome {
    let before = r.hash();
    match r.run(op) {
        Err(e) => {
            assert_eq!(r.hash(), before, "a refused op leaves the state as it was");
            Outcome::Refused(e.to_string())
        }
        Ok(_) => {
            let t = r.layer().topo();
            let issues = sm_topo_api::validate(t).len() + sm_topo_api::validate_geometry(t).len();
            let mut gap = 0.0f64;
            if let Some(ov) = r.layer().overlay() {
                for op in ov.ops() {
                    if let sm_topo_api::VirtualOp::PartitionFace { face, halves, .. } = op {
                        let host = r.layer().host();
                        let whole = host.face_moments(*face).map_or(0.0, |m| m.area);
                        let kids: f64 = halves
                            .iter()
                            .map(|h| t.face_moments(*h).map_or(f64::NAN, |m| m.area))
                            .sum();
                        gap = gap.max((whole - kids).abs());
                    }
                }
            }
            Outcome::Accepted {
                issues,
                tiling_gap: gap,
            }
        }
    }
}

fn is_sound(o: &Outcome) -> bool {
    match o {
        Outcome::Refused(m) => m.contains("virtual topology") || m.contains("partition"),
        Outcome::Accepted { issues, tiling_gap } => *issues == 0 && *tiling_gap < 1e-9,
    }
}

/// Partition the face that has vertices at `p` and `q` by the chord between them.
fn chord(r: &mut Rig, p: [f64; 3], q: [f64; 3]) -> EntityId {
    let (face, vs) = face_at(r, &[p, q]);
    r.run(&PartitionFace {
        face,
        start: vs[0],
        end: vs[1],
    })
    .expect("partition_face");
    face
}

fn bricks(gap: f64) -> Rig {
    let mut r = Rig::cc();
    r.brick([1.0; 3], [0.5; 3]);
    r.brick([1.0; 3], [1.5 + gap, 0.5, 0.5]);
    r
}

fn heal_op(tol: f64) -> Heal {
    Heal {
        entities: None,
        tol: Some(tol),
        fixes: None,
    }
}

/// Two bricks fused into an L (not regularized): the top faces are coplanar neighbours.
fn l_shape() -> Rig {
    let mut r = Rig::cc();
    r.brick([2.0, 1.0, 1.0], [1.0, 0.5, 0.5]);
    r.brick([1.0, 1.0, 1.0], [0.5, 1.5, 0.5]);
    let bodies = r.layer().topo().cells(Dim::Body).to_vec();
    r.run(&Unite {
        bodies,
        keep: None,
        regularize: Some(false),
        tol: None,
    })
    .expect("unite");
    r
}

fn all_volumes(r: &Rig) -> Vec<EntityId> {
    r.layer().host().cells(Dim::Volume).to_vec()
}

#[test]
fn audit_regularize_join_faces_chord() {
    let mut r = l_shape();
    chord(&mut r, [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]);
    let volumes = all_volumes(&r);
    let o = judge(&mut r, &Regularize { volumes });
    // The kept face grows over its neighbour: the chord would silently split a bigger face.
    assert!(
        matches!(&o, Outcome::Refused(m) if m.contains("carries virtual topology")),
        "regularize join_faces, chord on kept face: {o:?}"
    );
}

#[test]
fn audit_regularize_join_faces_composite() {
    let mut r = l_shape();
    let (a, _) = face_at(&r, &[[0.0, 0.0, 1.0], [2.0, 1.0, 1.0]]);
    let (b, _) = face_at(&r, &[[0.0, 2.0, 1.0], [1.0, 1.0, 1.0]]);
    r.run(&CompositeFaces { faces: vec![a, b] }).expect("composite");
    let volumes = all_volumes(&r);
    let o = judge(&mut r, &Regularize { volumes });
    assert!(is_sound(&o), "regularize join_faces, composite of both: {o:?}");
}

#[test]
fn audit_merge_and_heal_weld() {
    for gap in [0.0, 1e-4] {
        // Chord on the face that survives (brick A's top) and on the one that is dropped.
        for (tag, p, q) in [
            ("kept side", [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]),
            ("far side", [1.0 + gap, 0.0, 1.0], [2.0 + gap, 1.0, 1.0]),
        ] {
            let mut r = bricks(gap);
            chord(&mut r, p, q);
            let o = judge(
                &mut r,
                &Merge {
                    entities: None,
                    kinds: None,
                    tol: Some(2e-4),
                    check_only: None,
                },
            );
            assert!(is_sound(&o), "merge gap {gap}, chord {tag}: {o:?}");
            let mut r = bricks(gap);
            chord(&mut r, p, q);
            let o = judge(&mut r, &heal_op(2e-4));
            assert!(is_sound(&o), "heal gap {gap}, chord {tag}: {o:?}");
        }
    }
}

#[test]
fn audit_webcut_and_imprint() {
    for z in [0.5, 1.0] {
        let mut r = Rig::cc();
        r.brick([1.0; 3], [0.5; 3]);
        chord(&mut r, [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]);
        let body = r.layer().host().cells(Dim::Body)[0];
        let o = judge(
            &mut r,
            &WebcutPlane {
                targets: vec![body],
                plane: PlaneSpec::Axis(AxisPlane {
                    axis: PlaneAxis::Z,
                    offset: z,
                }),
                keep: None,
                merge: None,
                imprint: None,
            },
        );
        assert!(is_sound(&o), "webcut z={z}, chord on the top: {o:?}");
    }
    let mut r = Rig::cc();
    r.brick([1.0; 3], [0.5; 3]);
    r.brick([1.0; 3], [1.0, 0.5, 0.5]);
    chord(&mut r, [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]);
    let bodies = r.layer().host().cells(Dim::Body).to_vec();
    let o = judge(
        &mut r,
        &Imprint {
            bodies,
            merge: None,
            tol: None,
        },
    );
    assert!(is_sound(&o), "imprint of overlapping bricks, chord on a top: {o:?}");
}

/// A brick cut at z = 0.5 and fused again without regularizing: every side face and vertical
/// edge is in two pieces.
fn cut_brick() -> Rig {
    let mut r = Rig::cc();
    r.brick([1.0; 3], [0.5; 3]);
    let body = r.layer().host().cells(Dim::Body)[0];
    r.run(&WebcutPlane {
        targets: vec![body],
        plane: PlaneSpec::Axis(AxisPlane {
            axis: PlaneAxis::Z,
            offset: 0.5,
        }),
        keep: None,
        merge: None,
        imprint: None,
    })
    .expect("webcut");
    let bodies = r.layer().topo().cells(Dim::Body).to_vec();
    r.run(&Unite {
        bodies,
        keep: None,
        regularize: Some(false),
        tol: None,
    })
    .expect("unite");
    r
}

fn vertical_edge(r: &Rig, z0: f64, z1: f64) -> EntityId {
    let t = r.layer().host();
    *t.cells(Dim::Edge)
        .iter()
        .find(|&&e| {
            let Some((a, b)) = t.edge_ends(e) else {
                return false;
            };
            let (pa, pb) = (vertex_point(t, a), vertex_point(t, b));
            pa.x.abs() < 1e-9
                && pa.y.abs() < 1e-9
                && pb.x.abs() < 1e-9
                && pb.y.abs() < 1e-9
                && ((pa.z - z0).abs() < 1e-9 && (pb.z - z1).abs() < 1e-9
                    || (pa.z - z1).abs() < 1e-9 && (pb.z - z0).abs() < 1e-9)
        })
        .expect("vertical edge")
}

#[test]
fn audit_regularize_join_edges() {
    for (tag, z0, z1, param) in [("lower half", 0.0, 0.5, 0.25), ("upper half", 0.5, 1.0, 0.75)] {
        let mut r = cut_brick();
        let e = vertical_edge(&r, z0, z1);
        r.run(&PartitionEdge { edge: e, param })
            .unwrap_or_else(|e| panic!("partition_edge {tag}: {e}"));
        let volumes = all_volumes(&r);
        let o = judge(&mut r, &Regularize { volumes });
        assert!(
            matches!(&o, Outcome::Refused(m) if m.contains("virtual topology")),
            "regularize join_edges, partition_edge on the {tag}: {o:?}"
        );
    }
}

/// Heal's orientation fix reverses faces and re-adds the uses with the face ids kept. At the
/// library level (the overlay is rebased by hand, as `topo_txn` does): a chord on a face of an
/// inside-out brick survives the fix, and the rebased view validates.
#[test]
fn audit_heal_orient_with_a_chord() {
    use sm_boolean::{HealApply, HealFixes, heal};
    use sm_geom_api::{CurveDef, Interval, Line};
    use sm_topo_api::fixtures::brick;
    use sm_topo_api::{CellComplex, OpId, Overlay, Side, TopologyEdit, VirtualCut};
    use std::sync::Arc;

    let mut h = CellComplex::new();
    h.begin_op(OpId(1)).unwrap();
    let b = brick(&mut h, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).unwrap();
    h.end_op().unwrap();
    h.begin_op(OpId(2)).unwrap();
    for &f in &b.faces {
        h.remove_face_use(b.volume, f, Side::Left).unwrap();
    }
    for &f in &b.faces {
        h.reverse_face(f).unwrap();
    }
    for &f in &b.faces {
        let u = sm_topo_api::FaceUseSpec {
            face: f,
            side: Side::Left,
        };
        h.add_face_use(b.volume, 0, u).unwrap();
    }
    h.end_op().unwrap();
    // Chord on the top face between two opposite corners.
    let top = *h
        .cells(Dim::Face)
        .iter()
        .find(|&&f| {
            face_vertices(&h, f)
                .iter()
                .all(|&v| (vertex_point(&h, v).z - 1.0).abs() < 1e-9)
        })
        .expect("top face");
    let vs: Vec<EntityId> = face_vertices(&h, top).into_iter().collect();
    let far = |a: EntityId, b: EntityId| (vertex_point(&h, a) - vertex_point(&h, b)).norm() > 1.2;
    let (from, to) = if far(vs[0], vs[1]) {
        (vs[0], vs[1])
    } else if far(vs[0], vs[2]) {
        (vs[0], vs[2])
    } else {
        (vs[0], vs[3])
    };
    let (pf, pt) = (vertex_point(&h, from), vertex_point(&h, to));
    let line = Line::new(pf, pt - pf).unwrap();
    let mut ov = Overlay::new(Arc::new(h.clone()));
    let cut = VirtualCut {
        from,
        to,
        curve: CurveDef::Line(line),
        range: Interval::new(0.0, (pt - pf).norm()),
    };
    ov.partition_face(top, cut).expect("virtual chord");
    let mut edited = h.clone();
    edited.begin_op(OpId(3)).unwrap();
    let apply = HealApply {
        entities: None,
        tol: None,
        fixes: HealFixes::default(),
    };
    let out = heal(&mut edited, &apply).unwrap();
    edited.end_op().unwrap();
    assert_eq!(out.reoriented, vec![b.volume]);
    let rebased = ov.rebase(Arc::new(edited));
    rebased.expect("the chord survives the orientation fix");
    let issues = sm_topo_api::validate(&ov).len() + sm_topo_api::validate_geometry(&ov).len();
    assert_eq!(issues, 0);
}

/// Control: a chord on a face no join touches does not stop `model.regularize` elsewhere.
#[test]
fn regularize_elsewhere_keeps_an_unrelated_chord() {
    let mut r = l_shape();
    r.brick([1.0; 3], [20.5, 0.5, 0.5]);
    chord(&mut r, [20.0, 0.0, 1.0], [21.0, 1.0, 1.0]);
    let volumes = all_volumes(&r);
    let o = judge(&mut r, &Regularize { volumes });
    assert!(
        matches!(o, Outcome::Accepted { issues: 0, tiling_gap } if tiling_gap < 1e-9),
        "{o:?}"
    );
}

/// The default regularize of a boolean joins coplanar faces too: a chord on a face the unite
/// joins is refused (typed, state untouched), as `model.regularize` does.
#[test]
fn unite_default_regularize_refuses_a_joined_chord_face() {
    for reg in [None, Some(true)] {
        let mut r = Rig::cc();
        r.brick([2.0, 1.0, 1.0], [1.0, 0.5, 0.5]);
        r.brick([1.0, 1.0, 1.0], [0.5, 1.5, 0.5]);
        chord(&mut r, [0.0, 0.0, 1.0], [2.0, 1.0, 1.0]);
        let bodies = r.layer().topo().cells(Dim::Body).to_vec();
        let o = judge(
            &mut r,
            &Unite {
                bodies: bodies.clone(),
                keep: None,
                regularize: reg,
                tol: None,
            },
        );
        assert!(
            matches!(&o, Outcome::Refused(m) if m.contains("carries virtual topology")),
            "unite regularize={reg:?}: {o:?}"
        );
        // Escape hatch: regularize false.
        let o = judge(
            &mut r,
            &Unite {
                bodies,
                keep: None,
                regularize: Some(false),
                tol: None,
            },
        );
        assert!(is_sound(&o), "unite regularize=false: {o:?}");
    }
}

/// Control: a chord on a face the unite does not join does not stop it.
#[test]
fn unite_default_regularize_elsewhere_is_accepted() {
    let mut r = Rig::cc();
    r.brick([2.0, 1.0, 1.0], [1.0, 0.5, 0.5]);
    r.brick([1.0, 1.0, 1.0], [0.5, 1.5, 0.5]);
    r.brick([1.0; 3], [20.5, 0.5, 0.5]);
    chord(&mut r, [20.0, 0.0, 1.0], [21.0, 1.0, 1.0]);
    let all = r.layer().topo().cells(Dim::Body).to_vec();
    let two: Vec<EntityId> = all[..2].to_vec();
    let o = judge(
        &mut r,
        &Unite {
            bodies: two,
            keep: None,
            regularize: None,
            tol: None,
        },
    );
    assert!(
        matches!(o, Outcome::Accepted { issues: 0, tiling_gap } if tiling_gap < 1e-9),
        "{o:?}"
    );
}
