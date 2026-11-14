//! W4-CSG-PROBE: what the kernel cuts today (draft csgbrep.md WEBCUT-PROBE items 1-4).
//!
//! Every case runs through the ops on a fresh `Rig`, compares volumes with analytic values and
//! records `pass`, a typed error or a panic in a report table (printed with `--nocapture`). The
//! test pins the measured outcome of each case, so a kernel change that fixes or breaks one shows
//! up here. Item 4 decides the D2 partition (webcut + merge + unite) or the per-cell boolean
//! fallback.

mod common;

use common::rig::*;
use common::*;
use core::f64::consts::PI;
use sm_boolean::ops::{
    AxisPlane, Intersect, Merge, PlaneAxis, PlaneSpec, PointNormalPlane, Subtract, Unite,
    WebcutCylinder, WebcutPlane,
};
use sm_boolean::{CutTool, KeepSide, WebcutOptions, webcut};
use sm_core::{EntityId, Point3};
use sm_geom_api::{Frame, ImplicitDef, Quadric, SurfaceDef};
use sm_model::{CreateFrustum, CreateTorus};
use sm_op_api::Delta;
use sm_topo_api::Dim;

const TOL: f64 = 1e-9;

type Out = Result<String, String>;

fn vols(r: &Rig, d: &Delta, role: &str) -> Vec<f64> {
    let t = r.topo();
    let of = |x: EntityId| -> f64 {
        if t.dim_of(x) == Some(Dim::Body) {
            t.boundary(x).iter().map(|e| volume_of(t, e.cell)).sum()
        } else {
            volume_of(t, x)
        }
    };
    d.outputs
        .get(role)
        .map(|v| v.iter().map(|&x| of(x)).collect())
        .unwrap_or_default()
}

fn close(what: &str, got: f64, want: f64) -> Result<(), String> {
    if (got - want).abs() <= TOL * want.abs().max(1.0) {
        Ok(())
    } else {
        Err(format!("{what}: got {got:.12}, want {want:.12}"))
    }
}

fn valid(r: &Rig) -> Result<(), String> {
    let t = r.topo();
    let i = sm_topo_api::validate(t);
    let g = sm_topo_api::validate_geometry(t);
    if i.is_empty() && g.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "invalid: {} topo, {} geometry issues",
            i.len(),
            g.len()
        ))
    }
}

/// `keep` discards the other side: its role is empty and exactly one volume is left in the model.
fn only_kept(r: &Rig, discarded: &[f64]) -> Result<(), String> {
    let n = r.topo().cells(Dim::Volume).len();
    if discarded.is_empty() && n == 1 {
        Ok(())
    } else {
        Err(format!(
            "discarded side holds {} volume(s), model has {n} volume(s), want 0 and 1",
            discarded.len()
        ))
    }
}

fn body_of(d: &Delta, role: &str) -> EntityId {
    d.outputs[role][0]
}

fn brick(r: &mut Rig, size: [f64; 3], c: [f64; 3]) -> EntityId {
    r.brick(size, c).body
}

fn run<T: sm_op_api::Op>(r: &mut Rig, op: &T) -> Result<Delta, String> {
    r.run(op).map_err(|e| format!("{e}"))
}

fn merge_all(r: &mut Rig) -> Result<Delta, String> {
    run(
        r,
        &Merge {
            entities: None,
            kinds: None,
            tol: None,
            check_only: None,
        },
    )
}

fn cut_y(r: &mut Rig, a: EntityId, y: f64) -> Result<Delta, String> {
    run(
        r,
        &WebcutPlane {
            targets: vec![a],
            plane: PlaneSpec::Axis(AxisPlane {
                axis: PlaneAxis::Y,
                offset: y,
            }),
            keep: None,
            merge: None,
            imprint: None,
        },
    )
}

fn faces_at_x(r: &Rig, v: EntityId, x: f64) -> usize {
    let t = r.topo();
    t.volume_faces(v)
        .iter()
        .filter(|&&f| {
            t.face_moments(f)
                .unwrap()
                .centroid()
                .is_some_and(|c| (c.x - x).abs() < 1e-9)
        })
        .count()
}

// ---- item 1: webcut by a tilted offset cylinder and a non-axis plane ------------------------

fn tilted_cylinder(keep: KeepSide) -> Out {
    let mut r = Rig::cc();
    let b = brick(&mut r, [10.0, 10.0, 4.0], [0.0, 0.0, 0.0]);
    let (th, rad) = (30f64.to_radians(), 1.0);
    let op = WebcutCylinder {
        targets: vec![b],
        radius: rad,
        axis: [th.sin(), 0.0, th.cos()],
        center: [0.3, -0.2, 0.0],
        keep: Some(keep),
        merge: None,
        imprint: None,
    };
    let d = run(&mut r, &op)?;
    valid(&r)?;
    let inside = PI * rad * rad * 4.0 / th.cos(); // axis crosses the two z faces: pi r^2 L
    let (neg, pos) = (vols(&r, &d, "negative"), vols(&r, &d, "positive"));
    match keep {
        KeepSide::Both => {
            close("inside", neg.iter().sum(), inside)?;
            close("outside", pos.iter().sum(), 400.0 - inside)?;
        }
        KeepSide::Negative => {
            close("inside", neg.iter().sum(), inside)?;
            only_kept(&r, &pos)?;
        }
        KeepSide::Positive => {
            close("outside", pos.iter().sum(), 400.0 - inside)?;
            only_kept(&r, &neg)?;
        }
    }
    Ok(format!("inside {inside:.6}"))
}

fn plane_cut(
    name: &str,
    size: f64,
    normal: [f64; 3],
    point: [f64; 3],
    keep: KeepSide,
    neg_vol: f64,
) -> Out {
    let mut r = Rig::cc();
    let b = brick(&mut r, [size; 3], [size / 2.0; 3]);
    let op = WebcutPlane {
        targets: vec![b],
        plane: PlaneSpec::PointNormal(PointNormalPlane {
            origin: point,
            normal,
        }),
        keep: Some(keep),
        merge: None,
        imprint: None,
    };
    let d = run(&mut r, &op)?;
    valid(&r)?;
    let total = size.powi(3);
    let (neg, pos) = (vols(&r, &d, "negative"), vols(&r, &d, "positive"));
    match keep {
        KeepSide::Both => {
            close(&format!("{name} neg"), neg.iter().sum(), neg_vol)?;
            close(&format!("{name} pos"), pos.iter().sum(), total - neg_vol)?;
        }
        KeepSide::Negative => {
            close(&format!("{name} neg"), neg.iter().sum(), neg_vol)?;
            only_kept(&r, &pos)?;
        }
        KeepSide::Positive => {
            close(&format!("{name} pos"), pos.iter().sum(), total - neg_vol)?;
            only_kept(&r, &neg)?;
        }
    }
    Ok(format!("neg {neg_vol:.6}"))
}

// ---- item 2: primitives against a brick ----------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum Prim {
    Sphere,
    Frustum,
    Torus,
}

#[derive(Clone, Copy, Debug)]
enum Place {
    Inside,
    Crossing,
}

#[derive(Clone, Copy, Debug)]
enum Rule {
    Subtract,
    Intersect,
}

/// Brick `6 x 6 x 6` over `[-3, 3]^2 x [z0, z0 + 6]`; primitive centred at the origin. `Inside`
/// puts the brick at `z0 = -3` (the primitive is interior), `Crossing` at `z0 = 0` (the plane
/// `z = 0` cuts the primitive in two equal halves by symmetry for sphere and torus; the frustum
/// keeps its upper part).
fn prim_case(prim: Prim, place: Place, rule: Rule) -> Out {
    let mut r = Rig::cc();
    let z0 = match place {
        Place::Inside => -3.0,
        Place::Crossing => 0.0,
    };
    let b = brick(&mut r, [6.0; 3], [0.0, 0.0, z0 + 3.0]);
    let (p, full, half) = match prim {
        Prim::Sphere => (
            r.sphere(1.5, [0.0; 3]).body,
            4.0 / 3.0 * PI * 1.5f64.powi(3),
            2.0 / 3.0 * PI * 1.5f64.powi(3),
        ),
        Prim::Frustum => {
            // Bottom radius 2, top 1, height 4, spans z in [-2, 2]; radius at z = 0 is 1.5.
            let d = run(
                &mut r,
                &CreateFrustum {
                    height: 4.0,
                    radius: 2.0,
                    minor: None,
                    top: Some(1.0),
                    center: Some([0.0; 3]),
                },
            )?;
            let f = |h: f64, a: f64, b: f64| PI * h * (a * a + a * b + b * b) / 3.0;
            (body_of(&d, "body"), f(4.0, 2.0, 1.0), f(2.0, 1.5, 1.0))
        }
        Prim::Torus => {
            let d = run(
                &mut r,
                &CreateTorus {
                    major: 2.0,
                    minor: 0.5,
                    center: Some([0.0; 3]),
                },
            )?;
            (
                body_of(&d, "body"),
                2.0 * PI * PI * 2.0 * 0.25,
                PI * PI * 2.0 * 0.25,
            )
        }
    };
    let part = match place {
        Place::Inside => full,
        Place::Crossing => half,
    };
    let d = match rule {
        Rule::Subtract => run(
            &mut r,
            &Subtract {
                targets: vec![b],
                tools: vec![p],
                keep: None,
                regularize: None,
                tol: None,
            },
        )?,
        Rule::Intersect => run(
            &mut r,
            &Intersect {
                bodies: vec![b, p],
                keep: None,
                regularize: None,
                tol: None,
            },
        )?,
    };
    valid(&r)?;
    let v = vols(&r, &d, "volumes");
    let want = match rule {
        Rule::Subtract => 216.0 - part,
        Rule::Intersect => part,
    };
    if v.len() != 1 {
        return Err(format!("{} volumes, want 1", v.len()));
    }
    close("volume", v[0], want)?;
    Ok(format!("vol {want:.6}"))
}

// ---- item 3: Implicit quadric --------------------------------------------------------------

fn implicit_webcut() -> Out {
    let mut r = Rig::cc();
    let b = brick(&mut r, [4.0; 3], [0.0; 3]);
    let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0);
    let tool = CutTool::Surface {
        surface: SurfaceDef::Implicit(ImplicitDef::Quadric(q)),
        window: None,
    };
    r.lib(|h| webcut(h, &[b], &tool, &WebcutOptions::default()))
        .map_err(|e| e.to_string())?;
    Ok("cut".into())
}

// ---- item 4: what D2 relies on -------------------------------------------------------------

/// Two unit bricks A (x in [0,1]) and B (x in [1,2]) merged; webcut A only with `y = 0.5`.
fn d2_cut_one_piece() -> Out {
    let mut r = Rig::cc();
    let a = brick(&mut r, [1.0; 3], [0.5, 0.5, 0.5]);
    let b = r.brick([1.0; 3], [1.5, 0.5, 0.5]);
    merge_all(&mut r)?;
    let before = r.topo().volume_faces(b.volume).len();
    let d = cut_y(&mut r, a, 0.5)?;
    valid(&r)?;
    let t = r.topo();
    let (neg, pos) = (vols(&r, &d, "negative"), vols(&r, &d, "positive"));
    close("A lower", neg.iter().sum(), 0.5)?;
    close("A upper", pos.iter().sum(), 0.5)?;
    close("B", volume_of(t, b.volume), 1.0)?;
    // The neighbour keeps its own body, and that body still bounds it.
    if !t.boundary(b.body).iter().any(|e| e.cell == b.volume) {
        return Err("B's body no longer bounds B's volume".into());
    }
    if t.cells(Dim::Volume).len() != 3 {
        return Err(format!("{} volumes, want 3", t.cells(Dim::Volume).len()));
    }
    let after = t.volume_faces(b.volume).len();
    let shared = faces_at_x(&r, b.volume, 1.0);
    if shared != 2 {
        return Err(format!(
            "B has {shared} faces on x = 1 (B faces {before} -> {after}), want 2 split halves"
        ));
    }
    Ok(format!(
        "B faces {before} -> {after}, B volume 1, shared face split in {shared}"
    ))
}

/// A (x in [0,2]) and B (x in [2,4]) merged; A cut by a cylinder along x through its centre:
/// the circle lands on the shared face.
fn d2_cut_one_piece_cylinder() -> Out {
    let mut r = Rig::cc();
    let a = brick(&mut r, [2.0; 3], [1.0, 1.0, 1.0]);
    let b = r.brick([2.0; 3], [3.0, 1.0, 1.0]);
    merge_all(&mut r)?;
    let d = run(
        &mut r,
        &WebcutCylinder {
            targets: vec![a],
            radius: 0.5,
            axis: [1.0, 0.0, 0.0],
            center: [0.0, 1.0, 1.0],
            keep: None,
            merge: None,
            imprint: None,
        },
    )?;
    valid(&r)?;
    let t = r.topo();
    let (neg, pos) = (vols(&r, &d, "negative"), vols(&r, &d, "positive"));
    close("inside", neg.iter().sum(), PI * 0.25 * 2.0)?;
    close("outside", pos.iter().sum(), 8.0 - PI * 0.25 * 2.0)?;
    close("B", volume_of(t, b.volume), 8.0)?;
    let on_x2 = faces_at_x(&r, b.volume, 2.0);
    if on_x2 != 2 {
        return Err(format!("B has {on_x2} faces on x = 2, want disc + annulus"));
    }
    Ok(format!("B faces on x=2: {on_x2}"))
}

fn count_faces(r: &Rig) -> usize {
    r.topo().cells(Dim::Face).len()
}

/// `unite` of the two pieces of a webcut (they share the merged cut face).
fn d2_unite_pieces() -> Out {
    let mut r = Rig::cc();
    let a = brick(&mut r, [1.0; 3], [0.5, 0.5, 0.5]);
    let d = cut_y(&mut r, a, 0.5)?;
    let (n, p) = (d.outputs["negative"][0], d.outputs["positive"][0]);
    let before = count_faces(&r);
    let u = run(
        &mut r,
        &Unite {
            bodies: vec![n, p],
            keep: None,
            regularize: None,
            tol: None,
        },
    )?;
    valid(&r)?;
    let v = vols(&r, &u, "volumes");
    if v.len() != 1 {
        return Err(format!("{} volumes, want 1", v.len()));
    }
    close("united", v[0], 1.0)?;
    let after = count_faces(&r);
    if after != 6 {
        return Err(format!(
            "faces {before} -> {after}, want 6 (shared face removed)"
        ));
    }
    Ok(format!("faces {before} -> {after}"))
}

/// `unite` of two merged neighbouring bricks removes their shared face.
fn d2_unite_merged_neighbours() -> Out {
    let mut r = Rig::cc();
    let a = brick(&mut r, [1.0; 3], [0.5, 0.5, 0.5]);
    let b = brick(&mut r, [1.0; 3], [1.5, 0.5, 0.5]);
    merge_all(&mut r)?;
    let before = count_faces(&r);
    let u = run(
        &mut r,
        &Unite {
            bodies: vec![a, b],
            keep: None,
            regularize: None,
            tol: None,
        },
    )?;
    valid(&r)?;
    let v = vols(&r, &u, "volumes");
    if v.len() != 1 {
        return Err(format!("{} volumes, want 1", v.len()));
    }
    close("united", v[0], 2.0)?;
    let after = count_faces(&r);
    // Unite also merges the coplanar faces, so the result is a plain 2 x 1 x 1 brick.
    if after != 6 {
        return Err(format!(
            "faces {before} -> {after}, want 6 (shared face removed, coplanar faces merged)"
        ));
    }
    Ok(format!("faces {before} -> {after}"))
}

/// The D2 chain: cut one piece of a merged pair, then unite the cut pieces back; the neighbour
/// must stay valid and keep its volume.
fn d2_chain() -> Out {
    let mut r = Rig::cc();
    let a = brick(&mut r, [1.0; 3], [0.5, 0.5, 0.5]);
    let b = r.brick([1.0; 3], [1.5, 0.5, 0.5]);
    merge_all(&mut r)?;
    let d = cut_y(&mut r, a, 0.5)?;
    let (n, p) = (d.outputs["negative"][0], d.outputs["positive"][0]);
    let u = run(
        &mut r,
        &Unite {
            bodies: vec![n, p],
            keep: None,
            regularize: None,
            tol: None,
        },
    )?;
    valid(&r)?;
    let v = vols(&r, &u, "volumes");
    close("A again", v.iter().sum(), 1.0)?;
    close("B", volume_of(r.topo(), b.volume), 1.0)?;
    let on_x1 = faces_at_x(&r, b.volume, 1.0);
    // Pinned: the neighbour's face on the cut plane stays one face, the model has 11 faces.
    if on_x1 != 1 || count_faces(&r) != 11 {
        return Err(format!(
            "{} faces total, B has {on_x1} on x = 1, want 11 and 1",
            count_faces(&r)
        ));
    }
    Ok(format!(
        "{} faces total, B has {on_x1} on x = 1",
        count_faces(&r)
    ))
}

// ---- driver --------------------------------------------------------------------------------

fn guarded(f: impl FnOnce() -> Out) -> String {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(Ok(s)) => format!("pass  {s}"),
        Ok(Err(e)) => format!("FAIL  {e}"),
        Err(p) => format!(
            "PANIC {}",
            p.downcast_ref::<String>()
                .cloned()
                .or_else(|| p.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                .unwrap_or_default()
        ),
    }
}

type Case = (String, Box<dyn FnOnce() -> Out>);

fn cases() -> Vec<Case> {
    let mut v: Vec<Case> = Vec::new();
    for k in [KeepSide::Both, KeepSide::Negative, KeepSide::Positive] {
        v.push((
            format!("1a webcut tilted offset cylinder keep={k:?}"),
            Box::new(move || tilted_cylinder(k)),
        ));
    }
    for k in [KeepSide::Both, KeepSide::Negative, KeepSide::Positive] {
        // Corner tetrahedron x + y + z = 1 on a 2-cube: negative side 1/6.
        v.push((
            format!("1b plane x+y+z=1 corner keep={k:?}"),
            Box::new(move || {
                plane_cut(
                    "corner",
                    2.0,
                    [1.0, 1.0, 1.0],
                    [1.0, 0.0, 0.0],
                    k,
                    1.0 / 6.0,
                )
            }),
        ));
        // Vertical plane x + y = 1: a triangular prism 0.5 x 2 = 1.
        v.push((
            format!("1b plane x+y=1 prism keep={k:?}"),
            Box::new(move || plane_cut("prism", 2.0, [1.0, 1.0, 0.0], [1.0, 0.0, 0.0], k, 1.0)),
        ));
        // Generic normal through the cube centre: halves by symmetry.
        v.push((
            format!("1b plane (1,2,3) through centre keep={k:?}"),
            Box::new(move || plane_cut("half", 2.0, [1.0, 2.0, 3.0], [1.0, 1.0, 1.0], k, 4.0)),
        ));
    }
    for prim in [Prim::Sphere, Prim::Frustum, Prim::Torus] {
        for place in [Place::Inside, Place::Crossing] {
            for rule in [Rule::Subtract, Rule::Intersect] {
                v.push((
                    format!("2 brick {rule:?} {prim:?} {place:?}"),
                    Box::new(move || prim_case(prim, place, rule)),
                ));
            }
        }
    }
    v.push((
        "3 webcut with an Implicit quadric tool".into(),
        Box::new(implicit_webcut),
    ));
    v.push((
        "4a webcut one piece, merged neighbour (plane)".into(),
        Box::new(d2_cut_one_piece),
    ));
    v.push((
        "4a webcut one piece, merged neighbour (cylinder)".into(),
        Box::new(d2_cut_one_piece_cylinder),
    ));
    v.push((
        "4b unite two pieces sharing a cut face".into(),
        Box::new(d2_unite_pieces),
    ));
    v.push((
        "4b unite two merged neighbours".into(),
        Box::new(d2_unite_merged_neighbours),
    ));
    v.push((
        "4 chain: cut one piece then unite its pieces".into(),
        Box::new(d2_chain),
    ));
    v
}

#[test]
fn webcut_probe_report() {
    let mut report = Vec::new();
    for (name, f) in cases() {
        report.push(format!("{name:<58} {}", guarded(f)));
    }
    println!("\nWEBCUT-PROBE report\n{}", report.join("\n"));
    let text = report.join("\n");
    assert!(!text.contains("PANIC"), "a case panicked:\n{text}");
    // Pinned outcome: everything passes except the Implicit tool, a typed `Unsupported` today
    // (webcut closes a quadric tool into a solid; an implicit-only surface has no such form).
    for l in &report {
        if l.starts_with("3 ") {
            assert!(l.contains("FAIL  unsupported: webcut supports plane, cylinder, sphere, cone, torus and elliptic tools"), "{l}");
        } else {
            assert!(l.contains(" pass  "), "{l}");
        }
    }
}
