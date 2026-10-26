//! Acceptance 9: lost-particle audit over mixed models (debug: 2e5 histories; the full 1e7
//! run is `#[ignore]`, run with `cargo test -p sm-rt --release -- --ignored`).
mod common;
use common::*;
use rayon::prelude::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundaryAction, BoundaryDef, BoundarySelector, BoundarySpec, BuildOptions, CellId, FaceAction, RecoveryPolicy, ResponseKind, Scratch, StepEnd, Tracker};
use sm_testkit::meshes::{mixed_transition, nonconvex_poly};

struct Model {
    name: &'static str,
    tracker: sm_rt::MeshTracker,
    lo: [f64; 3],
    hi: [f64; 3],
}

fn face(n: u64) -> sm_core::EntityId {
    ent(sm_core::EntityKind::TopoFace, n)
}

fn models(opts: &BuildOptions) -> Vec<Model> {
    let mut v = Vec::new();
    let mut add = |name, t: sm_rt::MeshTracker, m: Option<&dyn MeshRead>| {
        let (lo, hi) = match m {
            Some(m) => bbox(m),
            None => {
                let b = t.bounds();
                ([b.lo.x, b.lo.y, b.lo.z], [b.hi.x, b.hi.y, b.hi.z])
            }
        };
        v.push(Model { name, tracker: t, lo, hi });
    };
    let hb = hex_block([6; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // White boundaries: cosine-law re-emission stresses grazing directions.
    let white = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::White), vec![]);
    add("hex_block", build_mesh_tracker(&hb, &white, opts), Some(&hb));
    let refl = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Reflective), vec![]);
    let nc = nonconvex_poly().unwrap();
    add("nonconvex_poly", build_mesh_tracker(&nc.mesh, &refl, opts), Some(&nc.mesh));
    let mt = mixed_transition().unwrap();
    add("mixed_transition", build_mesh_tracker(&mt.mesh, &refl, opts), Some(&mt.mesh));
    let f = nested_facets();
    let spec = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Reflective), vec![]);
    let (ft, _) = MeshTrackerBuilder.build_concrete(&sm_rt_api::BuildInput::Facets(&f, &spec), opts).unwrap();
    add("icosphere_shells", ft, None);
    // Closed box of mirrors (decks applied in step).
    let mut mirrors = BoundarySpec::default();
    for k in 1..=6u64 {
        mirrors.rules.push((BoundarySelector::Entity(face(k)), BoundaryDef::Reflect));
    }
    add("reflective_box", build_mesh_tracker(&hb, &mirrors, opts), Some(&hb));
    v
}

#[derive(Default, Clone, Copy)]
struct Tally {
    histories: u64,
    steps: u64,
    lost: u64,
    nudged: u64,
}

fn run(models: &[Model], n: usize, steps: usize) -> Tally {
    let chunk = 1000usize;
    let parts: Vec<Tally> = (0..n.div_ceil(chunk))
        .into_par_iter()
        .map(|ci| {
            let mut r = SplitMix64(seed_of("acc09_audit") ^ sm_testkit::surf3::mix64(ci as u64 + 1));
            let mut sc = Scratch::new();
            let mut t = Tally::default();
            for h in 0..chunk.min(n - ci * chunk) {
                let m = &models[(ci * chunk + h) % models.len()];
                // Start: a random point of the model's box that locates inside a cell.
                let mut st = None;
                for _ in 0..200 {
                    let u = |k: usize, r: &mut SplitMix64| m.lo[k] + (m.hi[k] - m.lo[k]) * (r.unit() + 1.0) / 2.0;
                    let p = [u(0, &mut r), u(1, &mut r), u(2, &mut r)];
                    let (d, _, _) = random_frame(&mut r);
                    if let Ok(s) = m.tracker.init_state(p3(p), v3(d), None, &mut sc)
                        && !s.cell.is_none()
                        && s.cell != CellId::COMPLEMENT
                    {
                        st = Some(s);
                        break;
                    }
                }
                let Some(mut st) = st else { continue };
                t.histories += 1;
                for _ in 0..steps {
                    let s = m.tracker.step(&mut st, f64::INFINITY, &mut sc).unwrap();
                    t.steps += 1;
                    match s.end {
                        StepEnd::Lost(l) => {
                            t.lost += 1;
                            eprintln!("LOST in {}: {l:?}", m.name);
                            break;
                        }
                        StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } => {
                            let u = [(r.unit() + 1.0) / 2.0, (r.unit() + 1.0) / 2.0];
                            if m.tracker.resolve_boundary(&mut st, face, k, u).unwrap() == BoundaryAction::Terminate {
                                break;
                            }
                        }
                        StepEnd::RegionBoundary { to_cell, .. } if to_cell.is_none() || to_cell == CellId::COMPLEMENT => break,
                        _ => {}
                    }
                }
            }
            t
        })
        .collect();
    parts.iter().fold(Tally::default(), |a, b| Tally { histories: a.histories + b.histories, steps: a.steps + b.steps, lost: a.lost + b.lost, nudged: 0 })
}

fn audit(n: usize, steps: usize) {
    let fail = models(&opts_fail());
    let t = run(&fail, n, steps);
    eprintln!("audit: {} histories, {} steps, {} lost (Fail policy)", t.histories, t.steps, t.lost);
    assert_eq!(t.lost, 0, "lost particles under RecoveryPolicy::Fail");
    assert!(t.histories as usize >= n * 9 / 10);
    // With Nudge{3} the nudge count must stay 0 on this set.
    let mut o = BuildOptions::default();
    o.recovery = RecoveryPolicy::Nudge { tries: 3 };
    let nudge = models(&o);
    let t2 = run(&nudge, n.min(200_000), steps);
    let nudges: u64 = nudge.iter().map(|m| m.tracker.nudge_count()).sum();
    eprintln!("audit: Nudge{{3}} run: {} lost, {nudges} nudges", t2.lost);
    assert_eq!(t2.lost, 0);
    assert_eq!(nudges, 0, "nudge count must be 0");
}

#[test]
fn acc09_lost_particle_audit_debug_size() {
    audit(200_000, 60);
}

#[test]
#[ignore = "full 1e7-history gate: cargo test -p sm-rt --release -- --ignored"]
fn acc09_lost_particle_audit_full() {
    audit(10_000_000, 100);
}

/// Histories started within 3e-7 of the re-entrant edge of `nonconvex_poly` (where a fixed
/// tolerance, a reflex corner and reflecting walls meet): the case the first full audit found.
#[test]
fn acc09_reflex_edge_stress() {
    let nc = nonconvex_poly().unwrap();
    let spec = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Reflective), vec![]);
    let t = build_mesh_tracker(&nc.mesh, &spec, &opts_fail());
    let chunks = if cfg!(debug_assertions) { 20 } else { 1000 };
    let bad: usize = (0..chunks)
        .into_par_iter()
        .map(|ci| {
            let mut r = SplitMix64(seed_of("reflex_edge") ^ sm_testkit::surf3::mix64(ci as u64 + 1));
            let mut sc = Scratch::new();
            let mut bad = 0;
            for _ in 0..1000 {
                let z = (r.unit() + 1.0) / 2.0;
                let mut jitter = || 3e-7 * r.unit();
                let p = [1.0 + jitter(), 1.0 + jitter(), z];
                let (d, _, _) = random_frame(&mut r);
                let Ok(mut st) = t.init_state(p3(p), v3(d), None, &mut sc) else { continue };
                if st.cell.is_none() {
                    continue;
                }
                for _ in 0..60 {
                    match t.step(&mut st, f64::INFINITY, &mut sc).unwrap().end {
                        StepEnd::Lost(_) => {
                            bad += 1;
                            break;
                        }
                        StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } => {
                            t.resolve_boundary(&mut st, face, k, [0.5, 0.5]).unwrap();
                        }
                        _ => {}
                    }
                }
            }
            bad
        })
        .sum();
    assert_eq!(bad, 0, "lost particles near the re-entrant edge");
}
