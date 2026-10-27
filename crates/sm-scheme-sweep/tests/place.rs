//! Acceptance 10: `place_layers` on synthetic data (the Shepard path no fixture needs).

mod common;

use common::rng::Rng;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_scheme_api::NodeKey;
use sm_scheme_sweep::{Boundary, Interior, LayerMode, PlaceOptions, Placement, place_layers};

fn key(i: u32) -> NodeKey {
    NodeKey::new(EntityId::new(EntityKind::TopoEdge, 1).expect("id"), i)
}

/// A ring of boundary nodes of the square [0,4]^2 (spacing 1) and a 3x3 interior block.
fn square() -> (Vec<NodeKey>, Vec<Point3>, Vec<NodeKey>, Vec<Point3>) {
    let (mut bk, mut bp, mut ik, mut ip) = (vec![], vec![], vec![], vec![]);
    for j in 0..=4 {
        for i in 0..=4 {
            let p = Point3::new(f64::from(i), f64::from(j), 0.0);
            if i == 0 || j == 0 || i == 4 || j == 4 {
                bk.push(key(bk.len() as u32));
                bp.push(p);
            } else {
                ik.push(key(100 + ik.len() as u32));
                ip.push(p);
            }
        }
    }
    (bk, bp, ik, ip)
}

fn columns(bp: &[Point3], layers: usize, f: impl Fn(usize, Point3) -> Point3) -> Vec<Point3> {
    let mut out = Vec::new();
    for p in bp {
        for l in 0..=layers {
            out.push(if l == 0 { *p } else { f(l, *p) });
        }
    }
    out
}

fn run(
    bk: &[NodeKey],
    bp: &[Point3],
    cols: &[Point3],
    layers: usize,
    ik: &[NodeKey],
    ip: &[Point3],
    opts: PlaceOptions,
) -> Vec<Point3> {
    place_layers(
        &Boundary {
            keys: bk,
            src: bp,
            cols,
            layers,
        },
        &Interior { keys: ik, pos: ip },
        &opts,
    )
    .expect("place")
    .positions
}

fn rot(l: usize, p: Point3) -> Point3 {
    let g = Transform::rotation(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        0.2 * l as f64,
    )
    .expect("rz");
    g.apply_point(p) + Vector3::new(0.0, 0.0, l as f64)
}

#[test]
fn a_uniform_boundary_residual_shifts_every_interior_node_by_exactly_r() {
    let (bk, bp, ik, ip) = square();
    let layers = 4;
    let r = Vector3::new(0.013, -0.027, 0.0041);
    // Non-uniform displacement field (a rotation) plus a uniform residual r.
    let base = columns(&bp, layers, rot);
    let shifted = columns(&bp, layers, |l, p| rot(l, p) + r);
    for placement in [Placement::Affine, Placement::Interpolate, Placement::Auto] {
        for support in [0usize, 5, 16] {
            let opts = PlaceOptions {
                placement,
                shepard_power: 3.0,
                support,
            };
            let a = run(&bk, &bp, &base, layers, &ik, &ip, opts);
            let b = run(&bk, &bp, &shifted, layers, &ik, &ip, opts);
            assert_eq!(a.len(), ik.len() * (layers - 1));
            for (p, q) in a.iter().zip(&b) {
                assert!(
                    (*q - *p - r).norm() < 1e-12,
                    "{placement:?} support {support}: {p:?} {q:?}"
                );
            }
        }
    }
}

#[test]
fn an_interior_node_on_a_boundary_node_is_placed_on_its_column() {
    let (bk, bp, mut ik, mut ip) = square();
    // Put one interior node exactly on boundary node 3.
    ik.push(key(500));
    ip.push(bp[3]);
    let layers = 3;
    let cols = columns(&bp, layers, |l, p| {
        // Non-affine: a bump that depends on the node.
        rot(l, p) + Vector3::new(0.0, 0.0, 0.05 * p.x * p.y)
    });
    for placement in [Placement::Affine, Placement::Interpolate] {
        let out = run(
            &bk,
            &bp,
            &cols,
            layers,
            &ik,
            &ip,
            PlaceOptions {
                placement,
                shepard_power: 3.0,
                support: 8,
            },
        );
        let last = ik.len() - 1;
        for l in 1..layers {
            let want = cols[3 * (layers + 1) + l];
            let got = out[last * (layers - 1) + l - 1];
            assert!((got - want).norm() < 1e-12, "{placement:?} layer {l}");
        }
    }
}

#[test]
fn interpolate_and_affine_agree_when_the_residual_is_zero() {
    let (bk, bp, ik, ip) = square();
    let layers = 5;
    // Pure translation per layer: no residual of either kind.
    let cols = columns(&bp, layers, |l, p| {
        p + Vector3::new(0.1 * l as f64, 0.0, l as f64)
    });
    let mk = |placement| PlaceOptions {
        placement,
        shepard_power: 3.0,
        support: 16,
    };
    let a = run(&bk, &bp, &cols, layers, &ik, &ip, mk(Placement::Affine));
    let i = run(
        &bk,
        &bp,
        &cols,
        layers,
        &ik,
        &ip,
        mk(Placement::Interpolate),
    );
    for (p, q) in a.iter().zip(&i) {
        assert!((*p - *q).norm() < 1e-12);
    }
    // The translation fast path: x + d (bit-exact for lattice displacements, see the brick tests).
    let placed = place_layers(
        &Boundary {
            keys: &bk,
            src: &bp,
            cols: &cols,
            layers,
        },
        &Interior {
            keys: &ik,
            pos: &ip,
        },
        &mk(Placement::Auto),
    )
    .expect("place");
    assert!(placed.modes.iter().all(|m| *m == LayerMode::Translation));
    for (n, x) in ip.iter().enumerate() {
        for l in 1..layers {
            let d = Vector3::new(0.1 * l as f64, 0.0, l as f64);
            assert!((placed.positions[n * (layers - 1) + l - 1] - (*x + d)).norm() < 1e-14);
        }
    }
}

#[test]
fn a_rigid_motion_is_reproduced_by_the_affine_part_alone() {
    let (bk, bp, ik, ip) = square();
    let layers = 4;
    let cols = columns(&bp, layers, rot);
    let out = run(
        &bk,
        &bp,
        &cols,
        layers,
        &ik,
        &ip,
        PlaceOptions {
            placement: Placement::Auto,
            shepard_power: 3.0,
            support: 16,
        },
    );
    for (n, x) in ip.iter().enumerate() {
        for l in 1..layers {
            let want = rot(l, *x);
            assert!((out[n * (layers - 1) + l - 1] - want).norm() < 1e-12);
        }
    }
}

/// Four boundary nodes equidistant from the interior node at the origin: the support picks
/// the two smallest keys, whatever the input order.
#[test]
fn ties_are_broken_by_node_key_and_input_order_does_not_matter() {
    let pts = [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(5.0, 5.0, 0.0),
        Point3::new(-5.0, 5.0, 0.0),
        Point3::new(5.0, -5.0, 0.0),
        Point3::new(-5.0, -5.0, 0.0),
    ];
    let keys: Vec<NodeKey> = (0..pts.len() as u32).map(key).collect();
    let delta = |j: usize| {
        Vector3::new(
            0.1 * (j + 1) as f64,
            -0.03 * (j * j) as f64,
            0.5 + 0.01 * j as f64,
        )
    };
    let layers = 2;
    let cols = columns(&pts, layers, |_, p| p);
    // Per-node displacements (non-uniform, so no fast path).
    let mut cols = cols;
    for (j, p) in pts.iter().enumerate() {
        cols[j * (layers + 1) + 1] = *p + delta(j);
        cols[j * (layers + 1) + 2] = *p + delta(j) * 2.0;
    }
    let ik = [key(99)];
    let ip = [Point3::new(0.0, 0.0, 0.0)];
    let opts = PlaceOptions {
        placement: Placement::Interpolate,
        shepard_power: 3.0,
        support: 2,
    };
    let base = run(&keys, &pts, &cols, layers, &ik, &ip, opts);
    // The two nearest by (distance, key) are nodes 0 and 1 with equal weights.
    let want = ip[0] + (delta(0) + delta(1)) * 0.5;
    assert!((base[0] - want).norm() < 1e-12, "{:?} vs {want:?}", base[0]);
    // Permute the input order of the boundary arrays.
    let mut rng = Rng(3);
    for _ in 0..10 {
        let mut perm: Vec<usize> = (0..pts.len()).collect();
        for i in (1..perm.len()).rev() {
            perm.swap(i, (rng.next_u64() % (i as u64 + 1)) as usize);
        }
        let k2: Vec<NodeKey> = perm.iter().map(|j| keys[*j]).collect();
        let p2: Vec<Point3> = perm.iter().map(|j| pts[*j]).collect();
        let mut c2 = Vec::new();
        for j in &perm {
            c2.extend_from_slice(&cols[j * (layers + 1)..(j + 1) * (layers + 1)]);
        }
        let out = run(&k2, &p2, &c2, layers, &ik, &ip, opts);
        for (a, b) in base.iter().zip(&out) {
            assert_eq!(
                (a.x.to_bits(), a.y.to_bits(), a.z.to_bits()),
                (b.x.to_bits(), b.y.to_bits(), b.z.to_bits())
            );
        }
    }
}

/// An independent dense Shepard reference for random scattered data.
#[test]
fn the_grid_neighbour_search_matches_a_dense_reference() {
    let mut rng = Rng(99);
    let nb = 60;
    let bp: Vec<Point3> = (0..nb)
        .map(|_| Point3::new(rng.range(0.0, 10.0), rng.range(0.0, 10.0), 0.0))
        .collect();
    let bk: Vec<NodeKey> = (0..nb as u32).map(key).collect();
    let ip: Vec<Point3> = (0..40)
        .map(|_| Point3::new(rng.range(1.0, 9.0), rng.range(1.0, 9.0), 0.0))
        .collect();
    let ik: Vec<NodeKey> = (0..40u32).map(|i| key(1000 + i)).collect();
    let layers = 2;
    let disp: Vec<Vector3> = (0..nb)
        .map(|_| {
            Vector3::new(
                rng.range(-0.2, 0.2),
                rng.range(-0.2, 0.2),
                1.0 + rng.range(-0.1, 0.1),
            )
        })
        .collect();
    let cols = {
        let mut c = Vec::new();
        for (p, d) in bp.iter().zip(&disp) {
            c.extend([*p, *p + *d, *p + *d * 2.0]);
        }
        c
    };
    let (support, power) = (7usize, 2.5);
    let out = run(
        &bk,
        &bp,
        &cols,
        layers,
        &ik,
        &ip,
        PlaceOptions {
            placement: Placement::Interpolate,
            shepard_power: power,
            support,
        },
    );
    for (n, x) in ip.iter().enumerate() {
        let mut d: Vec<(f64, usize)> = bp
            .iter()
            .enumerate()
            .map(|(j, p)| ((*p - *x).norm().powi(2), j))
            .collect();
        d.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        d.truncate(support);
        let ws: Vec<f64> = d.iter().map(|(d2, _)| d2.powf(-0.5 * power)).collect();
        let sum: f64 = ws.iter().sum();
        let mut shift = Vector3::default();
        for ((_, j), w) in d.iter().zip(&ws) {
            shift = shift + disp[*j] * (w / sum);
        }
        assert!((out[n] - (*x + shift)).norm() < 1e-12, "node {n}");
    }
}

#[test]
fn shepard_weights_stay_finite_for_tiny_and_huge_geometry() {
    // d^-3 overflows (or underflows) at these scales without normalising by the nearest distance.
    for scale in [1e-120_f64, 1e120] {
        let (bk, bp, ik, ip) = square();
        let sc = |p: &Point3| Point3::new(p.x * scale, p.y * scale, p.z * scale);
        let bp: Vec<Point3> = bp.iter().map(sc).collect();
        let ip: Vec<Point3> = ip.iter().map(sc).collect();
        let layers = 2;
        let cols = columns(&bp, layers, |l, p| {
            p + Vector3::new(0.0, 0.0, l as f64 * scale)
        });
        let opts = PlaceOptions {
            placement: Placement::Interpolate,
            shepard_power: 3.0,
            support: 0,
        };
        let out = run(&bk, &bp, &cols, layers, &ik, &ip, opts);
        for (n, p) in out.iter().enumerate() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "scale {scale}: {p:?}"
            );
            assert!(
                (p.z - scale).abs() <= 1e-12 * scale,
                "scale {scale} node {n}: {p:?}"
            );
        }
    }
}

#[test]
fn inconsistent_input_is_an_error() {
    let (bk, bp, ik, ip) = square();
    let cols = columns(&bp, 2, |_, p| p);
    let bad = place_layers(
        &Boundary {
            keys: &bk,
            src: &bp,
            cols: &cols[1..],
            layers: 2,
        },
        &Interior {
            keys: &ik,
            pos: &ip,
        },
        &PlaceOptions::default(),
    );
    assert!(bad.is_err());
    let dup = [bk[0], bk[0]];
    let two = [bp[0], bp[1]];
    let c2 = columns(&two, 2, |_, p| p);
    assert!(
        place_layers(
            &Boundary {
                keys: &dup,
                src: &two,
                cols: &c2,
                layers: 2
            },
            &Interior {
                keys: &ik,
                pos: &ip
            },
            &PlaceOptions::default(),
        )
        .is_err()
    );
}
