//! Corner cells of `pave` on a circle: the facts behind `PAVE_FLAT_COUNTS` of `sm-scheme-ogrid`.
//!
//! A quad with two consecutive edges on a smooth boundary has the corner angle `pi - 2 pi / n`
//! (n circle nodes) wherever its other nodes sit, so its sine is `sin(2 pi / n)` and uniform
//! refinement, which doubles n, halves it. The nodes are fixed, so only the topology can avoid
//! the cell: `pave` closes 6 to 14 circle nodes with such cells and 16 and more without.

use sm_scheme_pave::{PaveOptions, pave_region};

/// The quad of every `(a, b, c)` run of three consecutive circle nodes in it, and the quad count.
fn corner_cells(n: usize) -> (usize, Vec<[[f64; 2]; 4]>) {
    let w: Vec<[f64; 2]> = (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            [a.cos(), a.sin()]
        })
        .collect();
    let m = pave_region(&[w], &|_| [1.0, 0.0, 1.0], None, &PaveOptions::default()).expect("pave");
    let on =
        |a: u32, b: u32| (a as usize) < n && (b as usize) < n && (a as usize + 1) % n == b as usize;
    let mut out = Vec::new();
    for q in &m.quads {
        if (0..4).any(|k| on(q[k], q[(k + 1) % 4]) && on(q[(k + 1) % 4], q[(k + 2) % 4])) {
            out.push(q.map(|i| m.points[i as usize]));
        }
    }
    (m.quads.len(), out)
}

/// Sine of the corner of `q` at node `k`.
fn sine(q: &[[f64; 2]; 4], k: usize) -> f64 {
    let (p0, p1, p2) = (q[(k + 3) % 4], q[k], q[(k + 1) % 4]);
    let (e1, e2) = (
        [p1[0] - p0[0], p1[1] - p0[1]],
        [p2[0] - p1[0], p2[1] - p1[1]],
    );
    (e1[0] * e2[1] - e1[1] * e2[0]) / (e1[0].hypot(e1[1]) * e2[0].hypot(e2[1]))
}

#[test]
fn six_to_fourteen_circle_nodes_leave_corner_cells_and_sixteen_up_do_not() {
    for n in (6..=48usize).step_by(2) {
        let (_, cells) = corner_cells(n);
        if n <= 14 {
            assert!(!cells.is_empty(), "n = {n}: expected corner cells");
        } else {
            assert!(cells.is_empty(), "n = {n}: {} corner cells", cells.len());
        }
    }
}

#[test]
fn the_corner_cell_angle_is_pi_minus_two_pi_over_n_at_the_shared_node() {
    // n = 8: four quads around one centre, each owning two circle edges (analytic: 135 degrees).
    let (quads, cells) = corner_cells(8);
    assert_eq!((quads, cells.len()), (4, 4));
    let want = (std::f64::consts::TAU / 8.0).sin();
    for q in &cells {
        // The shared circle node is the one at unit radius between two neighbours at unit radius.
        let k = (0..4)
            .find(|&k| {
                let on = |p: [f64; 2]| (p[0].hypot(p[1]) - 1.0).abs() < 1e-12;
                on(q[(k + 3) % 4]) && on(q[k]) && on(q[(k + 1) % 4])
            })
            .expect("a corner at three circle nodes");
        assert!(
            (sine(q, k) - want).abs() < 1e-12,
            "sine {} vs sin(2 pi / 8) = {want}",
            sine(q, k)
        );
    }
}
