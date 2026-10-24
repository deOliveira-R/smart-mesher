//! Polygon-with-holes triangulation by ear clipping, with exact predicates.
//!
//! Input points are snapped once to an integer lattice (`2^30` steps over the larger bbox
//! side) and every predicate runs in `i128`, so the result cannot depend on rounding. Holes are
//! joined to the outer ring by a visible bridge (nearest vertex whose segment crosses no ring
//! edge and lies inside the region). Boundary vertices are never added or dropped: all of them
//! appear in some triangle, which keeps neighbouring faces free of T-junctions.

type Pt = [i64; 2];

const LATTICE: f64 = (1u64 << 30) as f64;

/// A ring vertex: uv position and the mesh vertex it stands for.
pub(crate) type RingPt = ([f64; 2], u32);

fn orient(a: Pt, b: Pt, c: Pt) -> i128 {
    (b[0] - a[0]) as i128 * (c[1] - a[1]) as i128 - (b[1] - a[1]) as i128 * (c[0] - a[0]) as i128
}

fn area2(ring: &[Pt]) -> i128 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            a[0] as i128 * b[1] as i128 - b[0] as i128 * a[1] as i128
        })
        .sum()
}

fn on_segment(a: Pt, b: Pt, p: Pt) -> bool {
    orient(a, b, p) == 0 && p[0] >= a[0].min(b[0]) && p[0] <= a[0].max(b[0]) && p[1] >= a[1].min(b[1]) && p[1] <= a[1].max(b[1])
}

/// True if the closed segments share a point.
fn segments_touch(p1: Pt, p2: Pt, p3: Pt, p4: Pt) -> bool {
    let (d1, d2) = (orient(p3, p4, p1).signum(), orient(p3, p4, p2).signum());
    let (d3, d4) = (orient(p1, p2, p3).signum(), orient(p1, p2, p4).signum());
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    on_segment(p3, p4, p1) || on_segment(p3, p4, p2) || on_segment(p1, p2, p3) || on_segment(p1, p2, p4)
}

#[derive(Clone, Copy)]
struct Node {
    p: Pt,
    g: u32,
    prev: usize,
    next: usize,
}

struct Arena {
    n: Vec<Node>,
}

impl Arena {
    fn ring(&mut self, pts: &[(Pt, u32)]) -> usize {
        let base = self.n.len();
        let k = pts.len();
        for (i, (p, g)) in pts.iter().enumerate() {
            self.n.push(Node { p: *p, g: *g, prev: base + (i + k - 1) % k, next: base + (i + 1) % k });
        }
        base
    }

    fn cycle(&self, head: usize) -> Vec<usize> {
        let mut v = vec![head];
        let mut i = self.n[head].next;
        while i != head {
            v.push(i);
            i = self.n[i].next;
        }
        v
    }

    fn clone_node(&mut self, i: usize) -> usize {
        let c = self.n[i];
        self.n.push(c);
        self.n.len() - 1
    }
}

/// Triangulate `rings[0]` (outer) minus `rings[1..]` (holes). Triangles are counter-clockwise
/// in the input's `(u, v)` frame and reference the rings' mesh vertices.
pub(crate) fn triangulate(rings: &[Vec<RingPt>]) -> Result<Vec<[u32; 3]>, String> {
    let Some(outer_in) = rings.first() else { return Err("no outer loop".into()) };
    if outer_in.len() < 3 {
        return Err("outer loop has fewer than 3 vertices".into());
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (p, _) in rings.iter().flatten() {
        if !(p[0].is_finite() && p[1].is_finite()) {
            return Err("non-finite parameter coordinates".into());
        }
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    if span <= 0.0 {
        return Err("degenerate (zero-extent) loop".into());
    }
    let scale = LATTICE / span;
    let snap = |p: &[f64; 2]| -> Pt { [((p[0] - lo[0]) * scale).round() as i64, ((p[1] - lo[1]) * scale).round() as i64] };

    // Snap, drop consecutive duplicates, fix orientation (outer ccw, holes cw).
    let mut prepared: Vec<Vec<(Pt, u32)>> = Vec::with_capacity(rings.len());
    for (ri, ring) in rings.iter().enumerate() {
        let mut pts: Vec<(Pt, u32)> = Vec::with_capacity(ring.len());
        for (p, g) in ring {
            let q = snap(p);
            if pts.last().is_none_or(|l| l.0 != q) {
                pts.push((q, *g));
            }
        }
        while pts.len() > 1 && pts[0].0 == pts[pts.len() - 1].0 {
            pts.pop();
        }
        if pts.len() < 3 {
            if ri == 0 {
                return Err("outer loop collapses to fewer than 3 distinct vertices".into());
            }
            continue;
        }
        let coords: Vec<Pt> = pts.iter().map(|x| x.0).collect();
        let a = area2(&coords);
        if a == 0 {
            if ri == 0 {
                return Err("outer loop has zero area".into());
            }
            continue;
        }
        if (ri == 0) != (a > 0) {
            pts.reverse();
        }
        prepared.push(pts);
    }

    let mut arena = Arena { n: Vec::new() };
    let heads: Vec<usize> = prepared.iter().map(|r| arena.ring(r)).collect();
    let outer = heads[0];
    let mut holes: Vec<usize> = heads[1..].to_vec();
    // Process holes by leftmost vertex (then lowest y), then arena index.
    let leftmost = |arena: &Arena, head: usize| -> usize {
        let cyc = arena.cycle(head);
        *cyc.iter().min_by_key(|&&i| (arena.n[i].p[0], arena.n[i].p[1], i)).unwrap_or(&head)
    };
    let mut keyed: Vec<(usize, usize)> = holes.iter().map(|&h| (leftmost(&arena, h), h)).collect();
    keyed.sort_by_key(|&(l, h)| (arena.n[l].p[0], arena.n[l].p[1], h));
    holes = keyed.iter().map(|&(_, h)| h).collect();
    let hole_pts: Vec<usize> = keyed.iter().map(|&(l, _)| l).collect();

    let mut merged = outer;
    let mut pending: Vec<usize> = holes.clone();
    for (k, &hole_head) in holes.iter().enumerate() {
        let h = hole_pts[k];
        pending.retain(|&x| x != hole_head);
        let c = find_bridge(&arena, merged, &pending, hole_head, h).ok_or("could not bridge a hole to the outer loop")?;
        merged = split(&mut arena, c, h);
    }

    let mut tris = Vec::new();
    ear_clip(&mut arena, merged, &mut tris);
    Ok(tris)
}

/// Splice hole vertex `b` to polygon vertex `a` (earcut's `splitPolygon`); returns a node of the joined ring.
fn split(arena: &mut Arena, a: usize, b: usize) -> usize {
    let a2 = arena.clone_node(a);
    let b2 = arena.clone_node(b);
    let an = arena.n[a].next;
    let bp = arena.n[b].prev;
    arena.n[a].next = b;
    arena.n[b].prev = a;
    arena.n[a2].next = an;
    arena.n[an].prev = a2;
    arena.n[b2].next = a2;
    arena.n[a2].prev = b2;
    arena.n[bp].next = b2;
    arena.n[b2].prev = bp;
    arena.n[a2].prev = b2;
    a
}

fn find_bridge(arena: &Arena, merged: usize, pending: &[usize], hole_head: usize, h: usize) -> Option<usize> {
    let hp = arena.n[h].p;
    let mut cands = arena.cycle(merged);
    let d2 = |i: usize| {
        let p = arena.n[i].p;
        let (dx, dy) = ((p[0] - hp[0]) as i128, (p[1] - hp[1]) as i128);
        dx * dx + dy * dy
    };
    cands.sort_by_key(|&i| (d2(i), i));
    let mut edges: Vec<(Pt, Pt)> = Vec::new();
    for head in std::iter::once(merged).chain(pending.iter().copied()).chain(std::iter::once(hole_head)) {
        for i in arena.cycle(head) {
            edges.push((arena.n[i].p, arena.n[arena.n[i].next].p));
        }
    }
    'cand: for c in cands {
        let cp = arena.n[c].p;
        if cp == hp {
            continue;
        }
        for &(a, b) in &edges {
            if a == hp || a == cp || b == hp || b == cp {
                continue;
            }
            if segments_touch(hp, cp, a, b) {
                continue 'cand;
            }
        }
        // Midpoint (doubled coordinates) must be inside the region: odd crossing count.
        let q = [hp[0] + cp[0], hp[1] + cp[1]];
        let mut inside = false;
        for &(a, b) in &edges {
            let (a, b) = ([a[0] * 2, a[1] * 2], [b[0] * 2, b[1] * 2]);
            if (a[1] > q[1]) != (b[1] > q[1]) {
                let t = orient(a, b, q);
                if (b[1] > a[1]) == (t > 0) {
                    inside = !inside;
                }
            }
        }
        if inside {
            return Some(c);
        }
    }
    None
}

fn convex(arena: &Arena, i: usize) -> bool {
    let n = &arena.n[i];
    orient(arena.n[n.prev].p, n.p, arena.n[n.next].p) > 0
}

fn ear_clip(arena: &mut Arena, head: usize, tris: &mut Vec<[u32; 3]>) {
    let cyc = arena.cycle(head);
    let mut remaining = cyc.len();
    let mut alive = vec![false; arena.n.len()];
    for &i in &cyc {
        alive[i] = true;
    }
    // Convex vertices never turn reflex when an ear is cut, so this list only shrinks.
    let mut reflex: Vec<usize> = cyc.iter().copied().filter(|&i| !convex(arena, i)).collect();
    let mut ear = head;
    let mut stop = ear;
    while remaining > 3 {
        let (prev, next) = (arena.n[ear].prev, arena.n[ear].next);
        if is_ear(arena, ear, &reflex, &alive) {
            tris.push([arena.n[prev].g, arena.n[ear].g, arena.n[next].g]);
            remove(arena, &mut alive, ear);
            remaining -= 1;
            ear = arena.n[next].next;
            stop = ear;
            reflex.retain(|&r| alive[r] && !convex(arena, r));
            continue;
        }
        ear = next;
        if ear == stop {
            // No ear: clip any convex tip ignoring the containment test, else drop a flat vertex.
            let cyc = arena.cycle(ear);
            if let Some(&t) = cyc.iter().find(|&&i| convex(arena, i)) {
                let (p, n) = (arena.n[t].prev, arena.n[t].next);
                tris.push([arena.n[p].g, arena.n[t].g, arena.n[n].g]);
                remove(arena, &mut alive, t);
                ear = n;
            } else {
                let t = cyc[0];
                let n = arena.n[t].next;
                remove(arena, &mut alive, t);
                ear = n;
            }
            remaining -= 1;
            stop = ear;
            reflex.retain(|&r| alive[r] && !convex(arena, r));
        }
    }
    if remaining == 3 {
        let (a, b, c) = (ear, arena.n[ear].next, arena.n[arena.n[ear].next].next);
        if orient(arena.n[a].p, arena.n[b].p, arena.n[c].p) != 0 {
            tris.push([arena.n[a].g, arena.n[b].g, arena.n[c].g]);
        }
    }
}

fn remove(arena: &mut Arena, alive: &mut [bool], i: usize) {
    let (p, n) = (arena.n[i].prev, arena.n[i].next);
    arena.n[p].next = n;
    arena.n[n].prev = p;
    alive[i] = false;
}

fn is_ear(arena: &Arena, i: usize, reflex: &[usize], alive: &[bool]) -> bool {
    let (p, n) = (arena.n[i].prev, arena.n[i].next);
    let (a, b, c) = (arena.n[p].p, arena.n[i].p, arena.n[n].p);
    if orient(a, b, c) <= 0 {
        return false;
    }
    for &r in reflex {
        if !alive[r] || r == p || r == i || r == n {
            continue;
        }
        let q = arena.n[r].p;
        if q == a || q == b || q == c {
            continue;
        }
        if orient(a, b, q) >= 0 && orient(b, c, q) >= 0 && orient(c, a, q) >= 0 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(pts: &[[f64; 2]], base: u32) -> Vec<RingPt> {
        pts.iter().enumerate().map(|(i, p)| (*p, base + i as u32)).collect()
    }

    fn coords(rings: &[Vec<RingPt>]) -> std::collections::BTreeMap<u32, [f64; 2]> {
        rings.iter().flatten().map(|&(p, g)| (g, p)).collect()
    }

    fn area(tris: &[[u32; 3]], pos: &std::collections::BTreeMap<u32, [f64; 2]>) -> f64 {
        tris.iter()
            .map(|t| {
                let (a, b, c) = (pos[&t[0]], pos[&t[1]], pos[&t[2]]);
                0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            })
            .sum()
    }

    fn all_ccw(tris: &[[u32; 3]], pos: &std::collections::BTreeMap<u32, [f64; 2]>) -> bool {
        tris.iter().all(|t| {
            let (a, b, c) = (pos[&t[0]], pos[&t[1]], pos[&t[2]]);
            (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.0
        })
    }

    #[test]
    fn square_gives_two_triangles() {
        let r = vec![ring(&[[0.0, 0.0], [10.0, 0.0], [10.0, 6.0], [0.0, 6.0]], 0)];
        let t = triangulate(&r).unwrap();
        assert_eq!(t.len(), 2);
        assert_eq!(area(&t, &coords(&r)), 60.0);
    }

    #[test]
    fn clockwise_input_is_reoriented() {
        let r = vec![ring(&[[0.0, 0.0], [0.0, 6.0], [10.0, 6.0], [10.0, 0.0]], 0)];
        let t = triangulate(&r).unwrap();
        assert!(all_ccw(&t, &coords(&r)));
        assert_eq!(area(&t, &coords(&r)), 60.0);
    }

    #[test]
    fn l_shape_area() {
        let r = vec![ring(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0], [1.0, 2.0], [0.0, 2.0]], 0)];
        let t = triangulate(&r).unwrap();
        assert_eq!(t.len(), 4);
        assert!((area(&t, &coords(&r)) - 3.0).abs() < 1e-12);
        assert!(all_ccw(&t, &coords(&r)));
    }

    #[test]
    fn collinear_boundary_vertices_are_used() {
        let r = vec![ring(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], 0)];
        let t = triangulate(&r).unwrap();
        assert_eq!(t.len(), 3);
        let used: std::collections::BTreeSet<u32> = t.iter().flatten().copied().collect();
        assert_eq!(used.len(), 5);
        assert!((area(&t, &coords(&r)) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn disc_with_hole() {
        let n = 24;
        let circ = |rad: f64, base: u32| -> Vec<RingPt> {
            (0..n).map(|i| {
                let a = core::f64::consts::TAU * i as f64 / n as f64;
                ([rad * a.cos(), rad * a.sin()], base + i as u32)
            }).collect()
        };
        let rings = vec![circ(3.0, 0), circ(1.0, 100)];
        let t = triangulate(&rings).unwrap();
        let pos = coords(&rings);
        let poly = |rad: f64| 0.5 * n as f64 * rad * rad * (core::f64::consts::TAU / n as f64).sin();
        assert!((area(&t, &pos) - (poly(3.0) - poly(1.0))).abs() < 1e-9, "{}", area(&t, &pos));
        assert!(all_ccw(&t, &pos));
        let used: std::collections::BTreeSet<u32> = t.iter().flatten().copied().collect();
        assert_eq!(used.len(), 2 * n);
        assert_eq!(t.len(), 2 * n);
    }

    #[test]
    fn two_holes() {
        let sq = |x: f64, y: f64, s: f64, base: u32| ring(&[[x, y], [x, y + s], [x + s, y + s], [x + s, y]], base);
        let rings = vec![ring(&[[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]], 0), sq(1.0, 1.0, 2.0, 10), sq(5.0, 1.0, 3.0, 20)];
        let t = triangulate(&rings).unwrap();
        assert!((area(&t, &coords(&rings)) - (50.0 - 4.0 - 9.0)).abs() < 1e-12);
        assert!(all_ccw(&t, &coords(&rings)));
    }

    #[test]
    fn degenerate_inputs_are_errors() {
        assert!(triangulate(&[]).is_err());
        assert!(triangulate(&[ring(&[[0.0, 0.0], [1.0, 1.0]], 0)]).is_err());
        assert!(triangulate(&[ring(&[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]], 0)]).is_err());
    }

    use proptest::prelude::*;

    /// A star-shaped polygon about the origin: angles `2 pi (i + j_i) / n`, radii in `[1, 10]`.
    fn star(n: usize, jit: &[f64], rad: &[f64], base: u32) -> Vec<RingPt> {
        (0..n)
            .map(|i| {
                let a = core::f64::consts::TAU * (i as f64 + 0.5 * jit[i]) / n as f64;
                ([rad[i] * a.cos(), rad[i] * a.sin()], base + i as u32)
            })
            .collect()
    }

    fn shoelace(r: &[RingPt]) -> f64 {
        let n = r.len();
        0.5 * (0..n).map(|i| r[i].0[0] * r[(i + 1) % n].0[1] - r[(i + 1) % n].0[0] * r[i].0[1]).sum::<f64>()
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn random_star_polygons(n in 12usize..48, jit in prop::collection::vec(0.0f64..1.0, 48), rad in prop::collection::vec(1.0f64..10.0, 48)) {
            let outer = star(n, &jit, &rad, 0);
            let rings = vec![outer.clone()];
            let t = triangulate(&rings).unwrap();
            let pos = coords(&rings);
            prop_assert_eq!(t.len(), n - 2);
            prop_assert!(all_ccw(&t, &pos));
            let want = shoelace(&outer);
            prop_assert!((area(&t, &pos) - want).abs() < 1e-6 * want, "{} vs {}", area(&t, &pos), want);
        }

        #[test]
        fn random_star_polygons_with_a_hole(n in 12usize..48, m in 3usize..20, jit in prop::collection::vec(0.0f64..1.0, 48), rad in prop::collection::vec(1.0f64..10.0, 48)) {
            let outer = star(n, &jit, &rad, 0);
            // A regular polygon of radius 0.5 (clockwise) around the origin lies inside the star.
            let hole: Vec<RingPt> = (0..m).map(|i| {
                let a = -core::f64::consts::TAU * i as f64 / m as f64;
                ([0.5 * a.cos(), 0.5 * a.sin()], 1000 + i as u32)
            }).collect();
            let rings = vec![outer.clone(), hole.clone()];
            let t = triangulate(&rings).unwrap();
            let pos = coords(&rings);
            prop_assert_eq!(t.len(), n + m);
            prop_assert!(all_ccw(&t, &pos));
            let want = shoelace(&outer) + shoelace(&hole); // the hole ring is clockwise: negative area
            prop_assert!((area(&t, &pos) - want).abs() < 1e-6 * want, "{} vs {}", area(&t, &pos), want);
            let used: std::collections::BTreeSet<u32> = t.iter().flatten().copied().collect();
            prop_assert_eq!(used.len(), n + m);
        }
    }
}
