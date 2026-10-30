//! Facet-input diagnostics: grouped watertightness report (diagnostic only, never repairs).

use sm_core::{EntityId, Point3};
use sm_rt_api::{BoundaryDef, BoundarySpec, BuildIssue, FacetInput};
use sm_topo_api::Region;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Default)]
struct EdgeUse {
    fwd: u32,
    bwd: u32,
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Facets of different shells that sit on top of each other (touching, unmerged volumes).
///
/// Corner positions are snapped to a grid of `1e-9` of the model diagonal; two triangles with the
/// same three snapped corners are coincident. One `facet.coincident` issue per pair of B-rep
/// faces, located at the mean centroid of the pairs.
fn coincident_facets(f: &FacetInput, out: &mut Vec<BuildIssue>) {
    if f.triangles.len() < 2 {
        return;
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for v in &f.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    let diag = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    let q = if diag > 0.0 { diag * 1e-9 } else { return };
    let snap = |v: u32| -> [i64; 3] {
        let p = f.vertices[v as usize];
        [0, 1, 2].map(|k| ((p[k] - lo[k]) / q).round() as i64)
    };
    let mut keyed: Vec<([[i64; 3]; 3], u32)> = f
        .triangles
        .iter()
        .enumerate()
        .map(|(i, t)| {
            let mut k = [snap(t[0]), snap(t[1]), snap(t[2])];
            k.sort_unstable();
            (k, i as u32)
        })
        .collect();
    keyed.sort_unstable();
    let mut groups: BTreeMap<(EntityId, EntityId), (u32, [f64; 3], u32)> = BTreeMap::new();
    for w in keyed.windows(2) {
        if w[0].0 != w[1].0 {
            continue;
        }
        let (i, j) = (w[0].1 as usize, w[1].1 as usize);
        let (a, b) = (f.face[i].min(f.face[j]), f.face[i].max(f.face[j]));
        let t = f.triangles[i];
        let e = groups.entry((a, b)).or_insert((0, [0.0; 3], t[0]));
        e.0 += 1;
        for v in t {
            for k in 0..3 {
                e.1[k] += f.vertices[v as usize][k] / 3.0;
            }
        }
    }
    for ((a, _), (n, s, _)) in groups {
        let n_f = f64::from(n);
        out.push(
            BuildIssue::new("facet.coincident", format!("{n} facet pair(s) of these faces coincide: touching shells that were not imprinted and merged"))
                .with_entity(a)
                .at(Point3::new(s[0] / n_f, s[1] / n_f, s[2] / n_f))
                .suggest("imprint and merge the touching volumes so they share one set of facets, as DAGMC requires"),
        );
    }
}

/// Located, grouped findings for a facet soup and its boundary rules.
///
/// Unlike [`FacetInput::check_watertight`] (one issue per bad edge) the bad edges of one
/// volume and one kind are grouped into connected components, so a single missing triangle is
/// one `facet.open_edge` issue: `location` is the mean of the group's vertices (on the missing
/// triangle for a one-triangle hole), `gap` the longest open edge of the group. Codes:
/// `facet.open_edge`, `facet.nonmanifold_edge`, `facet.inconsistent_orientation`,
/// `facet.coincident` (touching shells whose facets lie on top of each other, i.e. not imprinted
/// and merged), `deck.mismatch` (a periodic rule whose mate face is not in the input). Output order is
/// deterministic (volume id, code, group's lowest edge). Structural problems
/// ([`FacetInput::validate`]) are reported first and stop the scan.
pub fn check_facets(f: &FacetInput, spec: &BoundarySpec) -> Vec<BuildIssue> {
    if let Err(sm_rt_api::RtError::Build(issues)) = f.validate() {
        return issues;
    }
    let mut per_volume: BTreeMap<EntityId, BTreeMap<(u32, u32), EdgeUse>> = BTreeMap::new();
    for (i, t) in f.triangles.iter().enumerate() {
        for (reg, flip) in [(&f.front[i], false), (&f.back[i], true)] {
            let Region::Volume(vol) = reg else { continue };
            let mut tri = *t;
            if flip {
                tri.swap(1, 2);
            }
            let edges = per_volume.entry(*vol).or_default();
            for k in 0..3 {
                let (a, c) = (tri[k], tri[(k + 1) % 3]);
                let e = edges.entry((a.min(c), a.max(c))).or_default();
                if a < c { e.fwd += 1 } else { e.bwd += 1 }
            }
        }
    }
    let p = |v: u32| {
        let q = f.vertices[v as usize];
        Point3::new(q[0], q[1], q[2])
    };
    let mut out = Vec::new();
    for (vol, edges) in &per_volume {
        for (code, msg, fix) in [
            ("facet.open_edge", "boundary of this volume has a gap (edges used by one triangle only)", "imprint and merge the touching volumes, or re-tessellate so shared curves use one polyline"),
            ("facet.nonmanifold_edge", "edges shared by more than two triangles of this volume", "check for a baffle or duplicate face"),
            ("facet.inconsistent_orientation", "two triangles traverse an edge in the same direction", "flip the face sense in the B-rep or the facet winding"),
        ] {
            let bad: Vec<(u32, u32)> = edges
                .iter()
                .filter(|(_, e)| (e.fwd, e.bwd) != (1, 1))
                .filter(|(_, e)| match code {
                    "facet.open_edge" => e.fwd + e.bwd == 1,
                    "facet.nonmanifold_edge" => e.fwd + e.bwd > 2,
                    _ => e.fwd + e.bwd == 2 && e.fwd != e.bwd,
                })
                .map(|(k, _)| *k)
                .collect();
            if bad.is_empty() {
                continue;
            }
            let verts: BTreeSet<u32> = bad.iter().flat_map(|&(a, b)| [a, b]).collect();
            let index: BTreeMap<u32, usize> = verts.iter().enumerate().map(|(i, &v)| (v, i)).collect();
            let mut parent: Vec<usize> = (0..verts.len()).collect();
            for &(a, b) in &bad {
                let (ra, rb) = (find(&mut parent, index[&a]), find(&mut parent, index[&b]));
                if ra != rb {
                    parent[ra.max(rb)] = ra.min(rb);
                }
            }
            let mut groups: BTreeMap<usize, Vec<(u32, u32)>> = BTreeMap::new();
            for &(a, b) in &bad {
                let r = find(&mut parent, index[&a]);
                groups.entry(r).or_default().push((a, b));
            }
            for edges in groups.values() {
                let gv: BTreeSet<u32> = edges.iter().flat_map(|&(a, b)| [a, b]).collect();
                let n = gv.len() as f64;
                let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
                for &v in &gv {
                    let q = p(v);
                    x += q.x;
                    y += q.y;
                    z += q.z;
                }
                let longest = edges.iter().map(|&(a, b)| (p(b) - p(a)).norm()).fold(0.0, f64::max);
                let mut issue = BuildIssue::new(code, format!("{msg} ({} edge(s) in this group)", edges.len()))
                    .with_entity(*vol)
                    .at(Point3::new(x / n, y / n, z / n))
                    .suggest(fix);
                if code == "facet.open_edge" {
                    issue = issue.with_gap(longest);
                }
                out.push(issue);
            }
        }
    }
    coincident_facets(f, &mut out);
    let faces: BTreeSet<EntityId> = f.face.iter().copied().collect();
    for (_, def) in &spec.rules {
        if let BoundaryDef::Wrap { mate, .. } = def
            && !faces.contains(mate)
        {
            out.push(BuildIssue::new("deck.mismatch", "a periodic rule names a mate face that is not in the facet input").with_entity(*mate));
        }
    }
    out
}
