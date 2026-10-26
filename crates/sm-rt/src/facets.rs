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

/// Located, grouped findings for a facet soup and its boundary rules.
///
/// Unlike [`FacetInput::check_watertight`] (one issue per bad edge) the bad edges of one
/// volume and one kind are grouped into connected components, so a single missing triangle is
/// one `facet.open_edge` issue: `location` is the mean of the group's vertices (on the missing
/// triangle for a one-triangle hole), `gap` the longest open edge of the group. Codes:
/// `facet.open_edge`, `facet.nonmanifold_edge`, `facet.inconsistent_orientation`,
/// `deck.mismatch` (a periodic rule whose mate face is not in the input). Output order is
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
