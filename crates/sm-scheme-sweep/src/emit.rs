//! Generation: column assembly, layer placement, the quality gate and cell emission.

use crate::discover::Columns;
use crate::error::SweepError;
use crate::params::{Placement, SweepParams};
use crate::place::{Boundary, Interior, PAR_MIN, PlaceOptions, place_layers};
use rayon::prelude::*;
use sm_core::{EntityId, Point3, Vector3};
use sm_mesh_api::{CellShape, Param};
use sm_scheme_api::{
    ElemShape, FailureKind, GenCtx, GenerateFailure, NodeKey, Patch, PatchNode, Target,
};
use sm_topo_api::{Dim, TopologyView};
use std::collections::BTreeSet;

type GResult<T> = Result<T, GenerateFailure>;

/// Hex corner frames `[node, a, b, c]`: the determinant of the three edge vectors is positive
/// for a right-handed element.
const HEX_CORNERS: [[u8; 4]; 8] = [
    [0, 1, 3, 4],
    [1, 2, 0, 5],
    [2, 3, 1, 6],
    [3, 0, 2, 7],
    [4, 7, 5, 0],
    [5, 4, 6, 1],
    [6, 5, 7, 2],
    [7, 6, 4, 3],
];
/// Wedge corner frames.
const WEDGE_CORNERS: [[u8; 4]; 6] = [
    [0, 1, 2, 3],
    [1, 2, 0, 4],
    [2, 0, 1, 5],
    [3, 5, 4, 0],
    [4, 3, 5, 1],
    [5, 4, 3, 2],
];

/// Scaled Jacobian of one corner: `det(e_a, e_b, e_c) / (|e_a| |e_b| |e_c|)`; 1 for an
/// orthogonal corner, 0 for a flat or collapsed one.
fn corner_jacobian(p: &[Point3], c: [u8; 4]) -> f64 {
    let o = p[c[0] as usize];
    let ea = p[c[1] as usize] - o;
    let eb = p[c[2] as usize] - o;
    let ec = p[c[3] as usize] - o;
    let len = ea.norm() * eb.norm() * ec.norm();
    if len > 0.0 {
        ea.dot(eb.cross(ec)) / len
    } else {
        0.0
    }
}

fn newell(ring: &[Point3]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..ring.len() {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        n = n + Vector3::new(a.x, a.y, a.z).cross(Vector3::new(b.x, b.y, b.z));
    }
    n
}

/// Pyramid volumes of a polyhedron from its centroid: `(total, min over faces)`.
fn polyhedron_volumes(faces: &[Vec<Point3>]) -> (f64, f64) {
    let (mut c, mut n) = (Vector3::default(), 0.0);
    let mut seen: Vec<Point3> = Vec::new();
    for f in faces {
        for p in f {
            if !seen.contains(p) {
                seen.push(*p);
                c = c + Vector3::new(p.x, p.y, p.z);
                n += 1.0;
            }
        }
    }
    let c = c * (1.0 / n);
    let (mut total, mut least) = (0.0, f64::INFINITY);
    for f in faces {
        let mut fc = Vector3::default();
        for p in f {
            fc = fc + Vector3::new(p.x, p.y, p.z);
        }
        let fc = fc * (1.0 / f.len() as f64);
        let v = newell(f).dot(fc - c) / 6.0;
        total += v;
        least = least.min(v);
    }
    (total, least)
}

/// Source elements in a form the layer loops can share.
struct Elems {
    /// `(shape, start, len)` into `ring`.
    meta: Vec<(ElemShape, u32, u32)>,
    /// Source-node indices (into the sorted key table), flat.
    ring: Vec<u32>,
}

impl Elems {
    fn nodes(&self, e: usize) -> &[u32] {
        let (_, s, n) = self.meta[e];
        &self.ring[s as usize..(s + n) as usize]
    }
}

/// Order the ring so that it runs counter-clockwise about the direction of the sweep.
fn ring_order(n: usize, forward: bool) -> impl Iterator<Item = usize> {
    (0..n).map(move |i| if forward || i == 0 { i } else { n - i })
}

fn ordered<T: Send>(n: usize, f: impl Fn(usize) -> T + Sync + Send) -> Vec<T> {
    if n >= PAR_MIN {
        (0..n).into_par_iter().map(f).collect()
    } else {
        (0..n).map(f).collect()
    }
}

struct Fail {
    elem: usize,
    layer: usize,
    nodes: Vec<NodeKey>,
    what: String,
}

/// Check one source element over all layers; the first failing layer is reported.
#[allow(clippy::too_many_arguments)]
fn check_element(
    e: usize,
    elems: &Elems,
    forward: bool,
    pos: &[Point3],
    lk: &[NodeKey],
    stride: usize,
    layers: usize,
    min_sj: f64,
) -> Option<Fail> {
    let ring = elems.nodes(e);
    let n = ring.len();
    let order: Vec<usize> = ring_order(n, forward).collect();
    for l in 0..layers {
        let at = |i: usize, layer: usize| ring[order[i]] as usize * stride + layer;
        let fail = |what: String| {
            let mut nodes: Vec<NodeKey> = Vec::with_capacity(2 * n);
            for layer in [l, l + 1] {
                nodes.extend((0..n).map(|i| lk[at(i, layer)]));
            }
            Fail {
                elem: e,
                layer: l,
                nodes,
                what,
            }
        };
        if n <= 4 {
            let mut p = [Point3::default(); 8];
            for (k, layer) in [l, l + 1].into_iter().enumerate() {
                for i in 0..n {
                    p[k * n + i] = pos[at(i, layer)];
                }
            }
            let frames: &[[u8; 4]] = if n == 4 { &HEX_CORNERS } else { &WEDGE_CORNERS };
            for (ci, c) in frames.iter().enumerate() {
                let sj = corner_jacobian(&p[..2 * n], *c);
                // `!(a > b)` also rejects NaN.
                if !(sj > min_sj) {
                    return Some(fail(format!(
                        "corner {ci} has scaled Jacobian {sj:.6e} (must exceed {min_sj})"
                    )));
                }
            }
        } else {
            let bot: Vec<Point3> = (0..n).rev().map(|i| pos[at(i, l)]).collect();
            let top: Vec<Point3> = (0..n).map(|i| pos[at(i, l + 1)]).collect();
            let mut faces = vec![bot, top];
            for i in 0..n {
                let j = (i + 1) % n;
                faces.push(vec![
                    pos[at(i, l)],
                    pos[at(j, l)],
                    pos[at(j, l + 1)],
                    pos[at(i, l + 1)],
                ]);
            }
            let (total, least) = polyhedron_volumes(&faces);
            if !(total > 0.0 && least > 0.0) {
                return Some(fail(format!(
                    "polyhedron volume {total:.6e}, smallest face-pyramid volume {least:.6e} (must be positive)"
                )));
            }
        }
    }
    None
}

/// Rotate layers `1..` about the x axis through the boundary centroid by `bend * l / layers`.
#[cfg(feature = "test-hooks")]
fn apply_bend(pos: &mut [Point3], stride: usize, centre: Point3, bend: f64) {
    let layers = stride - 1;
    for row in pos.chunks_mut(stride) {
        for (l, p) in row.iter_mut().enumerate().skip(1) {
            let (s, c) = (bend * l as f64 / layers as f64).sin_cos();
            let (y, z) = (p.y - centre.y, p.z - centre.z);
            *p = Point3::new(p.x, centre.y + c * y - s * z, centre.z + s * y + c * z);
        }
    }
}

fn column_from_path(cx: &GenCtx<'_>, path: &[(EntityId, bool)]) -> Option<Vec<NodeKey>> {
    let mut c: Vec<NodeKey> = Vec::new();
    for (e, fwd) in path {
        let mut en = cx.edge_nodes(*e)?;
        if !*fwd {
            en.reverse();
        }
        let skip = usize::from(!c.is_empty());
        c.extend_from_slice(&en[skip..]);
    }
    Some(c)
}

/// Generate the cells of a sweep (the target face is a copy meshed by the driver).
pub(crate) fn generate_prism(
    cx: &GenCtx<'_>,
    target: Target,
    st: &Columns,
    sp: &SweepParams,
) -> GResult<Patch> {
    let name = crate::NAME;
    let fatal = |r: String| GenerateFailure::fatal(target, name, r);
    let t: &dyn TopologyView = cx.input.topo;
    let volume = target
        .entity()
        .ok_or_else(|| fatal("a sweep target is a single piece".into()))?;
    let src = cx
        .patches
        .get(Target::Surface(st.source))
        .ok_or_else(|| fatal("source patch missing".into()))?;
    let first = st
        .columns
        .first()
        .ok_or_else(|| fatal("no columns".into()))?;
    let mut layers = 0usize;
    for (e, _) in &first.path {
        layers += cx
            .intervals(*e)
            .ok_or_else(|| fatal(format!("column edge {e} has no interval count")))?
            as usize;
    }
    if layers == 0 {
        return Err(fatal("zero layers".into()));
    }
    let stride = layers + 1;
    for (shape, nodes) in src.elements.iter() {
        let ok = match shape {
            ElemShape::Tri | ElemShape::Quad => true,
            ElemShape::Polygon => nodes.len() >= 3,
            _ => false,
        };
        if !ok {
            return Err(GenerateFailure::new(
                target,
                name,
                FailureKind::Unsupported,
                format!("cannot sweep a source element of shape {shape:?}"),
            ));
        }
    }

    // Source nodes in canonical order.
    let mut keys: Vec<NodeKey> = src
        .nodes
        .iter()
        .map(|n| n.key)
        .chain(src.refs.iter().copied())
        .collect();
    keys.sort_unstable();
    keys.dedup();
    let n_src = keys.len();
    let interior: Vec<bool> = keys.iter().map(|k| k.owner == st.source).collect();
    let n_int = interior.iter().filter(|b| **b).count();
    let n_created = n_int * (layers - 1);

    // Column keys.
    let end_edges: BTreeSet<EntityId> = t.face_edges(st.target).iter().copied().collect();
    let mut lk: Vec<NodeKey> = vec![NodeKey::new(volume, u32::MAX); n_src * stride];
    let mut slot = 0usize;
    for (i, k) in keys.iter().enumerate() {
        let col: Vec<NodeKey> = if interior[i] {
            let mut c = Vec::with_capacity(stride);
            c.push(*k);
            c.extend(
                (1..layers).map(|l| NodeKey::new(volume, (slot * (layers - 1) + l - 1) as u32)),
            );
            c.push(NodeKey::new(st.target.to_owned(), k.index));
            slot += 1;
            c
        } else {
            match t.dim_of(k.owner) {
                Some(Dim::Vertex) => {
                    let ci = st
                        .columns
                        .binary_search_by_key(&k.owner, |c| c.vertex)
                        .map_err(|_| fatal(format!("no column at vertex {}", k.owner)))?;
                    column_from_path(cx, &st.columns[ci].path)
                        .ok_or_else(|| fatal("column edge nodes unavailable".into()))?
                }
                Some(Dim::Edge) => {
                    let mut col = vec![*k];
                    let mut used: Vec<EntityId> = Vec::new();
                    let mut cur = *k;
                    for _ in 0..16 {
                        let l = st
                            .linking
                            .iter()
                            .copied()
                            .find(|l| !used.contains(l) && t.face_edges(*l).contains(&cur.owner))
                            .ok_or_else(|| {
                                fatal(format!("edge {} has no linking face", cur.owner))
                            })?;
                        used.push(l);
                        let line = cx.strip_from(l, cur).ok_or_else(|| {
                            fatal(format!(
                                "node {cur:?} starts no quad strip of linking face {l}"
                            ))
                        })?;
                        col.extend_from_slice(&line[1..]);
                        cur = *line.last().ok_or_else(|| fatal("empty strip".into()))?;
                        if end_edges.contains(&cur.owner) {
                            break;
                        }
                    }
                    col
                }
                _ => {
                    return Err(fatal(format!(
                        "source node {k:?} is owned by {}, outside the source",
                        k.owner
                    )));
                }
            }
        };
        if col.len() != stride {
            return Err(fatal(format!(
                "column at {k:?} has {} nodes, expected {stride}",
                col.len()
            )));
        }
        lk[i * stride..(i + 1) * stride].copy_from_slice(&col);
    }

    // Positions of every node but the created interior ones.
    let mut pos: Vec<Point3> = vec![Point3::default(); n_src * stride];
    for i in 0..n_src {
        for l in 0..stride {
            if interior[i] && l != 0 && l != layers {
                continue;
            }
            let k = lk[i * stride + l];
            pos[i * stride + l] = cx
                .point(k)
                .ok_or_else(|| fatal(format!("node {k:?} has no position")))?;
        }
    }
    #[cfg(feature = "test-hooks")]
    if let Some(bend) = sp.bend {
        let (mut c, mut nb) = (Vector3::default(), 0.0f64);
        for i in (0..n_src).filter(|i| !interior[*i]) {
            let p = pos[i * stride];
            c = c + Vector3::new(p.x, p.y, p.z);
            nb += 1.0;
        }
        let c = c * (1.0 / nb.max(1.0));
        apply_bend(&mut pos, stride, Point3::new(c.x, c.y, c.z), bend);
    }

    // Place the interior layers.
    let b_idx: Vec<usize> = (0..n_src).filter(|i| !interior[*i]).collect();
    let i_idx: Vec<usize> = (0..n_src).filter(|i| interior[*i]).collect();
    let b_keys: Vec<NodeKey> = b_idx.iter().map(|i| keys[*i]).collect();
    let b_src: Vec<Point3> = b_idx.iter().map(|i| pos[*i * stride]).collect();
    let mut b_cols: Vec<Point3> = Vec::with_capacity(b_idx.len() * stride);
    for i in &b_idx {
        b_cols.extend_from_slice(&pos[*i * stride..(*i + 1) * stride]);
    }
    let i_keys: Vec<NodeKey> = i_idx.iter().map(|i| keys[*i]).collect();
    let i_pos: Vec<Point3> = i_idx.iter().map(|i| pos[*i * stride]).collect();

    // Elements and orientation (decided at layer 0).
    let mut elems = Elems {
        meta: Vec::with_capacity(src.elements.len()),
        ring: Vec::new(),
    };
    for (shape, nodes) in src.elements.iter() {
        let start = elems.ring.len() as u32;
        for l in nodes {
            let k = src
                .key(*l)
                .ok_or_else(|| fatal("source element index out of range".into()))?;
            let at = keys
                .binary_search(&k)
                .map_err(|_| fatal(format!("source element node {k:?} not in the node table")))?;
            elems.ring.push(at as u32);
        }
        elems.meta.push((shape, start, nodes.len() as u32));
    }
    let n_el = elems.meta.len();

    let modes: &[Placement] = match sp.placement {
        Placement::Auto => &[Placement::Auto, Placement::Interpolate],
        Placement::Affine => &[Placement::Affine],
        Placement::Interpolate => &[Placement::Interpolate],
    };
    let mut first_fail: Option<Fail> = None;
    let mut forward: Vec<bool> = Vec::new();
    let mut placed_ok = false;
    for (attempt, mode) in modes.iter().enumerate() {
        let placed = place_layers(
            &Boundary {
                keys: &b_keys,
                src: &b_src,
                cols: &b_cols,
                layers,
            },
            &Interior {
                keys: &i_keys,
                pos: &i_pos,
            },
            &PlaceOptions {
                placement: *mode,
                shepard_power: sp.shepard_power,
                support: sp.support as usize,
            },
        )
        .map_err(|e: SweepError| {
            GenerateFailure::new(target, name, FailureKind::Geometry, e.to_string())
        })?;
        for (ii, i) in i_idx.iter().enumerate() {
            for l in 1..layers {
                pos[*i * stride + l] = placed.positions[ii * (layers - 1) + l - 1];
            }
        }
        let pos_ref = &pos;
        forward = ordered(n_el, |e| {
            let ring = elems.nodes(e);
            let bot: Vec<Point3> = ring.iter().map(|i| pos_ref[*i as usize * stride]).collect();
            let up = ring.iter().fold(Vector3::default(), |a, i| {
                let i = *i as usize * stride;
                a + (pos_ref[i + 1] - pos_ref[i])
            });
            newell(&bot).dot(up) > 0.0
        });
        let fails = |forward: &[bool]| -> Option<Fail> {
            let check = |e: usize| {
                check_element(
                    e,
                    &elems,
                    forward[e],
                    &pos,
                    &lk,
                    stride,
                    layers,
                    sp.min_scaled_jacobian,
                )
            };
            if n_el >= PAR_MIN / 8 {
                (0..n_el)
                    .into_par_iter()
                    .map(check)
                    .find_first(Option::is_some)
                    .flatten()
            } else {
                (0..n_el).find_map(check)
            }
        };
        match fails(&forward) {
            None => {
                placed_ok = true;
                break;
            }
            Some(f) => {
                if attempt == 0 {
                    first_fail = Some(f);
                } else if first_fail.is_none() {
                    first_fail = Some(f);
                }
            }
        }
    }
    if !placed_ok {
        let f = first_fail.ok_or_else(|| fatal("quality gate failed without a record".into()))?;
        let reason = format!(
            "layer {}, source element {}: {}; nodes {:?}",
            f.layer, f.elem, f.what, f.nodes
        );
        return Err(
            GenerateFailure::new(target, name, FailureKind::Quality, reason).with_nodes(f.nodes),
        );
    }

    // Emit: created nodes, then references, then cells.
    let mut out = Patch::new(target);
    out.nodes.reserve(n_created);
    let mut local: Vec<u32> = vec![0; n_src * stride];
    let mut next_ref = n_created as u32;
    let mut slot = 0usize;
    for i in 0..n_src {
        for l in 0..stride {
            let created = interior[i] && l != 0 && l != layers;
            if created {
                local[i * stride + l] = (slot * (layers - 1) + l - 1) as u32;
            } else {
                local[i * stride + l] = next_ref;
                next_ref += 1;
                out.refs.push(lk[i * stride + l]);
            }
        }
        if interior[i] {
            slot += 1;
        }
    }
    // Created nodes are pushed in slot-major, layer order.
    for i in 0..n_src {
        if !interior[i] {
            continue;
        }
        for l in 1..layers {
            out.nodes.push(PatchNode::new(
                lk[i * stride + l],
                pos[i * stride + l],
                Param::None,
            ));
        }
    }
    let n_cells = n_el * layers;
    out.elements = sm_scheme_api::Elements::with_capacity(n_cells, 8);
    let mut buf: Vec<u32> = Vec::with_capacity(16);
    for e in 0..n_el {
        let (shape, _, _) = elems.meta[e];
        let ring = elems.nodes(e);
        let n = ring.len();
        let order: Vec<usize> = ring_order(n, forward[e]).collect();
        let at = |i: usize, layer: usize| local[ring[order[i]] as usize * stride + layer];
        let _ = shape;
        for l in 0..layers {
            if n <= 4 {
                buf.clear();
                for layer in [l, l + 1] {
                    buf.extend((0..n).map(|i| at(i, layer)));
                }
                let cell = if n == 4 {
                    CellShape::Hex
                } else {
                    CellShape::Wedge
                };
                out.elements.push(ElemShape::Cell(cell), &buf);
            } else {
                let bot: Vec<u32> = (0..n).rev().map(|i| at(i, l)).collect();
                let top: Vec<u32> = (0..n).map(|i| at(i, l + 1)).collect();
                let sides: Vec<[u32; 4]> = (0..n)
                    .map(|i| {
                        let j = (i + 1) % n;
                        [at(i, l), at(j, l), at(j, l + 1), at(i, l + 1)]
                    })
                    .collect();
                let mut faces: Vec<&[u32]> = vec![&bot, &top];
                faces.extend(sides.iter().map(|s| s.as_slice()));
                out.elements.push_polyhedron(&faces);
            }
        }
    }
    Ok(out)
}
