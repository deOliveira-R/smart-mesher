//! The file-wide decode budget: an upper estimate of what decoding each solid produces, computed
//! from the entity graph before any geometry is decoded.
//!
//! Every face, edge, shell and geometry entity is costed at most once (memoized by id), so the
//! estimate itself costs no more than a pass over the input. A solid's cost is the sum of its
//! shells' costs; the total is the sum over the solids of cost times placements. The estimate is
//! an upper bound of the cells a decode produces (faces, surfaces, coedges, edges, curves,
//! vertices) plus the size of the geometry it decodes, so a total within a multiple of the file's
//! own size keeps both the decode work and the decoded model proportional to the input.
//!
//! The walk mirrors the decoder but is tolerant: whatever it cannot follow costs nothing here and
//! fails with a typed error in the decoder.

use std::collections::BTreeMap;

use crate::args::{Args, Ents};
use crate::gcodec::Geo;
use crate::model::SolidKind;
use crate::part21::{Entity, Param, StepFile};

/// Depth limit when costing a geometry entity's references (decoders nest far less).
const GEOM_DEPTH: usize = 8;

/// The size of a parameter, counting every scalar and reference once.
fn param_size(p: &Param) -> usize {
    match p {
        Param::List(v) => v.iter().fold(1usize, |s, q| s.saturating_add(param_size(q))),
        Param::Typed(_, b) => 1usize.saturating_add(param_size(b)),
        _ => 1,
    }
}

fn entity_size(e: &Entity) -> usize {
    let recs: &[crate::part21::Rec] = match e {
        Entity::Simple(r) => std::slice::from_ref(r),
        Entity::Complex(v) => v,
    };
    recs.iter().flat_map(|r| r.params.iter()).fold(1usize, |s, p| s.saturating_add(param_size(p)))
}

/// The size of the whole file in the units of [`Estimator`]: one per entity plus one per scalar
/// or reference.
pub(crate) fn file_size(file: &StepFile) -> usize {
    file.entities.values().fold(0usize, |s, e| s.saturating_add(entity_size(e)))
}

fn refs_of(p: &Param, out: &mut Vec<u64>) {
    match p {
        Param::Ref(r) => out.push(*r),
        Param::List(v) => v.iter().for_each(|q| refs_of(q, out)),
        Param::Typed(_, b) => refs_of(b, out),
        _ => {}
    }
}

/// Memoized cost estimates over one file.
pub(crate) struct Estimator<'a> {
    ents: Ents<'a>,
    geom: BTreeMap<u64, usize>,
    edges: BTreeMap<u64, usize>,
    faces: BTreeMap<u64, usize>,
    shells: BTreeMap<u64, usize>,
}

impl<'a> Estimator<'a> {
    pub fn new(ents: Ents<'a>) -> Self {
        Self { ents, geom: BTreeMap::new(), edges: BTreeMap::new(), faces: BTreeMap::new(), shells: BTreeMap::new() }
    }

    /// Size of a geometry entity with what it references (control points, placements).
    fn geom(&mut self, id: u64, depth: usize) -> usize {
        if let Some(c) = self.geom.get(&id) {
            return *c;
        }
        let Some(e) = self.ents.map.get(&id) else { return 0 };
        // Placeholder while in progress: a cycle costs nothing more (the decoder rejects it).
        self.geom.insert(id, 1);
        let mut cost = entity_size(e);
        if depth < GEOM_DEPTH {
            let mut refs = Vec::new();
            let recs: &[crate::part21::Rec] = match e {
                Entity::Simple(r) => std::slice::from_ref(r),
                Entity::Complex(v) => v,
            };
            for r in recs {
                r.params.iter().for_each(|p| refs_of(p, &mut refs));
            }
            for r in refs {
                cost = cost.saturating_add(self.geom(r, depth + 1));
            }
        }
        self.geom.insert(id, cost);
        cost
    }

    /// Edge, its two vertices and its curve (wrappers skipped as the decoder does).
    fn edge(&mut self, id: u64) -> usize {
        if let Some(c) = self.edges.get(&id) {
            return *c;
        }
        let ents = self.ents;
        let mut cost = 3usize;
        if let Ok(a) = ents.rec(id, id, "EDGE_CURVE")
            && let Ok(c) = a.reference(3)
        {
            let geo = Geo { ents, scale: 1.0, angle: 1.0 };
            if let Ok(c) = geo.curve_ref(c, id) {
                cost = cost.saturating_add(self.geom(c, 0));
            }
        }
        self.edges.insert(id, cost);
        cost
    }

    /// A face with its surface, bounds, coedges and their edges.
    fn face(&mut self, id: u64) -> usize {
        if let Some(c) = self.faces.get(&id) {
            return *c;
        }
        self.faces.insert(id, 0);
        let ents = self.ents;
        let mut cost = 2usize;
        let mut pieces = 1usize;
        if let Ok(a) = ents.one_of(id, id, &["ADVANCED_FACE", "FACE_SURFACE", "FACE"]) {
            if a.rec.name != "FACE"
                && let Ok(s) = a.reference(2)
            {
                cost = cost.saturating_add(self.geom(s, 0));
                // A face on a closed NURBS surface may be cut into `PIECES` faces.
                if ents.map.get(&s).is_some_and(|e| e.is("B_SPLINE_SURFACE_WITH_KNOTS") || e.is("B_SPLINE_SURFACE")) {
                    pieces = crate::split::PIECES;
                }
            }
            for b in a.refs(1).unwrap_or_default() {
                cost = cost.saturating_add(1);
                let Ok(ba) = ents.one_of(b, id, &["FACE_OUTER_BOUND", "FACE_BOUND"]) else { continue };
                let Ok(l) = ba.reference(1) else { continue };
                let Ok(le) = ents.get(l, b) else { continue };
                if let Some(lrec) = le.rec("EDGE_LOOP") {
                    for oe in (Args { id: l, rec: lrec }).refs(1).unwrap_or_default() {
                        cost = cost.saturating_add(1);
                        if let Ok((e, _)) = crate::decode::oriented_edge(ents, oe, l) {
                            cost = cost.saturating_add(self.edge(e));
                        }
                    }
                } else if let Some(prec) = le.rec("POLY_LOOP") {
                    // A point gives a vertex, an edge and its line.
                    let n = (Args { id: l, rec: prec }).refs(1).map_or(0, |v| v.len());
                    cost = cost.saturating_add(n.saturating_mul(4));
                } else {
                    // `VERTEX_LOOP`: a synthesized pole vertex, seam edge and circle.
                    cost = cost.saturating_add(6);
                }
            }
        }
        cost = cost.saturating_mul(pieces);
        self.faces.insert(id, cost);
        cost
    }

    /// A shell: the sum over its face references.
    fn shell(&mut self, id: u64) -> usize {
        if let Some(c) = self.shells.get(&id) {
            return *c;
        }
        self.shells.insert(id, 0);
        let ents = self.ents;
        let mut cost = 1usize;
        if let Ok(a) = ents.one_of(id, id, &["CLOSED_SHELL", "OPEN_SHELL"]) {
            for f in a.refs(1).unwrap_or_default() {
                cost = cost.saturating_add(self.face(f));
            }
        }
        self.shells.insert(id, cost);
        cost
    }

    /// Upper estimate of what decoding solid `id` once produces.
    pub fn solid(&mut self, id: u64, kind: SolidKind) -> usize {
        let ents = self.ents;
        let shells: Vec<u64> = match kind {
            SolidKind::Sheet => ents.rec(id, id, "SHELL_BASED_SURFACE_MODEL").and_then(|a| a.refs(1)).unwrap_or_default(),
            SolidKind::Faceted => ents.rec(id, id, "FACETED_BREP").and_then(|a| a.reference(1)).ok().and_then(|s| crate::decode::shell_of(ents, s, id).ok()).map(|(s, _)| s).into_iter().collect(),
            _ => crate::decode::solid_shell_refs(ents, id, kind).map(|v| v.into_iter().map(|(s, _)| s).collect()).unwrap_or_default(),
        };
        shells.into_iter().fold(1usize, |c, s| c.saturating_add(self.shell(s)))
    }
}
