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

/// Depth limit when costing a geometry entity's references: the curve decoder refuses nesting
/// deeper than 8 (`gcodec::curve_at`), and the others nest far less.
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
    /// Keyed by id and depth: the cost below depth `d` is truncated at [`GEOM_DEPTH`], so an
    /// entity first reached deep must not lend its truncated cost to a shallow reference.
    geom: BTreeMap<(u64, usize), usize>,
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
        if let Some(c) = self.geom.get(&(id, depth)) {
            return *c;
        }
        let Some(e) = self.ents.map.get(&id) else { return 0 };
        // Placeholder while in progress: a cycle costs nothing more (the decoder rejects it).
        self.geom.insert((id, depth), 1);
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
        self.geom.insert((id, depth), cost);
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
                // A face on a closed NURBS surface may be cut into `PIECES` faces. The split
                // itself costs a constant per coedge plus a few global projections (each a
                // constant times the surface size), so the surface needs no per-coedge charge.
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Cost of every solid of `bytes` over the file size.
    fn ratio(bytes: &[u8]) -> f64 {
        let file = crate::part21::parse(bytes).expect("parse");
        let mut est = Estimator::new(Ents::new(&file));
        let cost = crate::decode::solid_entities(&file).into_iter().fold(0usize, |c, (id, kind)| c + est.solid(id, kind));
        cost as f64 / file_size(&file) as f64
    }

    /// Legitimate files stay well inside the `8 x file size` budget (the loft, with its x4
    /// split allowance, is the largest).
    #[test]
    fn legitimate_fixtures_cost_a_few_times_their_size() {
        for (k, f) in sm_testkit::step::ALL.iter().enumerate() {
            let r = ratio(f.bytes);
            eprintln!("testkit fixture {k}: {r:.2}");
            assert!(r > 0.0 && r < 4.0, "fixture {k}: {r}");
        }
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
        let mut names: Vec<_> = std::fs::read_dir(&dir).expect("tests/data").filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "stp")).collect();
        names.sort();
        for p in names {
            let r = ratio(&std::fs::read(&p).expect("read"));
            eprintln!("{}: {r:.2}", p.display());
            assert!(r < 4.0, "{}: {r}", p.display());
        }
    }

    /// A geometry entity reached first deep in a reference chain is costed in full when it is
    /// later reached from the top (the memo is keyed by depth).
    #[test]
    fn geometry_cost_does_not_depend_on_visit_order() {
        let mut text = String::from("ISO-10303-21;HEADER;ENDSEC;DATA;\n#1=CARTESIAN_POINT('',(0.,0.,0.));\n#2=CARTESIAN_POINT('',(1.,0.,0.));\n#3=DIRECTION('',(1.,0.,0.));\n#4=VECTOR('',#3,1.);\n#5=LINE('',#1,#4);\n");
        for k in 6..=14 {
            text += &format!("#{k}=TRIMMED_CURVE('',#{},(#1),(#2),.T.,.CARTESIAN.);\n", k - 1);
        }
        text += "ENDSEC;END-ISO-10303-21;\n";
        let file = crate::part21::parse(text.as_bytes()).expect("parse");
        let fresh = Estimator::new(Ents::new(&file)).geom(6, 0);
        let mut est = Estimator::new(Ents::new(&file));
        // #6 is first reached from #14 at depth 8, where its references are cut off.
        est.geom(14, 0);
        assert_eq!(est.geom(6, 0), fresh);
        assert!(fresh > entity_size(&file.entities[&6]));
    }
}
