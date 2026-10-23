//! Validator for the semantic invariants of the model layer.
//!
//! Checks: coloring is an exact partition of the domain volumes; every boundary face of the
//! domain has a kind; interface intents agree with the colors; deck pairs are geometrically
//! consistent within tolerance; symmetry and instancing records are well-formed; the
//! decomposition tiles the domain and leaves coloring untouched. Output order is
//! deterministic (code, then entities).

use crate::action::{DeckKind, Generator};
use crate::boundary::{BoundaryKind, Conformity};
use crate::model::{GroupMembers, Model, Tolerance};
use crate::symmetry::{SymmetryOrigin, Verification};
use crate::topology::TopologyView;
use sm_core::EntityId;
use std::collections::{BTreeMap, BTreeSet};

/// How bad an issue is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Violates an invariant; downstream layers must not run.
    Error,
    /// Suspicious but legal.
    Warning,
}

/// Machine-readable issue code.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum IssueCode {
    /// A referenced entity does not exist in the topology or the model.
    UnknownEntity,
    /// A domain volume has no color.
    VolumeUncolored,
    /// A volume is in more than one color.
    VolumeMultiColor,
    /// A color has no members.
    EmptyColor,
    /// A boundary face has no boundary color.
    FaceUncolored,
    /// A boundary face of the domain has no kind.
    BoundaryFaceWithoutKind,
    /// A deck kind sits on a face that is not on the domain boundary.
    DeckOnInteriorFace,
    /// An interface's colors do not match the volumes on its sides.
    InterfaceColorMismatch,
    /// Paired deck faces do not map onto each other.
    DeckPairGeometry,
    /// Paired deck faces touch volumes of different colors.
    DeckPairColor,
    /// Deck pairing and face kinds disagree.
    DeckPairingIncomplete,
    /// The symmetry was rejected by verification or is incomplete.
    SymmetryInvalid,
    /// An instancing record is inconsistent (cycle, bad override, unknown volume).
    InstancingInvalid,
    /// Pieces do not tile the domain.
    DecompositionTiling,
    /// Coloring changed since the decomposition was made.
    DecompositionStale,
}

/// One finding.
#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    /// Severity.
    pub severity: Severity,
    /// Code.
    pub code: IssueCode,
    /// Entities involved, ascending.
    pub entities: Vec<EntityId>,
    /// Human-readable message.
    pub message: String,
}

/// The validator output.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// Issues, sorted by (code, entities).
    pub issues: Vec<Issue>,
}

impl Report {
    /// True if there are no errors (warnings allowed).
    pub fn is_ok(&self) -> bool {
        !self.issues.iter().any(|i| i.severity == Severity::Error)
    }
    /// Does any issue carry `code`?
    pub fn has(&self, code: IssueCode) -> bool {
        self.issues.iter().any(|i| i.code == code)
    }
    /// Issues with `code`.
    pub fn with_code(&self, code: IssueCode) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(move |i| i.code == code)
    }
}

struct Sink(Vec<Issue>);

impl Sink {
    fn push(&mut self, severity: Severity, code: IssueCode, mut entities: Vec<EntityId>, message: impl Into<String>) {
        entities.sort_unstable();
        self.0.push(Issue { severity, code, entities, message: message.into() });
    }
    fn err(&mut self, code: IssueCode, entities: Vec<EntityId>, message: impl Into<String>) {
        self.push(Severity::Error, code, entities, message);
    }
    fn warn(&mut self, code: IssueCode, entities: Vec<EntityId>, message: impl Into<String>) {
        self.push(Severity::Warning, code, entities, message);
    }
}

impl Model {
    /// Validate every semantic invariant against `topo`.
    pub fn validate(&self, topo: &dyn TopologyView, tol: &Tolerance) -> Report {
        validate(self, topo, tol)
    }
}

/// Validate `model` against `topo`. See the module docs.
pub fn validate(model: &Model, topo: &dyn TopologyView, tol: &Tolerance) -> Report {
    let mut s = Sink(Vec::new());
    let all_vols: BTreeSet<EntityId> = topo.volumes().into_iter().collect();
    let all_faces: BTreeSet<EntityId> = topo.faces().into_iter().collect();
    let domain = model.domain_volumes(topo);
    let boundary = model.domain_boundary_faces(topo);
    let boundary_set: BTreeSet<EntityId> = boundary.iter().copied().collect();

    // Coloring: exact partition of the domain volumes.
    let mut count: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for c in model.volume_colors() {
        if c.selection.is_empty() {
            s.warn(IssueCode::EmptyColor, vec![c.id], format!("color {:?} has no volumes", c.name));
        }
        for v in c.selection.iter() {
            if !all_vols.contains(&v) {
                s.err(IssueCode::UnknownEntity, vec![c.id, v], format!("color {:?} refers to a missing volume", c.name));
            }
            count.entry(v).or_default().push(c.id);
        }
    }
    for v in &domain {
        match count.get(v).map_or(0, Vec::len) {
            0 => s.err(IssueCode::VolumeUncolored, vec![*v], "volume has no color"),
            1 => {}
            _ => s.err(IssueCode::VolumeMultiColor, vec![*v], "volume has several colors"),
        }
    }
    for c in model.face_colors() {
        for f in c.selection.iter().filter(|f| !all_faces.contains(f)) {
            s.err(IssueCode::UnknownEntity, vec![c.id, f], format!("boundary color {:?} refers to a missing face", c.name));
        }
    }
    for f in boundary.iter().filter(|f| model.face_color_of(**f).is_none()) {
        s.warn(IssueCode::FaceUncolored, vec![*f], "boundary face has no boundary color");
    }
    for g in model.groups() {
        let (missing, what): (Vec<EntityId>, &str) = match &g.members {
            GroupMembers::Volumes(x) => (x.iter().filter(|v| !all_vols.contains(v)).collect(), "volume"),
            GroupMembers::Faces(x) => (x.iter().filter(|f| !all_faces.contains(f)).collect(), "face"),
            GroupMembers::Groups(_) => (Vec::new(), "group"),
        };
        for m in missing {
            s.err(IssueCode::UnknownEntity, vec![g.id, m], format!("group {:?} refers to a missing {what}", g.name));
        }
    }

    // Boundary kinds.
    for f in &boundary {
        if model.kind_of_face(*f).is_none() {
            s.err(IssueCode::BoundaryFaceWithoutKind, vec![*f], "boundary face has no kind (deck, response or interface)");
        }
    }
    for ks in model.kind_sets() {
        for f in ks.faces.iter() {
            if !all_faces.contains(&f) {
                s.err(IssueCode::UnknownEntity, vec![ks.id, f], "kind set refers to a missing face");
                continue;
            }
            if model.domain_sides(f, topo).is_none() {
                continue; // cut away with its volumes
            }
            match &ks.kind {
                BoundaryKind::Deck { .. } if !boundary_set.contains(&f) => {
                    s.err(IssueCode::DeckOnInteriorFace, vec![f], "deck kind on a face that is interior to the domain");
                }
                BoundaryKind::Interface(intent) => interface_check(model, topo, f, intent.between, &intent.conformity, &mut s),
                _ => {}
            }
        }
    }

    // Deck pairings.
    for p in model.pairings() {
        let g = p.generator;
        if let Err(e) = g.validate() {
            s.err(IssueCode::DeckPairGeometry, vec![p.id], e.to_string());
            continue;
        }
        let tr = g.transform();
        for &(a, b) in &p.pairs {
            let (Some(pa), Some(pb)) = (topo.face_properties(a), topo.face_properties(b)) else {
                s.err(IssueCode::UnknownEntity, vec![p.id, a, b], "deck pair refers to a missing face");
                continue;
            };
            for f in [a, b] {
                match model.kind_of_face(f) {
                    Some(BoundaryKind::Deck { pairing, .. }) if *pairing == p.id => {}
                    _ => s.err(IssueCode::DeckPairingIncomplete, vec![p.id, f], "paired face does not carry this deck kind"),
                }
            }
            let (Some(na), Some(nb)) = (model.outward_normal(a, topo), model.outward_normal(b, topo)) else {
                s.err(IssueCode::DeckOnInteriorFace, vec![p.id, a, b], "deck face is not on the domain boundary");
                continue;
            };
            let ok = match g {
                Generator::Mirror { plane } => {
                    a == b && plane.signed_distance(pa.centroid).abs() <= tol.length && na.dot(plane.normal).abs() >= 1.0 - tol.angle
                }
                _ => {
                    (tr.apply_point(pa.centroid) - pb.centroid).norm() <= tol.length
                        && (pa.area - pb.area).abs() <= tol.relative * pa.area.max(pb.area)
                        && nb.dot(tr.apply_vector(na)) <= -(1.0 - tol.angle)
                }
            };
            if !ok {
                s.err(
                    IssueCode::DeckPairGeometry,
                    vec![p.id, a, b],
                    format!("{:?} pair does not map under the generator within tolerance", g.kind()),
                );
            }
            let color = |f: EntityId| model.domain_sides(f, topo).and_then(|d| model.volume_color_of(d.near));
            if color(a) != color(b) {
                s.err(IssueCode::DeckPairColor, vec![p.id, a, b], "paired deck faces touch different volume colors");
            }
        }
    }

    // Symmetry.
    if let Some(sym) = model.symmetry() {
        if let SymmetryOrigin::Detected { verification: Verification::Rejected { .. } } = sym.origin {
            s.err(IssueCode::SymmetryInvalid, vec![], "symmetry was adopted from a rejected candidate");
        }
        for (i, g) in sym.generators.iter().enumerate() {
            if let Some(p) = g.pairing {
                match model.pairing(p) {
                    Some(pr) if pr.generator.kind() == g.generator.kind() => {}
                    _ => s.err(IssueCode::SymmetryInvalid, vec![p], format!("generator {i} has a missing or mismatched pairing")),
                }
            }
            if g.generator.kind() == DeckKind::Translation && g.repeat.is_none() {
                s.warn(IssueCode::SymmetryInvalid, vec![], format!("translation generator {i} has no repeat count; unfold will fail"));
            }
        }
    }

    // Instancing.
    let inst = model.instancing();
    for p in inst.prototypes() {
        for v in p.volumes.iter().filter(|v| !all_vols.contains(v)) {
            s.err(IssueCode::InstancingInvalid, vec![p.id, v], "prototype refers to a missing volume");
        }
    }
    let known_color = |c: EntityId| model.volume_color(c).is_some() || model.face_color(c).is_some();
    let check_map = |owner: EntityId, m: &crate::instancing::ColorMap, s: &mut Sink| {
        for (a, b) in m.iter() {
            for c in [a, b].into_iter().filter(|c| !known_color(*c)) {
                s.err(IssueCode::InstancingInvalid, vec![owner, c], "override refers to an unknown color");
            }
        }
    };
    for l in inst.lattices() {
        for f in l.slots.values().flatten().chain(l.default_fill.iter()) {
            check_map(l.id, &f.overrides, &mut s);
        }
    }
    for i in inst.instances() {
        check_map(i.id, &i.fill.overrides, &mut s);
    }
    if let Err(e) = inst.flatten_all(1 << 40) {
        s.err(IssueCode::InstancingInvalid, vec![], e.to_string());
    }

    // Decomposition.
    if let Some(d) = model.decomposition() {
        for (v, msg) in d.check_tiling(&domain) {
            s.err(IssueCode::DecompositionTiling, vec![v], msg);
        }
        if d.coloring_fingerprint != model.coloring_fingerprint() {
            s.warn(IssueCode::DecompositionStale, vec![], "coloring changed after the decomposition was made");
        }
    }

    s.0.sort_by(|a, b| (a.code, &a.entities, a.severity).cmp(&(b.code, &b.entities, b.severity)));
    Report { issues: s.0 }
}

fn interface_check(model: &Model, topo: &dyn TopologyView, f: EntityId, between: Option<(EntityId, EntityId)>, conf: &Conformity, s: &mut Sink) {
    if let Conformity::NonConformal { partner: Some(g) } = conf
        && model.groups().all(|x| x.id != *g)
    {
        s.err(IssueCode::UnknownEntity, vec![f, *g], "non-conformal partner group does not exist");
    }
    let Some((c1, c2)) = between else { return };
    let Some(sides) = topo.face_sides(f) else { return };
    let col = |v: Option<EntityId>| v.and_then(|v| model.volume_color_of(v));
    let (a, b) = (col(sides.left), col(sides.right));
    let ok = (a == Some(c1) && b == Some(c2)) || (a == Some(c2) && b == Some(c1));
    if !ok {
        s.err(IssueCode::InterfaceColorMismatch, vec![f, c1, c2], "interface colors do not match the volumes on the two sides");
    }
}
