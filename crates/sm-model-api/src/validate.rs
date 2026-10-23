//! Validator for the semantic invariants of the model layer.
//!
//! Checks (all deterministic, issues sorted):
//! 1. **Coloring**: every volume has exactly one volume-role color (partition), colors exist with the
//!    right role, no stale entities, groups acyclic.
//! 2. **Boundary kinds**: every boundary face has a kind; deck/response/interface tags are placed on
//!    faces where they make sense.
//! 3. **Deck pairs**: mirror faces lie on the plane with normal parallel to it; paired sets satisfy
//!    `B = g(A)` (centroid, area, and the sign rule `n_B = -g(n_A)`), face by face, or in aggregate with
//!    a warning when the face counts differ.
//! 4. **Instancing**: references resolve, no cycles, prototype volumes exist, recolor targets are
//!    volume colors.
//! 5. **Decomposition**: pieces cover every volume and face, composites do not mix colors, whole
//!    faces belong to one piece, virtual cuts are shared by two pieces, measures add up.

use crate::boundary::{BoundaryKind, DeckSide, DeckSides, InterfaceIntent};
use crate::coloring::ColorRole;
use crate::decomposition::{Decomposition, PieceFace, PieceShape};
use crate::group::DeckKind;
use crate::model::Model;
use crate::symmetry::{DomainState, deck_partner};
use crate::topo::GeometryView;
use crate::xf::Tolerance;
use sm_core::EntityId;
use std::collections::{BTreeMap, BTreeSet};

/// Severity of an issue.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Suspicious but usable.
    Warning,
    /// Violates an invariant.
    Error,
}

/// Machine-readable issue code.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum IssueCode {
    /// A topology volume has no color.
    UncoloredVolume,
    /// A volume/face is referenced by the model but absent from topology.
    StaleEntity,
    /// A color is unknown or has the wrong role for its use.
    BadColor,
    /// Group nesting is cyclic or a member is missing.
    BadGroup,
    /// A boundary face has no boundary kind.
    BoundaryFaceWithoutKind,
    /// A deck tag sits on an interior face.
    DeckOnInteriorFace,
    /// A deck record references a generator the symmetry does not have, or the family mismatches.
    DeckGeneratorMismatch,
    /// A mirror face is not on the mirror plane (or not parallel to it).
    MirrorFaceOffPlane,
    /// A paired face has no partner at `g(face)`.
    DeckPairUnmatched,
    /// Partners exist but their outward normals do not satisfy `n_B = -g(n_A)`.
    DeckNormalsNotOpposed,
    /// Paired sets differ in face count and only aggregate area/centroid were compared.
    DeckPairAggregateOnly,
    /// Paired sets differ in total area or centroid.
    DeckPairAggregateMismatch,
    /// A generator of a reduced model identifies no faces.
    GeneratorWithoutDeckFaces,
    /// An interface tag is inconsistent with the faces' sides.
    BadInterface,
    /// Instancing references are dangling or cyclic.
    BadInstancing,
    /// A prototype volume is absent from topology.
    PrototypeVolumeMissing,
    /// A volume has no piece.
    VolumeUncovered,
    /// A model face of a volume is touched by none of its pieces.
    FaceUncovered,
    /// A piece references a missing volume or a face not on its volume.
    BadPiece,
    /// A composite piece mixes volume colors.
    CompositeMixedColors,
    /// A face marked whole belongs to more than one piece.
    WholeFaceShared,
    /// A virtual cut is not shared by exactly two pieces.
    CutNotShared,
    /// Sweep roles reference faces the piece does not have.
    SweepRoleNotInPiece,
    /// Piece measures do not add up to the volume measure.
    MeasureMismatch,
}

/// One validation finding.
#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    /// Severity.
    pub severity: Severity,
    /// Code.
    pub code: IssueCode,
    /// Primary entity involved, if any.
    pub subject: Option<EntityId>,
    /// Human-readable detail.
    pub message: String,
}

/// Validation result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// Findings, sorted by (severity descending, code, subject).
    pub issues: Vec<Issue>,
}

impl Report {
    /// True if there are no errors (warnings allowed).
    pub fn is_ok(&self) -> bool {
        !self.issues.iter().any(|i| i.severity == Severity::Error)
    }
    /// True if some issue has `code`.
    pub fn has(&self, code: IssueCode) -> bool {
        self.issues.iter().any(|i| i.code == code)
    }
    /// Issues with `code`.
    pub fn with_code(&self, code: IssueCode) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(move |i| i.code == code)
    }
    fn push(&mut self, severity: Severity, code: IssueCode, subject: Option<EntityId>, message: String) {
        self.issues.push(Issue { severity, code, subject, message });
    }
    fn err(&mut self, code: IssueCode, subject: Option<EntityId>, message: String) {
        self.push(Severity::Error, code, subject, message);
    }
}

/// Validate `model` against `topo` (and `decomposition` if given).
pub fn validate(topo: &dyn GeometryView, model: &Model, decomposition: Option<&Decomposition>, tol: &Tolerance) -> Report {
    let mut r = Report::default();
    check_coloring(topo, model, &mut r);
    check_boundaries(topo, model, tol, &mut r);
    check_symmetry(model, &mut r);
    check_instancing(topo, model, &mut r);
    if let Some(d) = decomposition {
        check_decomposition(topo, model, d, tol, &mut r);
    }
    r.issues.sort_by(|a, b| b.severity.cmp(&a.severity).then(a.code.cmp(&b.code)).then(a.subject.cmp(&b.subject)).then(a.message.cmp(&b.message)));
    r
}

fn check_coloring(topo: &dyn GeometryView, model: &Model, r: &mut Report) {
    let c = &model.coloring;
    let vols: BTreeSet<EntityId> = topo.volumes().into_iter().collect();
    let faces: BTreeSet<EntityId> = topo.faces().into_iter().collect();
    for &v in &vols {
        if c.volume_color(v).is_none() {
            r.err(IssueCode::UncoloredVolume, Some(v), format!("volume {v} has no color"));
        }
    }
    for (v, col) in c.volume_assignments() {
        if !vols.contains(&v) {
            r.err(IssueCode::StaleEntity, Some(v), format!("colored volume {v} not in topology"));
        }
        if c.color_info(col).map(|i| i.role) != Some(ColorRole::Volume) {
            r.err(IssueCode::BadColor, Some(v), format!("volume {v} has color {col} that is unknown or not a volume color"));
        }
    }
    for (f, col) in c.face_assignments() {
        if !faces.contains(&f) {
            r.err(IssueCode::StaleEntity, Some(f), format!("colored face {f} not in topology"));
        }
        if c.color_info(col).map(|i| i.role) != Some(ColorRole::Boundary) {
            r.err(IssueCode::BadColor, Some(f), format!("face {f} has color {col} that is unknown or not a boundary color"));
        }
    }
    for (g, grp) in c.groups() {
        if c.has_group_cycle(g) {
            r.err(IssueCode::BadGroup, Some(g), format!("group `{}` is cyclic", grp.name));
        }
        for &m in &grp.members {
            let ok = match m.kind() {
                sm_core::EntityKind::TopoVolume => vols.contains(&m),
                sm_core::EntityKind::TopoFace => faces.contains(&m),
                sm_core::EntityKind::Color => c.color_info(m).is_some(),
                sm_core::EntityKind::Group => c.group(m).is_some(),
                _ => false,
            };
            if !ok {
                r.err(IssueCode::BadGroup, Some(g), format!("group `{}` has missing member {m}", grp.name));
            }
        }
    }
}

fn check_boundaries(topo: &dyn GeometryView, model: &Model, tol: &Tolerance, r: &mut Report) {
    let b = &model.boundaries;
    let faces: BTreeSet<EntityId> = topo.faces().into_iter().collect();
    for f in topo.boundary_faces() {
        if b.kind_of(f).is_none() {
            r.err(IssueCode::BoundaryFaceWithoutKind, Some(f), format!("boundary face {f} has no kind"));
        }
    }
    for (f, k) in b.iter() {
        if !faces.contains(&f) {
            r.err(IssueCode::StaleEntity, Some(f), format!("tagged face {f} not in topology"));
            continue;
        }
        let Some(sides) = topo.face_sides(f) else { continue };
        match k {
            BoundaryKind::Deck { .. } if !sides.is_boundary() => {
                r.err(IssueCode::DeckOnInteriorFace, Some(f), format!("deck face {f} is interior"));
            }
            BoundaryKind::Interface(InterfaceIntent::BetweenColors) => {
                match (sides.fwd, sides.rev) {
                    (Some(a), Some(c)) => {
                        if model.coloring.volume_color(a) == model.coloring.volume_color(c) {
                            r.push(Severity::Warning, IssueCode::BadInterface, Some(f), format!("interface {f} separates volumes of the same color"));
                        }
                    }
                    _ => r.err(IssueCode::BadInterface, Some(f), format!("BetweenColors interface {f} is on a boundary face")),
                }
            }
            BoundaryKind::Interface(InterfaceIntent::NonConformal { partner, .. })
                if (partner.is_empty() || partner.iter().any(|p| !faces.contains(p))) => {
                    r.err(IssueCode::BadInterface, Some(f), format!("non-conformal interface {f} has missing partner faces"));
                }
            _ => {}
        }
    }
    for d in b.decks() {
        let Some(gen_) = model.symmetry.as_ref().and_then(|s| s.generator(d.generator)) else {
            r.err(IssueCode::DeckGeneratorMismatch, Some(d.id), format!("deck {} references unknown generator {}", d.id, d.generator));
            continue;
        };
        match (&d.sides, gen_.op.kind()) {
            (DeckSides::SelfPaired(set), DeckKind::Mirror) => {
                let Ok(plane) = (match gen_.op {
                    crate::group::DeckOp::Mirror { plane } => Ok(plane),
                    _ => Err(()),
                }) else { continue };
                for &f in set {
                    let (Some(c), Some(n)) = (topo.face_centroid(f), topo.face_normal(f)) else { continue };
                    if plane.signed_distance(c).abs() > tol.linear || n.cross(plane.normal()).norm() > tol.angular {
                        r.err(IssueCode::MirrorFaceOffPlane, Some(f), format!("mirror face {f} is not on the plane"));
                    }
                }
            }
            (DeckSides::Paired { a, b: bset }, DeckKind::Rotation | DeckKind::Translation) => {
                check_pairs(topo, d.id, &gen_.op, a, bset, tol, r);
            }
            _ => r.err(IssueCode::DeckGeneratorMismatch, Some(d.id), format!("deck {} face sets do not fit a {:?} generator", d.id, gen_.op.kind())),
        }
    }
}

fn check_pairs(topo: &dyn GeometryView, deck: EntityId, op: &crate::group::DeckOp, a: &BTreeSet<EntityId>, b: &BTreeSet<EntityId>, tol: &Tolerance, r: &mut Report) {
    let x = op.transform();
    let sides = DeckSides::Paired { a: a.clone(), b: b.clone() };
    let mut matched = BTreeSet::new();
    let mut failed = false;
    for &fa in a {
        let (Some(ca), Some(aa), Some(na)) = (topo.face_centroid(fa), topo.face_area(fa), topo.face_normal(fa)) else {
            failed = true;
            continue;
        };
        match deck_partner(topo, op, &sides, DeckSide::A, fa, tol) {
            Some(fb) => {
                let (ab, nb) = (topo.face_area(fb), topo.face_normal(fb));
                let area_ok = ab.is_some_and(|ab| tol.close(aa, ab));
                let opp = nb.is_some_and(|nb| (nb + x.apply_vector(na)).norm() <= tol.angular);
                if !area_ok {
                    failed = true;
                    if a.len() == b.len() {
                        r.err(IssueCode::DeckPairUnmatched, Some(fa), format!("face {fa} and its image {fb} differ in area"));
                    }
                } else if !opp {
                    failed = true;
                    r.err(IssueCode::DeckNormalsNotOpposed, Some(fa), format!("face {fa}: n_B != -g(n_A) at {fb}"));
                }
                matched.insert(fb);
            }
            None => {
                failed = true;
                if a.len() == b.len() {
                    r.err(IssueCode::DeckPairUnmatched, Some(fa), format!("no partner for face {fa} (centroid {ca:?}) in B"));
                }
            }
        }
    }
    if !failed && matched.len() == b.len() {
        return;
    }
    if a.len() != b.len() && !r.has(IssueCode::DeckNormalsNotOpposed) {
        // aggregate fallback: total area and area-weighted centroid
        let agg = |set: &BTreeSet<EntityId>| -> Option<(f64, [f64; 3])> {
            let (mut area, mut c) = (0.0, [0.0; 3]);
            for &f in set {
                let (fa, fc) = (topo.face_area(f)?, topo.face_centroid(f)?);
                area += fa;
                c[0] += fa * fc.x;
                c[1] += fa * fc.y;
                c[2] += fa * fc.z;
            }
            Some((area, c))
        };
        if let (Some((aa, ac)), Some((ab, bc))) = (agg(a), agg(b)) {
            let img = x.apply_point(sm_core::Point3::new(ac[0] / aa, ac[1] / aa, ac[2] / aa));
            let cb = sm_core::Point3::new(bc[0] / ab, bc[1] / ab, bc[2] / ab);
            if tol.close(aa, ab) && (img - cb).norm() <= tol.linear + tol.relative * 10.0 {
                r.push(Severity::Warning, IssueCode::DeckPairAggregateOnly, Some(deck), format!("deck {deck}: face counts differ ({} vs {}); only aggregate area/centroid match", a.len(), b.len()));
            } else {
                r.err(IssueCode::DeckPairAggregateMismatch, Some(deck), format!("deck {deck}: paired sets differ in area or centroid"));
            }
            return;
        }
    }
    if a.len() == b.len() && matched.len() != b.len() && !failed {
        r.err(IssueCode::DeckPairUnmatched, Some(deck), format!("deck {deck}: partners are not a bijection"));
    }
}

fn check_symmetry(model: &Model, r: &mut Report) {
    let Some(s) = &model.symmetry else { return };
    if s.domain.state == DomainState::Reduced {
        for g in &s.generators {
            if !model.boundaries.decks().any(|d| d.generator == g.id) {
                r.push(Severity::Warning, IssueCode::GeneratorWithoutDeckFaces, Some(g.id), format!("generator {} of a reduced model identifies no faces", g.id));
            }
        }
    }
    if s.group().is_err() {
        r.err(IssueCode::DeckGeneratorMismatch, None, "deck group does not close".into());
    }
}

fn check_instancing(topo: &dyn GeometryView, model: &Model, r: &mut Report) {
    if let Err(e) = model.instancing.check_references() {
        r.err(IssueCode::BadInstancing, None, e.to_string());
    }
    let vols: BTreeSet<EntityId> = topo.volumes().into_iter().collect();
    for p in model.instancing.prototypes.values() {
        for v in &p.volumes {
            if !vols.contains(v) {
                r.err(IssueCode::PrototypeVolumeMissing, Some(*v), format!("prototype `{}` volume {v} not in topology", p.name));
            }
        }
    }
    let mut recolors = Vec::new();
    for i in model.instancing.instances.values() {
        recolors.push(&i.placed.recolor);
    }
    for l in model.instancing.lattices.values() {
        recolors.push(&l.default.recolor);
        recolors.extend(l.slots.values().map(|p| &p.recolor));
    }
    for rc in recolors {
        for &t in rc.by_volume.values().chain(rc.by_color.values()) {
            if model.coloring.color_info(t).map(|i| i.role) != Some(ColorRole::Volume) {
                r.err(IssueCode::BadColor, Some(t), format!("recolor target {t} is not a volume color"));
            }
        }
    }
}

fn check_decomposition(topo: &dyn GeometryView, model: &Model, d: &Decomposition, tol: &Tolerance, r: &mut Report) {
    let vols = topo.volumes();
    let volset: BTreeSet<EntityId> = vols.iter().copied().collect();
    let mut cut_users: BTreeMap<crate::decomposition::CutId, usize> = BTreeMap::new();
    let mut whole_users: BTreeMap<(EntityId, EntityId), usize> = BTreeMap::new();
    let mut touched: BTreeSet<EntityId> = BTreeSet::new();
    for p in d.pieces() {
        let mut colors = BTreeSet::new();
        for &v in &p.volumes {
            if !volset.contains(&v) {
                r.err(IssueCode::BadPiece, Some(p.id), format!("piece {} references unknown volume {v}", p.id));
                continue;
            }
            colors.insert(model.coloring.volume_color(v));
            let vf: BTreeSet<EntityId> = topo.volume_faces(v).into_iter().collect();
            for pf in &p.faces {
                if let PieceFace::Model { face, .. } = pf
                    && p.volumes.len() == 1 && !vf.contains(face) {
                        r.err(IssueCode::BadPiece, Some(p.id), format!("piece {} touches face {face} not on its volume", p.id));
                    }
            }
        }
        if colors.len() > 1 {
            r.err(IssueCode::CompositeMixedColors, Some(p.id), format!("piece {} mixes volume colors", p.id));
        }
        for pf in &p.faces {
            match pf {
                PieceFace::Model { face, whole } => {
                    touched.insert(*face);
                    if *whole {
                        for &v in &p.volumes {
                            *whole_users.entry((*face, v)).or_insert(0) += 1;
                        }
                    }
                }
                PieceFace::Virtual(c) => *cut_users.entry(*c).or_insert(0) += 1,
            }
        }
        if let PieceShape::Sweep(roles) = &p.shape {
            for f in roles.sources.iter().chain(&roles.targets) {
                if !p.faces.contains(f) {
                    r.err(IssueCode::SweepRoleNotInPiece, Some(p.id), format!("sweep role face {f:?} not on piece {}", p.id));
                }
            }
        }
    }
    for &v in &vols {
        if d.pieces_of_volume(v).is_empty() {
            r.err(IssueCode::VolumeUncovered, Some(v), format!("volume {v} has no piece"));
            continue;
        }
        for f in topo.volume_faces(v) {
            if !touched.contains(&f) {
                r.err(IssueCode::FaceUncovered, Some(f), format!("face {f} of volume {v} is touched by no piece"));
            }
        }
        let ps = d.pieces_of_volume(v);
        let ms: Option<Vec<f64>> = ps.iter().map(|&p| d.piece(p).and_then(|p| if p.volumes.len() == 1 { p.measure } else { None })).collect();
        if let (Some(ms), Some(m)) = (ms, topo.volume_moments(v)) {
            let sum: f64 = ms.iter().sum();
            if !tol.close(sum, m.measure) {
                r.err(IssueCode::MeasureMismatch, Some(v), format!("pieces of {v} sum to {sum}, volume is {}", m.measure));
            }
        }
    }
    for ((f, v), n) in whole_users {
        if n > 1 {
            r.err(IssueCode::WholeFaceShared, Some(f), format!("face {f} is whole in {n} pieces of volume {v}"));
        }
    }
    for (c, n) in cut_users {
        if n != 2 {
            r.err(IssueCode::CutNotShared, None, format!("virtual cut {c:?} is used by {n} pieces, expected 2"));
        }
    }
}
