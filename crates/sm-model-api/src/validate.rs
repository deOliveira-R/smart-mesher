//! Validator for the semantic invariants of the model tables against a topology.
//!
//! Checks: volume coloring is an exact partition, every boundary face has a kind, deck pairs are
//! geometrically consistent within tolerance, the symmetry record is sound, instancing
//! references resolve without cycles, and a decomposition preserves colors. The validator
//! never mutates and reports **all** issues, in a deterministic order.

use crate::action::{Placed, Plane, inverse};
use crate::boundary::{BoundaryKind, Deck, InterfaceIntent, Side};
use crate::color::ColorDomain;
use crate::decomposition::{CutSurface, PieceFace, PieceShape};
use crate::instancing::{Fill, Member, Override};
use crate::model::Model;
use crate::symmetry::UnfoldedModel;
use crate::topology::{Tolerance, TopologyView};
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::BTreeSet;

/// Machine-readable issue class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum IssueCode {
    /// A topology volume has no color.
    VolumeUncolored,
    /// A table row refers to an entity that the topology (or a table) does not have.
    DanglingReference,
    /// A color is used in the wrong domain.
    ColorDomain,
    /// A group refers to a missing member or is cyclic.
    GroupInvalid,
    /// A boundary face (one volume side) has no boundary kind.
    BoundaryFaceWithoutKind,
    /// A kind is attached to a face where it cannot apply (e.g. a deck on an interior face).
    KindMisplaced,
    /// A deck pair is missing a mate or the mates disagree.
    DeckPairing,
    /// A deck pair or mirror face does not satisfy its transform within tolerance.
    DeckGeometry,
    /// Deck mates differ in face color or in adjacent volume color.
    DeckColor,
    /// A deck record has invalid parameters.
    DeckParameters,
    /// The symmetry record is inconsistent.
    SymmetryInvalid,
    /// An instancing reference is unresolved, cyclic or out of range.
    InstancingInvalid,
    /// A piece is malformed.
    PieceInvalid,
    /// A piece mixes volume colors (decomposition would change coloring).
    PieceColorMixed,
    /// A volume is not covered by any piece.
    VolumeUncovered,
    /// A virtual cut is malformed.
    CutInvalid,
    /// An unfolded glue pair does not coincide geometrically.
    UnfoldGlue,
}

/// One validation finding.
#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    /// Class.
    pub code: IssueCode,
    /// The main entity involved, if any.
    pub entity: Option<EntityId>,
    /// Explanation.
    pub message: String,
}

/// All findings.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// Findings in deterministic order.
    pub issues: Vec<Issue>,
}

impl Report {
    /// True if there are no issues.
    pub fn is_ok(&self) -> bool {
        self.issues.is_empty()
    }

    /// Number of issues with `code`.
    pub fn count(&self, code: IssueCode) -> usize {
        self.issues.iter().filter(|i| i.code == code).count()
    }

    fn push(&mut self, code: IssueCode, entity: impl Into<Option<EntityId>>, message: String) {
        self.issues.push(Issue { code, entity: entity.into(), message });
    }
}

fn pts_match(a: &[Point3], b: &[Point3], tol: f64) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut used = vec![false; b.len()];
    a.iter().all(|p| {
        let hit = b.iter().enumerate().find(|(j, q)| !used[*j] && (*p - **q).norm() <= tol);
        match hit {
            Some((j, _)) => {
                used[j] = true;
                true
            }
            None => false,
        }
    })
}

/// Validate `model` against `topo`.
pub fn validate(model: &Model, topo: &dyn TopologyView, tol: &Tolerance) -> Report {
    let mut r = Report::default();
    let volumes = topo.volumes();
    let faces = topo.faces();
    let vset: BTreeSet<EntityId> = volumes.iter().copied().collect();
    let fset: BTreeSet<EntityId> = faces.iter().copied().collect();

    check_coloring(model, &vset, &fset, &mut r);
    check_groups(model, &vset, &fset, &mut r);
    check_boundaries(model, topo, &faces, &fset, tol, &mut r);
    check_symmetry(model, topo, &vset, &mut r);
    check_instancing(model, &vset, &mut r);
    check_decomposition(model, topo, &volumes, &vset, &mut r);
    r
}

fn check_coloring(m: &Model, vset: &BTreeSet<EntityId>, fset: &BTreeSet<EntityId>, r: &mut Report) {
    for v in vset {
        if !m.volume_color.contains_key(v) {
            r.push(IssueCode::VolumeUncolored, *v, format!("volume {v} has no color"));
        }
    }
    for (v, c) in &m.volume_color {
        if !vset.contains(v) {
            r.push(IssueCode::DanglingReference, *v, format!("colored volume {v} is not in the topology"));
        }
        match m.colors.get(c) {
            None => r.push(IssueCode::DanglingReference, *c, format!("color {c} of {v} does not exist")),
            Some(row) if row.domain != ColorDomain::Volume => r.push(IssueCode::ColorDomain, *c, format!("color {c} on volume {v} is a boundary color")),
            Some(_) => {}
        }
    }
    for (f, c) in &m.face_color {
        if !fset.contains(f) {
            r.push(IssueCode::DanglingReference, *f, format!("colored face {f} is not in the topology"));
        }
        match m.colors.get(c) {
            None => r.push(IssueCode::DanglingReference, *c, format!("color {c} of {f} does not exist")),
            Some(row) if row.domain != ColorDomain::Boundary => r.push(IssueCode::ColorDomain, *c, format!("color {c} on face {f} is a volume color")),
            Some(_) => {}
        }
    }
}

fn check_groups(m: &Model, vset: &BTreeSet<EntityId>, fset: &BTreeSet<EntityId>, r: &mut Report) {
    for (g, row) in &m.groups {
        for mem in &row.members {
            let ok = match mem.kind() {
                EntityKind::TopoVolume => vset.contains(mem),
                EntityKind::TopoFace => fset.contains(mem),
                EntityKind::Color => m.colors.contains_key(mem),
                EntityKind::Group => m.groups.contains_key(mem),
                _ => true,
            };
            if !ok {
                r.push(IssueCode::GroupInvalid, *g, format!("group {g} member {mem} does not exist"));
            }
        }
        if m.group_volumes(*g).is_err() {
            r.push(IssueCode::GroupInvalid, *g, format!("group {g} is cyclic or has unknown nested groups"));
        }
    }
}

fn check_boundaries(m: &Model, topo: &dyn TopologyView, faces: &[EntityId], fset: &BTreeSet<EntityId>, tol: &Tolerance, r: &mut Report) {
    for f in faces {
        if topo.face_sides(*f).len() == 1 && !m.face_boundary.contains_key(f) {
            r.push(IssueCode::BoundaryFaceWithoutKind, *f, format!("boundary face {f} has no boundary kind"));
        }
    }
    for (f, fb) in &m.face_boundary {
        if !fset.contains(f) {
            r.push(IssueCode::DanglingReference, *f, format!("face {f} in a boundary record is not in the topology"));
            continue;
        }
        let Some(row) = m.boundaries.get(&fb.boundary) else {
            r.push(IssueCode::DanglingReference, *f, format!("face {f} refers to missing boundary {}", fb.boundary));
            continue;
        };
        let sides = topo.face_sides(*f);
        let interior_ok = matches!(row.kind, BoundaryKind::Interface(InterfaceIntent::Conformal));
        if interior_ok && sides.len() < 2 {
            r.push(IssueCode::KindMisplaced, *f, format!("conformal interface face {f} must have two sides"));
        }
        if !interior_ok && sides.len() > 1 {
            r.push(IssueCode::KindMisplaced, *f, format!("face {f} is interior but belongs to {:?}", row.name));
        }
        match row.kind {
            BoundaryKind::Deck(d) => check_deck_face(m, topo, *f, fb.side, fb.mate, fb.boundary, &d, tol, r),
            _ => {
                if row.kind.is_paired() && fb.mate.is_none() {
                    // Non-conformal sets may be unpaired; nothing to check.
                }
            }
        }
    }
    for (b, row) in &m.boundaries {
        if let BoundaryKind::Deck(d) = &row.kind
            && d.check().is_err()
        {
            r.push(IssueCode::DeckParameters, *b, format!("deck {b} has invalid parameters"));
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn check_deck_face(m: &Model, topo: &dyn TopologyView, f: EntityId, side: Side, mate: Option<EntityId>, b: EntityId, deck: &Deck, tol: &Tolerance, r: &mut Report) {
    match deck {
        Deck::Mirror { plane } => {
            check_plane(plane, f, tol, r);
            let off = topo.face_outline(f).iter().map(|p| plane.signed_distance(*p).abs()).fold(0.0, f64::max);
            if off > tol.linear {
                r.push(IssueCode::DeckGeometry, f, format!("mirror face {f} is {off:e} off its plane"));
            }
            if mate.is_some() || side != Side::A {
                r.push(IssueCode::DeckPairing, f, format!("mirror face {f} must be unpaired on side A"));
            }
        }
        _ => {
            let Some(mt) = mate else {
                r.push(IssueCode::DeckPairing, f, format!("deck face {f} has no mate"));
                return;
            };
            match m.face_boundary.get(&mt) {
                Some(o) if o.boundary == b && o.side != side && o.mate == Some(f) => {}
                _ => {
                    r.push(IssueCode::DeckPairing, f, format!("mate {mt} of {f} is not its reciprocal partner in the same record"));
                    return;
                }
            }
            if side != Side::A {
                return; // geometry is checked once per pair, from side A
            }
            let t = deck.transform();
            let img: Vec<Point3> = topo.face_outline(f).iter().map(|p| t.apply_point(*p)).collect();
            if !pts_match(&img, &topo.face_outline(mt), tol.linear) {
                r.push(IssueCode::DeckGeometry, f, format!("T({f}) does not coincide with its mate {mt}"));
            }
            let adj = |x: EntityId| -> Vec<Option<EntityId>> { topo.face_sides(x).iter().map(|s| m.volume_color.get(&s.volume).copied()).collect() };
            if m.face_color.get(&f) != m.face_color.get(&mt) || adj(f) != adj(mt) {
                r.push(IssueCode::DeckColor, f, format!("deck mates {f} and {mt} differ in face color or adjacent volume color"));
            }
        }
    }
}

fn check_plane(plane: &Plane, f: EntityId, tol: &Tolerance, r: &mut Report) {
    if (plane.normal.norm() - 1.0).abs() > tol.angular {
        r.push(IssueCode::DeckParameters, f, "mirror plane normal is not a unit vector".to_owned());
    }
}

fn check_symmetry(m: &Model, topo: &dyn TopologyView, vset: &BTreeSet<EntityId>, r: &mut Report) {
    let Some(s) = &m.symmetry else { return };
    if s.generators.is_empty() {
        r.push(IssueCode::SymmetryInvalid, None, "symmetry has no generators".to_owned());
    }
    for g in &s.generators {
        if m.boundaries.get(g).and_then(|b| b.kind.as_deck()).is_none() {
            r.push(IssueCode::SymmetryInvalid, *g, format!("generator {g} is not a deck boundary"));
        }
    }
    for v in &s.domain {
        if !vset.contains(v) {
            r.push(IssueCode::SymmetryInvalid, *v, format!("domain volume {v} is not in the topology"));
        }
    }
    let dom: BTreeSet<_> = s.domain.iter().copied().collect();
    for (f, fb) in &m.face_boundary {
        if s.generators.contains(&fb.boundary) && !topo.face_sides(*f).iter().any(|x| dom.contains(&x.volume)) {
            r.push(IssueCode::SymmetryInvalid, *f, format!("deck face {f} does not bound the fundamental domain"));
        }
    }
}

fn check_instancing(m: &Model, vset: &BTreeSet<EntityId>, r: &mut Report) {
    let ins = &m.instancing;
    let fill_ok = |f: &Fill| match f {
        Fill::Prototype(p) => ins.prototypes.contains_key(p),
        Fill::Lattice(l) => ins.lattices.contains_key(l),
        Fill::Homogeneous(c) => m.colors.get(c).is_some_and(|c| c.domain == ColorDomain::Volume),
    };
    let ov_ok = |o: &Override| {
        let col = |c: &EntityId| m.colors.get(c).is_some_and(|c| c.domain == ColorDomain::Volume);
        match o {
            Override::ReplaceColor { from, to } => col(from) && col(to),
            Override::SetVolume { volume, to } => vset.contains(volume) && col(to),
            Override::SetAll { to } => col(to),
        }
    };
    for (p, proto) in &ins.prototypes {
        for mem in &proto.members {
            let ok = match mem {
                Member::Volume(v) => vset.contains(v),
                Member::Instance(i) => ins.instances.contains_key(i),
            };
            if !ok {
                r.push(IssueCode::InstancingInvalid, *p, format!("prototype {p} has an unresolved member {mem:?}"));
            }
        }
    }
    for (i, inst) in &ins.instances {
        if !fill_ok(&inst.fill) || !inst.overrides.iter().all(ov_ok) {
            r.push(IssueCode::InstancingInvalid, *i, format!("instance {i} has an unresolved fill or override"));
        }
    }
    for (l, lat) in &ins.lattices {
        if lat.outer.as_ref().is_some_and(|f| !fill_ok(f)) {
            r.push(IssueCode::InstancingInvalid, *l, format!("lattice {l} has an unresolved outer fill"));
        }
        for (idx, s) in &lat.slots {
            if !lat.layout.contains(*idx) {
                r.push(IssueCode::InstancingInvalid, *l, format!("lattice {l} slot {idx:?} is outside its layout"));
            }
            if !fill_ok(&s.fill) || !s.overrides.iter().all(ov_ok) {
                r.push(IssueCode::InstancingInvalid, *l, format!("lattice {l} slot {idx:?} has an unresolved fill or override"));
            }
        }
    }
    for root in &ins.roots {
        if !ins.instances.contains_key(root) {
            r.push(IssueCode::InstancingInvalid, *root, format!("root {root} is not an instance"));
        }
    }
    if r.count(IssueCode::InstancingInvalid) == 0
        && let Err(e) = m.flatten()
    {
        r.push(IssueCode::InstancingInvalid, None, format!("flatten failed: {e}"));
    }
}

fn check_decomposition(m: &Model, topo: &dyn TopologyView, volumes: &[EntityId], vset: &BTreeSet<EntityId>, r: &mut Report) {
    let Some(d) = &m.decomposition else { return };
    let mut covered = BTreeSet::new();
    for (id, p) in &d.pieces {
        if p.volumes.is_empty() {
            r.push(IssueCode::PieceInvalid, *id, format!("piece {id} has no volumes"));
        }
        let mut colors = BTreeSet::new();
        for v in &p.volumes {
            if !vset.contains(v) {
                r.push(IssueCode::PieceInvalid, *id, format!("piece {id} refers to missing volume {v}"));
            }
            covered.insert(*v);
            colors.insert(m.volume_color.get(v).copied());
        }
        if colors.len() > 1 {
            r.push(IssueCode::PieceColorMixed, *id, format!("piece {id} spans several colors"));
        }
        for f in &p.faces {
            match f {
                PieceFace::Original(face) => {
                    if !topo.face_sides(*face).iter().any(|s| p.volumes.contains(&s.volume)) {
                        r.push(IssueCode::PieceInvalid, *id, format!("piece {id}: face {face} does not bound its volumes"));
                    }
                }
                PieceFace::Cut(c) => {
                    if !d.cuts.contains_key(c) {
                        r.push(IssueCode::PieceInvalid, *id, format!("piece {id}: unknown cut {c:?}"));
                    }
                }
            }
        }
        if let PieceShape::Sweep { source, target } = p.shape
            && (source == target || !p.faces.contains(&source) || !p.faces.contains(&target))
        {
            r.push(IssueCode::PieceInvalid, *id, format!("piece {id}: sweep source/target must be distinct faces of the piece"));
        }
    }
    for v in volumes {
        if !covered.contains(v) {
            r.push(IssueCode::VolumeUncovered, *v, format!("volume {v} is not covered by any piece"));
        }
    }
    for (c, cut) in &d.cuts {
        if !vset.contains(&cut.volume) {
            r.push(IssueCode::CutInvalid, cut.volume, format!("cut {c:?} refers to missing volume"));
        }
        if let CutSurface::Surface(s) = cut.surface
            && s.kind() != EntityKind::Surface
        {
            r.push(IssueCode::CutInvalid, s, format!("cut {c:?} surface has the wrong kind"));
        }
        let sides = d.pieces_of_cut(*c);
        if sides.len() != 2 || !sides.iter().all(|p| d.volumes_of_piece(*p).contains(&cut.volume)) {
            r.push(IssueCode::CutInvalid, cut.volume, format!("cut {c:?} must bound exactly two pieces of its volume"));
        }
    }
}

/// Check that every glue pair of an unfolded model coincides geometrically (loops map onto each
/// other under the copies' transforms) and that no copy transform is singular.
pub fn validate_unfolded(topo: &dyn TopologyView, u: &UnfoldedModel, tol: &Tolerance) -> Report {
    let mut r = Report::default();
    let place = |c: u32| -> Option<&Placed> { u.copies.get(c as usize) };
    for g in &u.glued {
        let (Some(pa), Some(pb)) = (place(g.a.copy), place(g.b.copy)) else {
            r.push(IssueCode::UnfoldGlue, g.boundary, "glue refers to a missing copy".to_owned());
            continue;
        };
        if inverse(&pa.transform).is_err() || inverse(&pb.transform).is_err() {
            r.push(IssueCode::UnfoldGlue, g.boundary, "singular copy transform".to_owned());
            continue;
        }
        let a: Vec<Point3> = topo.face_outline(g.a.base).iter().map(|p| pa.transform.apply_point(*p)).collect();
        let b: Vec<Point3> = topo.face_outline(g.b.base).iter().map(|p| pb.transform.apply_point(*p)).collect();
        if !pts_match(&a, &b, tol.linear) {
            r.push(IssueCode::UnfoldGlue, g.boundary, format!("glued faces {} (copy {}) and {} (copy {}) do not coincide", g.a.base, g.a.copy, g.b.base, g.b.copy));
        }
    }
    r
}
