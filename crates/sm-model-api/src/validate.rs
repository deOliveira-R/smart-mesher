//! The validator for semantic invariants that need topology and geometry.
//!
//! All geometric checks run on integrated moments (`face_moments`), so they do not depend on where
//! a periodic seam sits and are sensitive to shape, not only to area and centroid. Deck checks
//! are scoped to each deck (a reflective cell with mirrors at `x = 0` and `x = 1` is valid; only
//! *declaring* a symmetry needs a finite point group). All checks of one call share one geometry
//! cache. Coincidence is read from the topology's `coincident_faces`, never inferred from moments.

use crate::boundary::{BoundaryKind, Deck, DeckSide, FaceRole, InterfaceIntent};
use crate::color::Domain;
use crate::decomposition::{PieceFace, PieceRef, PieceShape};
use crate::instancing::ColorMap;
use crate::model::Model;
use crate::symmetry::{MAX_GROUP_ORDER, Keep, plan_cut, point_group};
use crate::unfold::UnfoldedModel;
use crate::xf::{GeomCache, Tolerance, face_shape_close, transform_face_moments, vector_area_sign};
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use sm_topo_api::{Coincidence, Dim, FaceClass, Region, TopologyView, TopologyViewExt};
use smallvec::SmallVec;
use std::collections::BTreeSet;

/// How serious an issue is. Only errors make a report not [`Report::is_ok`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Severity {
    /// The model is inconsistent.
    Error,
    /// Suspicious but allowed (an uncolored boundary is an error only at export time).
    Warning,
}

/// What a validation issue is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum IssueCode {
    /// A live volume has no color (volume coloring must be an exact partition).
    ColoringNotPartition,
    /// A color of the wrong domain is used.
    ColorDomainMismatch,
    /// Two colors of one domain share a label.
    LabelDuplicate,
    /// A boundary face has no kind (deck, response or interface).
    BoundaryFaceNoKind,
    /// A boundary face has no face color (warning).
    BoundaryFaceUncolored,
    /// A deck or response kind sits on a face that is not a model boundary.
    KindOnInteriorFace,
    /// Deck mates are not reciprocal.
    DeckMateNotReciprocal,
    /// The image of a deck face does not coincide with its mate (shape or position).
    DeckGeometry,
    /// A deck pair breaks the sign rule `n_B = -g(n_A)`: both volumes would be on the same side.
    DeckOrientation,
    /// A mirror face is not on the mirror plane.
    MirrorOffPlane,
    /// A translation does not commute with the point group, so a translation window is invalid.
    TranslationNotNormalized,
    /// A rotation deck is not a primitive `1/n` rotation, so the wedge does not tile.
    RotationOrder,
    /// A symmetry generator has no faces.
    GeneratorUnused,
    /// A deck record with faces is not a generator of the declared symmetry (warning).
    DeckNotInSymmetry,
    /// The point group of the generators does not close.
    GroupNotFinite,
    /// The two face sets of an interface do not face each other.
    InterfaceNotFacing,
    /// An interface face has no counterpart, or is declared on a merged face.
    InterfaceUnmatched,
    /// Both sides of an interface have the same volume color (warning).
    InterfaceSameColor,
    /// A cap of a recorded cut is off the planned plane.
    CapOffPlane,
    /// Lattices contain themselves.
    InstanceCycle,
    /// A recoloring maps between colors of different domains.
    RecolorDomainMismatch,
    /// A recoloring key is not a known color, volume or face.
    RecolorUnknownKey,
    /// A piece lists faces that do not bound its volume.
    PieceFacesNotOfVolume,
    /// The pieces of a volume do not cover its faces, or a cut has only one side.
    PieceTilingBroken,
    /// A sweep's source and target share an edge.
    SweepSourceTargetAdjacent,
    /// A composite piece mixes volume colors.
    DecompositionChangedColoring,
    /// A table refers to an entity that is not alive in the topology.
    UnknownEntity,
    /// Moments are unavailable for a face, so a geometric check was skipped (warning).
    GeometryUnavailable,
}

impl IssueCode {
    /// The severity of this code.
    pub fn severity(self) -> Severity {
        match self {
            IssueCode::BoundaryFaceUncolored | IssueCode::DeckNotInSymmetry | IssueCode::InterfaceSameColor | IssueCode::GeometryUnavailable => Severity::Warning,
            _ => Severity::Error,
        }
    }
}

/// One finding.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Issue {
    /// What kind of issue.
    pub code: IssueCode,
    /// Error or warning.
    pub severity: Severity,
    /// The entities involved.
    pub entities: SmallVec<[EntityId; 2]>,
    /// Human-readable detail.
    pub detail: String,
}

/// The findings of a validation, sorted by `(code, entities, detail)`.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Report {
    /// The issues.
    pub issues: Vec<Issue>,
}

impl Report {
    /// True if there is no error (warnings are allowed).
    pub fn is_ok(&self) -> bool {
        self.issues.iter().all(|i| i.severity != Severity::Error)
    }

    /// True if an issue with `c` exists.
    pub fn has(&self, c: IssueCode) -> bool {
        self.issues.iter().any(|i| i.code == c)
    }

    /// The issues with code `c`.
    pub fn with(&self, c: IssueCode) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(move |i| i.code == c)
    }

    /// The errors.
    pub fn errors(&self) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(|i| i.severity == Severity::Error)
    }

    /// The warnings.
    pub fn warnings(&self) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(|i| i.severity == Severity::Warning)
    }
}

struct Sink(Vec<Issue>);

impl Sink {
    fn add(&mut self, code: IssueCode, ents: &[EntityId], detail: impl Into<String>) {
        self.0.push(Issue { code, severity: code.severity(), entities: ents.iter().copied().collect(), detail: detail.into() });
    }
    fn finish(mut self) -> Report {
        self.0.sort_by(|a, b| (a.code, &a.entities, &a.detail).cmp(&(b.code, &b.entities, &b.detail)));
        Report { issues: self.0 }
    }
}

fn alive(t: &dyn TopologyView, id: EntityId) -> bool {
    t.is_alive(id)
}

/// Validate the model against the topology. See the module docs for the check list.
pub fn validate(m: &Model, t: &dyn TopologyView, tol: Tolerance) -> Report {
    let mut s = Sink(Vec::new());
    let mut c = GeomCache::new(t);
    let l = c.length_scale();
    check_coloring(m, t, &mut s);
    check_faces(m, t, &mut s);
    check_decks(m, &mut c, l, &tol, &mut s);
    check_symmetry(m, t, &tol, &mut s);
    check_interfaces(m, t, &mut c, l, &tol, &mut s);
    check_instancing(m, t, &mut s);
    check_decomposition(m, t, &mut s);
    s.finish()
}

fn check_coloring(m: &Model, t: &dyn TopologyView, s: &mut Sink) {
    let mut seen: BTreeSet<(Domain, u32)> = BTreeSet::new();
    for c in m.colors() {
        if !seen.insert((c.domain, c.label)) {
            s.add(IssueCode::LabelDuplicate, &[c.id], format!("label {} is used twice", c.label));
        }
    }
    for v in t.cells(Dim::Volume) {
        if m.volume_color(*v).is_none() {
            s.add(IssueCode::ColoringNotPartition, &[*v], "volume has no color");
        }
    }
    for (v, col) in &m.volume_color {
        if !alive(t, *v) {
            s.add(IssueCode::UnknownEntity, &[*v], "colored volume is not alive");
        }
        match m.color(*col) {
            None => s.add(IssueCode::UnknownEntity, &[*v, *col], "volume color is not in the color table"),
            Some(r) if r.domain != Domain::Volume => s.add(IssueCode::ColorDomainMismatch, &[*v, *col], "volume has a face color"),
            Some(_) => {}
        }
    }
    for (f, col) in &m.face_color {
        if !alive(t, *f) {
            s.add(IssueCode::UnknownEntity, &[*f], "colored face is not alive");
        }
        match m.color(*col) {
            None => s.add(IssueCode::UnknownEntity, &[*f, *col], "face color is not in the color table"),
            Some(r) if r.domain != Domain::Face => s.add(IssueCode::ColorDomainMismatch, &[*f, *col], "face has a volume color"),
            Some(_) => {}
        }
    }
}

fn check_faces(m: &Model, t: &dyn TopologyView, s: &mut Sink) {
    for f in t.cells(Dim::Face) {
        let class = t.face_class(*f);
        let fb = m.face_boundary(*f);
        if class == Some(FaceClass::Boundary) {
            if fb.is_none() {
                s.add(IssueCode::BoundaryFaceNoKind, &[*f], "boundary face needs a deck, response or interface kind");
            }
            if m.face_color(*f).is_none() {
                s.add(IssueCode::BoundaryFaceUncolored, &[*f], "boundary face has no color (an error at export if the format needs one)");
            }
        }
        if let Some(fb) = fb
            && class != Some(FaceClass::Boundary)
            && matches!(fb.role, FaceRole::Deck { .. } | FaceRole::Response)
        {
            s.add(IssueCode::KindOnInteriorFace, &[*f], "deck and response kinds belong on boundary faces");
        }
    }
    for (f, fb) in m.face_boundary.iter() {
        if !alive(t, *f) {
            s.add(IssueCode::UnknownEntity, &[*f], "face with a kind is not alive");
        }
        if m.boundary(fb.boundary).is_none() {
            s.add(IssueCode::UnknownEntity, &[*f, fb.boundary], "face refers to an unknown boundary record");
        }
    }
}

fn check_decks(m: &Model, c: &mut GeomCache<'_>, l: f64, tol: &Tolerance, s: &mut Sink) {
    let planar = tol.linear.max(1e-9 * l);
    let dist = 10.0 * tol.linear + 1e-9 * l;
    for row in m.boundaries() {
        let BoundaryKind::Deck(deck) = row.kind else { continue };
        if let Err(e) = deck.check() {
            s.add(IssueCode::DeckGeometry, &[row.id], format!("invalid deck: {e}"));
            continue;
        }
        if let Deck::Rotation { angle, .. } = deck
            && !(angle.num == 1 || angle.num + 1 == angle.den as i32)
        {
            s.add(IssueCode::RotationOrder, &[row.id], format!("rotation {}/{} of a turn is not a primitive 1/n rotation; the wedge would not tile", angle.num, angle.den));
        }
        let g = deck.transform();
        for (f, fb) in m.boundary_faces(row.id) {
            let FaceRole::Deck { side, mate } = fb.role else {
                s.add(IssueCode::KindOnInteriorFace, &[f], "face of a deck record has a non-deck role");
                continue;
            };
            let Some(fm) = c.face(f) else {
                s.add(IssueCode::GeometryUnavailable, &[f], "no face moments; deck geometry not checked");
                continue;
            };
            match (deck, side) {
                (Deck::Mirror { plane }, DeckSide::SelfPaired) => {
                    match fm.plane(planar) {
                        Some((n, d)) => {
                            let dot = n.dot(plane.normal);
                            let off = if dot > 0.0 { d } else { -d };
                            if dot.abs() < 1.0 - 1e-6 || (off - plane.offset * dot.signum()).abs() > dist {
                                s.add(IssueCode::MirrorOffPlane, &[f], "mirror face is not on the mirror plane");
                            }
                        }
                        None => s.add(IssueCode::MirrorOffPlane, &[f], "mirror face is not planar"),
                    }
                    if mate.is_some() {
                        s.add(IssueCode::DeckMateNotReciprocal, &[f], "a mirror face has no mate");
                    }
                }
                (Deck::Mirror { .. }, _) | (_, DeckSide::SelfPaired) => {
                    s.add(IssueCode::DeckMateNotReciprocal, &[f], "pairing role does not fit the deck kind");
                }
                (_, DeckSide::A | DeckSide::B) => {
                    let Some(mt) = mate else {
                        s.add(IssueCode::DeckMateNotReciprocal, &[f], "paired face has no mate");
                        continue;
                    };
                    let back = m.face_boundary(mt).map(|x| (x.boundary, x.role));
                    let want = FaceRole::Deck { side: if side == DeckSide::A { DeckSide::B } else { DeckSide::A }, mate: Some(f) };
                    if back != Some((row.id, want)) {
                        s.add(IssueCode::DeckMateNotReciprocal, &[f, mt], "mate does not point back");
                        continue;
                    }
                    if side == DeckSide::A {
                        let Some(mm) = c.face(mt) else {
                            s.add(IssueCode::GeometryUnavailable, &[mt], "no face moments; deck geometry not checked");
                            continue;
                        };
                        let img = transform_face_moments(&fm, &g);
                        if !face_shape_close(&img, &mm, l, tol) {
                            s.add(IssueCode::DeckGeometry, &[f, mt], "the image of face A does not coincide with face B");
                        } else {
                            match vector_area_sign(img.vector_area, mm.vector_area, img.area, tol) {
                                -1 => {}
                                1 => s.add(IssueCode::DeckOrientation, &[f, mt], "n_B = -g(n_A) is violated: both volumes would be on the same side"),
                                _ => s.add(IssueCode::DeckGeometry, &[f, mt], "face normals are not parallel under the deck transform"),
                            }
                        }
                    }
                }
            }
        }
    }
    // Recorded cuts: caps of translation and rotation cuts lie on the planned planes.
    for p in m.provenance() {
        let crate::symmetry::Provenance::Cut { spec } = p else { continue };
        if spec.deck.is_self_paired() {
            continue;
        }
        let Ok(plan) = plan_cut(spec) else { continue };
        for row in m.boundaries().filter(|r| r.kind.as_deck().is_some_and(|d| d.same_as(&spec.deck))) {
            for (f, _) in m.boundary_faces(row.id) {
                let Some(fm) = c.face(f) else { continue };
                let on = fm.plane(planar).is_some_and(|(n, d)| {
                    plan.half_spaces.iter().any(|h| {
                        let dot = n.dot(h.normal);
                        dot.abs() > 1.0 - 1e-6 && ((if dot > 0.0 { d } else { -d }) - h.offset).abs() <= dist
                    })
                });
                if !on {
                    s.add(IssueCode::CapOffPlane, &[f], "cap is not on a planned plane of its cut");
                }
            }
        }
    }
}

fn check_symmetry(m: &Model, t: &dyn TopologyView, tol: &Tolerance, s: &mut Sink) {
    let gens: Vec<EntityId> = m.symmetry().map(|x| x.generators.clone()).unwrap_or_default();
    for row in m.boundaries() {
        if let BoundaryKind::Deck(_) = row.kind
            && !gens.contains(&row.id)
            && m.boundary_faces(row.id).next().is_some()
        {
            s.add(IssueCode::DeckNotInSymmetry, &[row.id], "deck record has faces but is not a generator of a declared symmetry");
        }
    }
    let Some(sym) = m.symmetry() else { return };
    for v in &sym.domain {
        if !alive(t, *v) {
            s.add(IssueCode::UnknownEntity, &[*v], "symmetry domain volume is not alive");
        }
    }
    let mut point: Vec<Transform> = Vec::new();
    let mut trans: Vec<(EntityId, Vector3)> = Vec::new();
    for b in &gens {
        let Some(deck) = m.boundary(*b).and_then(|r| r.kind.as_deck().copied()) else {
            s.add(IssueCode::UnknownEntity, &[*b], "generator is not a deck record");
            continue;
        };
        if m.boundary_faces(*b).next().is_none() {
            s.add(IssueCode::GeneratorUnused, &[*b], "generator has no faces");
        }
        match deck {
            Deck::Translation { vector } => trans.push((*b, vector)),
            d => point.push(d.transform()),
        }
    }
    match point_group(&point, MAX_GROUP_ORDER) {
        Err(_) => s.add(IssueCode::GroupNotFinite, &gens, "the point group of the generators does not close"),
        Ok(group) => {
            for (b, v) in trans {
                if group.iter().any(|p| (p.apply_vector(v) - v).norm() > tol.angular.max(1e-9) * v.norm().max(1.0)) {
                    s.add(IssueCode::TranslationNotNormalized, &[b], "the translation does not commute with the point group");
                }
            }
        }
    }
}

fn bounding_radius(fm: &sm_topo_api::FaceMoments) -> f64 {
    let c = fm.centroid().unwrap_or_default();
    let cv = Vector3::new(c.x, c.y, c.z);
    let tr = fm.second[0][0] + fm.second[1][1] + fm.second[2][2] - fm.area * cv.dot(cv);
    (3.0 * tr.max(0.0) / fm.area.max(1e-300)).sqrt() * 1.01
}

fn check_interfaces(m: &Model, t: &dyn TopologyView, c: &mut GeomCache<'_>, l: f64, tol: &Tolerance, s: &mut Sink) {
    for row in m.boundaries() {
        let BoundaryKind::Interface(intent) = row.kind else { continue };
        let faces: Vec<(EntityId, DeckSide)> = m
            .boundary_faces(row.id)
            .filter_map(|(f, fb)| match fb.role {
                FaceRole::Interface { side } => Some((f, side)),
                _ => None,
            })
            .collect();
        let a: Vec<EntityId> = faces.iter().filter(|x| x.1 == DeckSide::A).map(|x| x.0).collect();
        let b: Vec<EntityId> = faces.iter().filter(|x| x.1 == DeckSide::B).map(|x| x.0).collect();
        let class_of = |f: EntityId| t.face_class(f);
        match intent {
            InterfaceIntent::Conformal => {
                for f in &a {
                    if class_of(*f) != Some(FaceClass::Interface) {
                        s.add(IssueCode::InterfaceUnmatched, &[*f], "a conformal interface face must be shared by two volumes");
                    }
                }
            }
            InterfaceIntent::NonConformal { transform } => {
                if a.is_empty() || b.is_empty() {
                    s.add(IssueCode::InterfaceUnmatched, &[row.id], "a non-conformal interface needs faces on both sides");
                    continue;
                }
                for f in a.iter().chain(&b) {
                    if class_of(*f) == Some(FaceClass::Interface) {
                        s.add(IssueCode::InterfaceUnmatched, &[*f], "interface declared on a merged (shared) face");
                    }
                }
                // Facing: summed vector areas opposed (under the optional transform).
                let mut sa = Vector3::default();
                let mut sb = Vector3::default();
                let mut total = 0.0;
                let mut ok = true;
                for f in &a {
                    match c.face(*f) {
                        Some(fm) => {
                            sa = sa + fm.vector_area;
                            total += fm.area;
                        }
                        None => ok = false,
                    }
                }
                for f in &b {
                    match c.face(*f) {
                        Some(fm) => sb = sb + fm.vector_area,
                        None => ok = false,
                    }
                }
                if !ok {
                    s.add(IssueCode::GeometryUnavailable, &[row.id], "no face moments; interface facing not checked");
                } else {
                    let img = transform.map_or(sa, |x| x.apply_vector(sa));
                    let lim = (tol.relative + tol.angular) * total + 10.0 * tol.linear * l;
                    if (img + sb).norm() > lim {
                        s.add(IssueCode::InterfaceNotFacing, &[row.id], "the summed vector areas of the two sides are not opposed");
                    }
                }
                // Every face has a counterpart: a recorded coincidence or overlapping bounds.
                let (aset, bset): (BTreeSet<EntityId>, BTreeSet<EntityId>) = (a.iter().copied().collect(), b.iter().copied().collect());
                let img_t = transform.unwrap_or(Transform::IDENTITY);
                for (mine, other, forward) in [(&a, &bset, true), (&b, &aset, false)] {
                    for f in mine {
                        let pairs = t.coincident_faces(*f);
                        if pairs.iter().any(|p| p.status == Coincidence::Merged && p.other(*f).is_some_and(|o| other.contains(&o) || aset.contains(&o) || bset.contains(&o))) {
                            s.add(IssueCode::InterfaceUnmatched, &[*f], "the face is recorded as merged with a face of the interface");
                            continue;
                        }
                        if pairs.iter().any(|p| p.other(*f).is_some_and(|o| other.contains(&o))) {
                            continue;
                        }
                        let Some(fm) = c.face(*f) else { continue };
                        let Some(pc) = fm.centroid() else { continue };
                        // Compare in B's frame: A faces are moved by the optional transform.
                        let p1 = if forward { img_t.apply_point(pc) } else { pc };
                        let r = bounding_radius(&fm);
                        let found = other.iter().any(|g| {
                            let Some(gm) = c.face(*g) else { return true };
                            let Some(gc) = gm.centroid() else { return false };
                            let p2 = if forward { gc } else { img_t.apply_point(gc) };
                            (p1 - p2).norm() <= r + bounding_radius(&gm) + 10.0 * tol.linear
                        });
                        if !found {
                            s.add(IssueCode::InterfaceUnmatched, &[*f], "the face has no counterpart on the other side");
                        }
                    }
                }
                // Same volume color on both sides (a plain interface; a transform is explicit intent).
                if transform.is_none() {
                    let colors = |fs: &[EntityId]| -> BTreeSet<Option<EntityId>> {
                        fs.iter()
                            .filter_map(|f| t.face_sides(*f))
                            .flat_map(|sd| [sd.left, sd.right])
                            .filter_map(|r| if let Region::Volume(v) = r { Some(m.volume_color(v)) } else { None })
                            .collect()
                    };
                    let (ca, cb) = (colors(&a), colors(&b));
                    if ca.len() == 1 && ca == cb {
                        s.add(IssueCode::InterfaceSameColor, &[row.id], "both sides of the interface have the same volume color");
                    }
                }
            }
        }
    }
}

fn check_map(m: &Model, t: &dyn TopologyView, map: &ColorMap, ctx: EntityId, s: &mut Sink) {
    let mut one = |map: &std::collections::BTreeMap<EntityId, EntityId>, ent: EntityKind, dom: Domain| {
        for (k, v) in map {
            let key_ok = if k.kind() == ent {
                alive(t, *k)
            } else {
                k.kind() == EntityKind::Color && m.color(*k).is_some()
            };
            if !key_ok {
                s.add(IssueCode::RecolorUnknownKey, &[ctx, *k], "recolor key is not a known color, volume or face");
            }
            if k.kind() == EntityKind::Color && m.color(*k).is_some_and(|c| c.domain != dom) {
                s.add(IssueCode::RecolorDomainMismatch, &[ctx, *k], "recolor key has the wrong domain");
            }
            match m.color(*v) {
                None => s.add(IssueCode::RecolorUnknownKey, &[ctx, *v], "recolor target is not a known color"),
                Some(c) if c.domain != dom => s.add(IssueCode::RecolorDomainMismatch, &[ctx, *v], "recolor target has the wrong domain"),
                Some(_) => {}
            }
        }
    };
    one(&map.volume, EntityKind::TopoVolume, Domain::Volume);
    one(&map.face, EntityKind::TopoFace, Domain::Face);
}

fn check_instancing(m: &Model, t: &dyn TopologyView, s: &mut Sink) {
    let ins = m.instancing();
    if let Some(l) = ins.find_cycle() {
        s.add(IssueCode::InstanceCycle, &[l], "lattice contains itself");
    }
    let fill_ok = |f: crate::instancing::Fill| match f {
        crate::instancing::Fill::Void => true,
        crate::instancing::Fill::Prototype(p) => ins.prototypes.contains_key(&p),
        crate::instancing::Fill::Lattice(l) => ins.lattices.contains_key(&l),
    };
    for (id, p) in &ins.prototypes {
        for v in &p.volumes {
            if !alive(t, *v) {
                s.add(IssueCode::UnknownEntity, &[*id, *v], "prototype volume is not alive");
            }
        }
    }
    for (id, i) in &ins.instances {
        if !fill_ok(i.of) {
            s.add(IssueCode::UnknownEntity, &[*id], "instance fill does not exist");
        }
        check_map(m, t, &i.recolor, *id, s);
    }
    for (id, l) in &ins.lattices {
        for (ix, sl) in &l.slots {
            if !l.layout.contains(*ix) || !fill_ok(sl.fill) {
                s.add(IssueCode::UnknownEntity, &[*id], format!("slot {ix:?} is outside the layout or its fill does not exist"));
            }
            check_map(m, t, &sl.recolor, *id, s);
        }
        if !fill_ok(l.outer) {
            s.add(IssueCode::UnknownEntity, &[*id], "lattice outer fill does not exist");
        }
    }
    for o in &ins.overrides {
        let ctx = o.prefix.iter().find_map(|st| if let crate::instancing::PathStep::Instance(i) = st { Some(*i) } else { None });
        if let Some(ctx) = ctx {
            check_map(m, t, &o.recolor, ctx, s);
        }
    }
}

fn check_decomposition(m: &Model, t: &dyn TopologyView, s: &mut Sink) {
    let d = m.decomposition();
    for v in d.decomposed_volumes() {
        if !alive(t, v) {
            s.add(IssueCode::UnknownEntity, &[v], "decomposed volume is not alive");
            continue;
        }
        // Tiling: the leaves cover every face of the volume, and each cut has both sides.
        let mut leaves = Vec::new();
        let mut stack = vec![PieceRef::Whole(v)];
        while let Some(p) = stack.pop() {
            match d.split_of(p) {
                Some(rec) => stack.extend(rec.children.iter().map(|c| PieceRef::Cell(*c))),
                None => leaves.push(p),
            }
        }
        let vf: BTreeSet<EntityId> = t.volume_faces(v).iter().copied().collect();
        let mut covered: BTreeSet<EntityId> = BTreeSet::new();
        let mut cuts: std::collections::BTreeMap<crate::decomposition::CutId, (bool, bool)> = Default::default();
        for lf in &leaves {
            for pf in d.piece_faces(*lf, t) {
                match pf {
                    PieceFace::Cut { cut, side } => {
                        let e = cuts.entry(cut).or_default();
                        if side == Keep::Negative { e.0 = true } else { e.1 = true }
                    }
                    other => {
                        if let Some(f) = other.base() {
                            covered.insert(f);
                        }
                    }
                }
            }
        }
        if covered != vf {
            s.add(IssueCode::PieceTilingBroken, &[v], "the pieces do not cover exactly the faces of the volume");
        }
        if cuts.values().any(|(n, p)| !(*n && *p)) {
            s.add(IssueCode::PieceTilingBroken, &[v], "a cut has pieces on only one side");
        }
    }
    for c in d.active_cells() {
        let Some(p) = d.piece(c) else { continue };
        if !alive(t, p.volume) {
            s.add(IssueCode::UnknownEntity, &[c, p.volume], "piece volume is not alive");
            continue;
        }
        if !p.members.is_empty() && m.piece_color(PieceRef::Cell(c)).is_none() {
            s.add(IssueCode::DecompositionChangedColoring, &[c], "a composite mixes volume colors");
        }
        let vf: BTreeSet<EntityId> = t.volume_faces(p.volume).iter().copied().collect();
        if p.members.is_empty() && p.faces.iter().filter_map(PieceFace::base).any(|f| !vf.contains(&f)) {
            s.add(IssueCode::PieceFacesNotOfVolume, &[c, p.volume], "piece lists a face that does not bound its volume");
        }
        if let PieceShape::Sweep { source, target } = &p.shape {
            let known: BTreeSet<PieceFace> = d.piece_faces(PieceRef::Cell(c), t).into_iter().collect();
            if source.iter().chain(target).any(|f| !known.contains(f)) {
                s.add(IssueCode::PieceFacesNotOfVolume, &[c], "sweep source or target is not a face of the piece");
            }
            let edges = |fs: &[PieceFace]| -> BTreeSet<EntityId> { fs.iter().filter_map(PieceFace::base).flat_map(|f| t.face_edges(f).iter().copied()).collect() };
            let (es, et) = (edges(source), edges(target));
            if es.intersection(&et).next().is_some() {
                s.add(IssueCode::SweepSourceTargetAdjacent, &[c], "sweep source and target share an edge");
            }
        }
    }
}

/// Validate an unfolded view: every glued pair of deck faces must coincide geometrically with
/// opposed outward normals once both copies are placed.
pub fn validate_unfolded(u: &UnfoldedModel, m: &Model, t: &dyn TopologyView, tol: Tolerance) -> Report {
    let mut s = Sink(Vec::new());
    let mut c = GeomCache::new(t);
    let l = c.length_scale();
    for g in u.glued() {
        let (Some(xa), Some(xb)) = (u.element_transform(g.a.elem), u.element_transform(g.b.elem)) else {
            s.add(IssueCode::UnknownEntity, &[g.a.base, g.b.base], "glue refers to an unknown element");
            continue;
        };
        let (Some(fa), Some(fb)) = (c.face(g.a.base), c.face(g.b.base)) else {
            s.add(IssueCode::GeometryUnavailable, &[g.a.base, g.b.base], "no face moments; glue not checked");
            continue;
        };
        let (ia, ib) = (transform_face_moments(&fa, &xa), transform_face_moments(&fb, &xb));
        if !face_shape_close(&ia, &ib, l, &tol) {
            s.add(IssueCode::DeckGeometry, &[g.a.base, g.b.base], "glued faces do not coincide in the unfolded model");
        } else {
            match vector_area_sign(ia.vector_area, ib.vector_area, ia.area, &tol) {
                -1 => {}
                1 => s.add(IssueCode::DeckOrientation, &[g.a.base, g.b.base], "glued faces have the same outward direction"),
                _ => s.add(IssueCode::DeckGeometry, &[g.a.base, g.b.base], "glued faces are not parallel"),
            }
        }
    }
    for v in u.volumes() {
        if u.color_of(v) != m.volume_color(v.base) {
            s.add(IssueCode::DecompositionChangedColoring, &[v.base], "unfolded copy changed the volume color");
        }
    }
    s.finish()
}
