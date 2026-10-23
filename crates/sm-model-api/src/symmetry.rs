//! Symmetry: deck group, fundamental domain, cut-and-keep-chamber, unfold, detection.
//!
//! The fundamental domain is a *view*: a volume selection (the model domain). Cutting to a
//! chamber selects the volumes on the keep side and tags the faces the cut exposes. The
//! physical split of a volume that straddles the plane is a topology-layer operation done
//! before; this layer asserts the precondition by finding the cut faces. Unfold is derived
//! data: `{fundamental domain, generators}` fully determines it, so refolding is a lookup.

use crate::action::{compose, enumerate_elements, unit, DeckKind, EnumerateOptions, Generator, Plane, Placement, TransformKey};
use crate::boundary::{BoundaryKind, DeckSide};
use crate::error::ModelError;
use crate::model::{Model, Tolerance};
use crate::selection::Selection;
use crate::topology::{MassProps, TopologyView};
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use std::collections::{BTreeMap, BTreeSet};

/// Result of checking a candidate symmetry against the model.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Verification {
    /// Declared but not checked.
    Unverified,
    /// Maps volumes, faces, colors and boundary kinds onto themselves.
    Verified {
        /// Largest centroid displacement seen after mapping.
        max_residual: f64,
    },
    /// Fails the check.
    Rejected {
        /// First stage that failed.
        stage: RejectStage,
        /// Residual at the failure (`f64::INFINITY` if no counterpart exists).
        residual: f64,
    },
}

/// Verification stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RejectStage {
    /// Per-color barycentre and second moment are not invariant.
    ColorInvariants,
    /// Some volume has no same-colored image.
    Volumes,
    /// Some face has no image with the same area, sides' colors and boundary kind.
    Faces,
}

/// One generator of the deck group with the bookkeeping that ties it to the model.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GeneratorSpec {
    /// The generator.
    pub generator: Generator,
    /// Its deck pairing, once the chamber is cut or the faces are tagged.
    pub pairing: Option<EntityId>,
    /// Number of copies along a translation generator when unfolding (required for translations).
    pub repeat: Option<u32>,
}

/// How the symmetry came to be.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SymmetryOrigin {
    /// Stated by the user.
    Declared,
    /// Proposed by [`Model::detect_symmetry`].
    Detected {
        /// Its verification at declaration.
        verification: Verification,
    },
    /// Present in imported pre-cut geometry.
    Imported,
}

/// The symmetry of a model: generators of the deck group. The fundamental domain is the model domain.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SymmetrySpec {
    /// Generators in application order.
    pub generators: Vec<GeneratorSpec>,
    /// Provenance.
    pub origin: SymmetryOrigin,
}

/// Which side of a mirror plane to keep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeepSide {
    /// The side the plane normal points to.
    Positive,
    /// The opposite side.
    Negative,
}

/// A cut-and-keep-chamber request.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CutSpec {
    /// The generator whose chamber is kept.
    pub generator: Generator,
    /// Side to keep (mirror only).
    pub keep: KeepSide,
    /// Direction (perpendicular to the axis) of the wedge's starting plane (rotation only).
    /// `None` picks a deterministic default.
    pub wedge_start: Option<Vector3>,
}

/// What a cut did.
#[derive(Clone, Debug, PartialEq)]
pub struct CutReport {
    /// Volumes kept in the new domain, ascending.
    pub kept: Vec<EntityId>,
    /// The new deck pairing.
    pub pairing: EntityId,
    /// Faces tagged as deck boundaries (A side first, then B side), ascending within each side.
    pub tagged: Vec<EntityId>,
}

/// Half-spaces (kept where `signed_distance >= 0`) whose intersection is the chamber.
pub fn chamber_planes(spec: &CutSpec) -> Result<Vec<Plane>, ModelError> {
    spec.generator.validate()?;
    match spec.generator {
        Generator::Mirror { plane } => Ok(vec![if spec.keep == KeepSide::Negative { plane.flipped() } else { plane }]),
        Generator::Translation { origin, vector } => {
            let t = unit(vector).ok_or_else(|| ModelError::BadGenerator("zero translation".into()))?;
            Ok(vec![Plane { point: origin, normal: t }, Plane { point: origin + vector, normal: -t }])
        }
        Generator::Rotation { origin, axis, order } => {
            if order < 3 {
                return Err(ModelError::BadGenerator("a wedge needs rotation order >= 3".into()));
            }
            let helper = spec.wedge_start.unwrap_or_else(|| {
                // Coordinate axis least aligned with `axis`.
                let a = [axis.x.abs(), axis.y.abs(), axis.z.abs()];
                if a[0] <= a[1] && a[0] <= a[2] {
                    Vector3::new(1.0, 0.0, 0.0)
                } else if a[1] <= a[2] {
                    Vector3::new(0.0, 1.0, 0.0)
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                }
            });
            let e1 = unit(helper - axis * helper.dot(axis)).ok_or_else(|| ModelError::BadGenerator("wedge start is parallel to the axis".into()))?;
            let e2 = axis.cross(e1);
            let th = std::f64::consts::TAU / f64::from(order);
            let (s, c) = th.sin_cos();
            Ok(vec![Plane { point: origin, normal: e2 }, Plane { point: origin, normal: e1 * s - e2 * c }])
        }
    }
}

impl Model {
    /// Declare (or replace) the symmetry. Pairings, if given, must exist and match the generator.
    pub fn declare_symmetry(&mut self, generators: Vec<GeneratorSpec>, origin: SymmetryOrigin) -> Result<(), ModelError> {
        for g in &generators {
            g.generator.validate()?;
            if let Some(p) = g.pairing {
                let pr = self.pairings.get(&p).ok_or(ModelError::UnknownEntity(p))?;
                if pr.generator.kind() != g.generator.kind() {
                    return Err(ModelError::Invalid("pairing kind does not match generator".into()));
                }
            }
        }
        self.symmetry = Some(SymmetrySpec { generators, origin });
        Ok(())
    }

    /// Adopt a detected candidate. Unless `force`, it must be [`Verification::Verified`].
    pub fn declare_detected(&mut self, cand: &Candidate, force: bool) -> Result<(), ModelError> {
        if !force && !matches!(cand.verification, Verification::Verified { .. }) {
            return Err(ModelError::NotVerified);
        }
        let spec = GeneratorSpec { generator: cand.generator, pairing: None, repeat: None };
        self.declare_symmetry(vec![spec], SymmetryOrigin::Detected { verification: cand.verification })
    }

    /// Cut to the chamber of a generator and keep it.
    ///
    /// Selects the domain volumes whose barycentre is inside every half-space, makes that the
    /// domain, finds the domain-boundary faces lying on the cut planes and tags them as a deck
    /// pairing, then records the generator in the symmetry. On error the model is unchanged.
    pub fn cut_keep_chamber(&mut self, topo: &dyn TopologyView, spec: &CutSpec, tol: &Tolerance) -> Result<CutReport, ModelError> {
        let planes = chamber_planes(spec)?;
        let old_domain = self.domain.clone();
        let kept: Vec<EntityId> = self
            .domain_volumes(topo)
            .into_iter()
            .filter(|v| topo.volume_mass_properties(*v).is_some_and(|m| planes.iter().all(|p| p.signed_distance(m.centroid) >= -tol.length)))
            .collect();
        if kept.is_empty() {
            return Err(ModelError::Invalid("the chamber contains no volume".into()));
        }
        self.domain = Some(Selection::geometry(kept.iter().copied())?);
        match self.finish_cut(topo, spec, &planes, tol) {
            Ok((pairing, tagged)) => Ok(CutReport { kept, pairing, tagged }),
            Err(e) => {
                self.domain = old_domain;
                Err(e)
            }
        }
    }

    fn finish_cut(&mut self, topo: &dyn TopologyView, spec: &CutSpec, planes: &[Plane], tol: &Tolerance) -> Result<(EntityId, Vec<EntityId>), ModelError> {
        let boundary = self.domain_boundary_faces(topo);
        let on_plane = |p: &Plane| -> Vec<EntityId> {
            boundary
                .iter()
                .copied()
                .filter(|f| {
                    topo.face_properties(*f).is_some_and(|fp| {
                        p.signed_distance(fp.centroid).abs() <= tol.length && fp.normal.dot(p.normal).abs() >= 1.0 - tol.angle
                    })
                })
                .collect()
        };
        let sets: Vec<Vec<EntityId>> = planes.iter().map(on_plane).collect();
        if let Some(i) = sets.iter().position(Vec::is_empty) {
            return Err(ModelError::NoCutFaces { index: i });
        }
        let (pairs, tagged) = if spec.generator.kind() == DeckKind::Mirror {
            (sets[0].iter().map(|f| (*f, *f)).collect::<Vec<_>>(), sets[0].clone())
        } else {
            let pairs = match_pairs(&sets[0], &sets[1], &spec.generator.transform(), topo, tol)?;
            let tagged = sets[0].iter().chain(&sets[1]).copied().collect();
            (pairs, tagged)
        };
        let pairing = self.declare_deck(spec.generator, pairs)?;
        let sym = self.symmetry.get_or_insert(SymmetrySpec { generators: Vec::new(), origin: SymmetryOrigin::Declared });
        match sym.generators.iter_mut().find(|g| g.generator == spec.generator && g.pairing.is_none()) {
            Some(g) => g.pairing = Some(pairing),
            None => sym.generators.push(GeneratorSpec { generator: spec.generator, pairing: Some(pairing), repeat: None }),
        }
        Ok((pairing, tagged))
    }

    /// Apply a [`Transform`] to the domain and check that it maps the colored model onto itself.
    pub fn verify_transform(&self, topo: &dyn TopologyView, tr: &Transform, tol: &Tolerance) -> Verification {
        let vols: Vec<(EntityId, MassProps)> =
            self.domain_volumes(topo).into_iter().filter_map(|v| topo.volume_mass_properties(v).map(|m| (v, m))).collect();
        if vols.is_empty() {
            return Verification::Unverified;
        }
        // Stage 1: per-color invariants.
        for inv in color_invariants_of(self, &vols) {
            let img = tr.apply_point(inv.barycentre);
            let d = (img - inv.barycentre).norm();
            let s = &inv.second_moment;
            let ms = conj(tr, s);
            let scale = s[0][0] + s[1][1] + s[2][2];
            let dm = (0..3).flat_map(|i| (0..3).map(move |j| (i, j))).map(|(i, j)| (ms[i][j] - s[i][j]).abs()).fold(0.0, f64::max);
            if d > tol.length {
                return Verification::Rejected { stage: RejectStage::ColorInvariants, residual: d };
            }
            if dm > tol.relative * scale + 1e-12 {
                return Verification::Rejected { stage: RejectStage::ColorInvariants, residual: dm };
            }
        }
        let mut max_res = 0.0f64;
        // Stage 2: every volume has a same-colored image.
        for (v, m) in &vols {
            let img = tr.apply_point(m.centroid);
            let color = self.volume_color_of(*v);
            let ms = conj(tr, &m.second_moment);
            let scale = m.second_moment[0][0] + m.second_moment[1][1] + m.second_moment[2][2];
            let mut best = f64::INFINITY;
            for (w, n) in &vols {
                if self.volume_color_of(*w) != color || (n.volume - m.volume).abs() > tol.relative * m.volume.abs().max(n.volume.abs()) {
                    continue;
                }
                let d = (n.centroid - img).norm();
                let dm = (0..3).flat_map(|i| (0..3).map(move |j| (i, j))).map(|(i, j)| (ms[i][j] - n.second_moment[i][j]).abs()).fold(0.0, f64::max);
                if dm <= tol.relative * scale + 1e-12 {
                    best = best.min(d);
                }
            }
            if best > tol.length {
                return Verification::Rejected { stage: RejectStage::Volumes, residual: best };
            }
            max_res = max_res.max(best);
        }
        // Stage 3: faces, with side colors, boundary color and boundary kind.
        let faces: Vec<FaceSig> = topo.faces().into_iter().filter_map(|f| self.face_sig(f, topo)).collect();
        for f in &faces {
            let img = tr.apply_point(f.centroid);
            let n_img = tr.apply_vector(f.normal);
            let mut best = f64::INFINITY;
            for g in &faces {
                if g.sides != f.sides || g.color != f.color || g.kind != f.kind || (g.area - f.area).abs() > tol.relative * f.area.max(g.area) {
                    continue;
                }
                if n_img.dot(g.normal).abs() < 1.0 - tol.angle {
                    continue;
                }
                best = best.min((g.centroid - img).norm());
            }
            if best > tol.length {
                return Verification::Rejected { stage: RejectStage::Faces, residual: best };
            }
            max_res = max_res.max(best);
        }
        Verification::Verified { max_residual: max_res }
    }

    fn face_sig(&self, f: EntityId, topo: &dyn TopologyView) -> Option<FaceSig> {
        self.domain_sides(f, topo)?;
        let fp = topo.face_properties(f)?;
        let s = topo.face_sides(f)?;
        let col = |v: Option<EntityId>| v.filter(|v| self.in_domain(*v)).and_then(|v| self.volume_color_of(v));
        let (a, b) = (col(s.left), col(s.right));
        let kind = self.kind_of_face(f).map(|k| match k {
            BoundaryKind::Deck { pairing, side } => {
                (0u8, format!("{:?}{:?}", self.pairings.get(pairing).map(crate::boundary::DeckPairing::kind), matches!(side, DeckSide::SelfPaired)))
            }
            BoundaryKind::Response { tag } => (1, tag.clone()),
            BoundaryKind::Interface(_) => (2, String::new()),
        });
        Some(FaceSig { centroid: fp.centroid, normal: fp.normal, area: fp.area, sides: if a <= b { (a, b) } else { (b, a) }, color: self.face_color_of(f), kind })
    }

    /// Per-color invariants of the domain: measure, barycentre and second moment about it.
    pub fn color_invariants(&self, topo: &dyn TopologyView) -> Vec<ColorInvariant> {
        let vols: Vec<(EntityId, MassProps)> =
            self.domain_volumes(topo).into_iter().filter_map(|v| topo.volume_mass_properties(v).map(|m| (v, m))).collect();
        color_invariants_of(self, &vols)
    }

    /// Propose symmetries from colored invariants and verify each.
    ///
    /// Every symmetry of the colored domain fixes its barycentre `G`. Mirror candidates are the
    /// perpendicular bisectors of same-color volume pairs plus the coordinate planes through `G`;
    /// rotation candidates are axes through `G` along the coordinate axes and along differences
    /// of same-color barycentres, with the largest verified order. The search is a heuristic
    /// (it can miss symmetries); verification is exact up to tolerance. Translations are not
    /// detected (a finite model has none). Output order is deterministic.
    pub fn detect_symmetry(&self, topo: &dyn TopologyView, opts: &DetectOptions) -> DetectionResult {
        let vols: Vec<(EntityId, MassProps)> =
            self.domain_volumes(topo).into_iter().filter_map(|v| topo.volume_mass_properties(v).map(|m| (v, m))).collect();
        let invariants = color_invariants_of(self, &vols);
        let total: f64 = vols.iter().map(|(_, m)| m.volume).sum();
        if vols.is_empty() || total <= 0.0 {
            return DetectionResult { barycentre: None, invariants, candidates: Vec::new() };
        }
        let g = {
            let s = vols.iter().fold(Vector3::new(0.0, 0.0, 0.0), |a, (_, m)| a + Vector3::new(m.centroid.x, m.centroid.y, m.centroid.z) * m.volume);
            Point3::new(s.x / total, s.y / total, s.z / total)
        };
        let mut planes: BTreeMap<[i64; 4], (Plane, Evidence)> = BTreeMap::new();
        let mut axes: BTreeMap<[i64; 3], (Vector3, Evidence)> = BTreeMap::new();
        let q = |x: f64| (x * 1e6).round() as i64;
        let add_axis = |dir: Vector3, ev: Evidence, axes: &mut BTreeMap<[i64; 3], (Vector3, Evidence)>| {
            if let Some(mut d) = unit(dir) {
                let flip = [d.x, d.y, d.z].iter().find(|c| c.abs() > 1e-9).is_some_and(|c| *c < 0.0);
                if flip {
                    d = -d;
                }
                axes.entry([q(d.x), q(d.y), q(d.z)]).or_insert((d, ev));
            }
        };
        let add_plane = |p: Point3, n: Vector3, ev: Evidence, planes: &mut BTreeMap<[i64; 4], (Plane, Evidence)>| {
            if let Some(mut n) = unit(n) {
                let flip = [n.x, n.y, n.z].iter().find(|c| c.abs() > 1e-9).is_some_and(|c| *c < 0.0);
                if flip {
                    n = -n;
                }
                let d = n.dot(Vector3::new(p.x, p.y, p.z));
                planes.entry([q(n.x), q(n.y), q(n.z), q(d)]).or_insert((Plane { point: p, normal: n }, ev));
            }
        };
        for (i, e) in [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)].into_iter().enumerate() {
            add_plane(g, e, Evidence::CoordinatePlane { axis: i as u8 }, &mut planes);
            add_axis(e, Evidence::CoordinateAxis { axis: i as u8 }, &mut axes);
        }
        for (i, (a, ma)) in vols.iter().enumerate() {
            for (b, mb) in &vols[i + 1..] {
                if self.volume_color_of(*a) != self.volume_color_of(*b) {
                    continue;
                }
                let d = mb.centroid - ma.centroid;
                if d.norm() <= opts.tol.length {
                    continue;
                }
                let mid = Point3::new((ma.centroid.x + mb.centroid.x) / 2.0, (ma.centroid.y + mb.centroid.y) / 2.0, (ma.centroid.z + mb.centroid.z) / 2.0);
                add_plane(mid, d, Evidence::BisectsPair { a: *a, b: *b }, &mut planes);
                add_axis(d, Evidence::PairDirection { a: *a, b: *b }, &mut axes);
            }
        }
        let mut found: Vec<(u8, [i64; 4], Candidate)> = Vec::new();
        for (key, (plane, evidence)) in planes {
            let generator = Generator::Mirror { plane };
            let verification = self.verify_transform(topo, &generator.transform(), &opts.tol);
            found.push((0, key, Candidate { generator, evidence, verification }));
        }
        for (key, (axis, evidence)) in axes {
            let mut best: Option<Candidate> = None;
            for order in (2..=opts.max_rotation_order).rev() {
                let generator = Generator::Rotation { origin: g, axis, order };
                let verification = self.verify_transform(topo, &generator.transform(), &opts.tol);
                if matches!(verification, Verification::Verified { .. }) {
                    best = Some(Candidate { generator, evidence, verification });
                    break;
                }
                if order == 2 {
                    best = Some(Candidate { generator, evidence, verification });
                }
            }
            if let Some(c) = best {
                found.push((1, [key[0], key[1], key[2], 0], c));
            }
        }
        let mut candidates: Vec<(u8, u8, [i64; 4], Candidate)> = found
            .into_iter()
            .filter(|(_, _, c)| opts.include_rejected || matches!(c.verification, Verification::Verified { .. }))
            .map(|(k, key, c)| (u8::from(!matches!(c.verification, Verification::Verified { .. })), k, key, c))
            .collect();
        candidates.sort_by_key(|c| (c.0, c.1, c.2));
        DetectionResult { barycentre: Some(g), invariants, candidates: candidates.into_iter().map(|c| c.3).collect() }
    }

    /// Derive the full model from the fundamental domain and the deck group.
    ///
    /// Copy 0 keeps the original ids. Copies `c > 0` get derived ids `kind#(id_base + n)` in
    /// canonical order (copy, then source id). Faces of paired deck sets that coincide across
    /// copies are merged into one interior face; faces in reflected copies have their loop
    /// reversed (`loop_reversed`), with left/right kept so normals still point outward from `left`.
    pub fn unfold(&self, topo: &dyn TopologyView, opts: &UnfoldOptions) -> Result<UnfoldedModel, ModelError> {
        let sym = self.symmetry.as_ref().ok_or(ModelError::NoSymmetry)?;
        let mut point = Vec::new();
        let mut trans = Vec::new();
        for (i, g) in sym.generators.iter().enumerate() {
            g.generator.validate()?;
            match g.generator.kind() {
                DeckKind::Translation => trans.push((g.generator.transform(), g.repeat.ok_or(ModelError::UnboundedGenerator { index: i })?)),
                _ => point.push(g.generator.transform()),
            }
        }
        let elements = enumerate_elements(&point, &trans, &opts.enumerate)?;
        let index: BTreeMap<TransformKey, usize> = elements.iter().enumerate().map(|(i, e)| (TransformKey::new(&e.transform, 1e-7), i)).collect();
        let domain = self.domain_volumes(topo);
        let faces: Vec<(EntityId, crate::model::DomainSides)> =
            topo.faces().into_iter().filter_map(|f| self.domain_sides(f, topo).map(|d| (f, d))).collect();

        // Gluing: which (face, copy) instances are dropped, and what the surviving one gains on its right.
        let mut dropped: BTreeSet<(EntityId, usize)> = BTreeSet::new();
        let mut gained: BTreeMap<(EntityId, usize), (EntityId, usize)> = BTreeMap::new();
        let near: BTreeMap<EntityId, EntityId> = faces.iter().map(|(f, d)| (*f, d.near)).collect();
        for (f, _) in &faces {
            let Some(BoundaryKind::Deck { pairing, side }) = self.kind_of_face(*f) else { continue };
            let Some(pr) = self.pairings.get(pairing) else { continue };
            let s = pr.generator.transform();
            let partner = match side {
                DeckSide::SelfPaired => Some(*f),
                DeckSide::B => pr.pairs.iter().find(|p| p.1 == *f).map(|p| p.0),
                DeckSide::A => None,
            };
            let Some(partner) = partner else { continue };
            for (c, e) in elements.iter().enumerate() {
                let Some(&c2) = index.get(&TransformKey::new(&compose(&e.transform, &s), 1e-7)) else { continue };
                // Self-paired: the lower copy survives. Paired: the A instance survives.
                let drop = match side {
                    DeckSide::SelfPaired => c2 < c,
                    _ => c2 != c,
                };
                if drop {
                    dropped.insert((*f, c));
                    gained.insert((partner, c2), (near[f], c));
                }
            }
        }

        let mut counters: BTreeMap<EntityKind, u64> = BTreeMap::new();
        let mut derive = |kind: EntityKind, src: EntityId, copy: usize| -> Result<EntityId, ModelError> {
            if copy == 0 {
                return Ok(src);
            }
            let n = counters.entry(kind).or_insert(0);
            *n += 1;
            EntityId::new(kind, opts.id_base + *n).ok_or(ModelError::SerialOverflow(kind))
        };
        let mut vol_id: BTreeMap<(EntityId, usize), EntityId> = BTreeMap::new();
        let mut volumes = Vec::new();
        let mut face_rows = Vec::new();
        let mut origin = BTreeMap::new();
        for c in 0..elements.len() {
            for v in &domain {
                let id = derive(EntityKind::TopoVolume, *v, c)?;
                vol_id.insert((*v, c), id);
                origin.insert(id, (*v, c as u32));
                volumes.push(UnfoldedVolume { id, source: *v, copy: c as u32, color: self.volume_color_of(*v) });
            }
        }
        for c in 0..elements.len() {
            for (f, d) in &faces {
                if dropped.contains(&(*f, c)) {
                    continue;
                }
                let id = derive(EntityKind::TopoFace, *f, c)?;
                origin.insert(id, (*f, c as u32));
                let right = match d.far {
                    Some(far) => Some(vol_id[&(far, c)]),
                    None => gained.get(&(*f, c)).map(|(v, c2)| vol_id[&(*v, *c2)]),
                };
                let glued = d.far.is_none() && right.is_some();
                face_rows.push(UnfoldedFace {
                    id,
                    source: *f,
                    copy: c as u32,
                    loop_reversed: elements[c].orientation_reversed ^ d.swapped,
                    left: vol_id[&(d.near, c)],
                    right,
                    kind_set: if glued { None } else { self.kind_set_of_face(*f) },
                    color: self.face_color_of(*f),
                });
            }
        }
        Ok(UnfoldedModel { elements, volumes, faces: face_rows, origin, domain, generators: sym.generators.clone() })
    }
}

/// Pair faces of `a` with faces of `b` under `s` (`b = s(a)`): centroid within tolerance, equal area.
fn match_pairs(a: &[EntityId], b: &[EntityId], s: &Transform, topo: &dyn TopologyView, tol: &Tolerance) -> Result<Vec<(EntityId, EntityId)>, ModelError> {
    let mut used: BTreeSet<EntityId> = BTreeSet::new();
    let mut out = Vec::new();
    for fa in a {
        let pa = topo.face_properties(*fa).ok_or(ModelError::UnknownEntity(*fa))?;
        let target = s.apply_point(pa.centroid);
        let mut best: Option<(f64, EntityId)> = None;
        for fb in b.iter().filter(|x| !used.contains(x)) {
            let pb = topo.face_properties(*fb).ok_or(ModelError::UnknownEntity(*fb))?;
            let d = (pb.centroid - target).norm();
            if d <= tol.length && (pb.area - pa.area).abs() <= tol.relative * pa.area.max(pb.area) && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, *fb));
            }
        }
        let (_, fb) = best.ok_or(ModelError::PairingFailed { face: *fa })?;
        used.insert(fb);
        out.push((*fa, fb));
    }
    if let Some(left) = b.iter().find(|x| !used.contains(x)) {
        return Err(ModelError::PairingFailed { face: *left });
    }
    Ok(out)
}

/// `M S M^T`.
fn conj(t: &Transform, s: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let m = &t.m;
    let mut ms = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            ms[i][j] = (0..3).map(|k| m[i][k] * s[k][j]).sum();
        }
    }
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| ms[i][k] * m[j][k]).sum();
        }
    }
    out
}

struct FaceSig {
    centroid: Point3,
    normal: Vector3,
    area: f64,
    sides: (Option<EntityId>, Option<EntityId>),
    color: Option<EntityId>,
    kind: Option<(u8, String)>,
}

fn color_invariants_of(model: &Model, vols: &[(EntityId, MassProps)]) -> Vec<ColorInvariant> {
    let mut by: BTreeMap<Option<EntityId>, Vec<&MassProps>> = BTreeMap::new();
    for (v, m) in vols {
        by.entry(model.volume_color_of(*v)).or_default().push(m);
    }
    by.into_iter()
        .map(|(color, ms)| {
            let volume: f64 = ms.iter().map(|m| m.volume).sum();
            let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
            for m in &ms {
                sx += m.centroid.x * m.volume;
                sy += m.centroid.y * m.volume;
                sz += m.centroid.z * m.volume;
            }
            let b = if volume > 0.0 { Point3::new(sx / volume, sy / volume, sz / volume) } else { ms[0].centroid };
            let mut s = [[0.0; 3]; 3];
            for m in &ms {
                let d = m.centroid - b;
                let dv = [d.x, d.y, d.z];
                for i in 0..3 {
                    for j in 0..3 {
                        s[i][j] += m.second_moment[i][j] + m.volume * dv[i] * dv[j];
                    }
                }
            }
            ColorInvariant { color, volume, barycentre: b, second_moment: s }
        })
        .collect()
}

/// Invariants of one color class, the input of symmetry detection.
#[derive(Clone, Debug, PartialEq)]
pub struct ColorInvariant {
    /// The color (`None`: uncolored volumes).
    pub color: Option<EntityId>,
    /// Total measure.
    pub volume: f64,
    /// Barycentre of the color class.
    pub barycentre: Point3,
    /// Second moment tensor about the barycentre (inertia tensor is `tr(S) I - S`).
    pub second_moment: [[f64; 3]; 3],
}

/// Why a candidate was proposed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Evidence {
    /// The plane bisects the barycentres of two same-color volumes.
    BisectsPair {
        /// First volume.
        a: EntityId,
        /// Second volume.
        b: EntityId,
    },
    /// Coordinate plane through the barycentre.
    CoordinatePlane {
        /// 0, 1, 2 for x, y, z.
        axis: u8,
    },
    /// Axis along the barycentre difference of two same-color volumes.
    PairDirection {
        /// First volume.
        a: EntityId,
        /// Second volume.
        b: EntityId,
    },
    /// Coordinate axis through the barycentre.
    CoordinateAxis {
        /// 0, 1, 2 for x, y, z.
        axis: u8,
    },
}

/// A proposed generator with its verification status.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    /// The proposed generator (rotation: the largest verified order for the axis).
    pub generator: Generator,
    /// How it was found.
    pub evidence: Evidence,
    /// Verification status.
    pub verification: Verification,
}

/// Output of [`Model::detect_symmetry`].
#[derive(Clone, Debug, PartialEq)]
pub struct DetectionResult {
    /// Barycentre of the domain.
    pub barycentre: Option<Point3>,
    /// Per-color invariants used.
    pub invariants: Vec<ColorInvariant>,
    /// Candidates: verified first, planes before axes, then by canonical key.
    pub candidates: Vec<Candidate>,
}

/// Options for detection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetectOptions {
    /// Verification tolerances.
    pub tol: Tolerance,
    /// Highest rotation order tried per axis.
    pub max_rotation_order: u32,
    /// Also return candidates that failed verification.
    pub include_rejected: bool,
}

impl Default for DetectOptions {
    fn default() -> Self {
        Self { tol: Tolerance::default(), max_rotation_order: 12, include_rejected: false }
    }
}

/// Options for [`Model::unfold`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnfoldOptions {
    /// Derived ids are `kind#(id_base + n)`; pick a base above every serial in use.
    pub id_base: u64,
    /// Group enumeration cap.
    pub enumerate: EnumerateOptions,
}

impl Default for UnfoldOptions {
    fn default() -> Self {
        Self { id_base: 1 << 40, enumerate: EnumerateOptions::default() }
    }
}

/// A volume of the unfolded model.
#[derive(Clone, Debug, PartialEq)]
pub struct UnfoldedVolume {
    /// Derived id (equals `source` for copy 0).
    pub id: EntityId,
    /// Volume in the fundamental domain.
    pub source: EntityId,
    /// Group element index.
    pub copy: u32,
    /// Color (copied from the source).
    pub color: Option<EntityId>,
}

/// A face of the unfolded model.
#[derive(Clone, Debug, PartialEq)]
pub struct UnfoldedFace {
    /// Derived id (equals `source` for copy 0).
    pub id: EntityId,
    /// Face in the fundamental domain.
    pub source: EntityId,
    /// Group element index.
    pub copy: u32,
    /// The loop must be reversed relative to the source (improper copy, or source normal pointed inward).
    pub loop_reversed: bool,
    /// Volume the normal points away from.
    pub left: EntityId,
    /// Volume the normal points into (`None`: boundary of the unfolded model).
    pub right: Option<EntityId>,
    /// Boundary kind set, if the face is still a boundary face (merged deck faces become interior).
    pub kind_set: Option<EntityId>,
    /// Boundary color (copied from the source).
    pub color: Option<EntityId>,
}

/// The derived full model. Reversible: [`Self::source_of`] and [`Self::fundamental_domain`] recover the input.
#[derive(Clone, Debug, PartialEq)]
pub struct UnfoldedModel {
    /// One placement per group element; element 0 is the identity.
    pub elements: Vec<Placement>,
    /// All volumes, copy-major then source id.
    pub volumes: Vec<UnfoldedVolume>,
    /// All surviving faces, copy-major then source id.
    pub faces: Vec<UnfoldedFace>,
    origin: BTreeMap<EntityId, (EntityId, u32)>,
    domain: Vec<EntityId>,
    generators: Vec<GeneratorSpec>,
}

impl UnfoldedModel {
    /// The fundamental-domain entity and copy index a derived id comes from.
    pub fn source_of(&self, id: EntityId) -> Option<(EntityId, u32)> {
        self.origin.get(&id).copied()
    }
    /// The fundamental domain this was unfolded from.
    pub fn fundamental_domain(&self) -> &[EntityId] {
        &self.domain
    }
    /// The generators this was unfolded with.
    pub fn generators(&self) -> &[GeneratorSpec] {
        &self.generators
    }
    /// Refold: the volumes of copy 0, which are exactly the fundamental domain.
    pub fn refold(&self) -> Vec<EntityId> {
        self.volumes.iter().filter(|v| v.copy == 0).map(|v| v.id).collect()
    }
}
