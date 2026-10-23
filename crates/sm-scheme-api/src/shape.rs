//! [`ShapeView`]: the problem abstraction every scheme and the driver read.
//!
//! Schemes never walk `TopologyView` rows or the geometry store directly. They ask for loops of
//! curve uses, arc-length points, corner angles, face uses of a volume, deck links and sweep
//! hints, all named by [`ShapeKey`]. [`TopoShape`] answers these from an `sm-topo-api` view (real
//! complex or decomposition overlay), the `sm-geom-api` store behind it and, optionally, the
//! `sm-model-api` [`Model`] (decks, decomposition pieces). Schemes can be tested against a
//! synthetic `ShapeView` without a kernel.

use crate::error::SchemeError;
use crate::key::ShapeKey;
use crate::vertex::interior_angle;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_geom_api::{CurveDef, Interval};
use sm_model_api::{Deck, DeckSide, FaceRole, Model, PieceFace, PieceRef, PieceShape};
use sm_topo_api::{Carrier, CellFlags, CoedgeKey, CornerKey, Dim, Sense, Side, TopologyView, TopologyViewExt};

/// One use of a curve in a face loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurveUse {
    /// The curve.
    pub curve: ShapeKey,
    /// `Forward`: the loop runs along the curve's direction.
    pub sense: Sense,
    /// The vertex where this use starts (in loop direction).
    pub start: ShapeKey,
    /// Stable name of the corner at `start` in this face (vertex types are keyed by it).
    pub corner: Option<CornerKey>,
}

/// A loop of a face: curve uses in loop order (ring 0 = outer).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LoopUse {
    /// Loop index.
    pub ring: u16,
    /// The uses, cyclic.
    pub uses: Vec<CurveUse>,
}

/// One face of a volume (or piece) as the volume sees it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct FaceUseRef {
    /// The face.
    pub face: ShapeKey,
    /// `Forward`: the face normal (right-hand rule on its loops) points out of the volume.
    pub sense: Sense,
}

/// A deck pairing lowered to meshing terms (ir.md §5.1).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DeckLink {
    /// Translation / rotation pair: the mesh on `b` must equal `transform(mesh on a)`.
    Pair {
        /// Side A face.
        a: ShapeKey,
        /// Side B face, `B = transform(A)`.
        b: ShapeKey,
        /// A -> B.
        transform: Transform,
        /// The boundary record.
        boundary: EntityId,
    },
    /// Mirror face: nodes stay on the plane, no boundary layers.
    Mirror {
        /// The face.
        face: ShapeKey,
        /// Plane unit normal.
        normal: Vector3,
        /// Plane offset (`n . p = offset`).
        offset: f64,
        /// The boundary record.
        boundary: EntityId,
    },
}

/// Sweep roles declared by the decomposition (`PieceShape::Sweep`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SweepHint {
    /// Source faces.
    pub source: Vec<ShapeKey>,
    /// Target faces.
    pub target: Vec<ShapeKey>,
}

/// Read-only shape queries for planning and meshing. Object safe, `Send + Sync`, deterministic.
pub trait ShapeView: Send + Sync {
    /// Rank of `key` if it names a live entity of this view.
    fn dim(&self, key: ShapeKey) -> Option<Dim>;
    /// Global linear tolerance (model length units).
    fn linear_tolerance(&self) -> f64;
    /// Position of a vertex.
    fn vertex_point(&self, v: ShapeKey) -> Result<Point3, SchemeError>;
    /// `(start, end)` vertices of a curve (equal for a closed curve).
    fn curve_ends(&self, c: ShapeKey) -> Result<(ShapeKey, ShapeKey), SchemeError>;
    /// Arc length of a curve.
    fn curve_length(&self, c: ShapeKey) -> Result<f64, SchemeError>;
    /// The point at arc-length `fraction` in `[0, 1]` from the curve's start, and the curve
    /// parameter there (for `Param::Curve` classification).
    fn curve_point(&self, c: ShapeKey, fraction: f64) -> Result<(Point3, f64), SchemeError>;
    /// Loops of a face, outer first.
    fn face_loops(&self, f: ShapeKey) -> Result<Vec<LoopUse>, SchemeError>;
    /// Interior angle (radians) of face `f` at the start of use `index` of loop `ring`.
    fn corner_angle(&self, f: ShapeKey, ring: u16, index: usize) -> Result<f64, SchemeError>;
    /// Closest point on the face's surface and its parameters (identity without a surface).
    fn snap_to_face(&self, f: ShapeKey, p: Point3) -> Result<(Point3, Option<[f64; 2]>), SchemeError>;
    /// Faces of a volume or piece with their senses, ascending.
    fn volume_faces(&self, v: ShapeKey) -> Result<Vec<FaceUseRef>, SchemeError>;
    /// Deck links of the model (empty without a model), ascending by face.
    fn deck_links(&self) -> Vec<DeckLink>;
    /// Sweep roles declared on a piece, if any.
    fn sweep_hint(&self, v: ShapeKey) -> Option<SweepHint>;
    /// The model/partition entity mesh entities on `key` are classified on.
    fn classify_on(&self, key: ShapeKey) -> Result<EntityId, SchemeError>;

    /// Distinct curves of a face, ascending.
    fn face_curves(&self, f: ShapeKey) -> Result<Vec<ShapeKey>, SchemeError> {
        let mut v: Vec<ShapeKey> = self.face_loops(f)?.iter().flat_map(|l| l.uses.iter().map(|u| u.curve)).collect();
        v.sort();
        v.dedup();
        Ok(v)
    }

    /// Curve correspondence of a deck pair: for every curve of `b` that is the image under `t`
    /// of a *different* curve of `a`, `(curve of a, curve of b, relative sense)`. Matched by
    /// mapped midpoints and start points within 16 x the linear tolerance (scaled by length).
    fn corresponding_curves(&self, a: ShapeKey, b: ShapeKey, t: &Transform) -> Result<Vec<(ShapeKey, ShapeKey, Sense)>, SchemeError> {
        let ca = self.face_curves(a)?;
        let cb = self.face_curves(b)?;
        let mut out = Vec::new();
        for &y in &cb {
            let (my, _) = self.curve_point(y, 0.5)?;
            let (sy, _) = self.curve_point(y, 0.0)?;
            let tol = 16.0 * self.linear_tolerance() * (1.0 + self.curve_length(y)?);
            for &x in &ca {
                if x == y {
                    continue;
                }
                let (mx, _) = self.curve_point(x, 0.5)?;
                if (t.apply_point(mx) - my).norm() > tol {
                    continue;
                }
                let (sx, _) = self.curve_point(x, 0.0)?;
                let sense = if (t.apply_point(sx) - sy).norm() <= tol { Sense::Forward } else { Sense::Reversed };
                out.push((x, y, sense));
                break;
            }
        }
        Ok(out)
    }
}

/// [`ShapeView`] over a topology view, its geometry store and an optional model.
#[derive(Clone, Copy)]
pub struct TopoShape<'a> {
    topo: &'a dyn TopologyView,
    model: Option<&'a Model>,
}

const EPS_FRACTION: f64 = 1e-4;

impl<'a> TopoShape<'a> {
    /// Topology only (no decks, no pieces).
    pub fn new(topo: &'a dyn TopologyView) -> Self {
        TopoShape { topo, model: None }
    }

    /// Topology plus the model layer.
    pub fn with_model(topo: &'a dyn TopologyView, model: &'a Model) -> Self {
        TopoShape { topo, model: Some(model) }
    }

    /// The underlying topology.
    pub fn topology(&self) -> &'a dyn TopologyView {
        self.topo
    }

    fn cell(&self, key: ShapeKey, want: Dim) -> Result<EntityId, SchemeError> {
        match key.canonical() {
            ShapeKey::Cell(id) if self.topo.dim_of(id) == Some(want) && self.topo.is_alive(id) => Ok(id),
            ShapeKey::PieceFace(_) if want == Dim::Face => Err(SchemeError::Unsupported(format!("{key}: a partial or cut piece face needs an overlay realization"))),
            _ => Err(SchemeError::shape(key, format!("not a live {want:?} of this view"))),
        }
    }

    fn piece(&self, key: ShapeKey) -> Option<&'a sm_model_api::Piece> {
        match key {
            ShapeKey::Piece(PieceRef::Cell(pc)) => self.model?.decomposition().piece(pc),
            _ => None,
        }
    }

    /// Edge parameter at parametric fraction `s` along the edge direction.
    fn edge_param(cc: &sm_topo_api::CurveCarrier, s: f64) -> f64 {
        let (lo, hi) = (cc.range.lo, cc.range.hi);
        match cc.sense {
            Sense::Forward => lo + s * (hi - lo),
            Sense::Reversed => hi - s * (hi - lo),
        }
    }

    fn face_normal(&self, f: EntityId, p: Point3) -> Result<Vector3, SchemeError> {
        if let Carrier::Surface(sc) = self.topo.carrier(f)
            && let Some(s) = self.topo.shapes().surface(sc.surface)
        {
            let pr = s.project(p, None).map_err(|e| SchemeError::shape(f.into(), e.to_string()))?;
            let n = s.normal(pr.uv).map_err(|e| SchemeError::shape(f.into(), e.to_string()))?;
            return Ok(n * sc.sense.sign());
        }
        // Newell normal of the outline.
        let pts = self.topo.face_outline(f);
        let mut n = Vector3::new(0.0, 0.0, 0.0);
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            n = n + Vector3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
        }
        let l = n.norm();
        if l > 0.0 { Ok(n * (1.0 / l)) } else { Err(SchemeError::shape(f.into(), "degenerate face normal")) }
    }

    fn use_point(&self, u: &CurveUse, along: f64) -> Result<Point3, SchemeError> {
        let f = if u.sense == Sense::Forward { along } else { 1.0 - along };
        Ok(self.curve_point(u.curve, f)?.0)
    }
}

impl ShapeView for TopoShape<'_> {
    fn dim(&self, key: ShapeKey) -> Option<Dim> {
        match key.canonical() {
            ShapeKey::Cell(id) => self.topo.is_alive(id).then(|| self.topo.dim_of(id)).flatten(),
            k @ ShapeKey::Piece(_) => self.piece(k).map(|_| Dim::Volume),
            ShapeKey::PieceFace(_) => Some(Dim::Face),
        }
    }

    fn linear_tolerance(&self) -> f64 {
        self.topo.tolerance().linear
    }

    fn vertex_point(&self, v: ShapeKey) -> Result<Point3, SchemeError> {
        let id = self.cell(v, Dim::Vertex)?;
        match self.topo.carrier(id) {
            Carrier::Point(p) => Ok(*p),
            _ => Err(SchemeError::shape(v, "vertex without a point")),
        }
    }

    fn curve_ends(&self, c: ShapeKey) -> Result<(ShapeKey, ShapeKey), SchemeError> {
        let id = self.cell(c, Dim::Edge)?;
        let (a, b) = self.topo.edge_ends(id).ok_or_else(|| SchemeError::shape(c, "edge without ends"))?;
        Ok((ShapeKey::Cell(a), ShapeKey::Cell(b)))
    }

    fn curve_length(&self, c: ShapeKey) -> Result<f64, SchemeError> {
        let id = self.cell(c, Dim::Edge)?;
        match self.topo.carrier(id) {
            Carrier::Curve(cc) => {
                let curve = self.topo.shapes().curve(cc.curve).ok_or_else(|| SchemeError::shape(c, "missing curve"))?;
                curve.length(cc.range, self.linear_tolerance() * 1e-3).map_err(|e| SchemeError::shape(c, e.to_string()))
            }
            _ => {
                let (a, b) = self.curve_ends(c)?;
                Ok((self.vertex_point(b)? - self.vertex_point(a)?).norm())
            }
        }
    }

    fn curve_point(&self, c: ShapeKey, fraction: f64) -> Result<(Point3, f64), SchemeError> {
        if !(0.0..=1.0).contains(&fraction) {
            return Err(SchemeError::shape(c, format!("fraction {fraction} outside [0, 1]")));
        }
        let id = self.cell(c, Dim::Edge)?;
        let Carrier::Curve(cc) = self.topo.carrier(id) else {
            let (a, b) = self.curve_ends(c)?;
            let (pa, pb) = (self.vertex_point(a)?, self.vertex_point(b)?);
            return Ok((pa + (pb - pa) * fraction, fraction));
        };
        let curve = self.topo.shapes().curve(cc.curve).ok_or_else(|| SchemeError::shape(c, "missing curve"))?;
        let is_line = matches!(self.topo.shapes().curve_def(cc.curve), Some(CurveDef::Line(_)));
        let s = if is_line || fraction == 0.0 || fraction == 1.0 {
            fraction
        } else {
            // Invert arc length by bisection on the parametric fraction.
            let tol = self.linear_tolerance() * 1e-3;
            let total = curve.length(cc.range, tol).map_err(|e| SchemeError::shape(c, e.to_string()))?;
            let partial = |s: f64| -> Result<f64, SchemeError> {
                let t = Self::edge_param(cc, s);
                let r = if cc.sense == Sense::Forward { Interval::new(cc.range.lo, t) } else { Interval::new(t, cc.range.hi) };
                if r.hi <= r.lo { Ok(0.0) } else { curve.length(r, tol).map_err(|e| SchemeError::shape(c, e.to_string())) }
            };
            let (mut lo, mut hi) = (0.0, 1.0);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if partial(mid)? < fraction * total { lo = mid } else { hi = mid }
            }
            0.5 * (lo + hi)
        };
        let t = Self::edge_param(cc, s);
        Ok((curve.eval(t), t))
    }

    fn face_loops(&self, f: ShapeKey) -> Result<Vec<LoopUse>, SchemeError> {
        let id = self.cell(f, Dim::Face)?;
        let mut out = Vec::new();
        for l in self.topo.loops(id) {
            let real: Vec<&sm_topo_api::Incidence> = l.entries.iter().filter(|e| !self.topo.flags(e.cell).contains(CellFlags::DEGENERATE)).collect();
            let mut uses = Vec::with_capacity(real.len());
            for (i, e) in real.iter().enumerate() {
                let (s, t) = self.topo.edge_ends(e.cell).ok_or_else(|| SchemeError::shape(f, "edge without ends"))?;
                let sense = e.side.sense();
                let prev = real[(i + real.len() - 1) % real.len()];
                uses.push(CurveUse {
                    curve: ShapeKey::Cell(e.cell),
                    sense,
                    start: ShapeKey::Cell(if e.side == Side::Right { t } else { s }),
                    corner: Some(CornerKey { face: id, incoming: CoedgeKey { face: id, edge: prev.cell, side: prev.side } }),
                });
            }
            out.push(LoopUse { ring: l.ring, uses });
        }
        Ok(out)
    }

    fn corner_angle(&self, f: ShapeKey, ring: u16, index: usize) -> Result<f64, SchemeError> {
        let id = self.cell(f, Dim::Face)?;
        let loops = self.face_loops(f)?;
        let l = loops.iter().find(|l| l.ring == ring).ok_or_else(|| SchemeError::shape(f, format!("no loop {ring}")))?;
        if index >= l.uses.len() {
            return Err(SchemeError::shape(f, format!("no use {index} in loop {ring}")));
        }
        if l.uses.len() < 2 {
            return Ok(core::f64::consts::PI);
        }
        let u = &l.uses[index];
        let prev = &l.uses[(index + l.uses.len() - 1) % l.uses.len()];
        let v = self.vertex_point(u.start)?;
        let t_out = self.use_point(u, EPS_FRACTION)? - v;
        let t_in = v - self.use_point(prev, 1.0 - EPS_FRACTION)?;
        let n = self.face_normal(id, v)?;
        Ok(interior_angle(t_in, t_out, n))
    }

    fn snap_to_face(&self, f: ShapeKey, p: Point3) -> Result<(Point3, Option<[f64; 2]>), SchemeError> {
        let id = self.cell(f, Dim::Face)?;
        if let Carrier::Surface(sc) = self.topo.carrier(id)
            && let Some(s) = self.topo.shapes().surface(sc.surface)
        {
            let pr = s.project(p, None).map_err(|e| SchemeError::shape(f, e.to_string()))?;
            return Ok((pr.point, Some([pr.uv.u, pr.uv.v])));
        }
        Ok((p, None))
    }

    fn volume_faces(&self, v: ShapeKey) -> Result<Vec<FaceUseRef>, SchemeError> {
        let mut out: Vec<FaceUseRef> = match v.canonical() {
            ShapeKey::Cell(id) => {
                let id = self.cell(ShapeKey::Cell(id), Dim::Volume)?;
                self.topo.face_uses(id).map(|u| FaceUseRef { face: ShapeKey::Cell(u.face), sense: u.sense }).collect()
            }
            k @ ShapeKey::Piece(_) => {
                let p = self.piece(k).ok_or_else(|| SchemeError::shape(k, "unknown piece"))?;
                if !p.members.is_empty() {
                    return Err(SchemeError::Unsupported(format!("{k}: composite pieces need the overlay to provide their faces")));
                }
                let mut out = Vec::with_capacity(p.faces.len());
                for pf in &p.faces {
                    let PieceFace::Whole(f) = pf else {
                        return Err(SchemeError::Unsupported(format!("{k}: cut or partial piece faces need an overlay realization")));
                    };
                    let sense = *self.topo.face_use_in(*f, p.volume).first().ok_or_else(|| SchemeError::shape(k, format!("{f} is not on the piece's volume")))?;
                    out.push(FaceUseRef { face: ShapeKey::Cell(*f), sense });
                }
                out
            }
            k => return Err(SchemeError::shape(k, "not a volume")),
        };
        out.sort();
        Ok(out)
    }

    fn deck_links(&self) -> Vec<DeckLink> {
        let Some(m) = self.model else { return Vec::new() };
        let mut out = Vec::new();
        for (f, fb) in m.deck_faces() {
            let Some(deck) = m.boundary(fb.boundary).and_then(|r| r.kind.as_deck()) else { continue };
            match (fb.role, deck) {
                (FaceRole::Deck { side: DeckSide::B, mate: Some(a) }, _) => out.push(DeckLink::Pair { a: ShapeKey::Cell(a), b: ShapeKey::Cell(f), transform: deck.transform(), boundary: fb.boundary }),
                (FaceRole::Deck { side: DeckSide::SelfPaired, .. }, Deck::Mirror { plane }) => out.push(DeckLink::Mirror { face: ShapeKey::Cell(f), normal: plane.normal, offset: plane.offset, boundary: fb.boundary }),
                _ => {}
            }
        }
        out
    }

    fn sweep_hint(&self, v: ShapeKey) -> Option<SweepHint> {
        match &self.piece(v)?.shape {
            PieceShape::Sweep { source, target } => Some(SweepHint {
                source: source.iter().map(|f| ShapeKey::PieceFace(*f).canonical()).collect(),
                target: target.iter().map(|f| ShapeKey::PieceFace(*f).canonical()).collect(),
            }),
            _ => None,
        }
    }

    fn classify_on(&self, key: ShapeKey) -> Result<EntityId, SchemeError> {
        match key.canonical() {
            ShapeKey::Cell(id) if self.topo.dim_of(id).is_some() => Ok(id),
            ShapeKey::Piece(PieceRef::Cell(pc)) if pc.kind() == EntityKind::PartitionCell => Ok(pc),
            ShapeKey::PieceFace(PieceFace::Part { face, .. }) => Ok(face),
            k => Err(SchemeError::Unsupported(format!("{k}: no classification target (cut faces have no PartitionFace id yet)"))),
        }
    }
}
