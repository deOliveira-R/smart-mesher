//! [`Model::canonical_visit`]: the model's persisted tables as a canonical word stream, for the
//! state hash (quantized floats) and derived-cache keys (raw floats).

use crate::boundary::{BoundaryKind, BoundaryRow, FaceBoundary, Deck, DeckSide, FaceRole, InterfaceIntent};
use crate::color::{ColorRow, Domain, GroupRow};
use crate::decomposition::{CutCells, CutGeometry, Decomposition, Piece, PieceFace, PieceRef, PieceShape, SplitRec};
use crate::detect::Verification;
use crate::instancing::{ColorMap, Fill, HexOrient, InstanceRow, Instancing, Lattice, Layout, PathOverride, PathStep, Prototype, Slot};
use crate::model::Model;
use crate::symmetry::{CutSpec, Keep, Provenance, Symmetry};
use crate::xf::{Axis, Plane};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::ModelTolerance;
use sm_topo_api::HashWord;

/// Emits words with the Q2 float rule.
struct W<'a, 'f> {
    tol: &'a ModelTolerance,
    f: &'a mut (dyn FnMut(HashWord) + 'f),
}

impl W<'_, '_> {
    fn int(&mut self, v: u64) {
        (self.f)(HashWord::Int(v));
    }
    fn id(&mut self, id: EntityId) {
        self.int(id.to_raw());
    }
    fn ids<'i>(&mut self, ids: impl ExactSizeIterator<Item = &'i EntityId>) {
        self.int(ids.len() as u64);
        for i in ids {
            self.id(*i);
        }
    }
    fn opt_id(&mut self, id: Option<EntityId>) {
        match id {
            None => self.int(0),
            Some(i) => {
                self.int(1);
                self.id(i);
            }
        }
    }
    fn str(&mut self, s: &str) {
        self.int(s.len() as u64);
        for c in s.as_bytes().chunks(8) {
            let mut b = [0u8; 8];
            b[..c.len()].copy_from_slice(c);
            self.int(u64::from_le_bytes(b));
        }
    }
    /// A length: `round(v / (linear * 2^-10))`.
    fn len(&mut self, v: f64) {
        let q = self.tol.quantize(v);
        (self.f)(HashWord::Float { raw: v, q });
    }
    /// A direction component, matrix entry or angle: `angular * 2^-10`.
    fn ang(&mut self, v: f64) {
        let q = ModelTolerance::quantize_with(v, self.tol.angular_quantum());
        (self.f)(HashWord::Float { raw: v, q });
    }
    fn point(&mut self, p: Point3) {
        for v in [p.x, p.y, p.z] {
            self.len(v);
        }
    }
    fn offset(&mut self, v: Vector3) {
        for x in [v.x, v.y, v.z] {
            self.len(x);
        }
    }
    fn dir(&mut self, v: Vector3) {
        for x in [v.x, v.y, v.z] {
            self.ang(x);
        }
    }
    fn plane(&mut self, p: &Plane) {
        self.dir(p.normal);
        self.len(p.offset);
    }
    fn axis(&mut self, a: &Axis) {
        self.point(a.point);
        self.dir(a.dir);
    }
    fn transform(&mut self, t: &Transform) {
        for row in t.m {
            for v in row {
                self.ang(v);
            }
        }
        self.offset(t.t);
    }
    fn deck(&mut self, d: &Deck) {
        match d {
            Deck::Mirror { plane } => {
                self.int(1);
                self.plane(plane);
            }
            Deck::Translation { vector } => {
                self.int(2);
                self.offset(*vector);
            }
            Deck::Rotation { axis, angle } => {
                self.int(3);
                self.axis(axis);
                self.int(angle.num as i64 as u64);
                self.int(u64::from(angle.den));
            }
        }
    }
    fn side(&mut self, s: DeckSide) {
        self.int(match s {
            DeckSide::A => 1,
            DeckSide::B => 2,
            DeckSide::SelfPaired => 3,
        });
    }
    fn keep(&mut self, k: Keep) {
        self.int(match k {
            Keep::Positive => 1,
            Keep::Negative => 2,
        });
    }
    fn cut_spec(&mut self, c: &CutSpec) {
        self.deck(&c.deck);
        self.keep(c.keep);
        self.point(c.origin);
    }
    fn piece_ref(&mut self, p: PieceRef) {
        match p {
            PieceRef::Whole(v) => {
                self.int(1);
                self.id(v);
            }
            PieceRef::Cell(c) => {
                self.int(2);
                self.id(c);
            }
        }
    }
    fn piece_face(&mut self, pf: &PieceFace) {
        match *pf {
            PieceFace::Whole(f) => {
                self.int(1);
                self.id(f);
            }
            PieceFace::Part { face, piece } => {
                self.int(2);
                self.id(face);
                self.id(piece);
            }
            PieceFace::Cut { cut, side } => {
                self.int(3);
                self.int(u64::from(cut.0));
                self.keep(side);
            }
            PieceFace::CutPart { cut, side, piece } => {
                self.int(4);
                self.int(u64::from(cut.0));
                self.keep(side);
                self.id(piece);
            }
        }
    }
    fn piece_faces(&mut self, v: &[PieceFace]) {
        self.int(v.len() as u64);
        for pf in v {
            self.piece_face(pf);
        }
    }
    fn fill(&mut self, f: Fill) {
        match f {
            Fill::Prototype(p) => {
                self.int(1);
                self.id(p);
            }
            Fill::Lattice(l) => {
                self.int(2);
                self.id(l);
            }
            Fill::Void => self.int(3),
        }
    }
    fn color_map(&mut self, c: &ColorMap) {
        for m in [&c.volume, &c.face] {
            self.int(m.len() as u64);
            for (k, v) in m {
                self.id(*k);
                self.id(*v);
            }
        }
    }
}

impl Model {
    /// Visit every persisted table in canonical order (`BTreeMap` order, declaration order for
    /// lists): the id allocator, colors, volume and face coloring, groups, boundaries, face
    /// boundaries, symmetry, provenance, recorded cut lineage, instancing and decomposition.
    /// Each table starts with a tag and its length; strings are emitted as their length and
    /// 8-byte little-endian words.
    ///
    /// Floats follow the Q2 rule with `tol` (the model's topology tolerance): lengths (plane
    /// offsets, points, translations, pitches, deviations) as `round(v / (linear * 2^-10))`;
    /// direction components, matrix entries and angles with `angular * 2^-10`
    /// ([`ModelTolerance::angular_quantum`]). A state layer folds the words into an
    /// `sm-op-api` `Digest` (`HashWord::Float { raw, q }` via `f64_with(raw, q)`), so a
    /// sub-quantum change keeps the identity hash and changes the exact hash.
    pub fn canonical_visit(&self, tol: &ModelTolerance, f: &mut dyn FnMut(HashWord)) {
        let mut w = W { tol, f };
        // Exhaustive patterns (no `..`): adding a field is a compile error until it is hashed.
        let Model { ids: _, colors: _, volume_color: _, face_color: _, groups: _, boundaries: _, face_boundary: _, symmetry: _, provenance: _, cuts: _, instancing: _, decomposition: _ } = self;
        // 1. Id allocator.
        w.int(1);
        w.int(self.ids.next.len() as u64);
        for (k, n) in &self.ids.next {
            w.int(u64::from(*k));
            w.int(*n);
        }
        // 2. Colors.
        w.int(2);
        w.int(self.colors.len() as u64);
        for (id, c) in &self.colors {
            w.id(*id);
            let ColorRow { id: _, label: _, name: _, domain: _ } = c;
            w.id(c.id);
            w.int(u64::from(c.label));
            w.str(&c.name);
            w.int(match c.domain {
                Domain::Volume => 1,
                Domain::Face => 2,
            });
        }
        // 3-4. Coloring.
        for (tag, m) in [(3, &self.volume_color), (4, &self.face_color)] {
            w.int(tag);
            w.int(m.len() as u64);
            for (e, c) in m {
                w.id(*e);
                w.id(*c);
            }
        }
        // 5. Groups.
        w.int(5);
        w.int(self.groups.len() as u64);
        for (id, g) in &self.groups {
            w.id(*id);
            let GroupRow { id: _, name: _, members: _ } = g;
            w.id(g.id);
            w.str(&g.name);
            w.ids(g.members.iter());
        }
        // 6. Boundaries.
        w.int(6);
        w.int(self.boundaries.len() as u64);
        for (id, b) in &self.boundaries {
            w.id(*id);
            let BoundaryRow { id: _, name: _, kind: _ } = b;
            w.id(b.id);
            w.str(&b.name);
            match &b.kind {
                BoundaryKind::Deck(d) => {
                    w.int(1);
                    w.deck(d);
                }
                BoundaryKind::Response => w.int(2),
                BoundaryKind::Interface(InterfaceIntent::Conformal) => w.int(3),
                BoundaryKind::Interface(InterfaceIntent::NonConformal { transform }) => {
                    w.int(4);
                    match transform {
                        None => w.int(0),
                        Some(t) => {
                            w.int(1);
                            w.transform(t);
                        }
                    }
                }
            }
        }
        // 7. Face boundaries.
        w.int(7);
        w.int(self.face_boundary.len() as u64);
        for (face, fb) in &self.face_boundary {
            w.id(*face);
            let FaceBoundary { boundary: _, role: _ } = fb;
            w.id(fb.boundary);
            match fb.role {
                FaceRole::Deck { side, mate } => {
                    w.int(1);
                    w.side(side);
                    w.opt_id(mate);
                }
                FaceRole::Response => w.int(2),
                FaceRole::Interface { side } => {
                    w.int(3);
                    w.side(side);
                }
            }
        }
        // 8. Symmetry.
        w.int(8);
        match &self.symmetry {
            None => w.int(0),
            Some(s) => {
                w.int(1);
                let Symmetry { generators: _, domain: _ } = s;
                w.ids(s.generators.iter());
                w.ids(s.domain.iter());
            }
        }
        // 9. Provenance.
        w.int(9);
        w.int(self.provenance.len() as u64);
        for p in &self.provenance {
            match p {
                Provenance::Declared => w.int(1),
                Provenance::Imported => w.int(2),
                Provenance::Detected(v) => {
                    w.int(3);
                    match v {
                        Verification::Unverified => w.int(1),
                        Verification::Verified { max_dev } => {
                            w.int(2);
                            w.len(*max_dev);
                        }
                        Verification::Rejected { stage, residual, reason } => {
                            w.int(3);
                            w.int(*stage as u64);
                            w.len(*residual);
                            w.str(reason);
                        }
                    }
                }
                Provenance::Cut { spec } => {
                    w.int(4);
                    w.cut_spec(spec);
                }
            }
        }
        // 10. Recorded cut lineage.
        w.int(10);
        w.int(self.cuts.len() as u64);
        for cut in &self.cuts {
            w.int(cut.len() as u64);
            for l in cut {
                w.id(l.old);
                w.ids(l.new.iter());
            }
        }
        // 11. Instancing.
        let ins = &self.instancing;
        let Instancing { prototypes: _, lattices: _, instances: _, overrides: _ } = ins;
        w.int(11);
        w.int(ins.prototypes.len() as u64);
        for (id, p) in &ins.prototypes {
            w.id(*id);
            let Prototype { volumes: _ } = p;
            w.ids(p.volumes.iter());
        }
        w.int(ins.lattices.len() as u64);
        for (id, l) in &ins.lattices {
            w.id(*id);
            let Lattice { layout: _, origin: _, outer: _, slots: _ } = l;
            match l.layout {
                Layout::Cartesian { n, pitch } => {
                    w.int(1);
                    n.iter().for_each(|x| w.int(u64::from(*x)));
                    w.offset(pitch);
                }
                Layout::Hex { rings, pitch, orient, nz, dz } => {
                    w.int(2);
                    w.int(u64::from(rings));
                    w.len(pitch);
                    w.int(match orient {
                        HexOrient::XRows => 1,
                        HexOrient::YRows => 2,
                    });
                    w.int(u64::from(nz));
                    w.len(dz);
                }
            }
            w.transform(&l.origin);
            w.fill(l.outer);
            w.int(l.slots.len() as u64);
            for (ix, s) in &l.slots {
                for v in [ix.i, ix.j, ix.k] {
                    w.int(v as i64 as u64);
                }
                let Slot { fill: _, recolor: _, clip: _ } = s;
                w.fill(s.fill);
                w.color_map(&s.recolor);
                w.opt_id(s.clip);
            }
        }
        w.int(ins.instances.len() as u64);
        for (id, r) in &ins.instances {
            w.id(*id);
            let InstanceRow { of: _, at: _, recolor: _, clip: _ } = r;
            w.fill(r.of);
            w.transform(&r.at);
            w.color_map(&r.recolor);
            w.opt_id(r.clip);
        }
        w.int(ins.overrides.len() as u64);
        for o in &ins.overrides {
            let PathOverride { prefix: _, recolor: _ } = o;
            w.int(o.prefix.len() as u64);
            for step in &o.prefix {
                match *step {
                    PathStep::Instance(i) => {
                        w.int(1);
                        w.id(i);
                    }
                    PathStep::Slot { lattice, ix } => {
                        w.int(2);
                        w.id(lattice);
                        for v in [ix.i, ix.j, ix.k] {
                            w.int(v as i64 as u64);
                        }
                    }
                }
            }
            w.color_map(&o.recolor);
        }
        // 12. Decomposition (every serialized field).
        let d = &self.decomposition;
        let Decomposition { cells: _, splits: _, parent: _, composites: _, consumed: _, by_volume: _, by_face: _, cut_serial: _, whole_faces: _, composite_faces: _, realized: _, face_cells: _, split_cells: _ } = d;
        w.int(12);
        w.int(d.cells.len() as u64);
        for (id, p) in &d.cells {
            w.id(*id);
            let Piece { id: _, volume: _, faces: _, shape: _, members: _ } = p;
            w.id(p.id);
            w.id(p.volume);
            w.piece_faces(&p.faces);
            match &p.shape {
                PieceShape::Any => w.int(1),
                PieceShape::Sweep { source, target } => {
                    w.int(2);
                    w.piece_faces(source);
                    w.piece_faces(target);
                }
                PieceShape::Map => w.int(3),
            }
            w.int(p.members.len() as u64);
            for m in &p.members {
                w.piece_ref(*m);
            }
        }
        w.int(d.splits.len() as u64);
        for (pr, s) in &d.splits {
            w.piece_ref(*pr);
            let SplitRec { children: _, cut: _, geometry: _ } = s;
            w.ids(s.children.iter());
            w.int(u64::from(s.cut.0));
            match &s.geometry {
                CutGeometry::Plane(p) => {
                    w.int(1);
                    w.plane(p);
                }
                CutGeometry::Surface(sf) => {
                    w.int(2);
                    w.id(*sf);
                }
                CutGeometry::Faces(fs) => {
                    w.int(3);
                    w.ids(fs.iter());
                }
            }
        }
        w.int(d.parent.len() as u64);
        for (c, p) in &d.parent {
            w.id(*c);
            w.piece_ref(*p);
        }
        w.ids(d.composites.iter());
        w.int(d.consumed.len() as u64);
        for p in &d.consumed {
            w.piece_ref(*p);
        }
        let index: [Vec<(EntityId, &[PieceRef])>; 2] = [
            d.by_volume.iter().map(|(k, v)| (*k, v.as_slice())).collect(),
            d.by_face.iter().map(|(k, v)| (*k, v.as_slice())).collect(),
        ];
        for m in &index {
            w.int(m.len() as u64);
            for (k, v) in m {
                w.id(*k);
                w.int(v.len() as u64);
                for p in *v {
                    w.piece_ref(*p);
                }
            }
        }
        w.int(u64::from(d.cut_serial));
        w.int(d.whole_faces.len() as u64);
        for (k, v) in &d.whole_faces {
            w.id(*k);
            w.ids(v.iter());
        }
        w.int(d.composite_faces.len() as u64);
        for (k, v) in &d.composite_faces {
            w.id(*k);
            w.piece_faces(v);
        }
        w.int(d.realized.len() as u64);
        for (cut, cc) in &d.realized {
            w.int(u64::from(cut.0));
            let CutCells { faces: _, edges: _, vertices: _, split_faces: _, split_edges: _ } = cc;
            w.ids(cc.faces.iter());
            w.ids(cc.edges.iter());
            w.ids(cc.vertices.iter());
            w.int(cc.split_faces.len() as u64);
            for (f, h) in &cc.split_faces {
                w.id(*f);
                w.ids(h.iter());
            }
            w.int(cc.split_edges.len() as u64);
            for (e, v, h) in &cc.split_edges {
                w.id(*e);
                w.id(*v);
                w.ids(h.iter());
            }
        }
        w.int(d.face_cells.len() as u64);
        for (pf, cells) in &d.face_cells {
            w.piece_face(pf);
            w.ids(cells.iter());
        }
        w.int(d.split_cells.len() as u64);
        for (f, h) in &d.split_cells {
            w.id(*f);
            w.ids(h.iter());
        }
    }
}
