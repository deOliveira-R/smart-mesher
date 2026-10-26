//! Hierarchy and instancing: universes of placed components and lattices.
//!
//! Position in OpenMC terms: a `Leaf` is a universe whose cells are a mesh (or facets); `Union`
//! is a universe of placed children; `Lattice` is a regular array of one prototype. The
//! difference from OpenMC/DAGMC is that a leaf is a *mesh with exact adjacency*, so a lattice
//! element needs no per-element geometry copy, and every instance cell has a global dense
//! `CellId` for tallies and for the Sn sweep (cell numbering = prefix sums over instances in
//! ascending `InstanceId`).

use crate::boundary::hash_transform;
use crate::cache::{ContentHash, ContentHasher};
use crate::error::{BuildIssue, Result, RtError};
use crate::ids::ComponentId;
use sm_core::{Point3, Transform, Vector3};

/// A child placed in a `Union`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Placement {
    /// The prototype.
    pub child: ComponentId,
    /// Placement (may be improper: mirrored copy).
    pub transform: Transform,
}

impl Placement {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(child: ComponentId, transform: Transform) -> Self {
        Self { child, transform }
    }
}

/// Lattice shape.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum LatticeKind {
    /// Rectangular with pitch per axis; elements are centred on `origin + (i + 1/2) * pitch`.
    Rect {
        /// Pitch along x, y, z (z pitch ignored for 2D lattices, `dims[2] == 1`).
        pitch: [f64; 3],
    },
    /// Hexagonal (pointy-top) with flat-to-flat pitch, axial coordinates `(q, r)`, optional z stack.
    Hex {
        /// Flat-to-flat distance between neighbour centres.
        pitch: f64,
        /// Axial pitch (z) when `dims[2] > 1`.
        z_pitch: f64,
    },
}

/// A regular array of one or more prototypes.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Lattice {
    /// Shape.
    pub kind: LatticeKind,
    /// Lower corner (rect) or centre of element `(0, 0)` (hex).
    pub origin: Point3,
    /// Element counts along the three index axes.
    pub dims: [u32; 3],
    /// Prototype per element, index 0 fastest.
    pub fill: Vec<ComponentId>,
    /// What fills space outside the array (`None` = outside the model).
    pub outer: Option<ComponentId>,
}

impl Lattice {
    /// Construct (the type is `#[non_exhaustive]`); [`WorldSpec::validate`] checks the result.
    pub fn new(kind: LatticeKind, origin: Point3, dims: [u32; 3], fill: Vec<ComponentId>, outer: Option<ComponentId>) -> Self {
        Self { kind, origin, dims, fill, outer }
    }

    /// Centre of element `idx` in the lattice's parent frame.
    pub fn element_center(&self, idx: [i32; 3]) -> Point3 {
        match self.kind {
            LatticeKind::Rect { pitch } => Point3::new(
                self.origin.x + (f64::from(idx[0]) + 0.5) * pitch[0],
                self.origin.y + (f64::from(idx[1]) + 0.5) * pitch[1],
                self.origin.z + (f64::from(idx[2]) + 0.5) * pitch[2],
            ),
            LatticeKind::Hex { pitch, z_pitch } => {
                let (q, r) = (f64::from(idx[0]), f64::from(idx[1]));
                Point3::new(
                    self.origin.x + pitch * (q + r / 2.0),
                    self.origin.y + pitch * (3.0_f64.sqrt() / 2.0) * r,
                    self.origin.z + f64::from(idx[2]) * z_pitch,
                )
            }
        }
    }

    /// Placement of element `idx`: a translation to its centre.
    pub fn element_transform(&self, idx: [i32; 3]) -> Transform {
        let c = self.element_center(idx);
        Transform::translation(Vector3::new(c.x, c.y, c.z))
    }

    /// Element containing `p` for a rectangular lattice (`None` outside the array or for hex,
    /// whose lookup the implementor owns). Exact: uses floor division, ties go to the higher index.
    pub fn locate_rect(&self, p: Point3) -> Option<[i32; 3]> {
        let LatticeKind::Rect { pitch } = self.kind else { return None };
        let f = |v: f64, o: f64, h: f64, n: u32| {
            let i = ((v - o) / h).floor();
            (i >= 0.0 && i < f64::from(n)).then_some(i as i32)
        };
        Some([f(p.x, self.origin.x, pitch[0], self.dims[0])?, f(p.y, self.origin.y, pitch[1], self.dims[1])?, f(p.z, self.origin.z, pitch[2], self.dims[2])?])
    }

    /// Number of elements, `None` if the product overflows `usize`.
    pub fn element_count(&self) -> Option<usize> {
        self.dims.iter().try_fold(1usize, |acc, &d| acc.checked_mul(usize::try_from(d).ok()?))
    }

    /// Located problems with the shape (dims, pitch, origin), independent of the fill. Dims must
    /// be non-zero, at most `i32::MAX` (indices are `i32`) and their product must fit `usize`.
    fn shape_issues(&self, comp: usize) -> Vec<BuildIssue> {
        let mut out = Vec::new();
        if self.dims.contains(&0) || self.dims.iter().any(|&d| d > i32::MAX as u32) || self.element_count().is_none() {
            out.push(BuildIssue::new("world.lattice_dims", format!("component {comp}: dims {:?} must be in 1..=i32::MAX with a product that fits usize", self.dims)));
        }
        let pos = |v: f64| v.is_finite() && v > 0.0;
        let z_used = self.dims[2] > 1;
        let pitch_ok = match self.kind {
            LatticeKind::Rect { pitch } => pos(pitch[0]) && pos(pitch[1]) && (!z_used || pos(pitch[2])),
            LatticeKind::Hex { pitch, z_pitch } => pos(pitch) && (!z_used || pos(z_pitch)),
        };
        if !pitch_ok {
            out.push(BuildIssue::new("world.lattice_pitch", format!("component {comp}: lattice pitch must be positive and finite")));
        }
        if ![self.origin.x, self.origin.y, self.origin.z].iter().all(|v| v.is_finite()) {
            out.push(BuildIssue::new("world.lattice_origin", format!("component {comp}: lattice origin is not finite")));
        }
        out
    }
}

/// What a component is.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum ComponentKind {
    /// Cells from the `i`-th input given to the builder (a mesh or facet set).
    Leaf {
        /// Index into the inputs slice.
        input: usize,
    },
    /// Placed children; where placements overlap the lowest index wins (CSG-like priority).
    Union {
        /// Placements, in priority order.
        placements: Vec<Placement>,
    },
    /// A lattice.
    Lattice(Lattice),
}

/// Flatten or keep prototypes shared.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum InstancePolicy {
    /// Expand to one flat mesh with `sm_mesh_api::flatten`: fastest tracking, memory x instances.
    Flatten,
    /// Keep prototypes; the tracker carries a level stack and transforms rays into local frames.
    Hierarchical,
    /// The implementor chooses per world by its own documented heuristic (e.g. flatten small
    /// worlds); the choice never changes tracking results, only speed and memory.
    #[default]
    Auto,
}

/// A hierarchical world.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct WorldSpec {
    /// Components, dense by `ComponentId`.
    pub components: Vec<ComponentKind>,
    /// Root component.
    pub root: ComponentId,
    /// Instancing policy.
    pub policy: InstancePolicy,
}

impl WorldSpec {
    /// Construct (the type is `#[non_exhaustive]`); [`Self::validate`] checks the result.
    pub fn new(components: Vec<ComponentKind>, root: ComponentId, policy: InstancePolicy) -> Self {
        Self { components, root, policy }
    }

    /// Check ids, lattice fills and acyclicity. Errors are `RtError::Build` (located by message;
    /// components have no model entity). On success returns warnings: a hierarchy with more than
    /// [`crate::LEVEL_INLINE`] levels below the root is legal (the level stack spills to the
    /// heap) but slower, so it is a `world.deep_hierarchy` warning, never an error.
    pub fn validate(&self, n_inputs: usize) -> Result<Vec<BuildIssue>> {
        let n = self.components.len();
        let mut issues = Vec::new();
        let mut warnings = Vec::new();
        let ok = |c: ComponentId| c.get() < n;
        if !ok(self.root) {
            issues.push(BuildIssue::new("world.root", "root component does not exist"));
        }
        for (i, c) in self.components.iter().enumerate() {
            match c {
                ComponentKind::Leaf { input } if *input >= n_inputs => {
                    issues.push(BuildIssue::new("world.input", format!("component {i}: input {input} missing")))
                }
                ComponentKind::Union { placements } => {
                    for p in placements {
                        if !ok(p.child) {
                            issues.push(BuildIssue::new("world.child", format!("component {i}: child {} missing", p.child.0)));
                        }
                        let t = &p.transform;
                        if !(t.m.iter().flatten().all(|v| v.is_finite()) && t.t.x.is_finite() && t.t.y.is_finite() && t.t.z.is_finite()) {
                            issues.push(BuildIssue::new("world.placement_nonfinite", format!("component {i}: non-finite placement")));
                        } else if p.transform.inverse().is_none() {
                            issues.push(BuildIssue::new("world.singular_transform", format!("component {i}: singular placement")));
                        }
                    }
                }
                ComponentKind::Lattice(l) => {
                    let shape = l.shape_issues(i);
                    let shape_ok = shape.is_empty();
                    issues.extend(shape);
                    if let (true, Some(need)) = (shape_ok, l.element_count())
                        && l.fill.len() != need
                    {
                        issues.push(BuildIssue::new("world.lattice_fill", format!("component {i}: fill has {} entries, dims need {need}", l.fill.len())));
                    }
                    if l.fill.iter().chain(l.outer.iter()).any(|&c| !ok(c)) {
                        issues.push(BuildIssue::new("world.child", format!("component {i}: lattice fill references a missing component")));
                    }
                }
                _ => {}
            }
        }
        if issues.is_empty() {
            match self.depth() {
                None => issues.push(BuildIssue::new("world.cycle", "component graph has a cycle")),
                Some(d) if d > crate::state::LEVEL_INLINE + 1 => {
                    warnings.push(BuildIssue::new("world.deep_hierarchy", format!("hierarchy has {} levels below the root; more than {} spill to the heap", d - 1, crate::state::LEVEL_INLINE)));
                }
                _ => {}
            }
        }
        if issues.is_empty() { Ok(warnings) } else { Err(RtError::Build(issues)) }
    }

    /// Content hash (cache key `hierarchy_hash`): components, placements, lattices, root, policy.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.components.len() as u64);
        for c in &self.components {
            match c {
                ComponentKind::Leaf { input } => {
                    h.write_u64(1);
                    h.write_u64(*input as u64);
                }
                ComponentKind::Union { placements } => {
                    h.write_u64(2);
                    h.write_u64(placements.len() as u64);
                    for p in placements {
                        h.write_u64(u64::from(p.child.0));
                        hash_transform(&mut h, &p.transform);
                    }
                }
                ComponentKind::Lattice(l) => {
                    h.write_u64(3);
                    match l.kind {
                        LatticeKind::Rect { pitch } => {
                            h.write_u64(1);
                            pitch.iter().for_each(|&v| h.write_f64(v));
                        }
                        LatticeKind::Hex { pitch, z_pitch } => {
                            h.write_u64(2);
                            h.write_f64(pitch);
                            h.write_f64(z_pitch);
                        }
                    }
                    for v in [l.origin.x, l.origin.y, l.origin.z] {
                        h.write_f64(v);
                    }
                    l.dims.iter().for_each(|&d| h.write_u64(u64::from(d)));
                    h.write_u64(l.fill.len() as u64);
                    l.fill.iter().for_each(|f| h.write_u64(u64::from(f.0)));
                    h.write_u64(l.outer.map_or(u64::MAX, |c| u64::from(c.0)));
                }
            }
        }
        h.write_u64(u64::from(self.root.0));
        h.write_u64(self.policy as u64);
        h.finish()
    }

    /// Depth of the component graph from the root (a lone leaf has depth 1). `None` on a cycle,
    /// a missing root or a dangling child id (never panics, so it is safe before
    /// [`Self::validate`]). Shared children are memoized, so a lattice of lattices costs time
    /// linear in the number of components and fill entries, not the product of the fan-outs.
    pub fn depth(&self) -> Option<usize> {
        struct Frame {
            comp: usize,
            kids: Vec<usize>,
            next: usize,
            best: usize,
        }
        let n = self.components.len();
        let kids_of = |c: usize| -> Option<Vec<usize>> {
            Some(match self.components.get(c)? {
                ComponentKind::Leaf { .. } => vec![],
                ComponentKind::Union { placements } => placements.iter().map(|p| p.child.get()).collect(),
                ComponentKind::Lattice(l) => l.fill.iter().chain(l.outer.iter()).map(|c| c.get()).collect(),
            })
        };
        let root = self.root.get();
        let mut memo = vec![0usize; n];
        let mut on_stack = vec![false; n];
        let mut stack = vec![Frame { comp: root, kids: kids_of(root)?, next: 0, best: 0 }];
        on_stack[root] = true;
        while let Some(top) = stack.last_mut() {
            if let Some(&k) = top.kids.get(top.next) {
                top.next += 1;
                let known = *memo.get(k)?;
                if on_stack[k] {
                    return None;
                }
                if known > 0 {
                    top.best = top.best.max(known);
                } else {
                    on_stack[k] = true;
                    stack.push(Frame { comp: k, kids: kids_of(k)?, next: 0, best: 0 });
                }
            } else {
                let d = top.best + 1;
                let c = top.comp;
                memo[c] = d;
                on_stack[c] = false;
                stack.pop();
                match stack.last_mut() {
                    Some(parent) => parent.best = parent.best.max(d),
                    None => return Some(d),
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pin_lattice() -> WorldSpec {
        WorldSpec {
            components: vec![
                ComponentKind::Leaf { input: 0 },
                ComponentKind::Lattice(Lattice {
                    kind: LatticeKind::Rect { pitch: [1.26, 1.26, 1.0] },
                    origin: Point3::new(0.0, 0.0, 0.0),
                    dims: [17, 17, 1],
                    fill: vec![ComponentId(0); 289],
                    outer: None,
                }),
            ],
            root: ComponentId(1),
            policy: InstancePolicy::Hierarchical,
        }
    }

    #[test]
    fn lattice_world_validates_and_locates() {
        let w = pin_lattice();
        assert!(w.validate(1).unwrap().is_empty());
        assert_eq!(w.depth(), Some(2));
        let ComponentKind::Lattice(l) = &w.components[1] else { unreachable!() };
        assert_eq!(l.locate_rect(Point3::new(1.3, 0.1, 0.5)), Some([1, 0, 0]));
        assert_eq!(l.locate_rect(Point3::new(-0.1, 0.1, 0.5)), None);
        let c = l.element_center([1, 0, 0]);
        assert!((c.x - 1.89).abs() < 1e-12 && (c.y - 0.63).abs() < 1e-12);
    }

    #[test]
    fn depth_is_linear_on_nested_lattices_and_total_on_bad_ids() {
        // 24 levels of 17x17 lattices of the previous level: 289^24 paths, 25 components.
        let mut components = vec![ComponentKind::Leaf { input: 0 }];
        for i in 0..24u32 {
            components.push(ComponentKind::Lattice(Lattice {
                kind: LatticeKind::Rect { pitch: [1.0, 1.0, 1.0] },
                origin: Point3::new(0.0, 0.0, 0.0),
                dims: [17, 17, 1],
                fill: vec![ComponentId(i); 289],
                outer: None,
            }));
        }
        let w = WorldSpec { components, root: ComponentId(24), policy: InstancePolicy::Hierarchical };
        assert_eq!(w.depth(), Some(25));
        assert_eq!(w.validate(1).unwrap().len(), 1, "deep_hierarchy warning");
        // Bad ids give None, never a panic.
        let mut bad = pin_lattice();
        bad.root = ComponentId(99);
        assert_eq!(bad.depth(), None);
        if let ComponentKind::Lattice(l) = &mut bad.components[1] {
            l.fill[3] = ComponentId(77);
        }
        bad.root = ComponentId(1);
        assert_eq!(bad.depth(), None);
        // A diamond (shared child twice) is not a cycle.
        let id = Transform::translation(Vector3::new(0.0, 0.0, 0.0));
        let u = |c: u32| ComponentKind::Union { placements: vec![Placement { child: ComponentId(c), transform: id }] };
        let diamond = WorldSpec {
            components: vec![
                ComponentKind::Leaf { input: 0 },
                u(0),
                u(0),
                ComponentKind::Union { placements: vec![Placement { child: ComponentId(1), transform: id }, Placement { child: ComponentId(2), transform: id }] },
            ],
            root: ComponentId(3),
            policy: InstancePolicy::Hierarchical,
        };
        assert_eq!(diamond.depth(), Some(3));
    }

    #[test]
    fn hostile_lattice_shapes_are_rejected_not_panicked() {
        let code_of = |mutate: &dyn Fn(&mut Lattice)| {
            let mut w = pin_lattice();
            if let ComponentKind::Lattice(l) = &mut w.components[1] {
                mutate(l);
            }
            match w.validate(1) {
                Err(RtError::Build(v)) => v.iter().map(|i| i.code).collect::<Vec<_>>(),
                _ => vec![],
            }
        };
        assert!(code_of(&|l| l.dims = [u32::MAX; 3]).contains(&"world.lattice_dims"));
        assert!(code_of(&|l| l.dims = [1 << 31, 1, 1]).contains(&"world.lattice_dims"));
        assert!(code_of(&|l| l.dims = [0, 17, 1]).contains(&"world.lattice_dims"));
        assert!(code_of(&|l| l.kind = LatticeKind::Rect { pitch: [0.0, 1.0, 1.0] }).contains(&"world.lattice_pitch"));
        assert!(code_of(&|l| l.kind = LatticeKind::Rect { pitch: [f64::NAN, 1.0, 1.0] }).contains(&"world.lattice_pitch"));
        assert!(code_of(&|l| l.kind = LatticeKind::Hex { pitch: -1.0, z_pitch: 1.0 }).contains(&"world.lattice_pitch"));
        assert!(code_of(&|l| l.origin = Point3::new(f64::INFINITY, 0.0, 0.0)).contains(&"world.lattice_origin"));
        // 2D lattice: z pitch is ignored.
        assert!(code_of(&|l| l.kind = LatticeKind::Rect { pitch: [1.0, 1.0, 0.0] }).is_empty());
        assert_eq!(Lattice::new(LatticeKind::Rect { pitch: [1.0; 3] }, Point3::new(0.0, 0.0, 0.0), [u32::MAX; 3], vec![], None).element_count(), None);
    }

    #[test]
    fn cycles_and_bad_fills_are_rejected() {
        let mut w = pin_lattice();
        w.components[0] = ComponentKind::Union { placements: vec![Placement { child: ComponentId(1), transform: Transform::translation(Vector3::new(0.0, 0.0, 0.0)) }] };
        assert!(w.validate(1).is_err());
        let mut w = pin_lattice();
        if let ComponentKind::Lattice(l) = &mut w.components[1] {
            l.fill.pop();
        }
        assert!(matches!(w.validate(1), Err(RtError::Build(_))));
    }

    #[test]
    fn deep_hierarchy_warns_and_hash_is_sensitive() {
        // A chain of 10 unions around one leaf: 10 levels below the root, > LEVEL_INLINE.
        let id = Transform::translation(Vector3::new(0.0, 0.0, 0.0));
        let mut components = vec![ComponentKind::Leaf { input: 0 }];
        for i in 0..10u32 {
            components.push(ComponentKind::Union { placements: vec![Placement { child: ComponentId(i), transform: id }] });
        }
        let w = WorldSpec { components, root: ComponentId(10), policy: InstancePolicy::Hierarchical };
        let warn = w.validate(1).unwrap();
        assert_eq!(warn.len(), 1);
        assert_eq!(warn[0].code, "world.deep_hierarchy");
        let a = pin_lattice();
        let mut b = pin_lattice();
        assert_eq!(a.content_hash(), b.content_hash());
        if let ComponentKind::Lattice(l) = &mut b.components[1] {
            l.origin = Point3::new(1e-9, 0.0, 0.0);
        }
        assert_ne!(a.content_hash(), b.content_hash());
    }
}
