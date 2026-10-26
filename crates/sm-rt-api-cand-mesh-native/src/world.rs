//! Hierarchy and instancing: universes of placed components and lattices.
//!
//! Position in OpenMC terms: a `Leaf` is a universe whose cells are a mesh (or facets); `Union`
//! is a universe of placed children; `Lattice` is a regular array of one prototype. The
//! difference from OpenMC/DAGMC is that a leaf is a *mesh with exact adjacency*, so a lattice
//! element needs no per-element geometry copy, and every instance cell has a global dense
//! `CellId` for tallies and for the Sn sweep (cell numbering = prefix sums over instances in
//! ascending `InstanceId`).

use crate::error::{BuildIssue, Result, RtError};
use crate::ids::ComponentId;
use sm_core::{Point3, Transform, Vector3};

/// A child placed in a `Union`.
#[derive(Clone, Debug, PartialEq)]
pub struct Placement {
    /// The prototype.
    pub child: ComponentId,
    /// Placement (may be improper: mirrored copy).
    pub transform: Transform,
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

    fn element_count(&self) -> usize {
        self.dims.iter().map(|&d| d as usize).product()
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
    /// Hierarchical when a prototype is instanced more than `flatten_below` times in total cells.
    #[default]
    Auto,
}

/// A hierarchical world.
#[derive(Clone, Debug, PartialEq)]
pub struct WorldSpec {
    /// Components, dense by `ComponentId`.
    pub components: Vec<ComponentKind>,
    /// Root component.
    pub root: ComponentId,
    /// Instancing policy.
    pub policy: InstancePolicy,
}

impl WorldSpec {
    /// Check ids, lattice fills and acyclicity; depth against `MAX_LEVELS`. Issues are located by
    /// message (components have no model entity).
    pub fn validate(&self, n_inputs: usize) -> Result<()> {
        let n = self.components.len();
        let mut issues = Vec::new();
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
                        if p.transform.inverse().is_none() {
                            issues.push(BuildIssue::new("world.singular_transform", format!("component {i}: singular placement")));
                        }
                    }
                }
                ComponentKind::Lattice(l) => {
                    if l.fill.len() != l.element_count() {
                        issues.push(BuildIssue::new("world.lattice_fill", format!("component {i}: fill has {} entries, dims need {}", l.fill.len(), l.element_count())));
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
                Some(d) if d > crate::state::MAX_LEVELS + 1 => {
                    return Err(RtError::LevelOverflow { max: crate::state::MAX_LEVELS });
                }
                _ => {}
            }
        }
        if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
    }

    /// Depth of the component graph from the root (a lone leaf has depth 1); `None` on a cycle.
    /// Call only after the id checks of [`Self::validate`] pass.
    pub fn depth(&self) -> Option<usize> {
        fn go(w: &WorldSpec, c: ComponentId, stack: &mut Vec<u32>) -> Option<usize> {
            if stack.contains(&c.0) {
                return None;
            }
            stack.push(c.0);
            let kids: Vec<ComponentId> = match &w.components[c.get()] {
                ComponentKind::Leaf { .. } => vec![],
                ComponentKind::Union { placements } => placements.iter().map(|p| p.child).collect(),
                ComponentKind::Lattice(l) => l.fill.iter().chain(l.outer.iter()).copied().collect(),
            };
            let mut best = 0;
            for k in kids {
                best = best.max(go(w, k, stack)?);
            }
            stack.pop();
            Some(best + 1)
        }
        go(self, self.root, &mut Vec::new())
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
        w.validate(1).unwrap();
        assert_eq!(w.depth(), Some(2));
        let ComponentKind::Lattice(l) = &w.components[1] else { unreachable!() };
        assert_eq!(l.locate_rect(Point3::new(1.3, 0.1, 0.5)), Some([1, 0, 0]));
        assert_eq!(l.locate_rect(Point3::new(-0.1, 0.1, 0.5)), None);
        let c = l.element_center([1, 0, 0]);
        assert!((c.x - 1.89).abs() < 1e-12 && (c.y - 0.63).abs() < 1e-12);
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
}
