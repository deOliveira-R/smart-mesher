//! Assemblies: `MAPPED_ITEM` / `REPRESENTATION_MAP` and `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`
//! with `ITEM_DEFINED_TRANSFORMATION`, flattened into one placement per solid instance.
//!
//! A *link* places a child representation in a parent representation. The transform of a link
//! maps child coordinates to parent coordinates so that the child's placement item lands on the
//! parent's: `T = F2 o F1^-1` (both frames read in the target unit, so a translation written in
//! the parent's unit is converted). A solid's instances are the paths from its representation up
//! to a root representation; the placement of an instance is the composition along the path.

use std::collections::BTreeMap;

use sm_core::{Transform, Vector3};
use sm_geom_api::GeomError;

use crate::args::{Args, Ents};
use crate::error::StepError;
use crate::gcodec::Geo;
use crate::model::Solid;
use crate::part21::{Entity, Param, Rec, StepFile};

/// A placement of `child` in `parent`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Link {
    /// The entity carrying the link (for error messages).
    pub id: u64,
    pub child: u64,
    pub parent: u64,
    /// `AXIS2_PLACEMENT_3D` in the child representation.
    pub item1: u64,
    /// `AXIS2_PLACEMENT_3D` in the parent representation.
    pub item2: u64,
}

/// The representation graph of a file.
#[derive(Default)]
pub(crate) struct Assembly {
    /// Representation id to (item ids, context id).
    pub reps: BTreeMap<u64, (Vec<u64>, Option<u64>)>,
    /// Links by child representation, ascending by link id.
    pub links: BTreeMap<u64, Vec<Link>>,
    /// Relationships and mapped items that could not be understood: (id, why).
    pub unsupported: Vec<(u64, String)>,
}

fn recs(e: &Entity) -> Vec<&Rec> {
    match e {
        Entity::Simple(r) => vec![r],
        Entity::Complex(v) => v.iter().collect(),
    }
}

fn reference(p: Option<&Param>) -> Option<u64> {
    match p {
        Some(Param::Ref(r)) => Some(*r),
        _ => None,
    }
}

/// Scan the representation graph.
pub(crate) fn scan(file: &StepFile) -> Assembly {
    let ents = Ents::new(file);
    let mut asm = Assembly::default();
    for (id, e) in &file.entities {
        for r in recs(e) {
            if r.name.ends_with("REPRESENTATION")
                && r.name != "DEFINITIONAL_REPRESENTATION"
                && let (Some(Param::List(items)), Some(Param::Ref(ctx))) = (r.params.get(1), r.params.get(2))
            {
                let items: Vec<u64> = items.iter().filter_map(|p| if let Param::Ref(i) = p { Some(*i) } else { None }).collect();
                asm.reps.entry(*id).or_insert((items, Some(*ctx)));
            }
        }
    }
    // The first representation listing each item (a `MAPPED_ITEM` sits in exactly one).
    let mut item_rep: BTreeMap<u64, u64> = BTreeMap::new();
    for (rep, (items, _)) in &asm.reps {
        for it in items {
            item_rep.entry(*it).or_insert(*rep);
        }
    }
    for (id, e) in &file.entities {
        // Relationships with a transformation, simple or complex.
        if let Some(wt) = e.rec("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
            let (base, tr) = match (e.rec("REPRESENTATION_RELATIONSHIP"), e) {
                (Some(b), _) => (b, reference(wt.params.first())),
                (None, Entity::Simple(_)) => (wt, reference(wt.params.get(4))),
                _ => continue,
            };
            let (Some(rep1), Some(rep2), Some(tr)) = (reference(base.params.get(2)), reference(base.params.get(3)), tr) else {
                asm.unsupported.push((*id, "malformed REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION".into()));
                continue;
            };
            match ents.get(tr, *id).ok().and_then(|t| t.rec("ITEM_DEFINED_TRANSFORMATION")) {
                Some(t) => match (reference(t.params.get(2)), reference(t.params.get(3))) {
                    (Some(item1), Some(item2)) => asm.links.entry(rep1).or_default().push(Link { id: *id, child: rep1, parent: rep2, item1, item2 }),
                    _ => asm.unsupported.push((*id, "ITEM_DEFINED_TRANSFORMATION without placements".into())),
                },
                None => asm.unsupported.push((*id, "a transformation other than ITEM_DEFINED_TRANSFORMATION".into())),
            }
        }
        if let Some(m) = e.rec("MAPPED_ITEM") {
            let a = Args { id: *id, rec: m };
            let (Ok(src), Ok(target)) = (a.reference(1), a.reference(2)) else {
                asm.unsupported.push((*id, "malformed MAPPED_ITEM".into()));
                continue;
            };
            let parent = item_rep.get(id).copied();
            let map = ents.get(src, *id).ok().and_then(|x| x.rec("REPRESENTATION_MAP"));
            let origin_child = map.and_then(|r| Some((reference(r.params.first())?, reference(r.params.get(1))?)));
            let target_ok = ents.get(target, *id).is_ok_and(|t| t.is("AXIS2_PLACEMENT_3D"));
            match (parent, origin_child) {
                (Some(parent), Some((origin, child))) if target_ok => asm.links.entry(child).or_default().push(Link { id: *id, child, parent, item1: origin, item2: target }),
                _ => asm.unsupported.push((*id, "a MAPPED_ITEM this reader cannot place".into())),
            }
        }
    }
    asm
}

/// The identity, snapped: a transform within `1e-12` of the identity is exactly the identity.
fn snap(t: Transform) -> Transform {
    let id = Transform::IDENTITY;
    let near = (0..3).all(|i| (0..3).all(|j| (t.m[i][j] - id.m[i][j]).abs() < 1e-12)) && t.t.norm() < 1e-12;
    if near { id } else { t }
}

/// Deepest representation chain followed (the path stack also stops cycles).
const MAX_DEPTH: usize = 64;

/// State of one instance walk: the representations on the current path (cycle detection) and the
/// largest number of placements the walk may produce.
pub(crate) struct Walk {
    path: Vec<u64>,
    /// Limit on the placements of one solid (and on every intermediate list).
    pub limit: usize,
}

impl Walk {
    pub(crate) fn new(limit: usize) -> Self {
        Self { path: Vec::new(), limit }
    }
}

/// The placements of every instance of the representation `rep`, ascending by link id along each
/// path. A representation with no link is its own root: one identity instance.
///
/// `scale_of(rep)` is the factor from a representation's length unit to the target unit. A
/// representation reached again on its own path is a [`StepError::Cycle`] (at the first
/// revisit, whatever the fan-out); more than `walk.limit` placements, at any level, is
/// [`StepError::TooManyInstances`], so a small file cannot make the walk allocate without bound.
pub(crate) fn instances(file: &StepFile, asm: &Assembly, rep: u64, scale_of: &dyn Fn(u64) -> f64, walk: &mut Walk) -> Result<Vec<Transform>, StepError> {
    if walk.path.contains(&rep) {
        return Err(StepError::Cycle { id: rep });
    }
    if walk.path.len() >= MAX_DEPTH {
        return Err(StepError::TooManyInstances { id: rep, limit: walk.limit });
    }
    let Some(links) = asm.links.get(&rep) else { return Ok(vec![Transform::IDENTITY]) };
    let ents = Ents::new(file);
    let mut out = Vec::new();
    walk.path.push(rep);
    for l in links {
        let f1 = Geo { ents, scale: scale_of(l.child), angle: 1.0 }.frame(l.item1, l.id)?;
        let f2 = Geo { ents, scale: scale_of(l.parent), angle: 1.0 }.frame(l.item2, l.id)?;
        // Columns of M are the frame axes: R = M2 M1^T, t = o2 - R o1.
        let axes = |f: &sm_geom_api::Frame| [f.x, f.y, f.z];
        let (a1, a2) = (axes(&f1), axes(&f2));
        let comp = |v: Vector3, i: usize| [v.x, v.y, v.z][i];
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = (0..3).map(|k| comp(a2[k], i) * comp(a1[k], j)).sum();
            }
        }
        let rot = Transform { m, t: Vector3::new(0.0, 0.0, 0.0) };
        let o1 = rot.apply_point(f1.origin);
        let link = snap(Transform { m, t: Vector3::new(f2.origin.x - o1.x, f2.origin.y - o1.y, f2.origin.z - o1.z) });
        for p in instances(file, asm, l.parent, scale_of, walk)? {
            if out.len() >= walk.limit {
                return Err(StepError::TooManyInstances { id: rep, limit: walk.limit });
            }
            out.push(snap(p.compose(&link)));
        }
    }
    walk.path.pop();
    Ok(out)
}

/// Apply a proper rigid transform to a solid in place. Improper maps are refused.
pub(crate) fn transform_solid(s: &mut Solid, t: &Transform) -> Result<(), GeomError> {
    if t.det() <= 0.0 {
        return Err(GeomError::Unsupported("mirrored placement"));
    }
    let mut curves = Vec::with_capacity(s.curves.len());
    for c in &s.curves {
        curves.push(c.def.transformed(t)?);
    }
    let mut surfaces = Vec::with_capacity(s.surfaces.len());
    let mut flips = Vec::with_capacity(s.surfaces.len());
    for sf in &s.surfaces {
        let (d, sense) = sf.def.transformed(t)?;
        flips.push(sense == sm_geom_api::Sense::Reversed);
        surfaces.push(d);
    }
    for (c, d) in s.curves.iter_mut().zip(curves) {
        c.def = d;
    }
    for (sf, d) in s.surfaces.iter_mut().zip(surfaces) {
        sf.def = d;
    }
    for v in &mut s.vertices {
        v.point = t.apply_point(v.point);
    }
    for f in &mut s.faces {
        if flips[f.surface] {
            f.sense = f.sense.flip();
        }
    }
    Ok(())
}
