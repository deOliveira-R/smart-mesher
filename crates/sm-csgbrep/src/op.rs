//! `model.import_openmc`: an OpenMC deck becomes B-rep volumes in one history node.

use std::collections::{BTreeMap, BTreeSet};

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_cadio::{MAX_INLINE_BYTES, Source, StepInputs, sha256_hex};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_csgio::ir::BoundaryFlag;
use sm_fem::{BlockAdd, SidesetAdd};
use sm_geom_api::SurfaceDef;
use sm_boolean::{ImprintOptions, MergeOptions, imprint_bodies, merge};
use sm_layer_api::{ModelLayer, TopoHost, commit_topology, model_txn, topo_txn};
use sm_model::{CopyXform, ModelCopy};
use sm_topo_api::{Carrier, Dim, TopologyView};
use sm_csgio::ir::HexOrientation;
use sm_model_api::{HexOrient, LatticeFrame, Layout, RegionTag, SlotIx};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};

use crate::deck::{self, Placed};
use crate::error::CsgbrepError;
use crate::partition::{self, Built, BuildOptions, OverlapPolicy, Vol};
use crate::reuse::{self, ReusePlan};

/// Default cap on placed cells.
const DEFAULT_MAX_CELLS: usize = 20_000;

/// Output roles. `deck_volumes` and `deck_bodies` are the result; the rest are the roles of the
/// nested ops the import runs (they hold intermediate ids a later cut or union may have retired).
const OUTPUTS: &[OutputRole] = &[
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "Intermediate: the lattice element copies the import placed (a later merge may change their faces). Read `deck_volumes`."),
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "Intermediate: the bodies of the lattice element copies. Read `deck_bodies`."),
    OutputRole::new("deck_volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The final volumes of the deck, ascending."),
    OutputRole::new("deck_bodies", &[EntityKind::TopoBody], Cardinality::Many, "The final bodies of the deck, ascending."),
    OutputRole::new("block", &[EntityKind::Color], Cardinality::Many, "The volume colors that are the blocks, one per region name."),
    OutputRole::new("sideset", &[EntityKind::Color], Cardinality::Many, "The face colors that are the sidesets (vacuum, reflective, white)."),
];

/// What to do with a piece of space two cells own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum OverlapMode {
    /// Fail naming both cells (default).
    #[default]
    Error,
    /// Keep the piece in the first cell (canonical order).
    FirstWins,
    /// As `first_wins`, and warn for each overlap.
    Report,
}

/// Options of `model.import_openmc`; every field is omitted from the hash when it holds the default.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenmcOptions {
    /// Overlap policy (default `error`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overlap: Option<OverlapMode>,
    /// Bounds of the model `[xmin, ymin, zmin, xmax, ymax, zmax]`; default: derived from the cells.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bounds: Option<[f64; 6]>,
    /// Region names that replace the derived ones, by material id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material_names: Option<BTreeMap<u32, String>>,
    /// Cap on placed cells (default 20000).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_cells: Option<usize>,
    /// Build each distinct lattice universe once and copy it (default `true`); `false` cuts
    /// every element (the flat partition).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reuse_lattices: Option<bool>,
}

/// `model.import_openmc {geometry, materials?, options?}`: build the B-rep volumes of an OpenMC
/// `geometry.xml` (and `materials.xml`) by space partition.
///
/// `geometry` and `materials` are `{sha256}` or `{inline}` sources like `model.import_step`
/// (never a path). The deck is flattened as the tracker import does (universes, rectangular
/// lattices and hexagonal lattices, both orientations, with their `outer`, transforms), the model box is cut by the plane and cylinder
/// surfaces (a quadric that is a plane or a cylinder included), each piece is classified by the
/// cell regions, and the pieces of one cell that touch are united. Every cut is conformal.
///
/// Lattice reuse (draft D4, on by default, `options.reuse_lattices`): a rectangular lattice that
/// is a leaf, placed once without rotation and whose elements are whole boxes inside the cells
/// that enclose them gets one prototype element per distinct universe (cut like the flat
/// partition), the other slots are `model.copy` translations of the prototype's bodies, and the
/// rest of the deck (the `outer` ring, the surrounding cells) is cut flat with the array box left
/// empty. The copies are merged, the remainder is imprinted on the elements it touches (an
/// `outer` ring, a neighbouring cell) and merged, so the result is conformal and has the same
/// volumes, blocks and tags as the flat partition. Any other lattice keeps the flat partition.
/// The `import.partition` diagnostic reports the kernel cuts (prototypes and remainder), copies,
/// imprints and merged cells.
///
/// Result roles: `deck_volumes` and `deck_bodies` (ascending). Roles `volumes` and `bodies` are
/// the intermediate lattice copies (the nested `model.copy` records them; a later merge changes
/// their faces): read `deck_volumes`, never `volumes`. One block per region name (numbered from 1 in name order), one sideset per boundary
/// kind in use (`vacuum` 1, `reflective` 2, `white` 3 when present). Each deck lattice becomes a
/// lattice frame `lat<id>` (a hexagonal lattice a `Hex` layout of the matching orientation, slots
/// the centred axial `(q, r)` and the layer) and each volume inside lattice elements gets a region
/// tag with the deck indices. A hexagonal element is a hexagonal prism of six planes (a lattice
/// of `n` rings gives `3 n (n - 1) + 1` prisms per layer, each of area `sqrt(3)/2 p^2` for the
/// flat-to-flat pitch `p`); hexagonal lattices keep the flat partition (no prototype copies).
///
/// Refused with `csg.unsupported`, naming the deck surface: spheres, cones, tori, elliptic and
/// non-classifiable quadrics (until the sm-boolean tools exist) and a structured lattice
/// `outer` (the reader refuses that). Gaps inside the bounds are dropped with an
/// `import.gap` warning; overlaps fail (`csg.overlap`) unless `options.overlap` says otherwise.
/// Diagnostics: `import.partition` (info: kernel cuts, calls, pieces, unions),
/// `import.reflective_curved`, `import.lattice_rotated`, `import.lattice_multi_placement`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportOpenmc {
    /// The `geometry.xml`.
    pub geometry: Source,
    /// The `materials.xml` (without it every material id is an unnamed material).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub materials: Option<Source>,
    /// Options.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub options: Option<OpenmcOptions>,
}

fn is_sha256_hex(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c))
}

/// Resolve a source to text: inline base64 (capped) or a hash read through [`StepInputs`] and
/// re-hashed.
fn load_text(ctx: &ApplyCtx<'_>, source: &Source, what: &str) -> Result<String, OpError> {
    let bytes: Vec<u8> = match source {
        Source::Inline(i) => {
            if i.inline.len() > MAX_INLINE_BYTES.div_ceil(3) * 4 {
                return Err(OpError::Invalid(format!("csg.inline: the inline {what} is over {MAX_INLINE_BYTES} bytes (stage the file and use sha256)")));
            }
            STANDARD.decode(i.inline.as_bytes()).map_err(|e| OpError::Invalid(format!("csg.inline: the inline {what} is not valid base64: {e}")))?
        }
        Source::Sha256(s) => {
            if !is_sha256_hex(&s.sha256) {
                return Err(OpError::Invalid(format!("csg.bad_sha256: `{}` is not 64 lowercase hex digits", s.sha256)));
            }
            let store = ctx.service::<StepInputs>()?;
            let Some(b) = store.0.get(&s.sha256) else {
                return Err(OpError::Invalid(format!("csg.input_missing: input blob {} not found; stage it first", s.sha256)));
            };
            let got = sha256_hex(&b);
            if got != s.sha256 {
                return Err(OpError::Invalid(format!("csg.input_mismatch: the staged blob for {} hashes to {got}", s.sha256)));
            }
            b.to_vec()
        }
    };
    String::from_utf8(bytes).map_err(|_| OpError::Invalid(format!("csg.utf8: the {what} is not UTF-8 text")))
}

/// Frame definitions in parents-first order and the region tags, written in one model-only
/// transaction after the last geometry edit.
fn write_frames_and_tags(ctx: &mut ApplyCtx<'_>, placed: &Placed, built: &partition::Built, bounds: ([f64; 3], [f64; 3])) -> Result<(), OpError> {
    if placed.lattices.is_empty() {
        return Ok(());
    }
    let by_id: BTreeMap<u32, &deck::LatticePlacement> = placed.lattices.iter().map(|l| (l.id, l)).collect();
    // Layout data per lattice from the deck: dims, pitch and lower left come from the placement
    // record, filled in by `Placed::lattice_geometry`.
    let mut frames: Vec<(String, LatticeFrame)> = Vec::new();
    let (lo, hi) = bounds;
    for l in &placed.lattices {
        if let Some(h) = &l.hex {
            // Slot (0, 0, 0) is the centre of the first layer. Without z extent one slab of the
            // model height keeps every point in layer 0 (as for the rectangular frame).
            let finite = h.z_pitch.is_finite();
            let (dz, z) = if finite { (h.z_pitch, h.center[2] + h.z_pitch * (0.5 - f64::from(h.n_axial) / 2.0)) } else { (hi[2] - lo[2], 0.0) };
            let mut origin = l.xf.apply_point(Point3::new(h.center[0], h.center[1], z));
            if !finite {
                origin.z = 0.5 * (lo[2] + hi[2]);
            }
            let orient = match h.orientation {
                HexOrientation::X => HexOrient::XRows,
                HexOrientation::Y => HexOrient::YRows,
            };
            let layout = Layout::Hex { rings: h.n_rings, pitch: h.pitch, orient, nz: h.n_axial, dz };
            frames.push((format!("lat{}", l.id), LatticeFrame::new(layout, origin, l.parent.map(|p| format!("lat{p}")))));
            continue;
        }
        let g = &l.geometry;
        let mut pitch = [0.0; 3];
        let mut centre = [0.0; 3];
        for a in 0..3 {
            if g.pitch[a].is_finite() {
                pitch[a] = g.pitch[a];
                centre[a] = g.lower_left[a] + 0.5 * g.pitch[a];
            } else {
                // One element along an unbounded axis: a finite pitch that keeps every point of the model in slot 0.
                pitch[a] = hi[a] - lo[a];
                centre[a] = 0.0;
            }
        }
        let mut origin = l.xf.apply_point(Point3::new(centre[0], centre[1], centre[2]));
        let o = [&mut origin.x, &mut origin.y, &mut origin.z];
        for (a, v) in o.into_iter().enumerate() {
            if !g.pitch[a].is_finite() {
                *v = 0.5 * (lo[a] + hi[a]);
            }
        }
        let layout = Layout::Cartesian { n: g.dims, pitch: Vector3::new(pitch[0], pitch[1], pitch[2]) };
        frames.push((format!("lat{}", l.id), LatticeFrame::new(layout, origin, l.parent.map(|p| format!("lat{p}")))));
    }
    // Parents first.
    let depth = |mut id: u32| {
        let mut d = 0;
        while let Some(p) = by_id.get(&id).and_then(|l| l.parent) {
            d += 1;
            id = p;
        }
        d
    };
    let mut order: Vec<usize> = (0..frames.len()).collect();
    order.sort_by_key(|&i| (depth(placed.lattices[i].id), placed.lattices[i].id));

    let mut tags: Vec<(EntityId, RegionTag)> = Vec::new();
    for v in &built.vols {
        let mut slots = BTreeMap::new();
        for h in &placed.cells[v.cell].origin.path {
            if let Some((lid, [i, j, k])) = h.element {
                slots.insert(format!("lat{lid}"), SlotIx::new(i, j, k));
            }
        }
        if !slots.is_empty() {
            tags.push((v.volume, RegionTag::new(slots, None, None)));
        }
    }
    tags.sort_by_key(|t| t.0);
    model_txn(
        ctx,
        |model, _topo| {
            for &i in &order {
                let (name, f) = &frames[i];
                model.define_lattice_frame(name, f.clone()).map_err(|e| OpError::Failed(format!("csg.frame: {e}")))?;
            }
            for (v, t) in tags {
                model.set_region_tag(v, t).map_err(|e| OpError::Failed(format!("csg.tag: {e}")))?;
            }
            Ok(())
        },
        |_, _| Vec::new(),
    )
}

/// The bodies of one copy of a prototype and its slot.
type Placed3 = ([i32; 3], Vec<EntityId>);

fn live_bodies(ctx: &ApplyCtx<'_>) -> Result<BTreeSet<EntityId>, OpError> {
    let topo = ctx.layer::<ModelLayer>()?.host();
    Ok(topo.cells(Dim::Body).iter().copied().filter(|b| topo.is_alive(*b)).collect())
}

/// The volumes of a body, ascending.
fn volumes_of(topo: &dyn TopologyView, body: EntityId) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = topo.boundary(body).iter().map(|i| i.cell).filter(|c| topo.dim_of(*c) == Some(Dim::Volume)).collect();
    v.sort_unstable();
    v
}

fn axis_of(v: Vector3) -> Option<usize> {
    let a = [v.x.abs(), v.y.abs(), v.z.abs()];
    (0..3).find(|&i| (a[i] - 1.0).abs() < 1e-9 && a[(i + 1) % 3] < 1e-9 && a[(i + 2) % 3] < 1e-9)
}

/// True when some face of `vol` lies on the axis-aligned plane `axis = c`.
fn has_face_on(topo: &dyn TopoHost, vol: EntityId, axis: usize, c: f64, tol: f64) -> bool {
    topo.volume_faces(vol).iter().any(|&f| {
        let Carrier::Surface(sc) = topo.carrier(f) else { return false };
        let Some(SurfaceDef::Plane(p)) = topo.shapes().surface_def(sc.surface) else { return false };
        axis_of(p.frame.z) == Some(axis) && ([p.frame.origin.x, p.frame.origin.y, p.frame.origin.z][axis] - c).abs() <= tol
    })
}

/// The sides `(axis, upper)` of the array box `hull` that a cell of the remainder touches: a
/// probe grid just outside the side lies in the cell.
fn rest_sides(placed: &Placed, rest: &[usize], hull: ([f64; 3], [f64; 3])) -> Vec<(usize, bool)> {
    let mut out = Vec::new();
    for axis in 0..3 {
        for upper in [false, true] {
            let eps = 1e-6 * (hull.1[axis] - hull.0[axis]);
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let mut hit = false;
            'grid: for iu in 0..5 {
                for iv in 0..5 {
                    let mut p = [0.0; 3];
                    p[axis] = if upper { hull.1[axis] + eps } else { hull.0[axis] - eps };
                    p[u] = hull.0[u] + (f64::from(iu) + 0.5) / 5.0 * (hull.1[u] - hull.0[u]);
                    p[v] = hull.0[v] + (f64::from(iv) + 0.5) / 5.0 * (hull.1[v] - hull.0[v]);
                    if rest.iter().any(|&c| partition::contains(placed, &placed.cells[c].expr, p)) {
                        hit = true;
                        break 'grid;
                    }
                }
            }
            if hit {
                out.push((axis, upper));
            }
        }
    }
    out
}

/// Build the deck with lattice reuse (draft D4): one prototype element per distinct universe,
/// `model.copy` for the other slots, the remainder cut flat, then imprint and merge so the
/// result is conformal. Three kernel phases, each its own transaction (`model.copy` opens its own).
fn build_reusing(ctx: &mut ApplyCtx<'_>, placed: &Placed, plan: &ReusePlan, bopts: &BuildOptions) -> Result<Built, OpError> {
    let reserved = plan.reserved();
    let sub = BuildOptions { overlap: bopts.overlap, bounds: None, quiet_gaps: false };
    // Phase A: the prototypes and the remainder.
    let ((protos, rest), txn) = topo_txn(ctx, |topo, _model| {
        let mut protos: Vec<Vec<Built>> = Vec::new();
        for g in &plan.groups {
            let mut v = Vec::new();
            for p in &g.protos {
                v.push(partition::build_subset(topo, placed, &p.cells, &[], &sub)?);
            }
            protos.push(v);
        }
        let rest = if plan.rest.is_empty() { None } else { Some(partition::build_subset(topo, placed, &plan.rest, &reserved, bopts)?) };
        Ok((protos, rest))
    })?;
    commit_topology(ctx, &txn, &[])?;

    let mut out = Built::default();
    let mut rest_vols: Vec<Vol> = Vec::new();
    if let Some(r) = rest {
        out.stats.absorb(&r.stats);
        out.notes.extend(r.notes);
        out.start = r.start;
        rest_vols = r.vols;
    } else {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for g in &plan.groups {
            for a in 0..3 {
                lo[a] = lo[a].min(g.hull.0[a]);
                hi[a] = hi[a].max(g.hull.1[a]);
            }
        }
        out.start = (lo, hi);
    }

    // Phase B: the copies. Cell of a copy: the cell at its slot with the same deck origin.
    let by_origin: BTreeMap<&deck::Origin, usize> = placed.cells.iter().enumerate().map(|(i, c)| (&c.origin, i)).collect();
    let mut elems: Vec<Vol> = Vec::new();
    // (group, volumes by slot) for the imprint step.
    for (g, pb) in plan.groups.iter().zip(&protos) {
        for (p, b) in g.protos.iter().zip(pb) {
            out.stats.absorb(&b.stats);
            out.stats.proto_cuts += b.stats.cuts;
            out.notes.extend(b.notes.iter().cloned());
            if b.vols.is_empty() {
                return Err(OpError::Invalid(format!("csg.invalid: universe {} of lattice {} has no volume in an element", p.universe, g.lattice)));
            }
            elems.extend(b.vols.iter().cloned());
            let src_bodies: Vec<EntityId> = b.vols.iter().map(|v| v.body).collect();
            let at = |slot: [i32; 3], v: &Vol| -> Result<usize, OpError> {
                let o = reuse::at_slot(&placed.cells[v.cell].origin, slot).ok_or_else(|| OpError::Invalid("csg.invalid: a lattice cell has no element hop".into()))?;
                by_origin.get(&o).copied().ok_or_else(|| OpError::Invalid(format!("csg.invalid: no placed cell {o} for a lattice copy")))
            };
            let delta = |from: [i32; 3], to: [i32; 3]| [0, 1, 2].map(|a| f64::from(to[a] - from[a]) * g.pitch[a]);
            let mut have: BTreeMap<[i32; 3], Vec<EntityId>> = BTreeMap::new();
            have.insert(p.ref_slot, src_bodies.clone());
            let mut want: BTreeSet<[i32; 3]> = p.slots.iter().copied().collect();
            // One call places `count` copies from `from` (bodies `src`) along `step`; the new
            // bodies, ascending, are copy 1.. in order, each in source order.
            let place = |ctx: &mut ApplyCtx<'_>, from: [i32; 3], src: &[EntityId], step: [i32; 3], count: u32, out: &mut Built| -> Result<Vec<Placed3>, OpError> {
                let before = live_bodies(ctx)?;
                let d = delta([0; 3], step);
                TypedOp::apply(&ModelCopy { entities: src.to_vec(), count: Some(count), xform: Some(CopyXform::Move { delta: d }), lattice: None }, ctx)?;
                let after = live_bodies(ctx)?;
                let new: Vec<EntityId> = after.difference(&before).copied().collect();
                if new.len() != src.len() * count as usize {
                    return Err(OpError::Failed(format!("csg.copy: {} copies of {} bodies gave {} new bodies", count, src.len(), new.len())));
                }
                out.stats.copies += count as usize;
                let mut placed_at = Vec::new();
                for k in 1..=count as usize {
                    let slot = [0, 1, 2].map(|a| from[a] + step[a] * k as i32);
                    placed_at.push((slot, new[(k - 1) * src.len()..k * src.len()].to_vec()));
                }
                Ok(placed_at)
            };
            // Slots reached by runs from slots already placed: x, then y, then z.
            let record = |ctx: &mut ApplyCtx<'_>, slot: [i32; 3], bodies: &[EntityId], elems: &mut Vec<Vol>, have: &mut BTreeMap<[i32; 3], Vec<EntityId>>| -> Result<(), OpError> {
                let topo = ctx.layer::<ModelLayer>()?.host();
                for (v, &nb) in b.vols.iter().zip(bodies) {
                    let vs = volumes_of(topo, nb);
                    let [vol] = vs.as_slice() else { return Err(OpError::Failed(format!("csg.copy: a copied body holds {} volumes", vs.len()))) };
                    elems.push(Vol { volume: *vol, body: nb, cell: at(slot, v)? });
                }
                have.insert(slot, bodies.to_vec());
                Ok(())
            };
            for a in 0..3 {
                let mut step = [0; 3];
                step[a] = 1;
                let starts: Vec<[i32; 3]> = have.keys().copied().collect();
                for s in starts {
                    let mut c = 0;
                    while want.contains(&[0, 1, 2].map(|x| s[x] + step[x] * (c + 1))) {
                        c += 1;
                    }
                    if c == 0 {
                        continue;
                    }
                    let src = have[&s].clone();
                    for (slot, bodies) in place(ctx, s, &src, step, c as u32, &mut out)? {
                        want.remove(&slot);
                        record(ctx, slot, &bodies, &mut elems, &mut have)?;
                    }
                }
            }
            // Whatever is left comes straight from the prototype.
            let left: Vec<[i32; 3]> = want.iter().copied().collect();
            for slot in left {
                let d = delta(p.ref_slot, slot);
                let before = live_bodies(ctx)?;
                TypedOp::apply(&ModelCopy { entities: src_bodies.clone(), count: Some(1), xform: Some(CopyXform::Move { delta: d }), lattice: None }, ctx)?;
                let after = live_bodies(ctx)?;
                let new: Vec<EntityId> = after.difference(&before).copied().collect();
                if new.len() != src_bodies.len() {
                    return Err(OpError::Failed(format!("csg.copy: a copy of {} bodies gave {} new bodies", src_bodies.len(), new.len())));
                }
                out.stats.copies += 1;
                record(ctx, slot, &new, &mut elems, &mut have)?;
            }
        }
    }

    // Phase C: merge the elements, imprint the remainder against the elements it touches,
    // merge again, and read the exterior faces.
    let all_ids = |vs: &[Vol]| -> Vec<EntityId> { vs.iter().map(|v| v.volume).collect() };
    let elem_vols = all_ids(&elems);
    let rest_ids = all_ids(&rest_vols);
    let hulls: Vec<([f64; 3], [f64; 3])> = plan.groups.iter().map(|g| g.hull).collect();
    let sides: Vec<Vec<(usize, bool)>> = hulls.iter().map(|h| rest_sides(placed, &plan.rest, *h)).collect();
    let (stats, txn) = topo_txn(ctx, |topo, _model| {
        let mut imprints = 0usize;
        let mut merged = 0usize;
        let mo = MergeOptions::default();
        let r = merge(topo, &elem_vols, &[], &mo).map_err(|e| OpError::Failed(format!("csg.merge: merging the lattice copies: {e}")))?;
        merged += r.merged.len();
        let tol = 10.0 * topo.tolerance().linear;
        let body_of = |t: &dyn TopoHost, v: EntityId| -> Option<EntityId> { t.coboundary(v).iter().map(|i| i.cell).find(|c| t.dim_of(*c) == Some(Dim::Body)) };
        let mut pairs: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
        for (h, ss) in hulls.iter().zip(&sides) {
            for &(axis, upper) in ss {
                let c = if upper { h.1[axis] } else { h.0[axis] };
                let rs: Vec<EntityId> = rest_ids.iter().copied().filter(|&v| has_face_on(topo, v, axis, c, tol)).collect();
                let es: Vec<EntityId> = elem_vols.iter().copied().filter(|&v| has_face_on(topo, v, axis, c, tol)).collect();
                for &r in &rs {
                    for &e in &es {
                        if let (Some(rb), Some(eb)) = (body_of(topo, r), body_of(topo, e)) {
                            pairs.insert((rb, eb));
                        }
                    }
                }
            }
        }
        for (rb, eb) in pairs {
            let o = imprint_bodies(topo, &[rb, eb], &ImprintOptions { merge: true, tol: None }).map_err(|e| OpError::Failed(format!("csg.imprint: imprinting the remainder on a lattice element: {e}")))?;
            imprints += 1;
            merged += o.merge.map_or(0, |m| m.merged.len());
        }
        if !rest_ids.is_empty() {
            let mut all = elem_vols.clone();
            all.extend(rest_ids.iter().copied());
            let r = merge(topo, &all, &[], &mo).map_err(|e| OpError::Failed(format!("csg.merge: merging the remainder: {e}")))?;
            merged += r.merged.len();
        }
        Ok((imprints, merged))
    })?;
    out.stats.imprints = stats.0;
    out.stats.merged = stats.1;

    let mut vols = rest_vols;
    vols.extend(elems);
    vols.sort_by_key(|v| (v.body, v.cell));
    let volumes: Vec<EntityId> = vols.iter().map(|v| v.volume).collect();
    let bodies: Vec<EntityId> = vols.iter().map(|v| v.body).collect();
    commit_topology(ctx, &txn, &[("deck_volumes", &volumes), ("deck_bodies", &bodies)])?;
    out.boundary_faces = {
        let topo = ctx.layer::<ModelLayer>()?.topo_host();
        partition::boundary_faces(&**topo, placed, &vols)
    };
    out.vols = vols;
    Ok(out)
}

/// The hexagonal elements of a deck: each is built on its own (the brick of its hexagon cut by its
/// six walls and by what its universe holds), the rest of the deck is cut without the hex walls
/// (a hex lattice's `outer` cells are cut as the whole region around, `PCell::core`), and the
/// array, a union of whole prisms, is subtracted from those cells. The pieces are then merged, so
/// the result is conformal. No plane of the tiling ever cuts a piece: the triple junctions of a
/// hexagonal tiling lie on lines where three walls meet, and cutting a convex piece by the third
/// wall through two of its edges is what `sm_boolean::webcut` does not do robustly.
fn build_hex(ctx: &mut ApplyCtx<'_>, placed: &Placed, bopts: &BuildOptions) -> Result<Built, OpError> {
    use sm_boolean::{BoolOptions, BoolRule, boolean};
    let n_el = placed.hex_elements.len();
    // Cells by the innermost hex element whose path starts their path.
    let by_path: BTreeMap<&[deck::Hop], usize> = placed.hex_elements.iter().enumerate().map(|(i, e)| (e.path.as_slice(), i)).collect();
    if by_path.len() != n_el {
        return Err(CsgbrepError::Invalid("two hexagonal elements have the same path".into()).into());
    }
    for e in &placed.hex_elements {
        if (1..e.path.len()).any(|n| by_path.contains_key(&e.path[..n])) {
            return Err(CsgbrepError::Unsupported { line: None, what: format!("hex_lattice {}: a hexagonal lattice inside the element of another one is not supported by the B-rep import yet", e.lattice) }.into());
        }
    }
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); n_el];
    let mut rest: Vec<usize> = Vec::new();
    for (ci, c) in placed.cells.iter().enumerate() {
        let hit = (1..=c.origin.path.len()).rev().find_map(|n| by_path.get(&c.origin.path[..n]).copied());
        match hit {
            Some(e) => groups[e].push(ci),
            None => rest.push(ci),
        }
    }
    // z extent of every element: from its cells (or the given bounds).
    let mut zs: Vec<(f64, f64, bool, bool)> = Vec::with_capacity(n_el);
    for (e, g) in placed.hex_elements.iter().zip(&groups) {
        let b = placed.bounds_of_cells(g);
        let z = match bopts.bounds {
            Some((lo, hi)) => (lo[2], hi[2], true, true),
            None => (b.lo[2], b.hi[2], b.lo_plane[2], b.hi_plane[2]),
        };
        if !(z.0.is_finite() && z.1.is_finite() && z.0 < z.1) {
            return Err(CsgbrepError::Unbounded(format!("the z extent of hexagonal lattice {} is {:?} to {:?}; give `options.bounds`", e.lattice, z.0, z.1)).into());
        }
        zs.push(z);
    }
    // Cells of the rest are cut with the array exclusion left out of the outer cells.
    let mut view = placed.clone();
    for c in &mut view.cells {
        if let Some(core) = c.core.clone() {
            c.expr = core;
        }
    }
    let outer_cell = |i: usize| placed.cells[i].core.is_some();
    // Adjacent elements (centres closer than the pitch, z ranges touching) form one array solid.
    let centre = |e: &deck::HexElement| {
        let n = e.polygon.len() as f64;
        [e.polygon.iter().map(|p| p[0]).sum::<f64>() / n, e.polygon.iter().map(|p| p[1]).sum::<f64>() / n]
    };
    let pitch_of = |e: &deck::HexElement| {
        let (a, b) = (e.polygon[0], e.polygon[3]);
        (a[0] - b[0]).hypot(a[1] - b[1]) * 3f64.sqrt() / 2.0
    };

    let ((elem_built, rest_built, prism_notes, arrays), txn) = topo_txn(ctx, |topo, _model| {
        let rest_built = if rest.is_empty() { None } else { Some(partition::build_subset(topo, &view, &rest, &[], bopts)?) };
        let mut elem_built: Vec<partition::Built> = Vec::new();
        for (i, g) in groups.iter().enumerate() {
            let (lo, hi) = partition::hex_box(&placed.hex_elements[i].polygon, zs[i]);
            let sub = BuildOptions { overlap: bopts.overlap, bounds: Some((lo, hi)), quiet_gaps: true };
            elem_built.push(if g.is_empty() { partition::Built::default() } else { partition::build_subset(topo, &view, g, &[], &sub)? });
        }
        // The array solid(s): one prism per hexagon footprint (all layers of a column in one), united with its
        // neighbours.
        let key = |e: &deck::HexElement| (e.lattice, e.polygon.iter().flatten().map(|v| v.to_bits()).collect::<Vec<u64>>());
        let mut footprints: BTreeMap<(u32, Vec<u64>), (usize, f64, f64)> = BTreeMap::new();
        for (i, (e, z)) in placed.hex_elements.iter().zip(&zs).enumerate() {
            let f = footprints.entry(key(e)).or_insert((i, z.0, z.1));
            f.1 = f.1.min(z.0);
            f.2 = f.2.max(z.1);
        }
        let feet: Vec<(usize, f64, f64)> = footprints.into_values().collect();
        let mut prisms: Vec<EntityId> = Vec::with_capacity(feet.len());
        for &(i, z0, z1) in &feet {
            prisms.push(partition::hex_prism(topo, &placed.hex_elements[i].polygon, (z0, z1))?);
        }
        let mut notes = Vec::new();
        for (i, g) in elem_built.iter().enumerate() {
            let poly = &placed.hex_elements[i].polygon;
            let area = 0.5 * (0..6).map(|k| poly[k][0] * poly[(k + 1) % 6][1] - poly[(k + 1) % 6][0] * poly[k][1]).sum::<f64>().abs();
            let want = area * (zs[i].1 - zs[i].0);
            let got: f64 = g.vols.iter().map(|v| topo.volume_moments(v.volume).map_or(0.0, |m| m.volume)).sum();
            if (got - want).abs() > 1e-9 * want {
                notes.push(partition::Note { code: "import.gap", message: format!("the cells of hexagonal element {} cover {got:e} of its volume {want:e}; the rest is not part of the model", placed.hex_elements[i].path.last().map_or(String::new(), |h| format!("{}[{:?}]", h.cell, h.element.map(|e| e.1)))) });
            }
        }
        let n_pr = prisms.len();
        let mut seen = vec![false; n_pr];
        let mut arrays: Vec<EntityId> = Vec::new();
        for start in 0..n_pr {
            if seen[start] {
                continue;
            }
            // The component of prisms whose footprints touch: centres closer than the pitch.
            seen[start] = true;
            let mut comp = vec![start];
            let mut head = 0;
            while head < comp.len() {
                let a = comp[head];
                head += 1;
                for b in 0..n_pr {
                    if seen[b] {
                        continue;
                    }
                    let (ea, eb) = (&placed.hex_elements[feet[a].0], &placed.hex_elements[feet[b].0]);
                    let (ca, cb) = (centre(ea), centre(eb));
                    if (ca[0] - cb[0]).hypot(ca[1] - cb[1]) <= 1.01 * pitch_of(ea) {
                        seen[b] = true;
                        comp.push(b);
                    }
                }
            }
            if comp.len() == 1 {
                arrays.push(prisms[start]);
                continue;
            }
            let rest_bodies: Vec<EntityId> = comp[1..].iter().map(|&i| prisms[i]).collect();
            let r = boolean(topo, BoolRule::Unite, &[prisms[start]], &rest_bodies, &BoolOptions::default()).map_err(|e| partition::kernel("uniting the prisms of a hexagonal lattice", e))?;
            match r.bodies.as_slice() {
                [one] => arrays.push(*one),
                other => return Err(CsgbrepError::Kernel(format!("uniting the prisms of a hexagonal lattice gave {} bodies", other.len())).into()),
            }
        }
        Ok((elem_built, rest_built, notes, arrays))
    })?;
    commit_topology(ctx, &txn, &[])?;

    let mut out = Built::default();
    let mut elems: Vec<Vol> = Vec::new();
    for b in &elem_built {
        out.stats.absorb(&b.stats);
        out.stats.proto_cuts += b.stats.cuts;
        out.notes.extend(b.notes.iter().cloned());
        elems.extend(b.vols.iter().cloned());
    }
    out.notes.extend(prism_notes);
    let mut rest_vols: Vec<Vol> = Vec::new();
    if let Some(r) = rest_built {
        out.stats.absorb(&r.stats);
        out.notes.extend(r.notes);
        out.start = r.start;
        rest_vols = r.vols;
    }
    if out.start == ([0.0; 3], [0.0; 3]) {
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for (e, z) in placed.hex_elements.iter().zip(&zs) {
            let (l, h) = partition::hex_box(&e.polygon, *z);
            for a in 0..3 {
                lo[a] = lo[a].min(l[a]);
                hi[a] = hi[a].max(h[a]);
            }
        }
        out.start = (lo, hi);
    }

    // Subtract the array from the cells around it, then merge everything.
    let (vols, txn) = topo_txn(ctx, |topo, _model| {
        let mut vols: Vec<Vol> = Vec::new();
        let mut kept: Vec<Vol> = Vec::new();
        for v in rest_vols {
            if !outer_cell(v.cell) {
                kept.push(v);
                continue;
            }
            let r = boolean(topo, BoolRule::Subtract, &[v.body], &arrays, &BoolOptions { keep: true, ..BoolOptions::default() }).map_err(|e| partition::kernel(&format!("subtracting the hexagonal array from cell {}", placed.cells[v.cell].origin), e))?;
            topo.kill_cascade(v.body).map_err(|e| CsgbrepError::Kernel(e.to_string()))?;
            for &b in &r.bodies {
                let volume = partition::volume_of_body(&*topo, b).ok_or_else(|| CsgbrepError::Invalid("a result has no volume".into()))?;
                vols.push(Vol { volume, body: b, cell: v.cell });
            }
        }
        for a in &arrays {
            topo.kill_cascade(*a).map_err(|e| CsgbrepError::Kernel(e.to_string()))?;
        }
        vols.extend(kept);
        let mut all: Vec<EntityId> = elems.iter().map(|v| v.volume).collect();
        all.extend(vols.iter().map(|v| v.volume));
        // Faces that lie in one plane but are cut differently (a pin of another radius one layer up, a slab
        // or the gas above the layers) are imprinted on each other first; the shared walls match as built.
        let tol = 10.0 * topo.tolerance().linear;
        let body_of = |t: &dyn TopoHost, v: EntityId| -> Option<EntityId> { t.coboundary(v).iter().map(|i| i.cell).find(|c| t.dim_of(*c) == Some(Dim::Body)) };
        let mut pairs: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
        let mut pairs_rest: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
        let foot = |e: &deck::HexElement| (e.lattice, e.polygon.iter().flatten().map(|v| v.to_bits()).collect::<Vec<u64>>());
        for (i, a) in elem_built.iter().enumerate() {
            let z_tol = 1e-9 * (zs[i].1 - zs[i].0);
            for (j, b) in elem_built.iter().enumerate() {
                if foot(&placed.hex_elements[i]) == foot(&placed.hex_elements[j]) && (zs[i].1 - zs[j].0).abs() <= z_tol {
                    for va in &a.vols {
                        for vb in &b.vols {
                            if let (Some(x), Some(y)) = (body_of(topo, va.volume), body_of(topo, vb.volume)) {
                                pairs.insert((x.min(y), x.max(y)));
                            }
                        }
                    }
                }
            }
            for c in [zs[i].0, zs[i].1] {
                let es: Vec<&Vol> = a.vols.iter().filter(|v| has_face_on(topo, v.volume, 2, c, tol)).collect();
                if es.is_empty() {
                    continue;
                }
                for r in vols.iter().filter(|r| has_face_on(topo, r.volume, 2, c, tol)) {
                    for e in &es {
                        if let (Some(x), Some(y)) = (body_of(topo, r.volume), body_of(topo, e.volume)) {
                            pairs_rest.insert((x.min(y), x.max(y)));
                        }
                    }
                }
            }
        }
        let mo = MergeOptions::default();
        let r = merge(topo, &all, &[], &mo).map_err(|e| OpError::Failed(format!("csg.merge: merging the hexagonal array and its surroundings: {e}")))?;
        let mut merged = r.merged.len();
        for (x, y) in pairs {
            let o = imprint_bodies(topo, &[x, y], &ImprintOptions { merge: true, tol: None }).map_err(|e| OpError::Failed(format!("csg.imprint: imprinting a hexagonal element: {e}")))?;
            out.stats.imprints += 1;
            merged += o.merge.map_or(0, |m| m.merged.len());
        }
        // A cell that abuts the elements in a z plane (a slab above or below the layers) has one face under
        // many hexagons. The kernel's imprint stops at the second hexagon that shares an edge with the first
        // ("a cut runs from an inner loop back to the same inner loop"): the import is refused, typed, rather
        // than returned with a non-conformal interface.
        for (x, y) in pairs_rest {
            let o = imprint_bodies(topo, &[x, y], &ImprintOptions { merge: true, tol: None }).map_err(|e| {
                CsgbrepError::Unsupported { line: None, what: format!("hex_lattice: a cell that abuts the lattice in a z plane (a slab above or below its layers) needs the kernel to imprint one face on many hexagons, which it does not do yet: {e}") }
            })?;
            out.stats.imprints += 1;
            merged += o.merge.map_or(0, |m| m.merged.len());
        }
        let r = merge(topo, &all, &[], &mo).map_err(|e| OpError::Failed(format!("csg.merge: merging the hexagonal array and its surroundings: {e}")))?;
        out.stats.merged = merged + r.merged.len();
        Ok(vols)
    })?;
    let mut vols = vols;
    vols.extend(elems);
    vols.sort_by_key(|v| (v.body, v.cell));
    let volumes: Vec<EntityId> = vols.iter().map(|v| v.volume).collect();
    let bodies: Vec<EntityId> = vols.iter().map(|v| v.body).collect();
    commit_topology(ctx, &txn, &[("deck_volumes", &volumes), ("deck_bodies", &bodies)])?;
    out.boundary_faces = {
        let topo = ctx.layer::<ModelLayer>()?.topo_host();
        partition::boundary_faces(&**topo, placed, &vols)
    };
    out.vols = vols;
    Ok(out)
}

impl TypedOp for ImportOpenmc {
    const NAME: &'static str = "model.import_openmc";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    /// Bump when the builder changes the cells it creates for the same deck.
    const IMPL_VERSION: u32 = 2;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let geometry = load_text(ctx, &self.geometry, "geometry.xml")?;
        let materials = self.materials.as_ref().map(|m| load_text(ctx, m, "materials.xml")).transpose()?;
        let ir = sm_csgio::openmc::import(&geometry, materials.as_deref()).map_err(CsgbrepError::from)?;
        let o = self.options.clone().unwrap_or_default();
        if let Some(b) = o.bounds
            && !(b.iter().all(|v| v.is_finite()) && b[0] < b[3] && b[1] < b[4] && b[2] < b[5])
        {
            return Err(OpError::Invalid(format!("csg.invalid: options.bounds {b:?} is not a finite box")));
        }
        let placed = deck::place(&ir, &o.material_names.clone().unwrap_or_default(), o.max_cells.unwrap_or(DEFAULT_MAX_CELLS))?;
        for s in &placed.surfaces {
            if s.flag == BoundaryFlag::Reflective && !matches!(s.def, SurfaceDef::Plane(_)) {
                ctx.warn("import.reflective_curved", format!("deck surface {} is reflective but not planar; reflection is exact on planes only", s.deck_id), &[]);
            }
        }
        for l in &placed.lattices {
            if l.rotated {
                ctx.warn("import.lattice_rotated", format!("lattice {} is placed with a rotation: its frame cannot describe it, the region tags come from the deck indices", l.id), &[]);
            }
            if l.placements > 1 {
                let msg = format!("lattice {} is placed {} times; its frame describes the first placement, the tags carry the deck indices", l.id, l.placements);
                if l.parents_differ {
                    ctx.warn("import.lattice_multi_placement", msg, &[]);
                } else {
                    ctx.info("import.lattice_multi_placement", msg, &[]);
                }
            }
        }
        let policy = match o.overlap.unwrap_or_default() {
            OverlapMode::Error => OverlapPolicy::Error,
            OverlapMode::FirstWins => OverlapPolicy::FirstWins,
            OverlapMode::Report => OverlapPolicy::Report,
        };
        let bopts = BuildOptions { overlap: policy, bounds: o.bounds.map(|b| ([b[0], b[1], b[2]], [b[3], b[4], b[5]])), quiet_gaps: false };
        let has_hex = !placed.hex_elements.is_empty();
        let plan = if o.reuse_lattices.unwrap_or(true) && !has_hex { reuse::plan(&placed) } else { ReusePlan { groups: Vec::new(), rest: Vec::new() } };
        let built = if has_hex {
            build_hex(ctx, &placed, &bopts)?
        } else if plan.groups.is_empty() {
            let (built, txn) = topo_txn(ctx, |topo, _model| partition::build(topo, &placed, &bopts).map_err(OpError::from))?;
            let volumes: Vec<EntityId> = built.vols.iter().map(|v| v.volume).collect();
            let bodies: Vec<EntityId> = built.vols.iter().map(|v| v.body).collect();
            commit_topology(ctx, &txn, &[("deck_volumes", &volumes), ("deck_bodies", &bodies)])?;
            built
        } else {
            build_reusing(ctx, &placed, &plan, &bopts)?
        };
        let volumes: Vec<EntityId> = built.vols.iter().map(|v| v.volume).collect();
        let s = built.stats;
        ctx.info(
            "import.partition",
            format!(
                "{} kernel cuts in {} webcut calls, {} pieces, {} unions, {} volumes; lattice reuse: {} lattices, {} kernel cuts in prototypes, {} copies, {} imprints, {} cells merged",
                s.cuts,
                s.cut_calls,
                s.pieces,
                s.unions,
                volumes.len(),
                plan.groups.len(),
                s.proto_cuts,
                s.copies,
                s.imprints,
                s.merged
            ),
            &[],
        );
        for n in &built.notes {
            ctx.warn(n.code, n.message.clone(), &[]);
        }

        // Blocks: one per region name, numbered in name order.
        let mut by_name: BTreeMap<&str, Vec<EntityId>> = BTreeMap::new();
        for v in &built.vols {
            by_name.entry(placed.cells[v.cell].name.as_str()).or_default().push(v.volume);
        }
        for (n, (name, vols)) in by_name.iter().enumerate() {
            TypedOp::apply(&BlockAdd { block: n as u32 + 1, volumes: vols.clone(), faces: Vec::new(), name: Some((*name).to_owned()) }, ctx)?;
        }
        // Sidesets: vacuum 1, reflective 2, white 3, when present.
        for (k, name) in ["vacuum", "reflective", "white"].into_iter().enumerate() {
            if !built.boundary_faces[k].is_empty() {
                TypedOp::apply(&SidesetAdd { sideset: k as u32 + 1, faces: built.boundary_faces[k].clone(), wrt: None, name: Some(name.to_owned()) }, ctx)?;
            }
        }
        let names: BTreeSet<&str> = by_name.keys().copied().collect();
        ctx.info("import.regions", format!("{} region name(s): {}", names.len(), names.into_iter().collect::<Vec<_>>().join(", ")), &[]);
        write_frames_and_tags(ctx, &placed, &built, built.start)?;
        Ok(())
    }
}
