//! Instancing: a 61-pin hex lattice with 3 overrides nested in a core lattice, flatten with
//! instance paths, path-prefix overrides and deterministic never-reused ids.

mod common;

use common::*;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyViewExt};
use std::collections::BTreeMap;

struct World {
    t: CellComplex,
    m: Model,
    fuel: EntityId,
    clad: EntityId,
    absorber: EntityId,
    water: EntityId,
    pin: EntityId,
    assembly: EntityId,
    core: EntityId,
    inst: EntityId,
    absorber_slots: [SlotIx; 3],
}

fn recolor(from: EntityId, to: EntityId) -> ColorMap {
    ColorMap { volume: BTreeMap::from([(from, to)]), face: BTreeMap::new() }
}

fn world() -> World {
    let mut t = CellComplex::new();
    let fuel_v = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [0.4, 0.4, 1.0])).unwrap();
    let clad_v = with_op(&mut t, OpId(2), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 1.0), [0.4, 0.4, 1.0])).unwrap();
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume).unwrap();
    let clad = m.add_color(2, "clad", Domain::Volume).unwrap();
    let absorber = m.add_color(3, "absorber", Domain::Volume).unwrap();
    let water = m.add_color(4, "water", Domain::Volume).unwrap();
    m.color_volumes(fuel, &[fuel_v.volume]).unwrap();
    m.color_volumes(clad, &[clad_v.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let pin = m.add_prototype(&[clad_v.volume, fuel_v.volume]).unwrap();
    let absorber_slots = [SlotIx::new(0, 0, 0), SlotIx::new(2, -1, 0), SlotIx::new(-3, 3, 0)];
    let slots: BTreeMap<SlotIx, Slot> = absorber_slots.iter().map(|ix| (*ix, Slot { fill: Fill::Prototype(pin), recolor: recolor(fuel, absorber) })).collect();
    let assembly = m
        .add_lattice(Lattice { layout: Layout::Hex { rings: 5, pitch: 1.26, orient: HexOrient::XRows, nz: 1, dz: 0.0 }, origin: Transform::IDENTITY, outer: Fill::Prototype(pin), slots })
        .unwrap();
    let core = m
        .add_lattice(Lattice {
            layout: Layout::Cartesian { n: [3, 3, 1], pitch: Vector3::new(20.0, 20.0, 0.0) },
            origin: Transform::IDENTITY,
            outer: Fill::Lattice(assembly),
            slots: BTreeMap::new(),
        })
        .unwrap();
    let inst = m.add_instance(InstanceRow { of: Fill::Lattice(core), at: Transform::translation(Vector3::new(100.0, 0.0, 0.0)), recolor: ColorMap::default() }).unwrap();
    World { t, m, fuel, clad, absorber, water, pin, assembly, core, inst, absorber_slots }
}

fn path(w: &World, core_ix: SlotIx, pin_ix: SlotIx) -> InstancePath {
    [PathStep::Instance(w.inst), PathStep::Slot { lattice: w.core, ix: core_ix }, PathStep::Slot { lattice: w.assembly, ix: pin_ix }].into_iter().collect()
}

#[test]
fn hex_lattice_of_61_pins_nested_in_a_core_flattens_with_paths_and_overrides() {
    let w = world();
    assert_eq!(Layout::Hex { rings: 5, pitch: 1.0, orient: HexOrient::XRows, nz: 1, dz: 0.0 }.slot_count(), 61);
    let flat = flatten(&w.m).unwrap();
    assert_eq!(flat.paths.len(), 9 * 61);
    assert_eq!(flat.entries.len(), 9 * 61 * 2);
    assert!(flat.paths.windows(2).all(|p| p[0] < p[1]), "paths are strictly increasing");
    // An absorber pin: fuel recolored by the inner slot; the clad is not.
    let ap = path(&w, SlotIx::new(1, 1, 0), SlotIx::new(2, -1, 0));
    let r = flat.find(&ap).unwrap();
    assert_eq!(r.len(), 2);
    let colors: BTreeMap<EntityId, EntityId> = flat.entries[r.clone()].iter().map(|e| (e.volume, e.color)).collect();
    assert_eq!(colors.values().copied().collect::<std::collections::BTreeSet<_>>(), [w.clad, w.absorber].into_iter().collect());
    // A plain pin keeps fuel.
    let pp = path(&w, SlotIx::new(1, 1, 0), SlotIx::new(1, 0, 0));
    let plain: Vec<EntityId> = flat.entries[flat.find(&pp).unwrap()].iter().map(|e| e.color).collect();
    assert!(plain.contains(&w.fuel) && plain.contains(&w.clad) && !plain.contains(&w.absorber));
    // 3 absorber pins in each of 9 assemblies.
    assert_eq!(flat.entries.iter().filter(|e| e.color == w.absorber).count(), 27);
    // Placement: instance shift + core cell (1,2) + hex slot (1,0) at XRows pitch 1.26.
    let p = path(&w, SlotIx::new(1, 2, 0), SlotIx::new(1, 0, 0));
    let e = &flat.entries[flat.find(&p).unwrap()][0];
    let pl = flat.placement_of(e);
    assert!(!pl.orientation_reversed);
    let o = pl.transform.apply_point(Point3::new(0.0, 0.0, 0.0));
    assert!((o - Point3::new(100.0 + 20.0 + 1.26, 40.0, 0.0)).norm() < 1e-12, "{o:?}");
    // A hex ring-4 corner slot.
    let c = path(&w, SlotIx::new(0, 0, 0), SlotIx::new(4, 0, 0));
    let o = flat.placement_of(&flat.entries[flat.find(&c).unwrap()][0]).transform.apply_point(Point3::new(0.0, 0.0, 0.0));
    assert!((o - Point3::new(100.0 + 4.0 * 1.26, 0.0, 0.0)).norm() < 1e-12);
    // Unknown and out-of-range paths.
    assert!(flat.find(&path(&w, SlotIx::new(5, 5, 0), SlotIx::new(0, 0, 0))).is_none());
    assert!(flat.find(&[PathStep::Instance(w.inst)]).is_none(), "only leaves are copies");
}

#[test]
fn outer_level_path_override_reaches_one_assembly_and_wins_last() {
    let mut w = world();
    let before = flatten(&w.m).unwrap();
    // Absorbers (and only there) become water in assembly (1,1,0): prefix = instance + core slot.
    let prefix: InstancePath = [PathStep::Instance(w.inst), PathStep::Slot { lattice: w.core, ix: SlotIx::new(1, 1, 0) }].into_iter().collect();
    w.m.add_path_override(PathOverride { prefix: prefix.clone(), recolor: recolor(w.absorber, w.water) }).unwrap();
    // A deeper, conflicting override on one pin is applied first and so loses to the outer one.
    let deep: InstancePath = prefix.iter().copied().chain([PathStep::Slot { lattice: w.assembly, ix: SlotIx::new(1, 0, 0) }]).collect();
    w.m.add_path_override(PathOverride { prefix: deep, recolor: recolor(w.fuel, w.clad) }).unwrap();
    let after = flatten(&w.m).unwrap();
    assert_eq!(before.entries.len(), after.entries.len());
    let n_abs = |f: &FlatModel, c: EntityId| f.entries.iter().filter(|e| e.color == c).count();
    assert_eq!(n_abs(&after, w.absorber), 24, "one assembly lost its three absorbers");
    assert_eq!(n_abs(&after, w.water), 3);
    // Other assemblies are untouched, the deep override recolored one fuel to clad.
    let other = path(&w, SlotIx::new(0, 0, 0), SlotIx::new(0, 0, 0));
    assert_eq!(&after.entries[after.find(&other).unwrap()], &before.entries[before.find(&other).unwrap()]);
    let deep_pin = path(&w, SlotIx::new(1, 1, 0), SlotIx::new(1, 0, 0));
    assert!(after.entries[after.find(&deep_pin).unwrap()].iter().all(|e| e.color == w.clad));
    assert_ne!(before.fingerprint(), after.fingerprint());
    // Overrides are validated.
    assert!(w.m.add_path_override(PathOverride { prefix: InstancePath::new(), recolor: ColorMap::default() }).is_err());
}

#[test]
fn recolor_order_is_inner_first_outer_last_and_per_entity_beats_per_color() {
    let mut w = world();
    // Inner slot: fuel -> absorber (per color). Instance: absorber -> water. Outer wins on top.
    let inst2 = w.m.add_instance(InstanceRow { of: Fill::Lattice(w.assembly), at: Transform::IDENTITY, recolor: recolor(w.absorber, w.water) }).unwrap();
    let flat = flatten(&w.m).unwrap();
    let p: InstancePath = [PathStep::Instance(inst2), PathStep::Slot { lattice: w.assembly, ix: SlotIx::new(0, 0, 0) }].into_iter().collect();
    let cols: Vec<EntityId> = flat.entries[flat.find(&p).unwrap()].iter().map(|e| e.color).collect();
    assert!(cols.contains(&w.water) && cols.contains(&w.clad), "{cols:?}");
    // Per-volume entry beats the per-color one inside one map.
    let fuel_vol = *w.m.instancing().prototypes[&w.pin].volumes.iter().find(|v| w.m.volume_color(**v) == Some(w.fuel)).unwrap();
    let mut both = recolor(w.fuel, w.absorber);
    both.volume.insert(fuel_vol, w.water);
    assert_eq!(both.apply_volume(fuel_vol, w.fuel), w.water);
    assert_eq!(both.apply_volume(EntityId::new(EntityKind::TopoVolume, 99).unwrap(), w.fuel), w.absorber);
    // Domain check: a volume color may only map to a volume color.
    let fc = w.m.add_color(50, "bc", Domain::Face).unwrap();
    let bad = ColorMap { volume: BTreeMap::from([(w.fuel, fc)]), face: BTreeMap::new() };
    assert_eq!(w.m.add_instance(InstanceRow { of: Fill::Void, at: Transform::IDENTITY, recolor: bad }), Err(ModelError::ColorDomain(fc)));
    let bad_face = ColorMap { volume: BTreeMap::new(), face: BTreeMap::from([(fc, w.fuel)]) };
    assert!(w.m.add_instance(InstanceRow { of: Fill::Void, at: Transform::IDENTITY, recolor: bad_face }).is_err());
    // Face recolor resolves through the same chain.
    let face = w.t.faces()[0];
    let fc2 = w.m.add_color(51, "bc2", Domain::Face).unwrap();
    let inst3 = w.m.add_instance(InstanceRow { of: Fill::Prototype(w.pin), at: Transform::IDENTITY, recolor: ColorMap { volume: BTreeMap::new(), face: BTreeMap::from([(face, fc2)]) } }).unwrap();
    assert_eq!(w.m.resolve_face_color(&[PathStep::Instance(inst3)], face).unwrap(), Some(fc2));
    assert_eq!(w.m.resolve_face_color(&[PathStep::Instance(w.inst)], face).unwrap(), w.m.face_color(face));
}

#[test]
fn ids_are_deterministic_minted_in_order_and_never_reused() {
    let mut w = world();
    assert_eq!((w.pin.kind(), w.assembly.kind(), w.core.kind(), w.inst.kind()), (EntityKind::Prototype, EntityKind::Lattice, EntityKind::Lattice, EntityKind::Instance));
    assert_eq!((w.pin.serial(), w.assembly.serial(), w.core.serial(), w.inst.serial()), (1, 1, 2, 1));
    let next_inst = w.m.ids().peek(EntityKind::Instance);
    let f1 = flatten(&w.m).unwrap();
    let f2 = flatten(&w.m).unwrap();
    assert_eq!(f1, f2);
    assert_eq!(w.m.ids().peek(EntityKind::Instance), next_inst, "flattening mints no ids");
    // A hole does not shift anyone: paths name slots, not emission positions.
    let hole = SlotIx::new(1, 0, 0);
    w.m.set_lattice_slot(w.assembly, hole, Slot { fill: Fill::Void, recolor: ColorMap::default() }).unwrap();
    let f3 = flatten(&w.m).unwrap();
    assert_eq!(f3.paths.len(), f1.paths.len() - 9);
    let keep = path(&w, SlotIx::new(2, 2, 0), SlotIx::new(-1, 0, 0));
    assert_eq!(f3.entries[f3.find(&keep).unwrap()].iter().map(|e| (e.volume, e.color)).collect::<Vec<_>>(), f1.entries[f1.find(&keep).unwrap()].iter().map(|e| (e.volume, e.color)).collect::<Vec<_>>());
    assert!(f3.find(&path(&w, SlotIx::new(2, 2, 0), hole)).is_none());
    // A second instance never reuses id #1; its copies are distinct from the first's.
    let inst2 = w.m.add_instance(InstanceRow { of: Fill::Prototype(w.pin), at: Transform::IDENTITY, recolor: ColorMap::default() }).unwrap();
    assert_eq!(inst2.serial(), 2);
    let f4 = flatten(&w.m).unwrap();
    assert!(f4.find(&[PathStep::Instance(inst2)]).is_some());
    assert_eq!(f4.paths.len(), f3.paths.len() + 1);
}

#[test]
fn lattice_references_are_cycle_checked_and_validated() {
    let mut w = world();
    // The assembly may not contain the core, which contains the assembly.
    assert_eq!(w.m.set_lattice_slot(w.assembly, SlotIx::new(0, 1, 0), Slot { fill: Fill::Lattice(w.core), recolor: ColorMap::default() }), Err(ModelError::Cycle(w.assembly)));
    assert!(w.m.instancing().find_cycle().is_none(), "the failed edit was rolled back");
    assert!(w.m.set_lattice_slot(w.assembly, SlotIx::new(9, 9, 0), Slot { fill: Fill::Void, recolor: ColorMap::default() }).is_err());
    assert!(w.m.add_lattice(Lattice { layout: Layout::Hex { rings: 0, pitch: 1.0, orient: HexOrient::YRows, nz: 1, dz: 0.0 }, origin: Transform::IDENTITY, outer: Fill::Void, slots: BTreeMap::new() }).is_err());
    assert!(w.m.add_prototype(&[]).is_err());
    let ghost = EntityId::new(EntityKind::Prototype, 77).unwrap();
    assert!(w.m.add_instance(InstanceRow { of: Fill::Prototype(ghost), at: Transform::IDENTITY, recolor: ColorMap::default() }).is_err());
    // A clean model validates with no instancing complaints.
    let r = validate(&w.m, &w.t, Tolerance::default());
    assert!(!r.has(IssueCode::InstanceCycle) && !r.has(IssueCode::RecolorUnknownKey) && !r.has(IssueCode::RecolorDomainMismatch) && !r.has(IssueCode::UnknownEntity), "{:#?}", r.issues);
    let _ = (w.absorber_slots, w.clad);
}

#[test]
fn cartesian_layout_and_improper_instances() {
    let mut w = world();
    let lay = Layout::Cartesian { n: [2, 3, 2], pitch: Vector3::new(1.0, 2.0, 3.0) };
    let idx = lay.indices();
    assert_eq!(idx.len(), 12);
    assert_eq!(idx[1], SlotIx::new(1, 0, 0), "(k, j, i) order: i runs fastest");
    assert_eq!(idx[2], SlotIx::new(0, 1, 0));
    assert_eq!(idx[6], SlotIx::new(0, 0, 1));
    assert!(lay.contains(SlotIx::new(1, 2, 1)) && !lay.contains(SlotIx::new(2, 0, 0)) && !lay.contains(SlotIx::new(0, -1, 0)));
    assert_eq!(lay.offset(SlotIx::new(1, 2, 1)), Vector3::new(1.0, 4.0, 3.0));
    // A mirrored instance records the orientation flip.
    let mirror = Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap();
    let i = w.m.add_instance(InstanceRow { of: Fill::Prototype(w.pin), at: mirror, recolor: ColorMap::default() }).unwrap();
    let flat = flatten(&w.m).unwrap();
    let e = &flat.entries[flat.find(&[PathStep::Instance(i)]).unwrap()][0];
    assert!(flat.placement_of(e).orientation_reversed);
}
