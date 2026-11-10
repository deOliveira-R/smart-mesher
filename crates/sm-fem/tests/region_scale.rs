//! W4-REGIONS-SPLIT acceptance 6: the split scales (17 x 17 pins x 10 rings x 8 sectors), and the
//! planner's work counter is linear in the cells (no wall clock).

use std::collections::{BTreeMap, BTreeSet};

use sm_core::{Point3, Vector3};
use sm_fem::region::{PlanItem, RegionName, RegionSplit, plan_regions};
use sm_mesh_api::CellIx;
use sm_model_api::{LatticeFrame, Layout, RegionTag, SlotIx, SubIndex};

const N: i32 = 17;
const RINGS: u32 = 10;
const SECTORS: u32 = 8;

fn frames() -> BTreeMap<String, LatticeFrame> {
    let layout = Layout::Cartesian {
        n: [N as u32, N as u32, 1],
        pitch: Vector3::new(1.26, 1.26, 1.0),
    };
    BTreeMap::from([(
        "a".to_string(),
        LatticeFrame::new(layout, Point3::new(0.0, 0.0, 0.0), None),
    )])
}

/// One tag per fuel piece and one per water cell: `fuel_pieces` items per region.
fn tags() -> (Vec<RegionTag>, Vec<RegionTag>) {
    let mut fuel = Vec::new();
    let mut water = Vec::new();
    for j in 0..N {
        for i in 0..N {
            let slots: BTreeMap<String, SlotIx> = [("a".to_string(), SlotIx::new(i, j, 0))].into();
            water.push(RegionTag::new(slots.clone(), None, None));
            for r in 0..RINGS {
                for s in 0..SECTORS {
                    fuel.push(RegionTag::new(
                        slots.clone(),
                        Some(SubIndex::new(r, RINGS)),
                        Some(SubIndex::new(s, SECTORS)),
                    ));
                }
            }
        }
    }
    (fuel, water)
}

fn plan(cells_per_item: usize) -> sm_fem::region::RegionPlan {
    let (fuel, water) = tags();
    let total = (fuel.len() + water.len()) * cells_per_item;
    let cells: Vec<CellIx> = (0..total as u32).map(CellIx).collect();
    let mut items = Vec::new();
    let mut at = 0;
    for (block, tags) in [("fuel", &fuel), ("water", &water)] {
        for t in tags {
            items.push(PlanItem {
                block,
                tag: Some(t),
                cells: &cells[at..at + cells_per_item],
            });
            at += cells_per_item;
        }
    }
    plan_regions(
        &RegionSplit {
            lattice: true,
            ring: true,
            sector: true,
        },
        &frames(),
        items,
    )
    .expect("plan")
}

#[test]
fn a_17x17_assembly_with_10_rings_and_8_sectors_has_23409_unique_regions() {
    let p = plan(1);
    let fuel = (N * N) as u32 * RINGS * SECTORS;
    assert_eq!(fuel, 23_120);
    assert_eq!(p.regions.len() as u32, fuel + (N * N) as u32);
    let names: BTreeSet<String> = p.regions.iter().map(|r| r.name.format()).collect();
    assert_eq!(names.len(), p.regions.len(), "names are unique");
    // Every region has the cells of exactly one item and the grammar round-trips.
    assert!(p.regions.iter().all(|r| r.cells.len() == 1));
    assert!(
        p.regions
            .iter()
            .all(|r| RegionName::parse(&r.name.format()).as_ref() == Ok(&r.name))
    );
    let first = p
        .regions
        .iter()
        .find(|r| r.name.format() == "fuel@a[16,0]/r9/s7")
        .expect("a region");
    assert_eq!(first.tags.len(), 6);
}

#[test]
fn the_work_counter_is_linear_in_the_cells() {
    let (a, b) = (plan(1), plan(3));
    assert_eq!(a.items, b.items);
    assert_eq!(a.cells, a.items, "one cell per item");
    assert_eq!(
        b.cells,
        3 * a.cells,
        "three cells per item cost three cell moves, nothing more"
    );
    assert_eq!(a.regions.len(), b.regions.len());
    assert!(b.regions.iter().all(|r| r.cells.len() == 3));
}
