//! The region split (ruling D9): one region per lattice position, ring and sector, as an export
//! option over the model's region tags.
//!
//! A cell's region key is its block name, then (for each requested part) the volume's slot in
//! every lattice frame ordered outer frame to inner frame, its ring and its sector. The key
//! reads as a name by one grammar ([`RegionName`]):
//!
//! ```text
//! <block> @<frame>[i,j] ... /r<ring> /s<sector>      fuel@core[2,3]@asm[5,7]/r1/s2
//! ```
//!
//! `[i,j,k]` is written when the frame's layout has more than one layer. [`plan_regions`] is the
//! pure planner (one pass over the cells); [`lower_split`](crate::lower_split) applies it to a
//! mesh and adds one `region/<name>` cell set per region.

use std::collections::BTreeMap;
use std::fmt;

use sm_mesh_api::CellIx;
use sm_model_api::{LatticeFrame, Layout, RegionTag};

/// The parts of a region key that an export may split on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RegionSplit {
    /// Split by lattice position (every frame the volume is tagged in).
    pub lattice: bool,
    /// Split by ring of a subdivided pin.
    pub ring: bool,
    /// Split by sector of a subdivided pin.
    pub sector: bool,
}

impl RegionSplit {
    /// The empty split: regions are not made (today's export).
    pub const NONE: Self = Self {
        lattice: false,
        ring: false,
        sector: false,
    };

    /// Build a split from part names: any of `"lattice"`, `"ring"`, `"sector"`.
    ///
    /// # Errors
    /// The unknown name.
    pub fn from_names<S: AsRef<str>>(names: &[S]) -> Result<Self, String> {
        let mut s = Self::NONE;
        for n in names {
            match n.as_ref() {
                "lattice" => s.lattice = true,
                "ring" => s.ring = true,
                "sector" => s.sector = true,
                other => return Err(other.to_string()),
            }
        }
        Ok(s)
    }

    /// True when no part is requested.
    pub fn is_empty(&self) -> bool {
        !(self.lattice || self.ring || self.sector)
    }
}

/// Errors of the region split. [`RegionError::code`] is the stable kebab-case code.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
#[non_exhaustive]
pub enum RegionError {
    /// The blocks do not cover the mesh (or there are none): the split has no block to name.
    #[error("a region split needs every cell in a block: {0}")]
    NeedBlocks(String),
    /// A block name contains a character the region grammar reserves.
    #[error(
        "block name {0:?} contains a reserved character (`@`, `/`, `[`, `]` or `,`); rename the block to split regions"
    )]
    NameReserved(String),
    /// Two different regions would get the same name (different ring or sector counts).
    #[error("two regions share the name {0:?} but differ in ring or sector count")]
    Ambiguous(String),
    /// The model has instances. Region tags key `TopoVolume`s, and a placed instance shares its
    /// prototype's volumes, so every placement would get the prototype's tag: refused, not guessed.
    #[error(
        "the model has instances: a region split cannot tell the placements of a prototype apart yet (export without `regions`)"
    )]
    Instances,
    /// A region name does not follow the grammar.
    #[error("bad region name {0:?}")]
    BadName(String),
}

impl RegionError {
    /// A stable kebab-case code.
    pub fn code(&self) -> &'static str {
        match self {
            Self::NeedBlocks(_) => "regions-need-blocks",
            Self::NameReserved(_) => "region-name-reserved",
            Self::Ambiguous(_) => "region-name-ambiguous",
            Self::Instances => "unsupported-instances",
            Self::BadName(_) => "region-name-bad",
        }
    }
}

/// One slot of a region name: the lattice frame and the indices.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameSlot {
    /// The frame name.
    pub frame: String,
    /// The slot `[i, j, k]` (`k` is 0 and not written for a single-layer frame).
    pub ix: [i32; 3],
    /// True when `k` is written (the frame has more than one layer).
    pub layered: bool,
}

/// A structured region name; [`format`](RegionName::format) and [`parse`](RegionName::parse)
/// are exact inverses.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RegionName {
    /// The block (material) name.
    pub block: String,
    /// Slots from the outer frame to the inner one.
    pub slots: Vec<FrameSlot>,
    /// Ring index (0 innermost).
    pub ring: Option<u32>,
    /// Sector index.
    pub sector: Option<u32>,
}

impl RegionName {
    /// A name with only a block.
    pub fn new(block: impl Into<String>) -> Self {
        Self {
            block: block.into(),
            slots: Vec::new(),
            ring: None,
            sector: None,
        }
    }

    /// The name as text: `<block>@<frame>[i,j]...[/r<ring>][/s<sector>]`.
    pub fn format(&self) -> String {
        self.to_string()
    }

    /// Read a name written by [`RegionName::format`].
    ///
    /// # Errors
    /// [`RegionError::BadName`] when `text` does not follow the grammar.
    pub fn parse(text: &str) -> Result<Self, RegionError> {
        let bad = || RegionError::BadName(text.to_string());
        let end = text.find(['@', '/']).unwrap_or(text.len());
        let block = &text[..end];
        if block.is_empty() || block.contains([']', '[', ',']) {
            return Err(bad());
        }
        let mut rest = &text[end..];
        let mut out = Self::new(block);
        while let Some(r) = rest.strip_prefix('@') {
            let open = r.find('[').ok_or_else(bad)?;
            let close = r.find(']').ok_or_else(bad)?;
            let frame = &r[..open];
            if frame.is_empty()
                || !frame
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b == b'_')
                || close < open
            {
                return Err(bad());
            }
            let nums: Result<Vec<i32>, _> = r[open + 1..close]
                .split(',')
                .map(str::parse::<i32>)
                .collect();
            let nums = nums.map_err(|_| bad())?;
            let (ix, layered) = match nums[..] {
                [i, j] => ([i, j, 0], false),
                [i, j, k] => ([i, j, k], true),
                _ => return Err(bad()),
            };
            out.slots.push(FrameSlot {
                frame: frame.to_string(),
                ix,
                layered,
            });
            rest = &r[close + 1..];
        }
        if let Some(r) = rest.strip_prefix("/r") {
            let n = r.find('/').unwrap_or(r.len());
            out.ring = Some(r[..n].parse().map_err(|_| bad())?);
            rest = &r[n..];
        }
        if let Some(r) = rest.strip_prefix("/s") {
            out.sector = Some(r.parse().map_err(|_| bad())?);
            rest = "";
        }
        if rest.is_empty() { Ok(out) } else { Err(bad()) }
    }
}

impl fmt::Display for RegionName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.block)?;
        for s in &self.slots {
            write!(f, "@{}[{},{}", s.frame, s.ix[0], s.ix[1])?;
            if s.layered {
                write!(f, ",{}", s.ix[2])?;
            }
            f.write_str("]")?;
        }
        if let Some(r) = self.ring {
            write!(f, "/r{r}")?;
        }
        if let Some(s) = self.sector {
            write!(f, "/s{s}")?;
        }
        Ok(())
    }
}

/// True when a block name has no character the grammar reserves.
pub fn block_name_ok(name: &str) -> bool {
    !name.is_empty() && !name.contains(['@', '/', '[', ']', ','])
}

/// One group of cells that share a block and a region tag (usually one volume).
#[derive(Clone, Copy, Debug)]
pub struct PlanItem<'a> {
    /// The block name.
    pub block: &'a str,
    /// The region tag of the volume, if it has one.
    pub tag: Option<&'a RegionTag>,
    /// The cells.
    pub cells: &'a [CellIx],
}

/// One planned region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlannedRegion {
    /// The structured name.
    pub name: RegionName,
    /// Integer tags for the `.smrt`: `lat.<frame>.i`, `.j`, `.k` (layered frames only), `ring`,
    /// `ring_count`, `sector`, `sector_count`, for the requested parts only.
    pub tags: Vec<(String, i64)>,
    /// The cells, ascending.
    pub cells: Vec<CellIx>,
}

/// The planned regions, in name order, with the work done.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionPlan {
    /// The regions, ordered by [`RegionName`] (block, slots, ring, sector).
    pub regions: Vec<PlannedRegion>,
    /// Number of [`PlanItem`]s visited.
    pub items: u64,
    /// Number of cells moved into regions (work counter: linear in the cells).
    pub cells: u64,
}

fn layered(l: &Layout) -> bool {
    match *l {
        Layout::Cartesian { n, .. } => n[2] > 1,
        Layout::Hex { nz, .. } => nz > 1,
        _ => false,
    }
}

/// Depth of `frame` in the parent chain (0 for a root frame).
fn depth(frames: &BTreeMap<String, LatticeFrame>, frame: &str) -> usize {
    let (mut d, mut cur) = (0, frames.get(frame).and_then(|f| f.parent.as_deref()));
    while let Some(p) = cur {
        d += 1;
        if d > frames.len() {
            break;
        }
        cur = frames.get(p).and_then(|f| f.parent.as_deref());
    }
    d
}

type Key = (RegionName, Option<u32>, Option<u32>);

/// Group `items` into regions by `split`. One pass over the items and one append per cell, so the
/// work is linear in the cells; the output order is the order of [`RegionName`] and does not
/// depend on the order of `items`.
///
/// # Errors
/// [`RegionError::NameReserved`] for a block name the grammar reserves;
/// [`RegionError::Ambiguous`] when one name stands for two different ring or sector counts.
pub fn plan_regions<'a>(
    split: &RegionSplit,
    frames: &BTreeMap<String, LatticeFrame>,
    items: impl IntoIterator<Item = PlanItem<'a>>,
) -> Result<RegionPlan, RegionError> {
    let mut groups: BTreeMap<Key, Vec<CellIx>> = BTreeMap::new();
    let (mut n_items, mut n_cells) = (0u64, 0u64);
    let mut depths: BTreeMap<&str, usize> = BTreeMap::new();
    for it in items {
        n_items += 1;
        if !block_name_ok(it.block) {
            return Err(RegionError::NameReserved(it.block.to_string()));
        }
        let mut name = RegionName::new(it.block);
        let (mut ring_count, mut sector_count) = (None, None);
        if let Some(tag) = it.tag {
            if split.lattice {
                let mut slots: Vec<(usize, &String, FrameSlot)> = tag
                    .slots
                    .iter()
                    .map(|(f, ix)| {
                        let d = *depths.entry(f.as_str()).or_insert_with(|| depth(frames, f));
                        let l = frames.get(f).is_some_and(|fr| layered(&fr.layout));
                        (
                            d,
                            f,
                            FrameSlot {
                                frame: f.clone(),
                                ix: [ix.i, ix.j, if l { ix.k } else { 0 }],
                                layered: l,
                            },
                        )
                    })
                    .collect();
                slots.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
                name.slots = slots.into_iter().map(|s| s.2).collect();
            }
            if split.ring
                && let Some(r) = tag.ring
            {
                name.ring = Some(r.index);
                ring_count = Some(r.count);
            }
            if split.sector
                && let Some(s) = tag.sector
            {
                name.sector = Some(s.index);
                sector_count = Some(s.count);
            }
        }
        n_cells += it.cells.len() as u64;
        groups
            .entry((name, ring_count, sector_count))
            .or_default()
            .extend_from_slice(it.cells);
    }
    let mut regions: Vec<PlannedRegion> = Vec::with_capacity(groups.len());
    for ((name, ring_count, sector_count), mut cells) in groups {
        if let Some(prev) = regions.last()
            && prev.name == name
        {
            return Err(RegionError::Ambiguous(name.format()));
        }
        cells.sort_unstable();
        cells.dedup();
        let mut tags: Vec<(String, i64)> = Vec::new();
        for s in &name.slots {
            tags.push((format!("lat.{}.i", s.frame), i64::from(s.ix[0])));
            tags.push((format!("lat.{}.j", s.frame), i64::from(s.ix[1])));
            if s.layered {
                tags.push((format!("lat.{}.k", s.frame), i64::from(s.ix[2])));
            }
        }
        if let (Some(r), Some(c)) = (name.ring, ring_count) {
            tags.push(("ring".into(), i64::from(r)));
            tags.push(("ring_count".into(), i64::from(c)));
        }
        if let (Some(s), Some(c)) = (name.sector, sector_count) {
            tags.push(("sector".into(), i64::from(s)));
            tags.push(("sector_count".into(), i64::from(c)));
        }
        regions.push(PlannedRegion { name, tags, cells });
    }
    Ok(RegionPlan {
        regions,
        items: n_items,
        cells: n_cells,
    })
}

/// One region of a lowered mesh.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RegionSummary {
    /// The region name (text of the [`RegionName`], without the `region/` prefix).
    pub name: String,
    /// The cell set that holds the region (`region/<name>`).
    pub set: String,
    /// Integer tags (see [`PlannedRegion::tags`]).
    pub tags: Vec<(String, i64)>,
    /// Number of cells.
    pub len: usize,
}

/// Prefix of the cell set of every region.
pub const REGION_SET_PREFIX: &str = "region/";

#[cfg(test)]
mod tests {
    use super::*;
    use sm_model_api::{SlotIx, SubIndex};

    fn name(
        block: &str,
        slots: &[(&str, [i32; 3], bool)],
        ring: Option<u32>,
        sector: Option<u32>,
    ) -> RegionName {
        RegionName {
            block: block.into(),
            slots: slots
                .iter()
                .map(|(f, ix, l)| FrameSlot {
                    frame: (*f).into(),
                    ix: *ix,
                    layered: *l,
                })
                .collect(),
            ring,
            sector,
        }
    }

    #[test]
    fn grammar_round_trips_and_matches_the_documented_example() {
        let n = name(
            "fuel",
            &[("core", [2, 3, 0], false), ("asm", [5, 7, 0], false)],
            Some(1),
            Some(2),
        );
        assert_eq!(n.format(), "fuel@core[2,3]@asm[5,7]/r1/s2");
        for x in [
            n,
            name("water", &[], None, None),
            name("u_235", &[("a", [-1, 0, 4], true)], None, Some(3)),
            name("m", &[("h", [-2, 2, 0], false)], Some(0), None),
            name("x", &[], Some(10), Some(11)),
        ] {
            assert_eq!(RegionName::parse(&x.format()), Ok(x.clone()), "{x}");
        }
        for bad in [
            "",
            "@a[1,2]",
            "f@a[1]",
            "f@a[1,2,3,4]",
            "f/s1/r1",
            "f@a[1,2]x",
            "f/rx",
            "f@[1,2]",
            "f@a[1,2",
        ] {
            assert!(RegionName::parse(bad).is_err(), "{bad}");
        }
    }

    fn tag(
        slots: &[(&str, i32, i32)],
        ring: Option<(u32, u32)>,
        sector: Option<(u32, u32)>,
    ) -> RegionTag {
        RegionTag::new(
            slots
                .iter()
                .map(|(f, i, j)| ((*f).to_string(), SlotIx::new(*i, *j, 0)))
                .collect(),
            ring.map(|(i, c)| SubIndex::new(i, c)),
            sector.map(|(i, c)| SubIndex::new(i, c)),
        )
    }

    #[test]
    fn planning_is_independent_of_item_order_and_respects_the_split() {
        let tags: Vec<RegionTag> = (0..2)
            .flat_map(|i| (0..3).map(move |r| tag(&[("a", i, 0)], Some((r, 3)), Some((0, 1)))))
            .collect();
        let cells: Vec<[CellIx; 1]> = (0..6).map(|k| [CellIx(k)]).collect();
        let items = |rev: bool| {
            let mut v: Vec<PlanItem<'_>> = tags
                .iter()
                .zip(&cells)
                .map(|(t, c)| PlanItem {
                    block: "fuel",
                    tag: Some(t),
                    cells: c,
                })
                .collect();
            if rev {
                v.reverse();
            }
            v
        };
        let frames = BTreeMap::new();
        let all = RegionSplit {
            lattice: true,
            ring: true,
            sector: true,
        };
        let a = plan_regions(&all, &frames, items(false)).expect("plan");
        assert_eq!(a, plan_regions(&all, &frames, items(true)).expect("plan"));
        assert_eq!(a.regions.len(), 6);
        assert_eq!(
            a.regions[0].tags,
            [
                ("lat.a.i", 0),
                ("lat.a.j", 0),
                ("ring", 0),
                ("ring_count", 3),
                ("sector", 0),
                ("sector_count", 1)
            ]
            .map(|(k, v)| (k.to_string(), v))
        );
        let ring_only = plan_regions(
            &RegionSplit {
                ring: true,
                ..RegionSplit::NONE
            },
            &frames,
            items(false),
        )
        .expect("plan");
        assert_eq!(
            ring_only
                .regions
                .iter()
                .map(|r| r.name.format())
                .collect::<Vec<_>>(),
            ["fuel/r0", "fuel/r1", "fuel/r2"]
        );
        assert_eq!(ring_only.regions[1].cells, [CellIx(1), CellIx(4)]);
        let lat_only = plan_regions(
            &RegionSplit {
                lattice: true,
                ..RegionSplit::NONE
            },
            &frames,
            items(false),
        )
        .expect("plan");
        assert_eq!(
            lat_only
                .regions
                .iter()
                .map(|r| r.name.format())
                .collect::<Vec<_>>(),
            ["fuel@a[0,0]", "fuel@a[1,0]"]
        );
        assert!(
            lat_only.regions[0]
                .tags
                .iter()
                .all(|(k, _)| k.starts_with("lat."))
        );
    }

    #[test]
    fn reserved_block_names_and_count_clashes_are_refused() {
        let frames = BTreeMap::new();
        let cell = [CellIx(0)];
        for b in ["u@235", "a/b", "a[1]", "a,b", ""] {
            let e = plan_regions(
                &RegionSplit {
                    ring: true,
                    ..RegionSplit::NONE
                },
                &frames,
                [PlanItem {
                    block: b,
                    tag: None,
                    cells: &cell,
                }],
            )
            .expect_err("reserved");
            assert_eq!(e.code(), "region-name-reserved", "{b}");
        }
        let (t3, t4) = (tag(&[], Some((0, 3)), None), tag(&[], Some((0, 4)), None));
        let e = plan_regions(
            &RegionSplit {
                ring: true,
                ..RegionSplit::NONE
            },
            &frames,
            [
                PlanItem {
                    block: "f",
                    tag: Some(&t3),
                    cells: &cell,
                },
                PlanItem {
                    block: "f",
                    tag: Some(&t4),
                    cells: &cell,
                },
            ],
        )
        .expect_err("clash");
        assert_eq!(e.code(), "region-name-ambiguous");
    }
}
