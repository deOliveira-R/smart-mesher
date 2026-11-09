//! Lattice frames and region tags: provenance of "which lattice position, pin ring and sector a
//! volume came from" (ruling D9), as two attribute tables in the [`Model`].
//!
//! A [`LatticeFrame`] is a named lattice geometry (a [`Layout`] at an origin, optionally nested
//! in a parent frame). It is provenance only: unrelated to [`crate::Lattice`] instancing, no
//! prototype, no flatten. A [`RegionTag`] says, for one `TopoVolume`, which slot it occupies in
//! each frame and which ring and sector of a subdivided pin it is. Both tables stay empty (and
//! out of the serde shape and the canonical hash) until something writes them.

use crate::error::{ModelError, Result, expect_kind};
use crate::instancing::{HexOrient, Layout, SlotIx};
use crate::model::Model;
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::BTreeMap;

/// A named lattice geometry used only as provenance. Slot `(0, 0, 0)` is centred at `origin`,
/// as in [`Layout::offset`].
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct LatticeFrame {
    /// Slot geometry (Cartesian or Hex).
    pub layout: Layout,
    /// Centre of slot `(0, 0, 0)`.
    pub origin: Point3,
    /// The enclosing frame (core > assembly > pin), by name. Must exist and not form a cycle.
    pub parent: Option<String>,
}

impl LatticeFrame {
    /// Construct a frame.
    pub fn new(layout: Layout, origin: Point3, parent: Option<String>) -> Self {
        Self { layout, origin, parent }
    }

    /// The slot whose centre is nearest to `p`, or `None` when that slot is outside
    /// [`Layout::contains`] (or `p` or the pitch is not finite and positive where it is needed).
    ///
    /// Cartesian: each axis rounds `(p - origin) / pitch` to the nearest integer. Hex: axial
    /// coordinates of `(p - origin) / pitch` are cube-rounded (both [`HexOrient`] values).
    ///
    /// **Tie rule.** A point on the boundary between two slots belongs to the lower index.
    /// Cartesian: per axis, a quotient within `1e-12` (relative, floor 1) of `n + 0.5` gives `n`.
    /// Hex: the cube-rounding fix-up changes the coordinate with the largest rounding error and,
    /// on equal errors, prefers changing `q`, then `r` (the `s = -q - r` coordinate is
    /// recomputed last). The layer index `k` follows the Cartesian per-axis rule with the
    /// z pitch (`dz`). An axis of one slot with a zero pitch always gives index 0.
    pub fn locate(&self, p: Point3) -> Option<SlotIx> {
        let d = [p.x - self.origin.x, p.y - self.origin.y, p.z - self.origin.z];
        if d.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let ix = match self.layout {
            Layout::Cartesian { n, pitch } => SlotIx::new(axis_index(d[0], pitch.x, n[0])?, axis_index(d[1], pitch.y, n[1])?, axis_index(d[2], pitch.z, n[2])?),
            Layout::Hex { pitch, orient, nz, dz, .. } => {
                if !(pitch.is_finite() && pitch > 0.0) {
                    return None;
                }
                let h = 3f64.sqrt() / 2.0;
                let (x, y) = (d[0] / pitch, d[1] / pitch);
                let (q, r) = match orient {
                    HexOrient::XRows => {
                        let r = y / h;
                        (x - r / 2.0, r)
                    }
                    HexOrient::YRows => {
                        let q = x / h;
                        (q, y - q / 2.0)
                    }
                };
                let (qi, ri) = cube_round(q, r)?;
                SlotIx::new(qi, ri, axis_index(d[2], dz, nz)?)
            }
        };
        self.layout.contains(ix).then_some(ix)
    }
}

/// Nearest index along one axis; ties go to the lower index. `n` is the slot count on the axis.
fn axis_index(d: f64, pitch: f64, n: u32) -> Option<i32> {
    if pitch == 0.0 {
        return (n == 1).then_some(0);
    }
    if !pitch.is_finite() {
        return None;
    }
    let x = d / pitch;
    if !x.is_finite() || x.abs() > f64::from(i32::MAX / 2) {
        return None;
    }
    let f = x.floor();
    let frac = x - f;
    let tol = 1e-12 * x.abs().max(1.0);
    // Round half down: a fraction of 0.5 (within tol) keeps the floor.
    let up = frac > 0.5 + tol;
    Some(f as i32 + i32::from(up))
}

/// Cube rounding of axial `(q, r)`; see [`LatticeFrame::locate`] for the tie rule.
fn cube_round(q: f64, r: f64) -> Option<(i32, i32)> {
    let s = -q - r;
    if ![q, r, s].iter().all(|v| v.is_finite() && v.abs() < f64::from(i32::MAX / 4)) {
        return None;
    }
    let (mut rq, mut rr, rs) = (q.round(), r.round(), s.round());
    let (dq, dr, ds) = ((rq - q).abs(), (rr - r).abs(), (rs - s).abs());
    if dq >= dr && dq >= ds {
        rq = -rr - rs;
    } else if dr >= ds {
        rr = -rq - rs;
    }
    Some((rq as i32, rr as i32))
}

/// One index of a subdivision: piece `index` of `count` (0 innermost ring, or the first sector).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct SubIndex {
    /// Zero-based index, less than `count`.
    pub index: u32,
    /// Number of pieces of the subdivision.
    pub count: u32,
}

impl SubIndex {
    /// Construct a sub-index (`count > index` is checked when it is stored).
    pub fn new(index: u32, count: u32) -> Self {
        Self { index, count }
    }
}

/// Where a volume came from: its slot in each lattice frame (by frame name), and its ring and
/// sector when it is a piece of a subdivided pin.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct RegionTag {
    /// Slot per frame name. Every name must be a defined frame and the slot inside its layout.
    pub slots: BTreeMap<String, SlotIx>,
    /// Radial ring of a subdivided pin (0 innermost).
    pub ring: Option<SubIndex>,
    /// Azimuthal sector of a subdivided pin.
    pub sector: Option<SubIndex>,
}

impl RegionTag {
    /// Construct a tag.
    pub fn new(slots: BTreeMap<String, SlotIx>, ring: Option<SubIndex>, sector: Option<SubIndex>) -> Self {
        Self { slots, ring, sector }
    }
}

/// True for a non-empty `[A-Za-z0-9_]+` frame name.
pub(crate) fn valid_frame_name(s: &str) -> bool {
    !s.is_empty() && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
}

impl Model {
    /// Define a lattice frame. Errors: [`ModelError::BadFrameName`] (name not `[A-Za-z0-9_]+`),
    /// [`ModelError::NameTaken`], [`ModelError::UnknownFrame`] (the parent is not defined yet),
    /// or an invalid or non-finite layout or origin ([`ModelError::Invalid`]).
    pub fn define_lattice_frame(&mut self, name: &str, frame: LatticeFrame) -> Result<()> {
        if !valid_frame_name(name) {
            return Err(ModelError::BadFrameName(name.to_string()));
        }
        if self.lattice_frames.contains_key(name) {
            return Err(ModelError::NameTaken(name.to_string()));
        }
        frame.layout.check()?;
        if ![frame.origin.x, frame.origin.y, frame.origin.z].iter().all(|v| v.is_finite()) {
            return Err(ModelError::Invalid("lattice frame origin must be finite".into()));
        }
        if let Some(p) = &frame.parent
            && !self.lattice_frames.contains_key(p)
        {
            return Err(ModelError::UnknownFrame(p.clone()));
        }
        self.lattice_frames.insert(name.to_string(), frame);
        Ok(())
    }

    /// The frame called `name`.
    pub fn lattice_frame(&self, name: &str) -> Option<&LatticeFrame> {
        self.lattice_frames.get(name)
    }

    /// All frames, by name.
    pub fn lattice_frames(&self) -> &BTreeMap<String, LatticeFrame> {
        &self.lattice_frames
    }

    /// Set (replace) the region tag of a `TopoVolume`. Errors: [`ModelError::WrongKind`],
    /// [`ModelError::UnknownFrame`], [`ModelError::SlotOutsideLayout`], [`ModelError::BadSubIndex`].
    /// The volume itself is not looked up (the model never stores topology).
    pub fn set_region_tag(&mut self, volume: EntityId, tag: RegionTag) -> Result<()> {
        expect_kind(volume, EntityKind::TopoVolume)?;
        self.check_region_tag(&tag)?;
        self.region_tags.insert(volume, tag);
        Ok(())
    }

    /// Remove the tag of `volume`, returning it. An untagged volume is ignored.
    pub fn clear_region_tag(&mut self, volume: EntityId) -> Option<RegionTag> {
        self.region_tags.remove(&volume)
    }

    /// The tag of `volume`.
    pub fn region_tag(&self, volume: EntityId) -> Option<&RegionTag> {
        self.region_tags.get(&volume)
    }

    /// All tags, by volume id.
    pub fn region_tags(&self) -> &BTreeMap<EntityId, RegionTag> {
        &self.region_tags
    }

    fn check_region_tag(&self, tag: &RegionTag) -> Result<()> {
        for (name, ix) in &tag.slots {
            let f = self.lattice_frames.get(name).ok_or_else(|| ModelError::UnknownFrame(name.clone()))?;
            if !f.layout.contains(*ix) {
                return Err(ModelError::SlotOutsideLayout { frame: name.clone(), ix: *ix });
            }
        }
        for s in [tag.ring, tag.sector].into_iter().flatten() {
            if s.count <= s.index {
                return Err(ModelError::BadSubIndex { index: s.index, count: s.count });
            }
        }
        Ok(())
    }

    /// The part of [`Model::check_integrity`] for the two region tables.
    pub(crate) fn check_regions(&self) -> Result<()> {
        for (name, f) in &self.lattice_frames {
            if !valid_frame_name(name) {
                return Err(ModelError::BadFrameName(name.clone()));
            }
            f.layout.check()?;
            if ![f.origin.x, f.origin.y, f.origin.z].iter().all(|v| v.is_finite()) {
                return Err(ModelError::Invalid("lattice frame origin must be finite".into()));
            }
        }
        for (name, f) in &self.lattice_frames {
            let mut cur = f.parent.as_deref();
            let mut steps = 0usize;
            while let Some(p) = cur {
                let pf = self.lattice_frames.get(p).ok_or_else(|| ModelError::UnknownFrame(p.to_string()))?;
                steps += 1;
                if p == name || steps > self.lattice_frames.len() {
                    return Err(ModelError::FrameCycle(name.clone()));
                }
                cur = pf.parent.as_deref();
            }
        }
        for (v, tag) in &self.region_tags {
            expect_kind(*v, EntityKind::TopoVolume)?;
            self.check_region_tag(tag)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cart() -> LatticeFrame {
        LatticeFrame::new(Layout::Cartesian { n: [3, 3, 1], pitch: sm_core::Vector3::new(1.26, 1.26, 1.0) }, Point3::new(0.0, 0.0, 0.0), None)
    }
    fn vol(n: u64) -> EntityId {
        EntityId::new(EntityKind::TopoVolume, n).unwrap()
    }
    fn tag(frame: &str, ix: SlotIx) -> RegionTag {
        RegionTag::new([(frame.to_string(), ix)].into(), None, None)
    }

    #[test]
    fn bad_frame_name_is_refused() {
        for n in ["", "a b", "a@b", "a/b", "a[0]", "é"] {
            assert_eq!(Model::new().define_lattice_frame(n, cart()), Err(ModelError::BadFrameName(n.into())));
        }
        let mut m = Model::new();
        m.define_lattice_frame("core_1", cart()).unwrap();
        assert_eq!(m.define_lattice_frame("core_1", cart()), Err(ModelError::NameTaken("core_1".into())));
        let mut bad = Model::new();
        bad.lattice_frames.insert("a b".into(), cart());
        assert_eq!(bad.check_integrity(), Err(ModelError::BadFrameName("a b".into())));
    }

    #[test]
    fn unknown_frame_and_parent_are_refused() {
        let mut m = Model::new();
        let child = LatticeFrame::new(cart().layout, Point3::new(0.0, 0.0, 0.0), Some("nope".into()));
        assert_eq!(m.define_lattice_frame("c", child.clone()), Err(ModelError::UnknownFrame("nope".into())));
        assert_eq!(m.set_region_tag(vol(1), tag("nope", SlotIx::new(0, 0, 0))), Err(ModelError::UnknownFrame("nope".into())));
        let mut bad = Model::new();
        bad.lattice_frames.insert("c".into(), child);
        assert_eq!(bad.check_integrity(), Err(ModelError::UnknownFrame("nope".into())));
        let mut bad = Model::new();
        bad.region_tags.insert(vol(1), tag("nope", SlotIx::new(0, 0, 0)));
        assert_eq!(bad.check_integrity(), Err(ModelError::UnknownFrame("nope".into())));
    }

    #[test]
    fn parent_cycle_is_refused() {
        for names in [&["a"][..], &["a", "b"], &["a", "b", "c"]] {
            let mut bad = Model::new();
            for (i, n) in names.iter().enumerate() {
                let parent = names[(i + 1) % names.len()];
                bad.lattice_frames.insert((*n).into(), LatticeFrame::new(cart().layout, Point3::new(0.0, 0.0, 0.0), Some(parent.into())));
            }
            assert!(matches!(bad.check_integrity(), Err(ModelError::FrameCycle(_))), "{names:?}");
        }
        // A tail leading into a cycle is refused too.
        let mut bad = Model::new();
        for (n, p) in [("t", "a"), ("a", "b"), ("b", "a")] {
            bad.lattice_frames.insert(n.into(), LatticeFrame::new(cart().layout, Point3::new(0.0, 0.0, 0.0), Some(p.into())));
        }
        assert!(matches!(bad.check_integrity(), Err(ModelError::FrameCycle(_))));
        // A proper chain is fine.
        let mut ok = Model::new();
        ok.define_lattice_frame("core", cart()).unwrap();
        ok.define_lattice_frame("asm", LatticeFrame::new(cart().layout, Point3::new(0.0, 0.0, 0.0), Some("core".into()))).unwrap();
        ok.check_integrity().unwrap();
    }

    #[test]
    fn slot_outside_layout_and_bad_sub_index_are_refused() {
        let mut m = Model::new();
        m.define_lattice_frame("a", cart()).unwrap();
        for ix in [SlotIx::new(3, 0, 0), SlotIx::new(-1, 0, 0), SlotIx::new(0, 0, 1)] {
            let e = ModelError::SlotOutsideLayout { frame: "a".into(), ix };
            assert_eq!(m.set_region_tag(vol(1), tag("a", ix)), Err(e.clone()));
            let mut bad = m.clone();
            bad.region_tags.insert(vol(1), tag("a", ix));
            assert_eq!(bad.check_integrity(), Err(e));
        }
        for (i, c) in [(3, 3), (4, 3), (0, 0)] {
            let t = RegionTag::new(BTreeMap::new(), Some(SubIndex::new(i, c)), None);
            let e = ModelError::BadSubIndex { index: i, count: c };
            assert_eq!(m.set_region_tag(vol(1), t.clone()), Err(e.clone()));
            let mut bad = m.clone();
            bad.region_tags.insert(vol(1), t);
            assert_eq!(bad.check_integrity(), Err(e.clone()));
            let t = RegionTag::new(BTreeMap::new(), None, Some(SubIndex::new(i, c)));
            assert_eq!(m.set_region_tag(vol(1), t), Err(e));
        }
        m.set_region_tag(vol(1), RegionTag::new([("a".to_string(), SlotIx::new(2, 2, 0))].into(), Some(SubIndex::new(2, 3)), Some(SubIndex::new(0, 4)))).unwrap();
        m.check_integrity().unwrap();
    }

    #[test]
    fn tags_key_only_volumes() {
        let mut m = Model::new();
        let face = EntityId::new(EntityKind::TopoFace, 1).unwrap();
        assert!(matches!(m.set_region_tag(face, RegionTag::default()), Err(ModelError::WrongKind { .. })));
        let mut bad = Model::new();
        bad.region_tags.insert(face, RegionTag::default());
        assert!(matches!(bad.check_integrity(), Err(ModelError::WrongKind { .. })));
        m.set_region_tag(vol(1), RegionTag::default()).unwrap();
        assert!(m.region_tag(vol(1)).is_some());
        assert_eq!(m.clear_region_tag(vol(1)), Some(RegionTag::default()));
        assert_eq!(m.clear_region_tag(vol(1)), None);
        assert!(m.region_tags().is_empty());
    }
}
