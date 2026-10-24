//! Differential-testing support: build the same history on the reference `CellComplex` and on
//! `TopoStore`, and demand identical results and identical observable state.
#![allow(dead_code)]

use sm_core::EntityId;
use sm_topo::TopoStore;
use sm_topo_api::{CellComplex, CoedgeKey, Dim, Side, TopologyEdit, TopologyView, TopologyViewExt, validate, validate_geometry};
use std::fmt::Debug;

/// Both implementations, driven in lock step.
pub struct Twin {
    pub a: CellComplex,
    pub b: TopoStore,
}

impl Twin {
    pub fn new() -> Self {
        let mut b = TopoStore::new();
        b.set_paranoid(true);
        // Small thresholds so the positional and incremental paths run on small models.
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::new(), b }
    }

    pub fn with_tolerance(tol: sm_geom_api::ModelTolerance) -> Self {
        let mut b = TopoStore::with_tolerance(tol);
        b.set_paranoid(true);
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::with_tolerance(tol), b }
    }

    /// Run `f` on both and demand equal results (errors included).
    pub fn run<R: PartialEq + Debug>(&mut self, ctx: &str, f: impl Fn(&mut dyn TopologyEdit) -> R) -> R {
        let ra = f(&mut self.a);
        let rb = f(&mut self.b);
        assert_eq!(ra, rb, "{ctx}: results differ (reference vs sm-topo)");
        ra
    }

    /// Full state comparison.
    pub fn same(&self, ctx: &str) {
        same_state(&self.a, &self.b, ctx);
    }

    /// `begin_op`, `f`, `end_op` on both (abort if `f` fails), comparing every result and the
    /// final state. Returns `f`'s result.
    pub fn op<R: PartialEq + Debug>(&mut self, ctx: &str, op: u64, f: impl Fn(&mut dyn TopologyEdit) -> Result<R, sm_topo_api::TopoError>) -> Result<R, sm_topo_api::TopoError> {
        self.run(ctx, |t| t.begin_op(sm_topo_api::OpId(op))).expect("begin_op");
        let r = self.run(ctx, &f);
        if r.is_ok() {
            let end = self.run(ctx, |t| t.end_op());
            if let Err(e) = end {
                self.same(&format!("{ctx} (after failed end_op: {e})"));
                return Err(e);
            }
        } else {
            self.run(ctx, |t| {
                t.abort_op();
            });
        }
        self.same(ctx);
        r
    }
}

fn words(v: &dyn TopologyView) -> Vec<sm_topo_api::HashWord> {
    let mut out = Vec::new();
    v.canonical_visit(&mut |w| out.push(w));
    out
}

pub fn same_state(a: &dyn TopologyView, b: &dyn TopologyView, ctx: &str) {
    assert_eq!(a.canonical_hash(), b.canonical_hash(), "{ctx}: canonical hash");
    let (wa, wb) = (words(a), words(b));
    assert_eq!(wa.len(), wb.len(), "{ctx}: canonical word count");
    if let Some(i) = (0..wa.len()).find(|&i| wa[i] != wb[i]) {
        panic!("{ctx}: canonical words differ at {i}: {:?} vs {:?}", wa[i], wb[i]);
    }
    assert_eq!(a.revision(), b.revision(), "{ctx}: revision");
    assert_eq!(a.tolerance(), b.tolerance(), "{ctx}: tolerance");
    assert_eq!(a.shapes().curve_count(), b.shapes().curve_count(), "{ctx}: curve count");
    assert_eq!(a.shapes().surface_count(), b.shapes().surface_count(), "{ctx}: surface count");
    for d in Dim::ALL {
        assert_eq!(a.cells(d), b.cells(d), "{ctx}: live {d:?}");
        let mut n = 1u64;
        loop {
            let id = EntityId::new(d.kind(), n).unwrap();
            let (sa, sb) = (a.state(id), b.state(id));
            assert_eq!(sa, sb, "{ctx}: state of {id:?}");
            if sa.is_none() {
                break;
            }
            assert_eq!(a.dim_of(id), b.dim_of(id), "{ctx}: dim_of {id:?}");
            assert_eq!(a.born(id), b.born(id), "{ctx}: born {id:?}");
            assert_eq!(a.flags(id), b.flags(id), "{ctx}: flags {id:?}");
            assert_eq!(a.carrier(id), b.carrier(id), "{ctx}: carrier {id:?}");
            assert_eq!(a.boundary(id), b.boundary(id), "{ctx}: boundary {id:?}");
            assert_eq!(a.coboundary(id), b.coboundary(id), "{ctx}: coboundary {id:?}");
            assert_eq!(a.tolerance_override(id), b.tolerance_override(id), "{ctx}: override {id:?}");
            assert_eq!(a.radial_uses(id), b.radial_uses(id), "{ctx}: radial {id:?}");
            assert_eq!(a.volume_faces(id), b.volume_faces(id), "{ctx}: volume_faces {id:?}");
            assert_eq!(a.face_edges(id), b.face_edges(id), "{ctx}: face_edges {id:?}");
            assert_eq!(a.coincident_faces(id), b.coincident_faces(id), "{ctx}: coincident_faces {id:?}");
            for ring in 0..6 {
                assert_eq!(a.shell_kind(id, ring), b.shell_kind(id, ring), "{ctx}: shell kind {id:?}/{ring}");
            }
            if d == Dim::Face {
                for e in a.boundary(id) {
                    for side in [Side::Left, Side::Right] {
                        let k = CoedgeKey { face: id, edge: e.cell, side };
                        assert_eq!(a.pcurve(k), b.pcurve(k), "{ctx}: pcurve {k:?}");
                    }
                }
                assert_eq!(a.face_ordinal(id), b.face_ordinal(id), "{ctx}: face_ordinal {id:?}");
                assert_eq!(a.face_sides(id), b.face_sides(id), "{ctx}: face_sides {id:?}");
            }
            assert_eq!(a.live_cover(id), b.live_cover(id), "{ctx}: live_cover {id:?}");
            n += 1;
        }
    }
    assert_eq!(a.coincidences(), b.coincidences(), "{ctx}: coincidences");
    assert_eq!(a.lineage().records(), b.lineage().records(), "{ctx}: lineage records");
    assert_eq!(a.lineage(), b.lineage(), "{ctx}: lineage indexes");
    assert_eq!(validate(a), validate(b), "{ctx}: validate");
    assert_eq!(validate_geometry(a), validate_geometry(b), "{ctx}: validate_geometry");
    // The derived queries are provided methods on the trait, so they agree if the rows do; spot-check
    // the geometric ones that read many fields.
    for &v in a.cells(Dim::Volume) {
        let (ma, mb) = (a.volume_moments(v), b.volume_moments(v));
        assert_eq!(format!("{ma:?}"), format!("{mb:?}"), "{ctx}: volume_moments {v:?}");
    }
    for &f in a.cells(Dim::Face) {
        let (ma, mb) = (a.face_moments(f), b.face_moments(f));
        assert_eq!(format!("{ma:?}"), format!("{mb:?}"), "{ctx}: face_moments {f:?}");
    }
}

/// SplitMix64: a tiny seeded generator for the randomized drivers.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { (self.next() % n as u64) as usize }
    }
    pub fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn pick<T: Copy>(&mut self, xs: &[T]) -> Option<T> {
        if xs.is_empty() { None } else { Some(xs[self.below(xs.len())]) }
    }
}
