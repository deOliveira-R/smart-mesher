//! Acceptance 12: bit-identical meshes for the sequential executor, scoped threads and
//! one-thread / many-thread rayon pools, on meshes above the rayon thresholds.

mod common;

use common::{hex_gauss_volume, mesh_with, registry, registry_with_sources};
use sm_mesh_api::{MeshEntityKind, MeshRead};
use sm_model_api::Model;
use sm_scheme_api::{
    Executor, Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, ScopedThreads, Sequential,
    SizingSpec,
};
use sm_testkit::check::mesh_hash;
use sm_testkit::determinism::{check, hash_f64_bits};
use sm_testkit::topo::{SweepFixture, extruded_l, twisted_prism};

fn pool(n: usize) -> rayon::ThreadPool {
    rayon::ThreadPoolBuilder::new()
        .num_threads(n)
        .build()
        .expect("pool")
}

fn coords_hash(m: &dyn MeshRead) -> u64 {
    let c = m.coords();
    let mut all = Vec::with_capacity(3 * c.x.len());
    for i in 0..c.x.len() {
        all.extend([c.x[i], c.y[i], c.z[i]]);
    }
    hash_f64_bits(&all)
}

fn sweep_choice(fx: &SweepFixture) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [fx.source], "target": [fx.target] }),
        Firmness::Hard,
    )
}

type Hashes = (u64, u64, u64, usize);

/// An L-prism: 1728 lattice quads x 25 layers = 43 200 hexes.
fn l_prism(exec: &dyn Executor) -> Hashes {
    let fx = extruded_l(2.0, 1.0, 2.0).expect("L");
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 2.0 / 48.0 });
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(25)));
    }
    intent.set_scheme(fx.volume, sweep_choice(&fx));
    intent.set_scheme(
        fx.source,
        SchemeChoice::with_params(
            "gridface",
            serde_json::json!({ "h": 2.0 / 48.0 }),
            Firmness::Hard,
        ),
    );
    let out = mesh_with(
        &registry_with_sources(),
        &fx.topo.complex,
        &Model::new(),
        &[fx.volume],
        &intent,
        exec,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    (
        mesh_hash(m),
        coords_hash(m),
        hash_f64_bits(
            &out.patches
                .nodes()
                .flat_map(|(_, p, _)| [p.x, p.y, p.z])
                .collect::<Vec<_>>(),
        ),
        m.count(MeshEntityKind::Cell),
    )
}

/// A twisted prism with a 40 x 40 stub-Submap source and 25 layers: 40 000 hexes, placed by the
/// affine fit plus Shepard residual on every layer.
fn twisted(exec: &dyn Executor) -> Hashes {
    let fx = twisted_prism(2.0, 3.0, 1.2).expect("twisted");
    let mut intent = MeshIntent::default();
    let topo = &fx.topo.complex;
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(40)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(25)));
    }
    intent.set_scheme(fx.volume, sweep_choice(&fx));
    intent.set_scheme(fx.source, SchemeChoice::new("submap", Firmness::Hard));
    let out = mesh_with(
        &registry(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        exec,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    let (vol, least) = hex_gauss_volume(m);
    assert!(least > 0.0);
    let exact = 4.0 * 3.0 * (2.0 + 1.2f64.cos()) / 3.0;
    assert!((vol - exact).abs() / exact < 1e-11, "{vol} vs {exact}");
    (
        mesh_hash(m),
        coords_hash(m),
        hash_f64_bits(
            &out.patches
                .nodes()
                .flat_map(|(_, p, _)| [p.x, p.y, p.z])
                .collect::<Vec<_>>(),
        ),
        m.count(MeshEntityKind::Cell),
    )
}

fn executors(run: impl Fn(&dyn Executor) -> Hashes + Sync) -> Hashes {
    let base = pool(1).install(|| run(&Sequential));
    let scoped = pool(4).install(|| run(&ScopedThreads(4)));
    let default_pool = run(&Sequential);
    assert_eq!(
        base, scoped,
        "Sequential/1 thread vs ScopedThreads(4)/4 threads"
    );
    assert_eq!(base, default_pool, "one-thread pool vs the default pool");
    base
}

#[test]
fn l_prism_is_bit_identical_across_executors_and_pools() {
    let h = executors(l_prism);
    assert_eq!(h.3, 1728 * 25);
}

#[test]
fn twisted_prism_with_shepard_is_bit_identical_across_executors_and_pools() {
    let h = executors(twisted);
    assert_eq!(h.3, 1600 * 25);
}

#[test]
fn harness_check_over_repeated_and_concurrent_runs() {
    let r = check(1, 4, |threads| {
        if threads == 1 {
            pool(1).install(|| twisted(&Sequential))
        } else {
            pool(threads).install(|| twisted(&ScopedThreads(threads)))
        }
    });
    assert!(r.is_ok(), "{r:?}");
}
