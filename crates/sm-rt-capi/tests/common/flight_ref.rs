//! The Rust path of the transport "code" in `tests/c/flight_common.h`: the same algorithm over
//! `FlightCursor`, so the CASE lines the C program prints can be compared bit for bit. Only
//! `+ - * /` and `sqrt` are used, as in C.
#![allow(dead_code, clippy::neg_cmp_op_on_partial_ord)]

use sm_core::{Point3, Vector3};
use sm_rt_api::{FlightInput, FlightOptions, ModePolicy, NodeId, RegionId, Scratch, TraceEnd, TrackMode, Tracker};

/// splitmix64 as in `tests/c/check.h`.
pub struct Sm(pub u64);

impl Sm {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn unit01(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / 9_007_199_254_740_992.0
    }
}

/// The code's cross section per material name.
pub fn sigma_of(name: &str) -> f64 {
    let s: u32 = name.bytes().map(u32::from).sum();
    0.5 + f64::from(s % 5) * 0.25
}

/// Sigma per region and the majorant per node (max over the regions of the node's set).
pub fn model(t: &dyn Tracker) -> (Vec<f64>, Vec<f64>) {
    let sig: Vec<f64> = t.regions().0.iter().map(|r| sigma_of(&r.name)).collect();
    let h = t.majorant_view().expect("majorant view").hierarchy();
    let maj = (0..h.node_count())
        .map(|n| {
            let info = h.node(NodeId::new(n)).expect("node");
            h.set_regions(info.set).expect("set").iter().fold(0.0, |m: f64, r| {
                let v = sig[r.get()];
                if v > m { v } else { m }
            })
        })
        .collect();
    (sig, maj)
}

fn hex(v: &[f64]) -> String {
    v.iter().map(|x| format!("{:016x}", x.to_bits())).collect::<Vec<_>>().join(",")
}

/// Run one case and return its line (without the trailing newline); see `run_case` in
/// `flight_common.h` for the modes.
pub fn run_case(label: &str, t: &dyn Tracker, mode: u32, nhist: i64, seed: u64) -> String {
    let view = t.majorant_view().expect("majorant view");
    let (sig, maj) = model(t);
    let nreg = sig.len();
    let info = t.info();
    let (lo, hi) = (info.bounds.lo, info.bounds.hi);
    let (lo, hi) = ([lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z]);
    let block: Vec<NodeId> = if mode == 2 { (1..maj.len()).step_by(2).map(NodeId::new).collect() } else { Vec::new() };
    let mut rng = Sm(seed);
    let mut sc = Scratch::with_capacity(64);
    let (mut track, mut coll) = (vec![0.0f64; nreg], vec![0.0f64; nreg]);
    let (mut nseg, mut ncoll, mut nrej, mut nskip) = (0u64, 0u64, 0u64, 0u64);
    let mut kinds = [0u64; 6];
    let ridx = |r: RegionId| -> usize {
        let i = r.get();
        assert!(i < nreg, "region {r:?}");
        i
    };
    for _ in 0..nhist {
        let mut p = [0.0; 3];
        for k in 0..3 {
            p[k] = lo[k] + (hi[k] - lo[k]) * rng.unit01();
        }
        let mut d = [0.0; 3];
        let r2 = loop {
            for x in &mut d {
                *x = 2.0 * rng.unit01() - 1.0;
            }
            let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if r2 <= 1.0 && r2 > 1e-4 {
                break r2;
            }
        };
        let nrm = r2.sqrt();
        for x in &mut d {
            *x /= nrm;
        }
        let mut st = t.init_state(Point3::new(p[0], p[1], p[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc).expect("init_state");
        if st.cell.is_none() {
            nskip += 1;
            continue;
        }
        let policy = if mode == 0 { ModePolicy::Surface } else { ModePolicy::Delta };
        let mut cur = view.begin(&st, &FlightOptions::default().with_policy(policy)).expect("begin");
        let input = FlightInput::new(&maj).with_block(&block);
        let mut stopped = false;
        'flight: while let Some(seg) = cur.next_segment(view, input, &mut st, &mut sc).expect("next_segment") {
            nseg += 1;
            let len = seg.t1 - seg.t0;
            let ri = ridx(seg.region);
            if mode == 0 {
                assert_eq!(seg.mode, TrackMode::Surface);
                track[ri] += len;
                continue;
            }
            if seg.mode == TrackMode::Surface {
                let sg = sig[ri];
                let u = rng.unit01();
                // tau = u/(1-u) is a stand-in variate, not a physical exponential optical depth;
                // the C and Rust references only have to agree with each other.
                let tau = u / (1.0 - u);
                if sg > 0.0 && tau / sg < len {
                    let mut tt = seg.t0 + tau / sg;
                    if tt > seg.t1 {
                        tt = seg.t1;
                    }
                    cur.check_collision(sg).expect("check");
                    cur.stop_at(view, input, &mut st, &mut sc, tt).expect("stop_at");
                    coll[ri] += 1.0 / sg;
                    ncoll += 1;
                    stopped = true;
                    break 'flight;
                }
                continue;
            }
            let sm = seg.sigma_m;
            let mut s = 0.0;
            loop {
                let u = rng.unit01();
                let tau = u / (1.0 - u);
                if !(sm > 0.0) {
                    break;
                }
                s += tau / sm;
                if s >= len {
                    break;
                }
                let mut tt = seg.t0 + s;
                if tt > seg.t1 {
                    tt = seg.t1;
                }
                let loc = cur.probe(view, input, &mut sc, tt).expect("probe");
                let rj = ridx(loc.region);
                let stt = sig[rj];
                let u2 = rng.unit01();
                if u2 * sm < stt {
                    cur.check_collision(stt).expect("check");
                    cur.stop_at(view, input, &mut st, &mut sc, tt).expect("stop_at");
                    coll[rj] += 1.0 / stt;
                    ncoll += 1;
                    stopped = true;
                    break 'flight;
                }
                nrej += 1;
            }
        }
        let kind = match cur.end() {
            None => 0,
            Some(TraceEnd::Limit) => 1,
            Some(TraceEnd::LeftModel) => 2,
            Some(TraceEnd::Boundary { .. }) => 3,
            Some(TraceEnd::Lost(_)) => 4,
            Some(TraceEnd::Stopped) => 5,
            Some(_) => 4,
        };
        assert!(kind >= 1 && (stopped == (kind == 5)), "kind {kind}");
        kinds[kind] += 1;
    }
    format!(
        "CASE {label} hist={nhist} skip={nskip} seg={nseg} coll={ncoll} rej={nrej} kinds={} track={} colls={}",
        kinds.iter().map(u64::to_string).collect::<Vec<_>>().join(","),
        hex(&track),
        hex(&coll)
    )
}
