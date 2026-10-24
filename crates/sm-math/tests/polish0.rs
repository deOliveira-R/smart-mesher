//! polish0 regressions: svd3 cycling at the rounding floor, small-value preservation, sym3 fuzz.

#![allow(clippy::needless_range_loop)]

mod common;
use common::Rng;
use sm_math::linalg::*;

fn maxdiff(a: &Mat3, b: &Mat3) -> f64 {
    (0..3)
        .flat_map(|i| (0..3).map(move |j| (i, j)))
        .map(|(i, j)| (a[i][j] - b[i][j]).abs())
        .fold(0.0, f64::max)
}

const ID: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn check_svd(a: &Mat3) {
    let r = svd3(a).unwrap_or_else(|e| panic!("svd3 failed ({e}) for {a:?}"));
    let norm = a.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(
        maxdiff(&r.reconstruct(), a) <= 1e-13 * norm,
        "reconstruction {a:?}"
    );
    for m in [&r.u, &r.v] {
        let g = mat3_mul(&mat3_transpose(m), m);
        assert!(maxdiff(&g, &ID) <= 1e-13, "orthogonality {a:?}");
    }
    assert!(r.s[0] >= r.s[1] && r.s[1] >= r.s[2] && r.s[2] >= 0.0);
}

#[test]
fn svd3_reviewer_reproducers() {
    let a = [
        [
            -1.8168255503613585e26,
            -2.461734956253616e25,
            -3.663739020148964e26,
        ],
        [
            -4.476548292008179e26,
            1.063118874325685e27,
            1.1512639724800981e27,
        ],
        [
            1.168367138834097e27,
            7.912496542153914e25,
            -1.0462045330398398e27,
        ],
    ];
    let b = [
        [
            3.174903581458699e-35,
            -8.908251493832893e-36,
            -3.6250267996737813e-35,
        ],
        [
            -2.1282620308492364e-35,
            4.4418526924077095e-35,
            -7.177099656273105e-36,
        ],
        [
            4.144489710987125e-35,
            1.586169734103691e-35,
            -3.9881087363929566e-36,
        ],
    ];
    let c = [
        [
            6.612713272190578e31,
            6.612713272190578e31,
            3.6501297282523223e31,
        ],
        [
            3.6093125538581703e31,
            3.6093125538581703e31,
            5.572211152757841e30,
        ],
        [
            7.990968813510776e31,
            7.990968813510776e31,
            -3.360083978231905e31,
        ],
    ];
    for m in [a, b, c] {
        check_svd(&m);
    }
}

fn rel_close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-15 * b.abs()
}

#[test]
fn svd3_keeps_small_orthogonal_values() {
    for d in [[1.0, 1.0, 1e-17], [1.0, 1e-9, 1e-18], [1e-18, 1.0, 1e-9]] {
        let a = [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]];
        let r = svd3(&a).unwrap();
        let mut want = d;
        want.sort_by(|x, y| y.total_cmp(x));
        for k in 0..3 {
            assert!(rel_close(r.s[k], want[k]), "{d:?} -> {:?}", r.s);
        }
        let cond = r.condition();
        assert!(
            cond.is_finite() && rel_close(cond, want[0] / want[2])
                || (cond / (want[0] / want[2]) - 1.0).abs() < 1e-14
        );
    }
    let r = svd3(&[[1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 1e-16]]).unwrap();
    assert!(r.s[2] > 0.0 && r.condition().is_finite(), "{:?}", r.s);
}

fn unif(rng: &mut Rng) -> f64 {
    (rng.next() >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
}

fn pow2(k: i32) -> f64 {
    f64::from_bits(((1023 + k) as u64) << 52)
}

fn gen_matrix(rng: &mut Rng, kind: usize) -> Mat3 {
    let scale = pow2(rng.int(400) as i32);
    let mut m = [[0.0; 3]; 3];
    for row in m.iter_mut() {
        for v in row.iter_mut() {
            *v = unif(rng);
        }
    }
    match kind {
        0 => {}
        1 => {
            // Rank 1: outer product.
            let u = [unif(rng), unif(rng), unif(rng)];
            let w = [unif(rng), unif(rng), unif(rng)];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] = u[i] * w[j];
                }
            }
        }
        2 => {
            // Rank 2: third column is a combination of the first two.
            let (x, y) = (unif(rng), unif(rng));
            for row in m.iter_mut() {
                row[2] = x * row[0] + y * row[1];
            }
        }
        3 => {
            for row in m.iter_mut() {
                row[1] = row[0];
            }
        }
        _ => {
            m[2] = m[0];
        }
    }
    m.map(|row| row.map(|v| v * scale))
}

fn fuzz_svd(n: usize, seed: u64) {
    let mut rng = Rng(seed);
    for i in 0..n {
        check_svd(&gen_matrix(&mut rng, i % 5));
    }
}

#[test]
fn svd3_fuzz_default() {
    fuzz_svd(50_000, 0xC0FFEE);
}

#[test]
#[ignore = "1M-case fuzz; run with --release -- --ignored"]
fn svd3_fuzz_large() {
    fuzz_svd(1_000_000, 0xBADC0DE);
}

fn check_eigen(a: &Mat3) {
    let r = sym3_eigen(a).unwrap();
    let norm = a.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut d = [[0.0; 3]; 3];
    for k in 0..3 {
        d[k][k] = r.values[k];
    }
    let rec = mat3_mul(&mat3_mul(&r.vectors, &d), &mat3_transpose(&r.vectors));
    assert!(maxdiff(&rec, a) <= 1e-13 * norm, "reconstruction {a:?}");
    let g = mat3_mul(&mat3_transpose(&r.vectors), &r.vectors);
    assert!(maxdiff(&g, &ID) <= 1e-13, "orthogonality {a:?}");
    assert!(r.values[0] <= r.values[1] && r.values[1] <= r.values[2]);
}

fn fuzz_eigen(n: usize, seed: u64) {
    let mut rng = Rng(seed);
    for i in 0..n {
        let mut m = gen_matrix(&mut rng, i % 5);
        // Symmetrise (the special kinds give repeated / rank-deficient spectra).
        for r in 0..3 {
            for c in 0..r {
                m[c][r] = m[r][c];
            }
        }
        if i % 7 == 0 {
            // Near-repeated eigenvalues: equal diagonal plus tiny off-diagonals.
            let d = m[0][0];
            let (x, y, z) = (m[0][1] * 1e-12, m[0][2] * 1e-14, m[1][2] * 1e-13);
            m = [[d, x, y], [x, d, z], [y, z, d]];
        }
        check_eigen(&m);
    }
}

#[test]
fn sym3_eigen_fuzz_default() {
    fuzz_eigen(50_000, 0xFEED);
}

#[test]
#[ignore = "1M-case fuzz; run with --release -- --ignored"]
fn sym3_eigen_fuzz_large() {
    fuzz_eigen(1_000_000, 0xFACE);
}

#[test]
fn svd3_identical_row_fuzz_rerun() {
    let mut rng = Rng(0x5eed);
    let m = 1_000_000;
    for _ in 0..30000 {
        let k = 10f64.powi(rng.int(40) as i32 - 20) / m as f64;
        let row = [0, 1, 2].map(|_| rng.int(m) as f64 * k);
        check_svd(&[row, row, row]);
    }
}
