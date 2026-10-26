//! Canonical output: tet rotation and lexicographic sorting.

/// Rotate a positive tet to the lexicographically smallest even permutation of its vertices.
#[inline]
pub(crate) fn canon_tet(t: [u32; 4]) -> [u32; 4] {
    let mut idx = [0usize, 1, 2, 3];
    idx.sort_by_key(|&i| t[i]);
    // Parity of the sorting permutation.
    let mut inv = 0;
    for i in 0..4 {
        for j in i + 1..4 {
            if idx[i] > idx[j] {
                inv += 1;
            }
        }
    }
    let mut s = [t[idx[0]], t[idx[1]], t[idx[2]], t[idx[3]]];
    if inv % 2 == 1 {
        s.swap(2, 3);
    }
    s
}

/// Canonicalize and sort a tet list.
pub(crate) fn canonical_tets(tets: &mut [[u32; 4]]) {
    for t in tets.iter_mut() {
        *t = canon_tet(*t);
    }
    tets.sort_unstable();
}
