//! Canonical forms of rows (shared by operators and tests).

use sm_topo_api::{Dim, Incidence, Side};

/// `a < b`, false if either is NaN (used by operators).
pub(crate) fn lt(a: f64, b: f64) -> bool {
    a.partial_cmp(&b) == Some(core::cmp::Ordering::Less)
}

/// Rotate a cyclic ring so it starts at its smallest `(edge, side)`; ties are broken by comparing
/// the whole rotated sequences, so the result is unique.
pub(crate) fn canonical_rotate(ring: &mut [Incidence]) {
    let n = ring.len();
    if n < 2 {
        return;
    }
    let key = |e: &Incidence| (e.cell, e.side);
    let Some(min) = ring.iter().map(key).min() else { return };
    let mut best: Option<usize> = None;
    for i in 0..n {
        if key(&ring[i]) != min {
            continue;
        }
        best = match best {
            None => Some(i),
            Some(b) => {
                let less = (0..n).map(|k| (key(&ring[(i + k) % n]), key(&ring[(b + k) % n]))).find(|(x, y)| x != y).is_some_and(|(x, y)| x < y);
                Some(if less { i } else { b })
            }
        };
    }
    if let Some(b) = best {
        ring.rotate_left(b);
    }
}

/// Sort and merge the entries of a volume/body row group-locally: by `(ring, cell, side)`,
/// duplicates dropped, and for volumes `Left` + `Right` of one face in one shell fused to `Both`.
pub(crate) fn sort_merge(dim: Dim, row: &mut Vec<Incidence>) {
    row.sort_by_key(|e| (e.ring, e.cell, e.side));
    row.dedup();
    let mut out: Vec<Incidence> = Vec::with_capacity(row.len());
    for e in row.iter() {
        match out.last_mut() {
            Some(l) if l.cell == e.cell && l.ring == e.ring && l.side != e.side && dim == Dim::Volume => l.side = Side::Both,
            _ => out.push(*e),
        }
    }
    *row = out;
}

/// Normalize a row into canonical form for its rank.
pub(crate) fn normalize(dim: Dim, row: &mut Vec<Incidence>) {
    match dim {
        Dim::Vertex => {}
        Dim::Edge => {
            let merged = match row.as_slice() {
                [a, b] if a.cell == b.cell && a.side != Side::Both => Some(a.cell),
                _ => None,
            };
            if let Some(cell) = merged {
                *row = vec![Incidence::new(cell, Side::Both, 0)];
            }
        }
        Dim::Face => {
            let mut out = Vec::with_capacity(row.len());
            let mut embedded: Vec<Incidence> = Vec::new();
            for g in row.chunk_by(|a, b| a.ring == b.ring) {
                if g[0].ring == Incidence::EMBEDDED {
                    embedded.extend_from_slice(g);
                } else {
                    let mut ring = g.to_vec();
                    canonical_rotate(&mut ring);
                    out.extend(ring);
                }
            }
            embedded.sort();
            embedded.dedup();
            out.extend(embedded);
            *row = out;
        }
        Dim::Volume | Dim::Body => sort_merge(dim, row),
    }
}
