//! Columnar building blocks: compressed sparse row (CSR) relations.

use serde::{Deserialize, Serialize};

/// A CSR relation: row `i` is `data[offsets[i]..offsets[i+1]]`. Offsets are `u64` per the
/// storage decisions (addendum §7.2) so a mesh is not limited to 2^32 incidences.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Csr<T> {
    offsets: Vec<u64>,
    data: Vec<T>,
}

impl<T> Default for Csr<T> {
    fn default() -> Self {
        Self { offsets: vec![0], data: Vec::new() }
    }
}

impl<T> Csr<T> {
    /// An empty relation (zero rows).
    pub fn new() -> Self {
        Self::default()
    }
    /// Number of rows.
    pub fn len(&self) -> usize {
        self.offsets.len() - 1
    }
    /// True if there are no rows.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Row `i`; empty slice if out of range.
    pub fn row(&self, i: usize) -> &[T] {
        match (self.offsets.get(i), self.offsets.get(i + 1)) {
            (Some(&a), Some(&b)) => &self.data[a as usize..b as usize],
            _ => &[],
        }
    }
    /// Mutable row `i` (length cannot change); empty if out of range.
    pub fn row_mut(&mut self, i: usize) -> &mut [T] {
        match (self.offsets.get(i), self.offsets.get(i + 1)) {
            (Some(&a), Some(&b)) => &mut self.data[a as usize..b as usize],
            _ => &mut [],
        }
    }
    /// Append a row.
    pub fn push_row(&mut self, row: impl IntoIterator<Item = T>) {
        self.data.extend(row);
        self.offsets.push(self.data.len() as u64);
    }
    /// The offsets column (`len() + 1` entries).
    pub fn offsets(&self) -> &[u64] {
        &self.offsets
    }
    /// The flat data column.
    pub fn data(&self) -> &[T] {
        &self.data
    }
    /// Iterate rows in order.
    pub fn rows(&self) -> impl Iterator<Item = &[T]> {
        (0..self.len()).map(|i| self.row(i))
    }
}

impl<T: Clone> Csr<T> {
    /// Build from an iterator of rows.
    pub fn from_rows<R: AsRef<[T]>>(rows: impl IntoIterator<Item = R>) -> Self {
        let mut c = Self::new();
        for r in rows {
            c.push_row(r.as_ref().iter().cloned());
        }
        c
    }
}
