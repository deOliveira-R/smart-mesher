//! Typed tag columns and entity sets (the MOAB part of the design).

use serde::{Deserialize, Serialize};

use crate::error::{MeshError, MeshResult};
use crate::index::Dim;
use crate::schema::{TagDType, TagDef};

/// Borrowed value of one entity's tag row.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TagRow<'a> {
    /// Integer components.
    I64(&'a [i64]),
    /// Float components.
    F64(&'a [f64]),
    /// Raw entity-id components (`0` = none).
    Id(&'a [u64]),
}

/// Flat storage of a tag column, `len * width` elements.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TagData {
    /// Integers.
    I64(Vec<i64>),
    /// Floats.
    F64(Vec<f64>),
    /// Raw entity ids.
    Id(Vec<u64>),
}

/// One typed column: definition plus data, one row per entity of the tag's dimension.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TagColumn {
    /// The declaration.
    pub def: TagDef,
    /// The values.
    pub data: TagData,
}

impl TagColumn {
    /// A zero-filled column for `len` entities.
    pub fn new(def: TagDef, len: usize) -> Self {
        let n = len * def.width;
        let data = match def.dtype {
            TagDType::I64 => TagData::I64(vec![0; n]),
            TagDType::F64 => TagData::F64(vec![0.0; n]),
            TagDType::Id => TagData::Id(vec![0; n]),
        };
        Self { def, data }
    }

    /// Number of rows.
    pub fn len(&self) -> usize {
        let n = match &self.data {
            TagData::I64(v) => v.len(),
            TagData::F64(v) => v.len(),
            TagData::Id(v) => v.len(),
        };
        n / self.def.width.max(1)
    }

    /// True if the column has no rows.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Grow with zero rows to `len` rows (never shrinks).
    pub fn resize(&mut self, len: usize) {
        let n = len * self.def.width;
        match &mut self.data {
            TagData::I64(v) if v.len() < n => v.resize(n, 0),
            TagData::F64(v) if v.len() < n => v.resize(n, 0.0),
            TagData::Id(v) if v.len() < n => v.resize(n, 0),
            _ => {}
        }
    }

    /// Row `ix`, or `None` if out of range.
    pub fn row(&self, ix: usize) -> Option<TagRow<'_>> {
        let w = self.def.width;
        let (a, b) = (ix * w, ix * w + w);
        match &self.data {
            TagData::I64(v) => v.get(a..b).map(TagRow::I64),
            TagData::F64(v) => v.get(a..b).map(TagRow::F64),
            TagData::Id(v) => v.get(a..b).map(TagRow::Id),
        }
    }

    /// Overwrite row `ix`. Errors on type/width mismatch or out-of-range row.
    pub fn set_row(&mut self, ix: usize, value: TagRow<'_>) -> MeshResult<()> {
        let w = self.def.width;
        let mismatch = || MeshError::TagMismatch(self.def.name.clone());
        let (a, b) = (ix * w, ix * w + w);
        match (&mut self.data, value) {
            (TagData::I64(v), TagRow::I64(s)) if s.len() == w => {
                v.get_mut(a..b).ok_or_else(mismatch)?.copy_from_slice(s)
            }
            (TagData::F64(v), TagRow::F64(s)) if s.len() == w => {
                v.get_mut(a..b).ok_or_else(mismatch)?.copy_from_slice(s)
            }
            (TagData::Id(v), TagRow::Id(s)) if s.len() == w => {
                v.get_mut(a..b).ok_or_else(mismatch)?.copy_from_slice(s)
            }
            _ => return Err(mismatch()),
        }
        Ok(())
    }
}

/// A named set of same-dimension entities. Members are sorted and unique (canonical order).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntitySet {
    /// Dimension of the members.
    pub dim: Dim,
    members: Vec<u32>,
}

impl EntitySet {
    /// Build from arbitrary indices (sorted and deduplicated).
    pub fn new(dim: Dim, mut members: Vec<u32>) -> Self {
        members.sort_unstable();
        members.dedup();
        Self { dim, members }
    }
    /// Sorted member indices.
    pub fn members(&self) -> &[u32] {
        &self.members
    }
    /// Membership test.
    pub fn contains(&self, ix: u32) -> bool {
        self.members.binary_search(&ix).is_ok()
    }
}

/// Union of two sorted unique lists.
pub fn sorted_union(a: &[u32], b: &[u32]) -> Vec<u32> {
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        let next = match (a.get(i), b.get(j)) {
            (Some(&x), Some(&y)) if x == y => {
                i += 1;
                j += 1;
                x
            }
            (Some(&x), Some(&y)) if x < y => {
                i += 1;
                x
            }
            (Some(_), Some(&y)) => {
                j += 1;
                y
            }
            (Some(&x), None) => {
                i += 1;
                x
            }
            (None, Some(&y)) => {
                j += 1;
                y
            }
            (None, None) => break,
        };
        out.push(next);
    }
    out
}

/// Intersection of two sorted unique lists.
pub fn sorted_intersection(a: &[u32], b: &[u32]) -> Vec<u32> {
    a.iter().copied().filter(|x| b.binary_search(x).is_ok()).collect()
}

/// Difference `a \ b` of two sorted unique lists.
pub fn sorted_difference(a: &[u32], b: &[u32]) -> Vec<u32> {
    a.iter().copied().filter(|x| b.binary_search(x).is_err()).collect()
}
