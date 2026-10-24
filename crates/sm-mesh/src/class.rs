//! Compact classification columns and the reverse class index.

use crate::util::CHUNK;
use rayon::prelude::*;
use sm_core::EntityId;
use sm_mesh_api::{
    CellIx, ClassSet, Classification, EdgeIx, EdgeTable, FaceIx, Param, VertexIx,
};
use std::collections::BTreeMap;

/// One classification per entity: the model/partition entity densely (8 bytes), the parametric
/// position sparsely (most entities carry none).
#[derive(Clone, Debug, PartialEq, Default)]
pub(crate) struct ClassCol {
    pub(crate) ent: Vec<EntityId>,
    pub(crate) param: BTreeMap<u32, Param>,
}

impl ClassCol {
    pub(crate) fn get(&self, i: usize) -> Classification {
        Classification {
            entity: self.ent[i],
            param: self.param.get(&(i as u32)).copied().unwrap_or(Param::None),
        }
    }
    pub(crate) fn set(&mut self, i: usize, c: Classification) {
        self.ent[i] = c.entity;
        match c.param {
            Param::None => {
                self.param.remove(&(i as u32));
            }
            p => {
                self.param.insert(i as u32, p);
            }
        }
    }
    pub(crate) fn len(&self) -> usize {
        self.ent.len()
    }
}

/// Group the entities of one column by class entity (ascending indices per entity), in parallel
/// chunks merged in chunk order.
fn group(ent: &[EntityId]) -> BTreeMap<EntityId, Vec<u32>> {
    let parts: Vec<BTreeMap<EntityId, Vec<u32>>> = ent
        .par_chunks(CHUNK * 4)
        .enumerate()
        .map(|(k, chunk)| {
            let base = (k * CHUNK * 4) as u32;
            let mut at: BTreeMap<EntityId, usize> = BTreeMap::new();
            let mut last: Option<(EntityId, usize)> = None;
            let mut slots: Vec<(EntityId, Vec<u32>)> = Vec::new();
            for (i, e) in chunk.iter().enumerate() {
                let slot = match last {
                    Some((le, s)) if le == *e => s,
                    _ => {
                        let s = *at.entry(*e).or_insert_with(|| {
                            slots.push((*e, Vec::new()));
                            slots.len() - 1
                        });
                        last = Some((*e, s));
                        s
                    }
                };
                slots[slot].1.push(base + i as u32);
            }
            slots.into_iter().collect()
        })
        .collect();
    let mut out: BTreeMap<EntityId, Vec<u32>> = BTreeMap::new();
    for p in parts {
        for (e, v) in p {
            out.entry(e).or_default().extend(v);
        }
    }
    out
}

/// Build the reverse index and the sorted entity list from the columns (and the edge table).
pub(crate) fn build_index(
    v: &ClassCol,
    f: &ClassCol,
    c: &ClassCol,
    edges: Option<&EdgeTable>,
) -> (BTreeMap<EntityId, ClassSet>, Vec<EntityId>) {
    let mut idx: BTreeMap<EntityId, ClassSet> = BTreeMap::new();
    for (e, ix) in group(&v.ent) {
        idx.entry(e).or_default().vertices = ix.into_iter().map(VertexIx).collect();
    }
    if let Some(t) = edges {
        let ent: Vec<EntityId> = t.class.iter().map(|c| c.entity).collect();
        for (e, ix) in group(&ent) {
            idx.entry(e).or_default().edges = ix.into_iter().map(EdgeIx).collect();
        }
    }
    for (e, ix) in group(&f.ent) {
        idx.entry(e).or_default().faces = ix.into_iter().map(FaceIx).collect();
    }
    for (e, ix) in group(&c.ent) {
        idx.entry(e).or_default().cells = ix.into_iter().map(CellIx).collect();
    }
    let ents = idx.keys().copied().collect();
    (idx, ents)
}
