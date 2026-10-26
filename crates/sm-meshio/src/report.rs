//! Loss reports shared by every exporter: what a format could not carry.
//!
//! A report is a sorted list of [`LossItem`]s keyed by a stable `code` (`vtu.*`, `foam.*`), so
//! callers and tests match on codes, not on prose. Exporters never fail on a loss; they report it.

use crate::vtu::VtuOptions;
use serde::{Deserialize, Serialize};
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, SetMembers};

/// How bad a loss is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LossSeverity {
    /// Information moved to a different place or spelling; nothing is lost.
    Info,
    /// Information is kept in a weaker form (a type downgrade, a default assignment).
    Degraded,
    /// Information is absent from the output.
    Dropped,
}

/// One kind of loss with how many entities it touched.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LossItem {
    /// Stable machine code, for example `vtu.high_order_dropped`.
    pub code: String,
    /// Severity.
    pub severity: LossSeverity,
    /// Number of entities (cells, faces, sets, points) affected.
    pub count: u64,
    /// Human-readable detail; deterministic text (names in mesh order).
    pub detail: String,
}

impl LossItem {
    /// A loss item.
    pub fn new(code: impl Into<String>, severity: LossSeverity, count: u64, detail: impl Into<String>) -> Self {
        Self { code: code.into(), severity, count, detail: detail.into() }
    }
}

/// Everything an exporter could not carry, sorted by `code` then `detail`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LossReport {
    /// The items, sorted by `(code, detail)`; at most one item per pair.
    pub items: Vec<LossItem>,
}

impl LossReport {
    /// An empty (lossless) report.
    pub fn new() -> Self {
        Self::default()
    }

    /// True if nothing was lost (no items at all, `Info` items included).
    pub fn is_lossless(&self) -> bool {
        self.items.is_empty()
    }

    /// Add an item, keeping the list sorted. An item with the same `(code, detail)` as an
    /// existing one is merged into it: counts add and the worse severity wins.
    pub fn push(&mut self, item: LossItem) {
        match self.items.binary_search_by(|p| (p.code.as_str(), p.detail.as_str()).cmp(&(item.code.as_str(), item.detail.as_str()))) {
            Ok(i) => {
                let p = &mut self.items[i];
                p.count = p.count.saturating_add(item.count);
                p.severity = p.severity.max(item.severity);
            }
            Err(i) => {
                self.items.insert(i, item);
            }
        }
    }

    /// The first item with this code (items of one code are ordered by detail).
    pub fn get(&self, code: &str) -> Option<&LossItem> {
        self.items.iter().find(|i| i.code == code)
    }
}

/// What a VTU export of `mesh` under `opts` cannot carry. Pure and cheap: counts only.
///
/// Codes: `vtu.high_order_dropped` (cells with a high-order record: written as linear corners),
/// `vtu.face_curves_dropped`, `vtu.face_sets_dropped` (face, side and shell-side sets),
/// `vtu.edge_table_dropped`, `vtu.extra_points_unused` (`n_points - n_vertices`: extra
/// high-order nodes are written as unreferenced points).
pub fn vtu_loss_report(mesh: &dyn MeshRead, _opts: &VtuOptions) -> LossReport {
    let mut r = LossReport::new();
    let n = |c: usize| c as u64;
    let ho = (0..mesh.cell_count()).filter(|&c| mesh.high_order_cell(CellIx::new(c)).is_some()).count();
    if ho > 0 {
        r.push(LossItem::new("vtu.high_order_dropped", LossSeverity::Degraded, n(ho), "cells with high-order geometry are written as their linear corners"));
    }
    let fc = (0..mesh.face_count()).filter(|&f| mesh.face_curve(FaceIx::new(f)).is_some()).count();
    if fc > 0 {
        r.push(LossItem::new("vtu.face_curves_dropped", LossSeverity::Dropped, n(fc), "curved-face records are not written"));
    }
    let names: Vec<&str> = mesh
        .sets()
        .iter()
        .filter(|s| matches!(s.members, SetMembers::Faces(_) | SetMembers::Sides(_) | SetMembers::ShellSides(_)))
        .map(|s| s.name.as_str())
        .collect();
    if !names.is_empty() {
        r.push(LossItem::new("vtu.face_sets_dropped", LossSeverity::Dropped, n(names.len()), names.join(",")));
    }
    let ne = mesh.count(MeshEntityKind::Edge);
    if ne > 0 {
        r.push(LossItem::new("vtu.edge_table_dropped", LossSeverity::Dropped, n(ne), "the edge table and its classification are not written"));
    }
    let extra = mesh.coords().len().saturating_sub(mesh.vertex_count());
    if extra > 0 {
        r.push(LossItem::new("vtu.extra_points_unused", LossSeverity::Info, n(extra), "extra high-order nodes are written as points no cell references"));
    }
    r
}
