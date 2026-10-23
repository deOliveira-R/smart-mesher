//! The model-semantics aggregate: coloring + boundary kinds + symmetry + instancing, and the
//! operations that must update them together.

use crate::boundary::{BoundaryTable, DeckSides};
use crate::coloring::Coloring;
use crate::error::{ModelError, Result};
use crate::group::{DeckGenerator, DeckOp};
use crate::ids::IdAllocator;
use crate::instancing::Instancing;
use crate::symmetry::{CutPlan, CutReport, DomainState, FundamentalDomain, HalfSpace, Symmetry, SymmetryOrigin, same_transform, sides_from_caps};
use crate::detect::Verification;
use sm_core::EntityId;

/// Coloring, boundary kinds, symmetry and instancing of one model. Topology and shape live in their
/// own layers and are read through [`crate::topo::TopologyView`].
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Model {
    /// Volume/face colors and named groups.
    pub coloring: Coloring,
    /// Per-face boundary kinds and deck records.
    pub boundaries: BoundaryTable,
    /// Symmetry, if any.
    pub symmetry: Option<Symmetry>,
    /// Prototypes, instances and lattices.
    pub instancing: Instancing,
}

impl Model {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    /// The operation of a deck generator.
    pub fn deck_op(&self, generator: EntityId) -> Option<DeckOp> {
        self.symmetry.as_ref()?.generator(generator).map(|g| g.op)
    }

    /// Register `gen_` in the symmetry (reusing an existing generator with the same transform) and
    /// return the id deck boundaries should reference.
    fn register_generator(&mut self, gen_: DeckGenerator) -> EntityId {
        match &mut self.symmetry {
            Some(s) => {
                if let Some(e) = s.generators.iter().find(|g| same_transform(&g.op, &gen_.op)) {
                    return e.id;
                }
                s.generators.push(gen_);
                gen_.id
            }
            None => {
                self.symmetry = Some(Symmetry {
                    generators: vec![gen_],
                    domain: FundamentalDomain { state: DomainState::Reduced, chamber: Vec::new() },
                    origin: SymmetryOrigin::Declared,
                });
                gen_.id
            }
        }
    }

    /// Finish a cut-and-keep-chamber after the topology engine executed `plan` and returned
    /// `report`: children inherit colors/kinds/group membership, retired entities disappear, the cap
    /// faces become deck boundaries (self-paired for a mirror, `A`/`B` for rotation and translation),
    /// optionally get `cap_color`, and the symmetry records the new generator and chamber.
    /// Returns the deck boundary id.
    pub fn apply_cut(&mut self, plan: &CutPlan, report: &CutReport, cap_color: Option<EntityId>, ids: &mut dyn IdAllocator) -> Result<EntityId> {
        let sides = sides_from_caps(plan, report)?;
        for l in &report.volumes {
            self.coloring.inherit(l.old, l.new);
        }
        for l in &report.faces {
            self.coloring.inherit(l.old, l.new);
            self.boundaries.inherit(l.old, l.new);
        }
        // Ancestors that were replaced (and are not themselves descendants) are retired.
        let news: std::collections::BTreeSet<EntityId> = report.volumes.iter().chain(&report.faces).map(|l| l.new).collect();
        for l in report.volumes.iter().chain(&report.faces) {
            if l.old != l.new && !news.contains(&l.old) {
                self.coloring.retire(l.old);
                self.boundaries.retire(l.old);
            }
        }
        for &v in &report.removed_volumes {
            self.coloring.retire(v);
        }
        for &f in &report.removed_faces {
            self.coloring.retire(f);
            self.boundaries.retire(f);
        }
        // A survivor that kept its id also keeps its old kind unless it is now a cap.
        for c in &report.caps {
            self.boundaries.retire(c.face);
        }
        let gen_id = self.register_generator(plan.generator);
        let deck = self.boundaries.add_deck(ids, gen_id, sides)?;
        if let Some(color) = cap_color {
            for c in &report.caps {
                self.coloring.assign_face(c.face, color)?;
            }
        }
        if let Some(s) = &mut self.symmetry {
            s.domain.state = DomainState::Reduced;
            s.origin = SymmetryOrigin::Cut;
            s.domain.chamber.extend(plan.planes.iter().map(|p| p.keep));
        }
        Ok(deck)
    }

    /// Imported pre-cut geometry: declare that the model is already a fundamental domain and tag
    /// existing faces as a deck boundary of `op`.
    pub fn declare_deck(&mut self, ids: &mut dyn IdAllocator, op: DeckOp, sides: DeckSides, chamber: Vec<HalfSpace>) -> Result<EntityId> {
        let gen_ = crate::group::new_generator(ids, op)?;
        let gid = self.register_generator(gen_);
        let deck = self.boundaries.add_deck(ids, gid, sides)?;
        if let Some(s) = &mut self.symmetry {
            s.domain.state = DomainState::Reduced;
            s.domain.chamber.extend(chamber);
        }
        Ok(deck)
    }

    /// Record generators found by detection on the *full* model (no reduction yet).
    pub fn adopt_detected(&mut self, generators: Vec<DeckGenerator>, verification: Verification) -> Result<()> {
        if self.symmetry.is_some() {
            return Err(ModelError::State("model already has a symmetry"));
        }
        self.symmetry = Some(Symmetry {
            generators,
            domain: FundamentalDomain { state: DomainState::Full, chamber: Vec::new() },
            origin: SymmetryOrigin::Detected(verification),
        });
        Ok(())
    }
}
