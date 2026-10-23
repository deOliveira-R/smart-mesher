//! [`SchemeRegistry`]: scheme name -> code, per-scheme fallback chains and per-dimension auto
//! lists. All maps are ordered, so candidate lists never depend on registration order.

use crate::builtin::{CurveCopy, SurfaceCopy};
use crate::error::DriveError;
use crate::intent::{Firmness, SchemeChoice, SchemeId, SchemeSpec};
use crate::scheme::{CurveScheme, MeshTransform, SchemeCore, SurfaceScheme, VolumeScheme};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A registered scheme of one of the three dimensions.
#[derive(Clone)]
pub enum SchemeImpl {
    /// Curve scheme.
    Curve(Arc<dyn CurveScheme>),
    /// Surface scheme.
    Surface(Arc<dyn SurfaceScheme>),
    /// Volume scheme.
    Volume(Arc<dyn VolumeScheme>),
}

impl SchemeImpl {
    /// The shared phases.
    pub fn core(&self) -> &dyn SchemeCore {
        match self {
            SchemeImpl::Curve(s) => s.as_ref(),
            SchemeImpl::Surface(s) => s.as_ref(),
            SchemeImpl::Volume(s) => s.as_ref(),
        }
    }
}

/// Where a candidate came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Origin {
    /// Named in the intent chain.
    Explicit,
    /// From a registry fallback chain of an explicit scheme.
    Fallback,
    /// From auto selection.
    Auto,
}

/// One scheme to try, with its parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    /// Scheme and parameters.
    pub spec: SchemeSpec,
    /// Origin.
    pub origin: Origin,
}

/// Registered schemes, fallback chains and auto lists.
#[derive(Clone)]
pub struct SchemeRegistry {
    schemes: BTreeMap<SchemeId, SchemeImpl>,
    fallbacks: BTreeMap<SchemeId, Vec<SchemeId>>,
    auto: BTreeMap<u8, Vec<SchemeId>>,
    transforms: BTreeMap<SchemeId, Arc<dyn MeshTransform>>,
}

impl Default for SchemeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl SchemeRegistry {
    /// A registry holding the built-in copy schemes.
    pub fn new() -> Self {
        let mut r = SchemeRegistry { schemes: BTreeMap::new(), fallbacks: BTreeMap::new(), auto: BTreeMap::new(), transforms: BTreeMap::new() };
        r.schemes.insert(SchemeId::new(crate::builtin::CURVE_COPY), SchemeImpl::Curve(Arc::new(CurveCopy)));
        r.schemes.insert(SchemeId::new(crate::builtin::SURFACE_COPY), SchemeImpl::Surface(Arc::new(SurfaceCopy)));
        r
    }

    /// Register a scheme under its `id()`. Duplicate names are an error.
    pub fn register(&mut self, s: SchemeImpl) -> Result<(), DriveError> {
        let id = SchemeId::new(s.core().id());
        if self.schemes.contains_key(&id) {
            return Err(DriveError::Registry(format!("scheme {id} registered twice")));
        }
        self.schemes.insert(id, s);
        Ok(())
    }

    /// Register a mesh transform.
    pub fn register_transform(&mut self, t: Arc<dyn MeshTransform>) -> Result<(), DriveError> {
        let id = SchemeId::new(t.id());
        if self.transforms.contains_key(&id) {
            return Err(DriveError::Registry(format!("transform {id} registered twice")));
        }
        self.transforms.insert(id, t);
        Ok(())
    }

    /// Fallback chain tried after `id` fails (e.g. tridelaunay -> triadvance -> qtri).
    pub fn set_fallback(&mut self, id: &str, chain: &[&str]) -> Result<(), DriveError> {
        let all: Vec<SchemeId> = core::iter::once(id).chain(chain.iter().copied()).map(SchemeId::new).collect();
        if let Some(missing) = all.iter().find(|s| !self.schemes.contains_key(*s)) {
            return Err(DriveError::Registry(format!("unknown scheme {missing}")));
        }
        self.fallbacks.insert(all[0].clone(), all[1..].to_vec());
        Ok(())
    }

    /// Auto-selection list of a dimension (1, 2 or 3), in tie-break order.
    pub fn set_auto(&mut self, dim: u8, ids: &[&str]) -> Result<(), DriveError> {
        let list: Vec<SchemeId> = ids.iter().copied().map(SchemeId::new).collect();
        for s in &list {
            match self.schemes.get(s) {
                Some(i) if i.core().dim() as u8 == dim => {}
                Some(_) => return Err(DriveError::Registry(format!("{s} is not a dimension-{dim} scheme"))),
                None => return Err(DriveError::Registry(format!("unknown scheme {s}"))),
            }
        }
        self.auto.insert(dim, list);
        Ok(())
    }

    /// A scheme by name.
    pub fn get(&self, id: &SchemeId) -> Option<&SchemeImpl> {
        self.schemes.get(id)
    }

    /// A transform by name.
    pub fn transform(&self, id: &SchemeId) -> Option<&Arc<dyn MeshTransform>> {
        self.transforms.get(id)
    }

    /// Candidate list for an entity of dimension `dim`: the explicit chain, each entry followed
    /// by its fallback chain, then (unless the chain is `Hard`) the auto list. Duplicates keep
    /// their first position.
    pub fn candidates(&self, dim: u8, choice: &SchemeChoice) -> Vec<Candidate> {
        let mut out: Vec<Candidate> = Vec::new();
        let push = |spec: SchemeSpec, origin: Origin, out: &mut Vec<Candidate>| {
            if !out.iter().any(|c| c.spec.scheme == spec.scheme) {
                out.push(Candidate { spec, origin });
            }
        };
        let with_auto = match choice {
            SchemeChoice::Auto => true,
            SchemeChoice::Chain { schemes, firmness } => {
                for s in schemes {
                    push(s.clone(), Origin::Explicit, &mut out);
                    for f in self.fallbacks.get(&s.scheme).into_iter().flatten() {
                        push(SchemeSpec { scheme: f.clone(), params: s.params.clone() }, Origin::Fallback, &mut out);
                    }
                }
                *firmness != Firmness::Hard
            }
        };
        if with_auto {
            for s in self.auto.get(&dim).into_iter().flatten() {
                push(SchemeSpec { scheme: s.clone(), params: Default::default() }, Origin::Auto, &mut out);
            }
        }
        out
    }
}
