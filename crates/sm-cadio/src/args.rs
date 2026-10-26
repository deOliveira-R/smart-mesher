//! Typed access to the parameters of a record, with errors that name entity and index.

use std::collections::BTreeMap;

use crate::error::StepError;
use crate::part21::{Entity, Param, Rec, StepFile};

/// Read access to the entities of a file.
#[derive(Clone, Copy)]
pub(crate) struct Ents<'a> {
    pub map: &'a BTreeMap<u64, Entity>,
}

impl<'a> Ents<'a> {
    pub fn new(file: &'a StepFile) -> Self {
        Self { map: &file.entities }
    }

    /// The entity `id` referenced by `by`.
    pub fn get(&self, id: u64, by: u64) -> Result<&'a Entity, StepError> {
        self.map.get(&id).ok_or(StepError::MissingEntity { id, referenced_by: by })
    }

    /// The record `name` of entity `id` (simple or part of a complex entity).
    pub fn rec(&self, id: u64, by: u64, name: &str) -> Result<Args<'a>, StepError> {
        let e = self.get(id, by)?;
        match e.rec(name) {
            Some(rec) => Ok(Args { id, rec }),
            None => Err(StepError::WrongType { id, expected: name.to_owned(), found: e.display_name() }),
        }
    }

    /// The first record of entity `id` if its keyword is one of `names`.
    pub fn one_of(&self, id: u64, by: u64, names: &[&str]) -> Result<Args<'a>, StepError> {
        let e = self.get(id, by)?;
        for n in names {
            if let Some(rec) = e.rec(n) {
                return Ok(Args { id, rec });
            }
        }
        Err(StepError::WrongType { id, expected: names.join(" or "), found: e.display_name() })
    }
}

/// A record plus the id of its entity, for error messages.
#[derive(Clone, Copy)]
pub(crate) struct Args<'a> {
    pub id: u64,
    pub rec: &'a Rec,
}

impl<'a> Args<'a> {
    pub fn bad(&self, index: usize, msg: impl Into<String>) -> StepError {
        StepError::BadParam { id: self.id, entity: self.rec.name.clone(), index, msg: msg.into() }
    }

    pub fn param(&self, i: usize) -> Result<&'a Param, StepError> {
        self.rec.params.get(i).ok_or_else(|| self.bad(i, "missing parameter"))
    }

    pub fn real(&self, i: usize) -> Result<f64, StepError> {
        match self.param(i)? {
            Param::Real(v) => Ok(*v),
            Param::Int(v) => Ok(*v as f64),
            Param::Typed(_, inner) => match &**inner {
                Param::Real(v) => Ok(*v),
                Param::Int(v) => Ok(*v as f64),
                _ => Err(self.bad(i, "expected a real number")),
            },
            _ => Err(self.bad(i, "expected a real number")),
        }
    }

    pub fn finite(&self, i: usize) -> Result<f64, StepError> {
        let v = self.real(i)?;
        if v.is_finite() { Ok(v) } else { Err(self.bad(i, "number is not finite")) }
    }

    pub fn int(&self, i: usize) -> Result<i64, StepError> {
        match self.param(i)? {
            Param::Int(v) => Ok(*v),
            _ => Err(self.bad(i, "expected an integer")),
        }
    }

    pub fn reference(&self, i: usize) -> Result<u64, StepError> {
        match self.param(i)? {
            Param::Ref(v) => Ok(*v),
            _ => Err(self.bad(i, "expected an entity reference")),
        }
    }

    /// An optional reference: `$` and `*` give `None`.
    pub fn opt_ref(&self, i: usize) -> Result<Option<u64>, StepError> {
        match self.param(i)? {
            Param::Ref(v) => Ok(Some(*v)),
            Param::Null | Param::Derived => Ok(None),
            _ => Err(self.bad(i, "expected an entity reference or $")),
        }
    }

    pub fn list(&self, i: usize) -> Result<&'a [Param], StepError> {
        match self.param(i)? {
            Param::List(v) => Ok(v),
            _ => Err(self.bad(i, "expected a list")),
        }
    }

    pub fn string(&self, i: usize) -> String {
        match self.rec.params.get(i) {
            Some(Param::Str(s)) => s.clone(),
            _ => String::new(),
        }
    }

    /// An enumeration value (`T` for `.T.`), or a logical.
    pub fn enumeration(&self, i: usize) -> Result<&'a str, StepError> {
        match self.param(i)? {
            Param::Enum(s) => Ok(s),
            _ => Err(self.bad(i, "expected an enumeration value")),
        }
    }

    pub fn boolean(&self, i: usize) -> Result<bool, StepError> {
        match self.enumeration(i)? {
            "T" => Ok(true),
            "F" => Ok(false),
            other => Err(self.bad(i, format!("expected .T. or .F., found .{other}."))),
        }
    }

    /// A list of references.
    pub fn refs(&self, i: usize) -> Result<Vec<u64>, StepError> {
        self.list(i)?
            .iter()
            .map(|p| match p {
                Param::Ref(v) => Ok(*v),
                _ => Err(self.bad(i, "expected a list of entity references")),
            })
            .collect()
    }

    /// A list of reals.
    pub fn reals(&self, i: usize) -> Result<Vec<f64>, StepError> {
        self.list(i)?
            .iter()
            .map(|p| match p {
                Param::Real(v) => Ok(*v),
                Param::Int(v) => Ok(*v as f64),
                _ => Err(self.bad(i, "expected a list of real numbers")),
            })
            .collect()
    }

    /// A list of integers.
    pub fn ints(&self, i: usize) -> Result<Vec<i64>, StepError> {
        self.list(i)?
            .iter()
            .map(|p| match p {
                Param::Int(v) => Ok(*v),
                _ => Err(self.bad(i, "expected a list of integers")),
            })
            .collect()
    }
}
