//! Region (material) names, ruling C9.

use crate::ImportError;
use crate::ir::Material;
use std::collections::{BTreeMap, BTreeSet};

/// Name of the region of cells with no material.
pub const VOID_REGION: &str = "void";

/// The region name of each material.
///
/// A material is named by its deck `name` when that is non-empty, held by no other material,
/// not `void`, and not the fallback name of another material; otherwise `material_<id>`.
/// `overrides` rename materials explicitly and win over the rules. One region per material:
/// two materials that end up with one name are an error.
///
/// # Errors
/// [`ImportError::UnknownMaterial`] for an override of an undefined material and
/// [`ImportError::Invalid`] for an empty or `void` override or a name two materials share.
pub fn resolve_region_names(materials: &BTreeMap<u32, Material>, overrides: &BTreeMap<u32, String>) -> Result<BTreeMap<u32, String>, ImportError> {
    let named = |m: &Material| m.name.as_deref().filter(|n| !n.trim().is_empty()).map(str::to_owned);
    let mut count: BTreeMap<String, usize> = BTreeMap::new();
    for m in materials.values() {
        if let Some(n) = named(m) {
            *count.entry(n).or_default() += 1;
        }
    }
    let fallback = |id: u32| format!("material_{id}");
    let fallbacks: BTreeSet<String> = materials.keys().map(|k| fallback(*k)).collect();
    let mut out = BTreeMap::new();
    for (id, m) in materials {
        let own_fallback = fallback(*id);
        let name = match named(m) {
            Some(n) if count[&n] == 1 && n != VOID_REGION && (n == own_fallback || !fallbacks.contains(&n)) => n,
            _ => own_fallback,
        };
        out.insert(*id, name);
    }
    for (id, name) in overrides {
        if !materials.contains_key(id) {
            return Err(ImportError::UnknownMaterial { cell: 0, material: *id });
        }
        if name.trim().is_empty() || name == VOID_REGION {
            return Err(ImportError::Invalid(format!("the name override {name:?} for material {id} is not a usable region name")));
        }
        out.insert(*id, name.clone());
    }
    let mut seen: BTreeMap<&String, u32> = BTreeMap::new();
    for (id, n) in &out {
        if let Some(prev) = seen.insert(n, *id) {
            return Err(ImportError::Invalid(format!("materials {prev} and {id} would share the region name {n:?}")));
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mats(v: &[(u32, Option<&str>)]) -> BTreeMap<u32, Material> {
        v.iter().map(|(i, n)| (*i, Material { name: n.map(str::to_owned) })).collect()
    }

    #[test]
    fn unique_names_win_and_the_rest_fall_back() {
        let m = mats(&[(1, Some("fuel")), (2, Some("")), (3, Some("water")), (4, Some("water")), (5, None), (6, Some("void")), (7, Some("material_5"))]);
        let n = resolve_region_names(&m, &BTreeMap::new()).unwrap();
        let got: Vec<&str> = n.values().map(String::as_str).collect();
        assert_eq!(got, ["fuel", "material_2", "material_3", "material_4", "material_5", "material_6", "material_7"]);
    }

    #[test]
    fn overrides_win_and_must_stay_unique() {
        let m = mats(&[(1, Some("fuel")), (2, None)]);
        let o: BTreeMap<u32, String> = [(2, "water".to_string())].into();
        assert_eq!(resolve_region_names(&m, &o).unwrap()[&2], "water");
        let dup: BTreeMap<u32, String> = [(2, "fuel".to_string())].into();
        assert!(resolve_region_names(&m, &dup).is_err());
        let unknown: BTreeMap<u32, String> = [(9, "x".to_string())].into();
        assert!(matches!(resolve_region_names(&m, &unknown), Err(ImportError::UnknownMaterial { .. })));
    }
}
