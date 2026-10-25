//! The entity-kind alias table is part of the language's frozen surface.

use std::collections::BTreeSet;

use sm_core::EntityKind;
use sm_lang::kinds::*;

#[test]
fn table_covers_every_kind_exactly_once_in_order() {
    let kinds: Vec<EntityKind> = KIND_TABLE.iter().map(|r| r.kind).collect();
    assert_eq!(kinds, EntityKind::ALL, "KIND_TABLE must list EntityKind::ALL in order; a new kind needs a row");
}

#[test]
fn spellings_are_unique_and_do_not_collide_with_reserved_words() {
    // Script spellings (preferred, aliases, plurals) are unique across kinds. Canonical names are only a fallback in
    // `kind_from_singular`, and may be claimed as an alias of another kind (`Surface` is a face in scripts).
    let mut seen = BTreeSet::new();
    for r in KIND_TABLE {
        for s in std::iter::once(r.singular).chain(r.aliases.iter().copied()).chain(std::iter::once(r.plural)).chain(r.plural_aliases.iter().copied()) {
            assert!(seen.insert(s), "duplicate spelling {s}");
        }
        assert!(r.singular.chars().next().unwrap().is_ascii_uppercase(), "{}", r.singular);
        assert!(r.plural.chars().all(|c| c.is_ascii_lowercase() || c == '_'), "{}", r.plural);
    }
    // Plural keywords must not be reserved words (they are contextual) or builtin function names.
    let reserved = ["let", "set", "alias", "batch", "if", "else", "for", "in", "include", "print", "assert", "true", "false", "and", "or", "not", "all", "except", "help", "by"];
    for r in KIND_TABLE {
        assert!(!reserved.contains(&r.plural), "{}", r.plural);
        assert!(!sm_lang::eval::BUILTIN_NAMES.contains(&r.plural), "{} shadows a builtin", r.plural);
        let mut d = Vec::new();
        let toks = sm_lang::lexer::lex(0, r.plural, &mut d);
        assert!(d.is_empty() && toks[0].tok == sm_lang::lexer::Tok::Ident, "{} must lex as an identifier", r.plural);
    }
}

#[test]
fn alias_table_is_pinned() {
    let pairs = [
        ("Vertex", EntityKind::TopoVertex),
        ("Edge", EntityKind::TopoEdge),
        ("Face", EntityKind::TopoFace),
        ("Volume", EntityKind::TopoVolume),
        ("Body", EntityKind::TopoBody),
        ("Node", EntityKind::MeshVertex),
        ("Element", EntityKind::MeshCell),
        ("TopoVolume", EntityKind::TopoVolume),
        ("MeshCell", EntityKind::MeshCell),
        ("Color", EntityKind::Color),
        // Cubit vocabulary: a surface is a face and a curve is an edge; the geometry carriers are spelled out.
        ("Surface", EntityKind::TopoFace),
        ("Curve", EntityKind::TopoEdge),
        ("GeomSurface", EntityKind::Surface),
        ("GeomCurve", EntityKind::Curve),
    ];
    for (s, k) in pairs {
        assert_eq!(kind_from_singular(s), Some(k), "{s}");
    }
    let plurals = [("vertices", EntityKind::TopoVertex), ("faces", EntityKind::TopoFace), ("volumes", EntityKind::TopoVolume), ("bodies", EntityKind::TopoBody), ("nodes", EntityKind::MeshVertex), ("elements", EntityKind::MeshCell), ("surfaces", EntityKind::TopoFace), ("curves", EntityKind::TopoEdge), ("geom_surfaces", EntityKind::Surface), ("geom_curves", EntityKind::Curve)];
    for (s, k) in plurals {
        assert_eq!(kind_from_plural(s), Some(k), "{s}");
    }
    assert_eq!(kind_from_singular("Volumes"), None);
    assert_eq!(kind_from_singular("volume"), None, "typed ids are PascalCase");
}

#[test]
fn entity_text_round_trips_through_the_contract() {
    for &k in EntityKind::ALL {
        let id = sm_core::EntityId::new(k, 7).unwrap();
        // Canonical text and the alias text both parse to the same id.
        assert_eq!(parse_entity_text(&id.to_string()), Ok(id));
        assert_eq!(parse_entity_text(&format!("{}#7", singular_of(k))), Ok(id));
    }
    assert!(parse_entity_text("Volume").is_err());
    assert!(parse_entity_text("Nope#1").is_err());
    assert!(parse_entity_text("Volume#x").is_err());
}

#[test]
fn every_kind_stays_reachable_by_one_script_spelling() {
    for &k in EntityKind::ALL {
        assert_eq!(kind_from_singular(singular_of(k)), Some(k), "{k:?}");
        assert_eq!(kind_from_plural(plural_of(k).unwrap()), Some(k), "{k:?}");
    }
    // The canonical carrier names are claimed by the Cubit words, so only the explicit names reach them in source.
    assert_eq!(singular_of(EntityKind::Surface), "GeomSurface");
    assert_eq!(singular_of(EntityKind::TopoFace), "Face");
}

#[test]
fn script_singular_parses_to_the_same_kind_in_sm_core() {
    // Pins `singular_of` (what MCP respell shows) to `EntityKind::from_str` (what tool arguments accept).
    for &k in EntityKind::ALL {
        assert_eq!(singular_of(k).parse::<EntityKind>(), Ok(k), "{k}");
        let id = sm_core::EntityId::new(k, 3).unwrap();
        assert_eq!(parse_entity_text(&format!("{}#3", singular_of(k))), Ok(id));
        assert_eq!(format!("{}#3", singular_of(k)).parse::<sm_core::EntityId>(), Ok(id));
    }
}
