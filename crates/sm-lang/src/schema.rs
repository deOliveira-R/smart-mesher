//! A small JSON-Schema reader and checker over the schemas the op registry publishes.
//!
//! It interprets exactly what `schemars` 1 emits for op parameter types (`type`, `properties`,
//! `required`, `items`, `enum`/`oneOf`-of-`const`, `anyOf` for options, `$ref` into `$defs`, bounds) and the three string
//! patterns of the contract (entity id, batch reference, `$$` escape). Anything else passes: the op's own decoder stays the authority.

use serde_json::{Map, Value as Json};

/// A schema document (the root, so `$ref` can be resolved).
#[derive(Clone, Debug)]
pub struct SchemaDoc {
    root: Json,
}

impl SchemaDoc {
    /// Wrap a schema value.
    pub fn new(root: Json) -> Self {
        Self { root }
    }

    /// The root view.
    pub fn view(&self) -> View<'_> {
        View { root: &self.root, node: &self.root, site: &self.root }.resolve()
    }

    /// The raw schema.
    pub fn raw(&self) -> &Json {
        &self.root
    }
}

/// A node of a schema document.
#[derive(Clone, Copy, Debug)]
pub struct View<'a> {
    root: &'a Json,
    node: &'a Json,
    /// The node as written, before `$ref` was followed: annotations such as `x-entity-kind` sit beside the `$ref`.
    site: &'a Json,
}

/// One segment of a path into a value.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PathSeg {
    /// Object key.
    Key(String),
    /// Array index.
    Idx(usize),
}

/// A schema violation.
#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    /// Where in the value.
    pub path: Vec<PathSeg>,
    /// Stable code.
    pub code: &'static str,
    /// Message.
    pub message: String,
}

/// Static facts about one parameter, for help and completion.
#[derive(Clone, Debug, PartialEq)]
pub struct ParamInfo {
    /// Property name.
    pub name: String,
    /// In the schema's `required` list.
    pub required: bool,
    /// Human type, e.g. `number`, `[id]`, `"a" | "b"`.
    pub ty: String,
    /// Doc comment from the schema.
    pub description: Option<String>,
    /// Declared default.
    pub default: Option<Json>,
    /// Allowed values when the type is an enum, rendered.
    pub choices: Vec<String>,
    /// True when the value is an entity id or an array of ids.
    pub is_entity: bool,
    /// Entity kinds the id may have (`x-entity-kind`); empty when unrestricted or not an id.
    pub kinds: Vec<sm_core::EntityKind>,
}

impl<'a> View<'a> {
    /// Follow `$ref` chains (local refs only).
    pub fn resolve(self) -> View<'a> {
        let mut v = self;
        for _ in 0..32 {
            let Some(r) = v.node.get("$ref").and_then(Json::as_str) else { break };
            let Some(target) = r.strip_prefix('#').and_then(|p| if p.is_empty() { Some(v.root) } else { v.root.pointer(p) }) else { break };
            v = View { root: v.root, node: target, site: v.site };
        }
        v
    }

    fn sub(self, node: &'a Json) -> View<'a> {
        View { root: self.root, node, site: node }.resolve()
    }

    fn get(self, key: &str) -> Option<&'a Json> {
        self.node.get(key)
    }

    /// `description`.
    pub fn description(self) -> Option<&'a str> {
        self.get("description").and_then(Json::as_str)
    }

    /// Alternatives of `anyOf`/`oneOf`, resolved.
    pub fn alternatives(self) -> Vec<View<'a>> {
        let mut out = Vec::new();
        for k in ["anyOf", "oneOf"] {
            if let Some(a) = self.get(k).and_then(Json::as_array) {
                out.extend(a.iter().map(|n| self.sub(n)));
            }
        }
        out
    }

    fn is_null_schema(self) -> bool {
        match self.get("type") {
            Some(Json::String(s)) => s == "null",
            _ => self.get("const").is_some_and(Json::is_null),
        }
    }

    /// Strip the `null` alternative of an optional (`anyOf [X, null]`) and unwrap a lone remaining alternative.
    pub fn non_null(self) -> View<'a> {
        let alts = self.alternatives();
        if alts.is_empty() {
            return self;
        }
        let rest: Vec<View<'a>> = alts.into_iter().filter(|a| !a.is_null_schema()).collect();
        if rest.len() == 1 { rest[0].non_null() } else { self }
    }

    /// Declared types (`null` removed).
    pub fn types(self) -> Vec<&'a str> {
        match self.get("type") {
            Some(Json::String(s)) => vec![s.as_str()],
            Some(Json::Array(a)) => a.iter().filter_map(Json::as_str).filter(|s| *s != "null").collect(),
            _ => Vec::new(),
        }
    }

    /// `properties`.
    pub fn properties(self) -> Option<&'a Map<String, Json>> {
        self.get("properties").and_then(Json::as_object)
    }

    /// The view of one property.
    pub fn property(self, name: &str) -> Option<View<'a>> {
        self.properties().and_then(|p| p.get(name)).map(|n| self.sub(n))
    }

    /// `required` names.
    pub fn required(self) -> Vec<&'a str> {
        self.get("required").and_then(Json::as_array).map(|a| a.iter().filter_map(Json::as_str).collect()).unwrap_or_default()
    }

    /// `items` as a view.
    pub fn items(self) -> Option<View<'a>> {
        self.get("items").filter(|i| i.is_object()).map(|n| self.sub(n))
    }

    /// Allowed constant values (`enum`, `const`, or `oneOf`/`anyOf` whose alternatives are all constants).
    pub fn choices(self) -> Vec<Json> {
        if let Some(e) = self.get("enum").and_then(Json::as_array) {
            return e.clone();
        }
        if let Some(c) = self.get("const") {
            return vec![c.clone()];
        }
        let alts = self.alternatives();
        if !alts.is_empty() && alts.iter().all(|a| a.get("const").is_some() || a.get("enum").is_some()) {
            return alts.iter().flat_map(|a| a.choices()).collect();
        }
        Vec::new()
    }

    /// The `x-entity-kind` annotation of an id property, as written (canonical kind names). It sits on the property node;
    /// for `Option<EntityId>` that is the wrapper, for `Vec<EntityId>` the array (the contract says it applies to the items).
    pub fn entity_kind_names(self) -> Option<Vec<&'a str>> {
        let from = |v: View<'a>| v.site.get(sm_op_api::X_ENTITY_KIND).or_else(|| v.get(sm_op_api::X_ENTITY_KIND)).and_then(Json::as_array).map(|a| a.iter().filter_map(Json::as_str).collect::<Vec<_>>());
        let nn = self.non_null();
        from(self).or_else(|| from(nn)).or_else(|| nn.items().and_then(from)).or_else(|| self.alternatives().into_iter().find_map(from))
    }

    /// True if the schema is a single entity id (pattern or an alternative of ids).
    pub fn is_entity_id(self) -> bool {
        let v = self.non_null();
        if v.get("pattern").and_then(Json::as_str) == Some(sm_core::ENTITY_ID_PATTERN) {
            return true;
        }
        let alts = v.alternatives();
        !alts.is_empty() && alts.iter().any(|a| a.get("pattern").and_then(Json::as_str) == Some(sm_core::ENTITY_ID_PATTERN))
    }

    /// If the schema is an array of entity ids, the item view.
    pub fn id_array_items(self) -> Option<View<'a>> {
        let v = self.non_null();
        if v.types().contains(&"array") || v.get("items").is_some() {
            v.items().filter(|i| i.is_entity_id())
        } else {
            None
        }
    }

    /// True when `type` includes `number` but not `integer`.
    pub fn is_float_only(self) -> bool {
        let t = self.non_null().types();
        t.contains(&"number") && !t.contains(&"integer")
    }

    /// Human-readable type.
    pub fn describe(self) -> String {
        let v = self.non_null();
        let ch = v.choices();
        if !ch.is_empty() {
            return ch.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(" | ");
        }
        if v.is_entity_id() {
            return "id".into();
        }
        if let Some(i) = v.id_array_items() {
            let _ = i;
            return "[id]".into();
        }
        let types = v.types();
        if types.contains(&"array") {
            return match v.items() {
                Some(i) => format!("[{}]", i.describe()),
                None => "list".into(),
            };
        }
        if !types.is_empty() {
            return types.iter().map(|t| if *t == "boolean" { "bool" } else { t }).collect::<Vec<_>>().join(" | ");
        }
        let alts = v.alternatives();
        if !alts.is_empty() {
            return alts.iter().map(|a| a.describe()).collect::<Vec<_>>().join(" | ");
        }
        "any".into()
    }

    /// The parameters of an object schema, in `required`-first then alphabetical order.
    pub fn params(self) -> Vec<ParamInfo> {
        let v = self.non_null();
        let req = v.required();
        let mut out: Vec<ParamInfo> = v
            .properties()
            .map(|p| {
                p.iter()
                    .map(|(name, node)| {
                        let pv = v.sub(node);
                        let nn = pv.non_null();
                        ParamInfo {
                            name: name.clone(),
                            required: req.contains(&name.as_str()),
                            ty: pv.describe(),
                            description: node.get("description").and_then(Json::as_str).or_else(|| nn.description()).map(str::to_owned),
                            default: node.get("default").cloned().or_else(|| nn.get("default").cloned()),
                            choices: nn.choices().iter().map(|c| c.to_string()).collect(),
                            is_entity: nn.is_entity_id() || nn.id_array_items().is_some(),
                            kinds: pv.entity_kind_names().map(|ns| ns.iter().filter_map(|n| n.parse().ok()).collect()).unwrap_or_default(),
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
        out.sort_by(|a, b| b.required.cmp(&a.required).then_with(|| a.name.cmp(&b.name)));
        out
    }
}

fn kind_of(v: &Json) -> &'static str {
    match v {
        Json::Null => "null",
        Json::Bool(_) => "boolean",
        Json::Number(n) if n.is_i64() || n.is_u64() => "integer",
        Json::Number(_) => "number",
        Json::String(_) => "string",
        Json::Array(_) => "array",
        Json::Object(_) => "object",
    }
}

/// True if `s` matches `^[A-Za-z]+#[0-9]+$`.
pub fn is_id_text(s: &str) -> bool {
    let Some((k, n)) = s.split_once('#') else { return false };
    !k.is_empty() && k.bytes().all(|b| b.is_ascii_alphabetic()) && !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit())
}

/// `Some(matches)` for the patterns the contract defines, `None` for any other pattern (not interpreted).
fn pattern_matches(pat: &str, s: &str) -> Option<bool> {
    if pat == sm_core::ENTITY_ID_PATTERN {
        Some(is_id_text(s))
    } else if pat == sm_op_api::batch::REF_PATTERN {
        Some(sm_op_api::batch::is_reference(s))
    } else if pat == sm_op_api::batch::ESCAPED_LITERAL_PATTERN {
        Some(s.starts_with("$$"))
    } else {
        None
    }
}

/// Check `value` against `view`, appending issues. `top` makes unknown properties an error at the root object even
/// when the schema does not set `additionalProperties: false` (typo protection for call arguments).
pub fn check(view: View<'_>, value: &Json, top: bool, out: &mut Vec<Issue>) {
    let mut path = Vec::new();
    check_at(view, value, top, false, &mut path, out, 0);
}

/// Like [`check`] for a member of a `core.batch`: wherever the schema wants an entity id, a batch reference
/// (`$i.role`, `$i.role[k]`, `$i.role[*]`) is accepted too, as `Registry::batch_schema` allows.
pub fn check_in_batch(view: View<'_>, value: &Json, top: bool, out: &mut Vec<Issue>) {
    let mut path = Vec::new();
    check_at(view, value, top, true, &mut path, out, 0);
}

fn push(out: &mut Vec<Issue>, path: &[PathSeg], code: &'static str, message: String) {
    out.push(Issue { path: path.to_vec(), code, message });
}

fn check_at(view: View<'_>, value: &Json, top: bool, batch: bool, path: &mut Vec<PathSeg>, out: &mut Vec<Issue>, depth: usize) {
    if depth > 64 {
        return;
    }
    let v = view.resolve();
    // Plan-mode placeholders stand for unknown ids: accepted where an id is expected.
    if let Json::String(s) = value
        && s.starts_with("$plan.")
        && v.non_null().is_entity_id()
    {
        return;
    }
    // Optional: `anyOf [X, null]` checks as X.
    let nn = v.non_null();
    let alts = nn.alternatives();
    let choices = nn.choices();
    if !choices.is_empty() {
        if !choices.contains(value) {
            let list = choices.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(", ");
            push(out, path, "enum-violation", format!("{value} is not one of {list}"));
        }
        return;
    }
    if !alts.is_empty() {
        let mut best: Option<Vec<Issue>> = None;
        for a in &alts {
            if a.is_null_schema() {
                continue;
            }
            let mut sub = Vec::new();
            check_at(*a, value, top, batch, path, &mut sub, depth + 1);
            if sub.is_empty() {
                return;
            }
            if best.as_ref().is_none_or(|b| sub.len() < b.len()) {
                best = Some(sub);
            }
        }
        match best {
            Some(b) if alts.len() == 1 || b.len() == 1 => out.extend(b),
            Some(_) | None => push(out, path, "type-mismatch", format!("{} does not match any allowed form ({})", kind_of(value), nn.describe())),
        }
        return;
    }
    if let Some(all) = nn.get("allOf").and_then(Json::as_array) {
        for a in all {
            check_at(nn.sub(a), value, top, batch, path, out, depth + 1);
        }
    }
    if let Some(c) = nn.get("const")
        && c != value
    {
        push(out, path, "enum-violation", format!("expected {c}"));
        return;
    }
    let types = nn.types();
    if !types.is_empty() {
        let k = kind_of(value);
        let ok = types.iter().any(|t| *t == k || (*t == "number" && k == "integer"));
        if !ok {
            let want = types.iter().map(|t| if *t == "boolean" { "bool" } else { t }).collect::<Vec<_>>().join(" or ");
            let hint = if types.contains(&"integer") && k == "number" { " (use `int(..)`, `round(..)` or an integer literal)" } else { "" };
            push(out, path, "type-mismatch", format!("expected {want}, found {k}{hint}"));
            return;
        }
    }
    match value {
        Json::Number(n) => {
            if let Some(f) = n.as_f64() {
                let bound = |k: &str| nn.get(k).and_then(Json::as_f64);
                if let Some(m) = bound("minimum").filter(|m| f < *m) {
                    push(out, path, "out-of-range", format!("{n} is below the minimum {m}"));
                }
                if let Some(m) = bound("maximum").filter(|m| f > *m) {
                    push(out, path, "out-of-range", format!("{n} is above the maximum {m}"));
                }
                if let Some(m) = bound("exclusiveMinimum").filter(|m| f <= *m) {
                    push(out, path, "out-of-range", format!("{n} must be greater than {m}"));
                }
                if let Some(m) = bound("exclusiveMaximum").filter(|m| f >= *m) {
                    push(out, path, "out-of-range", format!("{n} must be less than {m}"));
                }
            }
        }
        Json::String(s) => {
            let len = s.chars().count() as u64;
            if let Some(m) = nn.get("minLength").and_then(Json::as_u64).filter(|m| len < *m) {
                push(out, path, "out-of-range", format!("string is shorter than {m} characters"));
            }
            if let Some(m) = nn.get("maxLength").and_then(Json::as_u64).filter(|m| len > *m) {
                push(out, path, "out-of-range", format!("string is longer than {m} characters"));
            }
            if let Some(pat) = nn.get("pattern").and_then(Json::as_str)
                && pattern_matches(pat, s) == Some(false)
                && !(batch && pat == sm_core::ENTITY_ID_PATTERN && (sm_op_api::batch::is_reference(s) || s.starts_with("$$")))
            {
                let msg = if pat == sm_core::ENTITY_ID_PATTERN { format!("{s:?} is not an entity id (expected `Kind#serial`)") } else { format!("{s:?} does not match {pat}") };
                push(out, path, "pattern-mismatch", msg);
            }
        }
        Json::Array(a) => {
            let n = a.len() as u64;
            if let Some(m) = nn.get("minItems").and_then(Json::as_u64).filter(|m| n < *m) {
                push(out, path, "out-of-range", format!("expected at least {m} items, found {n}"));
            }
            if let Some(m) = nn.get("maxItems").and_then(Json::as_u64).filter(|m| n > *m) {
                push(out, path, "out-of-range", format!("expected at most {m} items, found {n}"));
            }
            let prefix = nn.get("prefixItems").and_then(Json::as_array);
            for (i, el) in a.iter().enumerate() {
                path.push(PathSeg::Idx(i));
                if let Some(p) = prefix.and_then(|p| p.get(i)) {
                    check_at(nn.sub(p), el, false, batch, path, out, depth + 1);
                } else if let Some(items) = nn.items() {
                    check_at(items, el, false, batch, path, out, depth + 1);
                }
                path.pop();
            }
        }
        Json::Object(m) => {
            for r in nn.required() {
                if !m.contains_key(r) {
                    push(out, path, "missing-property", format!("missing required field `{r}`"));
                }
            }
            let props = nn.properties();
            let closed = nn.get("additionalProperties") == Some(&Json::Bool(false)) || (top && props.is_some());
            for (k, el) in m {
                path.push(PathSeg::Key(k.clone()));
                if let Some(node) = props.and_then(|p| p.get(k)) {
                    check_at(nn.sub(node), el, false, batch, path, out, depth + 1);
                } else if closed {
                    push(out, path, "unknown-property", format!("unknown field `{k}`"));
                } else if let Some(ap) = nn.get("additionalProperties").filter(|a| a.is_object()) {
                    check_at(nn.sub(ap), el, false, batch, path, out, depth + 1);
                }
                path.pop();
            }
        }
        _ => {}
    }
}
