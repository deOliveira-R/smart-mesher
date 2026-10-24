//! The surface `map` scheme. Skeleton: see W2 plan §2 task T5 (`docs/design/w2/plan.md`);
//! that task fills in the bodies.

use sm_scheme_api::SchemeRegistry;

/// Register the surface map scheme. Registers nothing yet.
pub fn register(_registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn register_is_callable() {
        let mut reg = sm_scheme_api::SchemeRegistry::new();
        super::register(&mut reg).expect("register");
    }
}
