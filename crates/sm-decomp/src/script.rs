//! Command-language text for a plan.

use crate::plan::{CutKind, CutStep, Target};
use std::fmt::Write;

fn f(x: f64) -> String {
    format!("{:?}", x + 0.0)
}
fn v(a: [f64; 3]) -> String {
    format!("[{}, {}, {}]", f(a[0]), f(a[1]), f(a[2]))
}

/// Render `steps` as script lines, one `let <prefix><n> = model.webcut_...(...)` per step.
///
/// `root` is the script expression for a [`Target::Id`] (the planned volume); a step reference
/// becomes `<prefix><n>.<role>`. The text is what `hex_only` quotes in its error message and
/// what a user pastes into a script.
pub fn render_script(steps: &[CutStep], root: &str, prefix: &str) -> String {
    let mut out = String::new();
    for s in steps {
        let targets: Vec<String> = s
            .targets
            .iter()
            .map(|t| match t {
                Target::Id { .. } => root.to_owned(),
                Target::Step { step, role } => format!("{prefix}{step}.{}", role.as_str()),
            })
            .collect();
        let t = targets.join(", ");
        let _ = match &s.cut {
            CutKind::Cylinder {
                radius,
                axis,
                center,
            } => {
                writeln!(
                    out,
                    "let {prefix}{} = model.webcut_cylinder(targets: {t}, radius: {}, axis: {}, center: {})",
                    s.step,
                    f(*radius),
                    v(*axis),
                    v(*center)
                )
            }
            CutKind::Plane { plane } => writeln!(
                out,
                "let {prefix}{} = model.webcut_plane(targets: {t}, plane: {{origin: {}, normal: {}}})",
                s.step,
                v(plane.origin),
                v(plane.normal)
            ),
        };
    }
    out
}
