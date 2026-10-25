//! [`AnglePicker`]: Cubit-style vertex types from interior angles, repaired to a corner rule
//! with the fewest changes.
//!
//! # Classification
//!
//! The interior angle `theta` of a face corner maps to a [`VertexType`] by [`classify_angle`]:
//! `End` for `theta <= 135 deg`, `Side` below `225 deg`, `Corner` below `315 deg`, `Reversal`
//! above. (`VertexType::nearest` differs only at exactly 225 and 315 deg, where it ties down.)
//!
//! # Rules
//!
//! * [`VertexRule::Free`]: classified types (a request replaces its corner's type); no sides.
//! * [`VertexRule::Map`] / [`VertexRule::Ends`]`(n)`: exactly `n` (4 for `Map`) `End` corners,
//!   every other corner `Side`; one loop.
//! * [`VertexRule::Submap`]: the corner sum `#End - #Corner - 2 #Reversal` is 4; one loop.
//!
//! # Repair
//!
//! Every corner starts at its classified type (change cost 1), at its requested type if a
//! request names it (cost 1 for a default-firmness request, 2 for a soft one), or is pinned (a
//! `Hard` request: it never changes). An exact dynamic program over the corner sum then picks the
//! final types minimizing, lexicographically, (1) the total change cost, (2) the total angular
//! distance `|theta - nominal(type)|`. Remaining ties go to the lowest vertex id (that corner is
//! the one adjusted; distances are quantized to 2^-24 rad so float noise cannot flip symmetric
//! ties), then to the type order `End < Side < Corner < Reversal`. So the picker
//! changes the fewest types, up to the weighting above (soft 2, other 1: the repair may change
//! two unrequested corners rather than one soft corner; see DESIGN.md), and among equal repairs
//! the least confident corner moves first.
//! Two `Hard` requests of different types on one corner are a conflict (an error naming both
//! types and the constraint ids), never last-wins.
//! A hard request wins over the rule; a hard type the rule forbids, or a rule that cannot be
//! met, is a [`SchemeError`] naming the face, the constraint ids and the hard requests.
//!
//! Logical sides are cut and oriented exactly as the reference picker does (starting after the
//! first `End`, or the first non-`Side` corner for `Submap`).

use core::f64::consts::FRAC_PI_4;
use sm_core::EntityId;
use sm_scheme_api::geom;
use sm_scheme_api::{
    ConstraintId, CornerAssignment, CornerPicker, FaceLayout, Firm, Firmness, Infeasible,
    LogicalSide, SchemeError, SideDir, VertexRule, VertexType,
};
use sm_topo_api::{CornerKey, TopologyView};

/// The vertex type of an interior angle (radians): `End` up to `3 pi/4`, `Side` below `5 pi/4`,
/// `Corner` below `7 pi/4`, `Reversal` above.
pub fn classify_angle(angle: f64) -> VertexType {
    if angle <= 3.0 * FRAC_PI_4 {
        VertexType::End
    } else if angle < 5.0 * FRAC_PI_4 {
        VertexType::Side
    } else if angle < 7.0 * FRAC_PI_4 {
        VertexType::Corner
    } else {
        VertexType::Reversal
    }
}

/// Cubit-style corner picker; see the [module docs](self).
#[derive(Clone, Copy, Debug, Default)]
pub struct AnglePicker;

/// Angular distance scale for the integer tie-break cost.
/// (2^24 per radian: coarse enough that float noise in corner angles cannot flip a tie.)
const DIST_SCALE: f64 = 16_777_216.0;
type Cost = (u64, u64);
const INF: Cost = (u64::MAX, u64::MAX);

fn add(a: Cost, b: Cost) -> Cost {
    if a == INF || b == INF { INF } else { (a.0 + b.0, a.1 + b.1) }
}

/// Starting state of one corner.
struct Start {
    ty: VertexType,
    weight: u64,
    hard: bool,
    req: Option<Firm<VertexType>>,
}

impl CornerPicker for AnglePicker {
    fn pick(
        &self,
        topo: &dyn TopologyView,
        face: EntityId,
        rule: VertexRule,
        requests: &[(CornerKey, Firm<VertexType>)],
        ids: &[ConstraintId],
    ) -> sm_scheme_api::Result<FaceLayout> {
        let corners = geom::corners(topo, face)?;
        let bad = |reason: String| -> SchemeError {
            Infeasible::new(reason, ids.to_vec(), vec![face]).into()
        };
        let structured = !matches!(rule, VertexRule::Free);
        let rings = corners.iter().map(|c| c.ring).max().map_or(0, |r| r + 1);
        if structured && rings != 1 {
            return Err(bad(format!("{rule:?} needs exactly one loop, face has {rings}")));
        }

        // Two hard requests with different types on one corner are a conflict, not last-wins.
        for (i, (k, t)) in requests.iter().enumerate() {
            if t.firmness != Firmness::Hard {
                continue;
            }
            if let Some((_, u)) = requests[..i]
                .iter()
                .find(|(k2, u)| k2 == k && u.firmness == Firmness::Hard && u.value != t.value)
            {
                let at = corners.iter().find(|c| c.key == *k).map_or_else(String::new, |c| format!(" at {}", c.vertex));
                return Err(bad(format!(
                    "conflicting hard vertex-type requests{at}: hard {:?} and hard {:?} on the same corner",
                    u.value, t.value
                )));
            }
        }

        // Starting state: the firmest request per corner (later wins ties), else the angle class.
        let starts: Vec<Start> = corners
            .iter()
            .map(|c| {
                let mut req: Option<Firm<VertexType>> = None;
                for (k, t) in requests {
                    if *k == c.key && req.is_none_or(|r| t.firmness >= r.firmness) {
                        req = Some(*t);
                    }
                }
                match req {
                    Some(r) => Start {
                        ty: r.value,
                        weight: if r.firmness == Firmness::Soft { 2 } else { 1 },
                        hard: r.firmness == Firmness::Hard,
                        req,
                    },
                    None => Start { ty: classify_angle(c.angle), weight: 1, hard: false, req: None },
                }
            })
            .collect();
        let hard_list = || {
            let mut v: Vec<String> = corners
                .iter()
                .zip(&starts)
                .filter(|(_, s)| s.hard)
                .map(|(c, s)| format!("hard {:?} at {}", s.ty, c.vertex))
                .collect();
            v.sort();
            v.join(", ")
        };

        let final_types: Vec<VertexType> = match rule {
            VertexRule::Free => starts.iter().map(|s| s.ty).collect(),
            VertexRule::Map | VertexRule::Ends(_) | VertexRule::Submap => {
                let (allowed, target): (&[VertexType], i64) = match rule {
                    VertexRule::Ends(n) => (&[VertexType::End, VertexType::Side], i64::from(n)),
                    VertexRule::Submap => (&VertexType::ALL, 4),
                    _ => (&[VertexType::End, VertexType::Side], 4),
                };
                for (c, s) in corners.iter().zip(&starts) {
                    if s.hard && !allowed.contains(&s.ty) {
                        return Err(bad(format!(
                            "hard {:?} at {} is not allowed by {rule:?}",
                            s.ty, c.vertex
                        )));
                    }
                }
                match repair(&corners, &starts, allowed, target) {
                    Some(t) => t,
                    None => {
                        let hards = hard_list();
                        return Err(bad(format!(
                            "{rule:?} cannot be met: {} corners, corner sum {target} needed; {}",
                            corners.len(),
                            if hards.is_empty() { "no hard requests".to_string() } else { hards }
                        )));
                    }
                }
            }
            other => return Err(bad(format!("corner rule {other:?} is not supported"))),
        };

        let out: Vec<CornerAssignment> = corners
            .iter()
            .zip(&starts)
            .zip(&final_types)
            .map(|((c, s), &t)| {
                let firmness = match s.req {
                    Some(r) if r.value == t => r.firmness,
                    _ => Firmness::Default,
                };
                CornerAssignment::new(c.key, t, firmness, c.angle)
            })
            .collect();
        let sides = if structured {
            sides_of(&out, matches!(rule, VertexRule::Submap))
        } else {
            Vec::new()
        };
        Ok(FaceLayout::new(face, rule, out, sides))
    }
}

/// Exact repair: final types over `allowed` with corner sum `target`, minimizing
/// (weighted changes, angular distance), ties to the lowest vertex id, then type order.
fn repair(
    corners: &[geom::Corner],
    starts: &[Start],
    allowed: &[VertexType],
    target: i64,
) -> Option<Vec<VertexType>> {
    let m = corners.len();
    let mut order: Vec<usize> = (0..m).collect();
    order.sort_by_key(|&i| (corners[i].vertex, i));
    let off = 2 * m as i64;
    let width = 3 * m + 1;
    let cost = |i: usize, t: VertexType| -> Cost {
        let s = &starts[i];
        let change = if t == s.ty { 0 } else { s.weight };
        let dist = ((corners[i].angle - t.nominal_angle()).abs() * DIST_SCALE).round() as u64;
        (change, dist)
    };
    let options = |i: usize| -> Vec<VertexType> {
        if starts[i].hard {
            vec![starts[i].ty]
        } else {
            allowed.to_vec()
        }
    };
    let mut best = vec![INF; (m + 1) * width];
    best[m * width + off as usize] = (0, 0);
    for k in (0..m).rev() {
        let i = order[k];
        let opts = options(i);
        for s in 0..width {
            let mut b = INF;
            for &t in &opts {
                let rem = s as i64 - i64::from(t.submap_weight());
                if (0..width as i64).contains(&rem) {
                    b = b.min(add(cost(i, t), best[(k + 1) * width + rem as usize]));
                }
            }
            best[k * width + s] = b;
        }
    }
    let mut s = target + off;
    if !(0..width as i64).contains(&s) || best[s as usize] == INF {
        return None;
    }
    let mut out = vec![VertexType::Side; m];
    for k in 0..m {
        let i = order[k];
        let mut pick: Option<((Cost, u8, usize), VertexType)> = None;
        for (rank, &t) in options(i).iter().enumerate() {
            let rem = s - i64::from(t.submap_weight());
            if !(0..width as i64).contains(&rem) {
                continue;
            }
            let total = add(cost(i, t), best[(k + 1) * width + rem as usize]);
            if total == INF {
                continue;
            }
            // Equal totals: prefer the corner that changes (lowest vertex id goes first).
            let key = (total, u8::from(t == starts[i].ty), rank);
            if pick.is_none_or(|p| key < p.0) {
                pick = Some((key, t));
            }
        }
        let (_, t) = pick?;
        out[i] = t;
        s -= i64::from(t.submap_weight());
    }
    Some(out)
}

/// Split the (single) loop into logical sides at `End` corners (or at every non-`Side` corner
/// when `any_turn`); directions follow the cumulative quarter turns. Same construction as the
/// reference picker.
fn sides_of(corners: &[CornerAssignment], any_turn: bool) -> Vec<LogicalSide> {
    let is_break = |c: &CornerAssignment| {
        if any_turn { c.vertex_type != VertexType::Side } else { c.vertex_type == VertexType::End }
    };
    let Some(first) = corners.iter().position(is_break) else {
        return Vec::new();
    };
    let n = corners.len();
    let mut sides = Vec::new();
    let mut cur = Vec::new();
    let mut dir = SideDir::PlusI;
    for s in 1..=n {
        let i = (first + s) % n;
        cur.push(corners[i].corner.incoming);
        if is_break(&corners[i]) {
            sides.push(LogicalSide::new(std::mem::take(&mut cur), dir));
            dir = dir.turned(if any_turn { corners[i].vertex_type.submap_weight() } else { 1 });
        }
    }
    sides
}
