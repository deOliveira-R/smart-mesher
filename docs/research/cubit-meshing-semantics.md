# Cubit meshing semantics, paraphrased, with smart-mesh decisions for the gaps

This is an offline reference for T3 (sm-sizing), T4 (sm-scheme-curve), T5/T6 (map) and T7 (sm-mesher).

- **Source:** Coreform Cubit manual, mesh_generation section (bias_dualbias, curvature, equal, stretch, pinpoint, mapping, submap, surface_vertex_types, meshing_the_geometry, interval_assignment/*, interval_matching). Read 2026-10-24.
- **Paraphrase only.** We match capabilities, not syntax.
- **Precedence:** where `docs/design/w2/plan.md` fixes a parameter set, threshold, objective or acceptance criterion (for example the T4 stretch params `{first_size, last_size?, max_ratio?}`, the T3 objective, or the picker thresholds), the plan wins. This document fills only what the plan leaves open.
- **Labels:**
  - **[Cubit]**: behavior the manual states.
  - **[Ours]**: our decision where the manual is silent. Implement it exactly and document it in rustdoc.

## 1. Curve schemes

### Equal
[Cubit] Equally spaced nodes; n comes from the curve's interval count or size.

### Bias
- [Cubit] Three quantities: interval count n, factor r (the ratio of successive edge lengths, so `e_{k+1} = r·e_k`; r may be above or below 1), and first edge a. The user gives any two and the third follows.
- [Cubit] The series starts at the curve's start vertex, or at a given start vertex. "Reverse bias" is the same as replacing r with 1/r.
- [Cubit] Allowed inputs:
  - n with a factor: a is derived.
  - n with a first size, or n with a fraction (`a = fraction·L`): r is derived.
  - Fine size with a factor, or fine size with a coarse size: n is derived.
- [Cubit] With no parameters, bias uses the vertex sizes, which are averaged from parents when not set directly.
- Formulas, for r ≠ 1; when r = 1, a = L/n:
  - `a = L(r−1)/(rⁿ−1)`
  - node `s_k = L(rᵏ−1)/(rⁿ−1)`, for k = 0..n
- [Ours] **Solving for r from (L, n, a):** if a·n = L then r = 1. Otherwise find r by bisection on (0, ∞) in log space, tolerance 1e-14 relative. The function is monotone in r. If no solution exists (a ≥ L with n > 1, or a ≤ 0), return a typed error.
- [Ours] **Solving for n from (L, a, r):** compute the real-valued n, round it to the nearest integer (minimum 1), then recompute a from (L, n, r) with r held fixed. Report the adjusted first size in diagnostics.
- [Ours] **Fine plus coarse:** use `r = (coarse/fine)^(1/(n−1))` together with the length constraint. Solve for n as `round(log(coarse/fine)/log(r)) + 1`, iterating to self-consistency: start from `n0 = round(L·2/(fine+coarse))`, compute r from n, and repeat until n stops changing (at most 8 iterations). Then fix a = fine and resolve r for the length.

### Dualbias
- [Cubit] Two geometric series start at both ends and meet in the middle.
- [Ours] **Even n = 2m:** both halves have m edges, `a·r^j` from each end, and `L = 2a(r^m−1)/(r−1)`.
- [Ours] **Odd n = 2m+1:** the middle edge is `a·r^m`, shared by both progressions, so the sequence mirrors around it. Then `L = 2a(r^m−1)/(r−1) + a·r^m`.
- The layout is symmetric in both cases.

### Stretch
- [Cubit] Takes first_size, optionally last_size (which requires first_size), or a stretch_factor alone (with no sizes). Interior edge lengths vary smoothly. It is not a strict geometric series. Sizes must be below L, and periodic curves are not supported.
- [Ours] With first size f and last size l and the given n: edge lengths follow linear interpolation `e_k = f + (l−f)·(k−1)/(n−1)`, scaled so they sum to L with endpoints pinned. If the scaling would change f or l by more than 1e-9 relative, choose n as `max(1, round(2L/(f+l)))` and re-solve instead. Only when n is hard, scale the interior.
- [Ours] With first size only: this is bias with that first size and the given n.
- [Ours] With a stretch factor s: this is bias with r = s.
- [Ours] Error on a periodic curve.

### Curvature
- [Cubit] A parameter c controls the adaptation:
  - c = 0: nearly equal intervals.
  - c > 0: the smallest edges are where curvature is largest.
  - c < 0: the largest edges are there.
- [Cubit] Lines and circular arcs come out near-uniform. The method is iterative and can fail to converge (c too large, or n too small). Periodic curves are not supported.
- [Ours] **Density:** `ρ(s) = (1 + |c|·L·κ(s))^{sign(c)}`, placing nodes so the integral of ρ between them is equal. κ(s) comes from 64·n samples, and placement uses trapezoid cumulative integration plus monotone inversion.
- [Ours] A constant κ gives an exactly uniform result, matching Cubit's arc behavior.
- [Ours] No iteration is needed, so convergence cannot fail. Still return an error for c with `|c|·L·κmax > 1e6`.

### Pinpoint
- [Cubit] Takes absolute arc-length positions from the start vertex, in any order.
- [Ours] Sort and deduplicate within the linear tolerance. Positions outside (0, L) are an error. n is the number of positions plus 1, and it overrides the interval count (hard).

## 2. Intervals

### Firmness
- [Cubit] There are three levels: default < soft < hard.
  - Every curve starts at default, and any interval command raises it to at least soft.
  - A command changes only curves whose firmness is at or below its own.
  - At the same level, the most recent command wins.
- [Cubit] An interval count set on a curve is hard. A count set on a surface, volume or body applies to the owned curves as soft. A count derived from a size is soft.

### Size to count
- [Cubit] Count = arc length / size. The manual does not give the rounding.
- [Ours] `n = max(1, round_half_even(L/size))`.

### Size inheritance
- [Cubit] A curve with no explicit size takes the average of its parents' sizes. Example: two volumes with sizes 1 and 3 give the shared face and its curves size 2. With no parents it uses auto size, and with auto off it uses 1.0.

### Auto size
- [Cubit] A factor from 1 (fine) to 10 (coarse), default 5, computed from all geometry in the model. It applies only when nothing explicit reaches the curve. The formula is not published.
- [Ours]
  - Let `d` = the bounding-box diagonal of the selected volumes, and `l_min` = the length of the shortest curve in them.
  - `auto_size(f) = clamp( d · 0.01 · 2^{(f−1)·0.5} , l_min/4 , d/2 )`.
    - f = 1 gives about d/100.
    - f = 5 gives about d/25.
    - f = 10 gives about d/4.5.
  - The result is deterministic and documented.
  - "Size smallest curve" = `l_min`, applied as soft.

### Vertex sizes
[Cubit] Vertex sizes are used only by biased curves. An unset vertex size is the average of its parents' sizes.

### Even
[Cubit] Forces an even count, raising an odd count by 1. "Odd" lifts the restriction.

### Extra constraints
[Cubit] Supported extras:
- lower and upper bounds per curve, and minimum and maximum per loop;
- linear relations between sums of curves, of the form ≥, ≤ or = plus an extra amount;
- "same" sets.

A constraint is honored only when its curves are part of the matching problem.

### Max arc span
[Cubit] For tri/tet with no user interval, an edge may span at most 60° of arc by default. This is deferred for W2.

## 3. Interval matching

- [Cubit] **Problem:** minimize f(x, goals) subject to Ax = b, lo ≤ x ≤ hi, with x integer. The constraints are:
  - map: opposite logical sides have equal sums;
  - any quad surface: an even boundary sum;
  - corner and vertex types;
  - sweep directions;
  - hard counts;
  - user constraints.

  The objective is the lexicographic min-max of max(achieved/goal, goal/achieved). Cubit's solver is the IIA (integer nullspace improvement); global optimality is not guaranteed.
- [Cubit] **Execution:**
  - Meshing runs matching first, globally over the listed entities by default ("individual" handles them one by one).
  - Lower-dimensional entities are included automatically; higher ones only when asked.
  - A diagnostic mode reports independent subproblems and the infeasible ones.
- [Ours] **Infeasible problems:** return a typed error that lists the infeasible subproblem's curves and the violated relation, for example "opposite sides 1 and 3 of surface 7: 4 hard ≠ 6 hard". Never relax hard counts silently.

## 4. Map

- [Cubit] **Surface:** four logical sides and four corners, where a side may span several curves and a corner may lie mid-curve. Opposite sides must match. The interior uses transfinite interpolation of the boundary and projects to the surface.
- [Cubit] **Volume:** six logical sides and eight corners; a side may span several surfaces.
- [Cubit] **Vertex types:**

  | Type | Elements at the vertex | Ideal angle |
  |---|---|---|
  | End | 1 | 90° |
  | Side | 2 | 180° |
  | Corner | 3 | 270° |
  | Reversal | 4 | 360° |

  Types are set per (surface, vertex) use, so the same vertex can differ between surfaces. Auto-picked types are soft and user-set types are hard. The picker weights cover corner angle, turn (how L-shaped the surface is) and interval change. Each weight is 0–10, default 1.
- [Ours]
  - **Auto-pick:** classify the interior angle θ to the nearest of 90/180/270/360°, using thresholds at the midpoints (135°, 225°, 315°).
  - **Map validity:** Σ(2 − elements at vertex) = 4 for a surface (End = +1, Side = 0, Corner = −1, Reversal = −2), which is the standard condition. When it fails, the angle picker adjusts the least-confident soft vertices, choosing by distance from the threshold with ties broken by ascending vertex id. If it still fails, it errors with the type counts.
- [Cubit] **Failure handling:** on a failure, Cubit continues meshing the remaining volumes by default ("continue meshing", on).
- [Ours] `mesh.run` fails atomically per op; partial success is a later option.

## 5. Precedence and bottom-up meshing

- [Cubit] Hard beats soft, which beats default. At the same level, the most recent command wins. An unset curve size is the average of its parents' sizes, then auto, then 1.0.
- [Cubit] Meshing an entity runs these steps:
  1. match intervals over it;
  2. mesh any unmeshed lower-dimensional entities;
  3. mesh the entity itself;
  4. check quality.

## Gaps left open

- Cubit's exact formulas for auto size, stretch and curvature.
- Its count rounding.
- Its default corner and end angle thresholds.

For all of these, our [Ours] choices above are binding for W2.
