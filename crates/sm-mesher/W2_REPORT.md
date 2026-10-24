# W2 report: sm-mesher (T7)

Branch: `claude/w2-sm-mesher-w8ijnt` (the session instruction named this branch; the task text
named `claude/w2-sm-mesher`. The session branch was used).

## Summary

`sm-mesher` implements the intent layer, the `mesh.*` ops, the `mesh.info` query, the
`MesherConfig` service, `RayonExec` and the mesh-driver node (`mesh.run`, `mesh.delete`) over
`sm-layer-api` (`insert_mesh`, `retire_mesh`, `MeshLayer`, `ModelLayer`) and the `sm-scheme-api`
`Driver`. It depends on no scheme crate and no sizing crate. All skeleton entry points
(`MeshPlugin`, `MesherConfig` and its builders) keep their signatures.

Additions beyond the plan text (both small, both justified):
* `mesh.default_size {size | function, params}`: the plan lists the `with_size_fields` hook but
  no op that could ever produce a `SizingSpec::Function`; this op sets `defaults.sizing`.
* `FirmnessArg` (`"default" | "soft" | "hard"`), the op-level spelling of firmness, and the
  firmness ladder with a `mesh.firmness_kept` warning (Cubit semantics).

## Acceptance bullet -> test

ACCEPTANCE_TABLE

## Deviations

* **mesh.size default firmness** is `default` (a real-valued goal), not Cubit's "soft" count; see
  DESIGN.md. `firmness: "soft"` gives the Cubit reading.
* **Plan acceptance wording "recorded in `MeshEntry.plan.attempts`"**: the driver records the
  failure that caused a re-plan in `plan.rejections` and the passed-over candidate in
  `plan.attempts`; the stored plan is unmodified (so it still equals a replay of
  `plan_with(rejections)`). Tests assert both.
* **Stub volume scheme**: plan says `"sweep"` with `source/target` params; that is what the tests
  use. (`"mapvol"` of the stubs only plans.)
* **Bench** uses the stub schemes, so its generate numbers are those of reference code.

## Known gaps

See DESIGN.md "Known gaps": no `on_succession` for the intent layer, fallbacks without
parameters, whole volumes only, neighbour guard by face adjacency only, no partial success.

## api_change_requests

None.

## Definition of done

DOD_TABLE
