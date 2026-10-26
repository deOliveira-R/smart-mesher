# sm-smooth

Classification-constrained mesh smoothing: Laplacian, smart Laplacian, centroid-area,
Winslow (2-D quads) and mean-ratio / condition optimisation. Nodes stay on their curves and
surfaces, no cell is made worse than the guard allows, and results are bit-identical for any
thread count. See `DESIGN.md` for the formulas and `W3_REPORT.md` for the acceptance table.

```rust,ignore
let input = SmoothInput { mesh: &mesh, topo: &topo, model: &model };
let outcome = smooth(&input, &SmoothRequest::default())?;
apply_outcome(&mut editable_mesh, &outcome)?;
```

`SmoothPlugin` registers the `mesh.smooth` op and the `mesh.smooth_preview` query (install
`CoreLayers` first; it writes only the `meshes` layer).

## Known gaps / deferred to W3b

- 3-D Winslow, equipotential, untangle, randomize.
- Analytic gradients or Hessians (Newton / feasible Newton). `Optimize` uses finite differences.
- Polyhedral cells and high-order meshes (polyhedra are frozen, high-order is `Unsupported`).
- Parallel colouring (it is sequential, `O(sum of stencil sizes)`).
- Targeting by stored mesh set or group (`NodeSelect::Vertices` meanwhile).
- Surface-feature preservation by dihedral angle (classification already gives features).
- Perf gates that depend on the bulk mesh commit (record-only here).
- Tied smoothing of translation / rotation deck pairs (W4): those nodes are pinned.
