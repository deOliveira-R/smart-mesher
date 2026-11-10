//! `source: model` exports: facets straight from the B-rep, no volume mesh. Only DAGMC `.h5m`
//! is written this way today.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value;
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_meshio::dagmc::{model_from_facets, write_h5m};
use sm_op_api::State;
use sm_protocol::ExportNote;
use sm_rt_api::FacetInput;
use sm_tess::{TessOptions, tessellate};
use sm_topo_api::{Region, TopologyViewExt};

use crate::error::SessionError;
use crate::mesh_export::{Written, dagmc_error, dagmc_options, notes_of};

/// Facet chord tolerance (model length units) when the `chord_tol` option is absent: the
/// `sm_tess` default. The value used is recorded in the file as `FACETING_TOL`.
pub(crate) const DEFAULT_CHORD_TOL: f64 = 1e-2;

/// What [`facets_of_model`] found besides the facets.
pub(crate) struct ModelFacets {
    pub facets: FacetInput,
    /// Volume to material name (the volume's color name).
    pub names: BTreeMap<EntityId, String>,
    /// Faces with the same volume (or none) on both sides, left out.
    pub skipped_faces: u64,
}

/// Tessellate every face of the model at `chord_tol` (the sagitta is the only sampling
/// control: the angle tolerance is loosened to pi). Triangles keep the face orientation, with
/// `front = left` and `back = right`, so a volume's triangles close with outward normals.
pub(crate) fn facets_of_model(model: &ModelLayer, chord_tol: f64) -> Result<ModelFacets, SessionError> {
    let m = model.model();
    if !m.instancing().instances.is_empty() {
        return Err(SessionError::new(
            "unsupported-instances",
            "`source: model` does not place instances yet: the model has instances (export a mesh instead)",
        ));
    }
    let topo = model.topo();
    let opts = TessOptions { chord_tol, angle_tol: core::f64::consts::PI, max_edge: f64::INFINITY };
    let (mut faces, mut skipped) = (Vec::new(), 0u64);
    let mut sides = Vec::new();
    for &f in topo.faces() {
        let s = topo.face_sides(f).ok_or_else(|| SessionError::internal(format!("face {f} has no sides")))?;
        if s.left == s.right {
            skipped += 1;
            continue;
        }
        faces.push(f);
        sides.push((s.left, s.right));
    }
    if faces.is_empty() {
        return Err(SessionError::new("no-model", "`source: model` found no faces bounding a volume"));
    }
    let t = tessellate(topo, &faces, &opts).map_err(|e| SessionError::new("tessellation-failed", e.to_string()))?;
    let mut fi = FacetInput::default();
    let mut map: BTreeMap<u32, u32> = BTreeMap::new();
    let mut vols: BTreeSet<EntityId> = BTreeSet::new();
    for (&face, &(left, right)) in faces.iter().zip(&sides) {
        let idx = t.face_indices(face).ok_or_else(|| SessionError::internal(format!("face {face} missing from the tessellation")))?;
        for tri in idx.as_chunks::<3>().0 {
            let mut out = [0u32; 3];
            for (k, &i) in tri.iter().enumerate() {
                out[k] = match map.get(&i) {
                    Some(&v) => v,
                    None => {
                        let v = u32::try_from(fi.vertices.len()).map_err(|_| SessionError::internal("too many vertices"))?;
                        let p = t.position(i);
                        fi.vertices.push([f64::from(p[0]), f64::from(p[1]), f64::from(p[2])]);
                        map.insert(i, v);
                        v
                    }
                };
            }
            fi.triangles.push(out);
            fi.front.push(left);
            fi.back.push(right);
            fi.face.push(face);
        }
        for r in [left, right] {
            if let Region::Volume(v) = r {
                vols.insert(v);
            }
        }
    }
    let mut names = BTreeMap::new();
    for v in vols {
        if let Some(row) = m.live_volume_color(v, topo).and_then(|c| m.color(c)) {
            names.insert(v, row.name.clone());
        }
    }
    Ok(ModelFacets { facets: fi, names, skipped_faces: skipped })
}

/// The `.h5m` of the model's B-rep facets. `options` are the `dagmc` options (`graveyard`,
/// `chord_tol`); the chord tolerance also becomes the file's `FACETING_TOL`.
pub(crate) fn write_dagmc(state: &State, options: Option<&Value>) -> Result<Written, SessionError> {
    let (o, chord) = dagmc_options(options, true)?;
    let chord = chord.unwrap_or(DEFAULT_CHORD_TOL);
    let model = state.layer::<ModelLayer>().ok_or_else(|| SessionError::new("no-model", "`source: model` needs the model layer"))?;
    let mf = facets_of_model(model, chord)?;
    let o = o.with_faceting_tol(chord);
    let (dm, loss) = model_from_facets(&mf.facets, &mf.names, &o).map_err(dagmc_error)?;
    let mut bytes = Vec::new();
    write_h5m(&dm, &o, &mut bytes).map_err(dagmc_error)?;
    let mut notes = notes_of(&loss);
    if mf.skipped_faces > 0 {
        notes.push(ExportNote::warning(
            "h5m.baffle_faces_dropped",
            mf.skipped_faces,
            "faces with one volume (or none) on both sides are not written",
        ));
    }
    Ok(Written { bytes, notes })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Server, standard_with};
    use serde_json::json;
    use sm_meshio::dagmc::{DagmcModel, DagmcOptions};
    use sm_protocol::Endpoint;

    const BRICKS: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body)
model.merge(entities: a.body | b.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "water")
"#;

    const PIN: &str = r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
fem.block_add(block: 1, volumes: fu.volume, name: "fuel")
fem.block_add(block: 2, volumes: mo.volumes, name: "moderator")
"#;

    fn server(script: &str) -> Server {
        let mut s = Server::new(standard_with(Some("tester".into()), Some("2026-10-24T00:00:00Z".into()))).expect("server");
        s.call(sm_protocol::Request::ScriptRun { text: script.into(), path: None, includes: Default::default() })
            .unwrap_or_else(|e| panic!("script: {e} {}", e.data));
        s
    }

    /// Signed volume of each volume of `m` from its oriented triangles (divergence theorem, f64).
    fn volumes_of(m: &DagmcModel) -> Vec<f64> {
        let mut out = vec![0.0; m.volumes.len()];
        for (t, &s) in m.triangles.iter().zip(&m.tri_surface) {
            let [a, b, c] = t.map(|i| m.vertices[i as usize]);
            let cross = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
            let det = (a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]) / 6.0;
            let sf = &m.surfaces[s as usize];
            if let Some(v) = sf.forward {
                out[v as usize] += det;
            }
            if let Some(v) = sf.backward {
                out[v as usize] -= det;
            }
        }
        out
    }

    fn model_of(s: &Server, tol: f64) -> (DagmcModel, ModelFacets) {
        let layer = s.workspace().state().layer::<ModelLayer>().expect("model layer");
        let mf = facets_of_model(layer, tol).expect("facets");
        let (m, _) = model_from_facets(&mf.facets, &mf.names, &DagmcOptions::default()).expect("closed model");
        (m, mf)
    }

    #[test]
    fn bricks_close_with_exact_volumes_and_names() {
        let s = server(BRICKS);
        let (m, mf) = model_of(&s, 0.01);
        assert_eq!(m.volumes.len(), 2);
        // 2 x 6 faces - 1 shared: 11 surfaces, 2 triangles each.
        assert_eq!((m.surfaces.len(), m.triangles.len()), (11, 22));
        for v in volumes_of(&m) {
            assert!((v - 8.0).abs() <= 8.0 * 1e-12, "{v}");
        }
        assert_eq!(m.groups.iter().map(|g| g.name.as_str()).collect::<Vec<_>>(), ["fuel", "water"]);
        assert_eq!(mf.names.len(), 2);
        // The shared face has both volumes on its sides.
        assert_eq!(m.surfaces.iter().filter(|x| x.forward.is_some() && x.backward.is_some()).count(), 1);
    }

    #[test]
    fn cylinder_is_below_the_analytic_volume_by_at_most_side_area_times_chord_tol() {
        let s = server(PIN);
        let (r, h) = (0.4_f64, 1.0_f64);
        let analytic = core::f64::consts::PI * r * r * h;
        let side = 2.0 * core::f64::consts::PI * r * h;
        let mut last = f64::INFINITY;
        for tol in [1e-2, 1e-3, 1e-4] {
            let (m, mf) = model_of(&s, tol);
            let vols = volumes_of(&m);
            let fuel = m.groups.iter().find(|g| g.name == "fuel").expect("fuel").volumes[0] as usize;
            let gap = analytic - vols[fuel];
            assert!(gap > 0.0 && gap <= side * tol, "tol {tol}: gap {gap} vs bound {}", side * tol);
            assert!(gap < last, "refining must reduce the gap");
            last = gap;
            // The moderator is the square minus the faceted fuel: its volume closes the cell.
            let moderator = m.groups.iter().find(|g| g.name == "moderator").expect("moderator").volumes[0] as usize;
            let cell = 1.26 * 1.26 * h;
            assert!((vols[fuel] + vols[moderator] - cell).abs() <= 1e-6 * cell, "{} {}", vols[fuel], vols[moderator]);
            assert_eq!(mf.skipped_faces, 0);
        }
    }

    #[test]
    fn written_bytes_are_deterministic_across_runs_and_thread_counts() {
        let s = server(PIN);
        let opts = json!({"graveyard": true, "chord_tol": 1e-3});
        let run = || write_dagmc(s.workspace().state(), Some(&opts)).expect("write").bytes;
        let a = run();
        assert_eq!(&a[..8], b"\x89HDF\r\n\x1a\n");
        assert_eq!(a, run());
        let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool").install(run);
        assert_eq!(a, one);
        // A finer tolerance gives a larger file.
        let fine = write_dagmc(s.workspace().state(), Some(&json!({"chord_tol": 1e-4}))).expect("fine").bytes;
        let coarse = write_dagmc(s.workspace().state(), Some(&json!({"chord_tol": 1e-2}))).expect("coarse").bytes;
        assert!(fine.len() > coarse.len());
    }
}
