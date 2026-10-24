//! The `view.tessellate` query and its wire encoding.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_op_api::{OpError, QueryCtx, State, TypedQuery, names};

use crate::options::TessOptions;
use crate::tess::{self, Tessellation};

/// Chord tolerance used when none is given, relative to the scene's bounding-box diagonal.
pub const DEFAULT_RELATIVE_CHORD_TOL: f64 = 1e-3;

/// Input of `view.tessellate`.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TessellateQuery {
    /// Faces, volumes, bodies (expanded to their faces) and edges (polyline only) to tessellate.
    #[schemars(extend("x-entity-kind" = ["TopoFace", "TopoVolume", "TopoBody", "TopoEdge"]))]
    pub entities: Vec<EntityId>,
    /// Chord tolerance in model length units; default `1e-3` of the scene's bounding-box diagonal.
    #[serde(default)]
    pub chord_tol: Option<f64>,
    /// Angle tolerance in radians, `(0, pi]`; default 0.3.
    #[serde(default)]
    pub angle_tol: Option<f64>,
    /// Longest segment in model length units; default none.
    #[serde(default)]
    pub max_edge: Option<f64>,
}

/// Triangle range of one face, as sent on the wire.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct FaceRangeOut {
    /// The face.
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub face: EntityId,
    /// Offset into `indices` (in indices, a multiple of 3).
    pub first_index: u32,
    /// Number of indices (a multiple of 3).
    pub index_count: u32,
}

/// Polyline range of one edge, as sent on the wire.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct EdgeRangeOut {
    /// The edge.
    #[schemars(extend("x-entity-kind" = ["TopoEdge"]))]
    pub edge: EntityId,
    /// Offset into `edge_indices` (in vertices).
    pub first: u32,
    /// Number of polyline vertices.
    pub count: u32,
}

/// Output of `view.tessellate`: the [`Tessellation`] with its buffers base64-encoded.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct TessellateReply {
    /// Number of vertices.
    pub vertex_count: u32,
    /// Number of triangles.
    pub triangle_count: u32,
    /// Base64 of `vertex_count * 3` little-endian `f32` (`x, y, z` per vertex).
    pub positions: String,
    /// Base64 of `triangle_count * 3` little-endian `u32` vertex indices.
    pub indices: String,
    /// Triangle ranges per face, ascending by face id.
    pub faces: Vec<FaceRangeOut>,
    /// Base64 of little-endian `u32` polyline vertex indices.
    pub edge_indices: String,
    /// Polyline ranges per edge, ascending by edge id.
    pub edges: Vec<EdgeRangeOut>,
    /// Chord tolerance that was used.
    pub chord_tol: f64,
    /// Angle tolerance that was used.
    pub angle_tol: f64,
    /// Maximum segment length that was used (`null` for none).
    pub max_edge: Option<f64>,
}

impl TessellateReply {
    /// Encode a tessellation.
    pub fn new(t: &Tessellation, o: &TessOptions) -> Self {
        let f32s: Vec<u8> = t.positions.iter().flat_map(|x| x.to_le_bytes()).collect();
        let u32s = |v: &[u32]| -> String { base64(&v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>()) };
        Self {
            vertex_count: t.vertex_count() as u32,
            triangle_count: t.triangle_count() as u32,
            positions: base64(&f32s),
            indices: u32s(&t.indices),
            faces: t.faces.iter().map(|r| FaceRangeOut { face: r.face, first_index: r.first_index, index_count: r.index_count }).collect(),
            edge_indices: u32s(&t.edge_indices),
            edges: t.edges.iter().map(|r| EdgeRangeOut { edge: r.edge, first: r.first, count: r.count }).collect(),
            chord_tol: o.chord_tol,
            angle_tol: o.angle_tol,
            max_edge: o.max_edge.is_finite().then_some(o.max_edge),
        }
    }
}

impl TypedQuery for TessellateQuery {
    const NAME: &'static str = "view.tessellate";
    type Output = TessellateReply;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<TessellateReply, OpError> {
        let model = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(names::MODEL.to_owned()))?;
        let view = model.topo();
        cx.step(0, 2, "expanding entities")?;
        let (faces, edges) = tess::expand(view, &self.entities).map_err(|e| OpError::Invalid(e.to_string()))?;
        let mut opts = TessOptions::default();
        opts.chord_tol = match self.chord_tol {
            Some(c) => c,
            None => {
                let d = tess::scene_diagonal(view, &faces, &edges);
                if d > 0.0 { DEFAULT_RELATIVE_CHORD_TOL * d } else { opts.chord_tol }
            }
        };
        if let Some(a) = self.angle_tol {
            opts.angle_tol = a;
        }
        if let Some(m) = self.max_edge {
            opts.max_edge = m;
        }
        cx.step(1, 2, "tessellating")?;
        let t = tess::tessellate_entities(view, &self.entities, &opts).map_err(|e| OpError::Invalid(e.to_string()))?;
        Ok(TessellateReply::new(&t, &opts))
    }
}

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// RFC 4648 base64 with padding.
pub(crate) fn base64(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        out.push(ALPHABET[(n >> 18) as usize & 63] as char);
        out.push(ALPHABET[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 { ALPHABET[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if chunk.len() > 2 { ALPHABET[n as usize & 63] as char } else { '=' });
    }
    out
}

/// Decode RFC 4648 base64 with padding (used by tests and by clients of this crate in Rust).
pub fn base64_decode(s: &str) -> Option<Vec<u8>> {
    let bytes = s.as_bytes();
    if !bytes.len().is_multiple_of(4) {
        return None;
    }
    let val = |c: u8| -> Option<u32> { ALPHABET.iter().position(|&a| a == c).map(|p| p as u32) };
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    for (i, q) in bytes.chunks(4).enumerate() {
        let last = i == bytes.len() / 4 - 1;
        let pad = q.iter().rev().take_while(|&&c| c == b'=').count();
        if pad > 2 || (pad > 0 && !last) {
            return None;
        }
        let mut n = 0u32;
        for &c in &q[..4 - pad] {
            n = (n << 6) | val(c)?;
        }
        n <<= 6 * pad as u32;
        let b = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        out.extend_from_slice(&b[..3 - pad]);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rfc4648_vectors() {
        for (raw, enc) in [("", ""), ("f", "Zg=="), ("fo", "Zm8="), ("foo", "Zm9v"), ("foob", "Zm9vYg=="), ("fooba", "Zm9vYmE="), ("foobar", "Zm9vYmFy")] {
            assert_eq!(base64(raw.as_bytes()), enc);
            assert_eq!(base64_decode(enc).unwrap(), raw.as_bytes());
        }
    }

    #[test]
    fn decode_rejects_garbage() {
        assert!(base64_decode("Zg=").is_none());
        assert!(base64_decode("Z!==").is_none());
        assert!(base64_decode("Zg==Zg==").is_none());
    }

    #[test]
    fn binary_round_trip() {
        let data: Vec<u8> = (0..=255u8).collect();
        assert_eq!(base64_decode(&base64(&data)).unwrap(), data);
    }
}
