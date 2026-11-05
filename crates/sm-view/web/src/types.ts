// Wire types of the sm-view queries (mirrors crates/sm-view/src/queries.rs) and the
// existing queries the client reads. Everything is read-only.

export type Vec3 = [number, number, number];
export type BBox = [Vec3, Vec3];

export interface VolumeOut {
  id: string;
  color_label: number | null;
  color: [number, number, number] | null;
}
export interface BodyOut {
  id: string;
  volumes: string[];
  color: [number, number, number] | null;
  color_label: number | null;
  name: string | null;
}
export interface MeshOut {
  id: string;
  name: string;
  dim: number;
  cells: number;
  facets: number;
  nodes: number;
  bbox: BBox;
  digest: string;
  derived_from: string[];
}
export interface SceneReply {
  schema_version: number;
  bbox: BBox;
  volumes: VolumeOut[];
  bodies: BodyOut[];
  meshes: MeshOut[];
}

export interface SkinReply {
  schema_version: number;
  mesh: string;
  tri_count: number;
  vertex_count: number;
  node_vertices: number;
  positions: string;
  indices: string;
  tri_cell: string;
  tri_owner: string;
  owners: string[];
  outline_indices: string;
  bbox: BBox;
  clipped_cells: number;
  kept_cells: number;
}

export interface WorstCell {
  cell: { id: string; index: number };
  value: number;
  location: Vec3;
}
export interface ScalarReply {
  schema_version: number;
  mesh: string;
  metric: string;
  count: number;
  values: string;
  min: number | null;
  max: number | null;
  mean: number | null;
  undefined_cells: number;
  bad_cells: number;
  worst_end: "low" | "high";
  worst: WorstCell[];
}

export interface FaceRange {
  face: string;
  first_index: number;
  index_count: number;
  boundary_only: boolean;
}
export interface EdgeRange {
  edge: string;
  first: number;
  count: number;
}
export interface TessReply {
  vertex_count: number;
  triangle_count: number;
  positions: string;
  indices: string;
  faces: FaceRange[];
  edge_indices: string;
  edges: EdgeRange[];
}

// mesh.validate / mesh.quality (only the fields the client shows)
export interface Violation {
  code: string;
  severity: "error" | "warning";
  detail: string;
  owner: string | null;
  location: Vec3 | null;
}
export interface ValidateReply {
  mesh: string;
  valid: boolean;
  total: number;
  violations: Violation[];
}
export interface Suggestion {
  id: string;
  kind: string;
  severity: "error" | "warning";
  metric: string;
  targets: string[];
  op: string | null;
  args: unknown;
  reason: string;
  available: boolean;
}
export interface WorstEntry {
  value: number;
  grade: string | null;
  location: Vec3 | null;
  owner: string | null;
}
export interface MetricReport {
  metric: string;
  worst: WorstEntry[];
}
export interface QualityReply {
  mesh: string;
  verdict: string;
  metrics: MetricReport[];
  suggestions: Suggestion[];
}

export interface LogEntry {
  node: string;
  parent: string | null;
  depth: number;
  op: string;
  author: string;
  time: string;
  message: string;
}
export interface StatusReply {
  head: unknown;
  position: string | null;
  state: string;
  branches: Record<string, string>;
  nodes: number;
}
export interface HashReply {
  state: string;
  layers: Record<string, string>;
}
export interface ScriptReply {
  state: string;
  output: unknown;
  diagnostics: unknown[];
}

/** A decoded skin: typed arrays ready for upload or CPU work. */
export interface Skin {
  triCount: number;
  vertexCount: number;
  positions: Float32Array;
  indices: Uint32Array;
  triCell: Uint32Array;
  triOwner: Uint32Array;
  owners: string[];
  outline: Uint32Array;
  bbox: BBox;
}
export interface Tess {
  positions: Float32Array;
  indices: Uint32Array;
  faces: FaceRange[];
  edgeIndices: Uint32Array;
  edges: EdgeRange[];
}
