// Pure buffer builders and color rules (no DOM, no GL): unit-testable in node.
import type { Skin, Tess } from "./types.js";

export type RGBA = [number, number, number, number];

/** Non-indexed triangle buffers for the flat-shaded pass and the ID pass. */
export interface TriBatch {
  vertexCount: number;
  positions: Float32Array;
  colors: Uint8Array;
  /** Triangle id + 1 as little-endian RGBA bytes. */
  ids: Uint8Array;
  hatch: Uint8Array;
}

/** Expand an indexed triangle list into per-corner buffers. `color(t)` picks each triangle's color. */
export function expandTriangles(
  positions: Float32Array,
  indices: Uint32Array,
  color: (tri: number) => RGBA,
  hatch: (tri: number) => boolean = () => false,
  idOf: (tri: number) => number = (t) => t + 1,
): TriBatch {
  const n = indices.length / 3;
  const out: TriBatch = {
    vertexCount: n * 3,
    positions: new Float32Array(n * 9),
    colors: new Uint8Array(n * 12),
    ids: new Uint8Array(n * 12),
    hatch: new Uint8Array(n * 3),
  };
  for (let t = 0; t < n; t++) {
    const c = color(t);
    const id = idOf(t);
    const h = hatch(t) ? 255 : 0;
    for (let k = 0; k < 3; k++) {
      const v = indices[t * 3 + k]!;
      const o = t * 3 + k;
      out.positions.set(positions.subarray(v * 3, v * 3 + 3), o * 3);
      out.colors.set(c, o * 4);
      out.ids[o * 4] = id & 255;
      out.ids[o * 4 + 1] = (id >> 8) & 255;
      out.ids[o * 4 + 2] = (id >> 16) & 255;
      out.ids[o * 4 + 3] = 255;
      out.hatch[o] = h;
    }
  }
  return out;
}

/** Line list positions (two vertices per segment) from index pairs. */
export function expandLines(positions: Float32Array, pairs: Uint32Array): Float32Array {
  const out = new Float32Array(pairs.length * 3);
  for (let i = 0; i < pairs.length; i++) out.set(positions.subarray(pairs[i]! * 3, pairs[i]! * 3 + 3), i * 3);
  return out;
}

/** Polyline ranges of a tessellation as a line list. */
export function edgeLines(t: Tess): Float32Array {
  const segs: number[] = [];
  for (const e of t.edges) {
    for (let i = 0; i + 1 < e.count; i++) segs.push(t.edgeIndices[e.first + i]!, t.edgeIndices[e.first + i + 1]!);
  }
  return expandLines(t.positions, Uint32Array.from(segs));
}

/** Serial of an id text `Kind#serial` (0 if malformed). */
export function serialOf(id: string): number {
  const n = Number(id.slice(id.indexOf("#") + 1));
  return Number.isFinite(n) ? n : 0;
}

const PALETTE: [number, number, number][] = [
  [76, 120, 168], [245, 133, 24], [82, 162, 59], [228, 87, 86], [178, 121, 162], [157, 117, 93],
  [242, 142, 171], [118, 183, 178], [237, 201, 72], [186, 176, 172], [95, 90, 166], [160, 203, 232],
];

/** Stable color of an owner id: palette by serial. */
export function ownerColor(id: string | null): RGBA {
  if (id === null) return [180, 180, 180, 255];
  const c = PALETTE[serialOf(id) % PALETTE.length]!;
  return [c[0], c[1], c[2], 255];
}

/** Sequential ramp, `t` in [0, 1]: dark blue to yellow. */
export function ramp(t: number): RGBA {
  const x = Math.max(0, Math.min(1, Number.isFinite(t) ? t : 0));
  const stops: [number, number, number][] = [[40, 30, 120], [30, 130, 160], [60, 180, 110], [250, 230, 60]];
  const s = x * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(s));
  const f = s - i;
  const a = stops[i]!, b = stops[i + 1]!;
  return [Math.round(a[0] + (b[0] - a[0]) * f), Math.round(a[1] + (b[1] - a[1]) * f), Math.round(a[2] + (b[2] - a[2]) * f), 255];
}

export const BAD_COLOR: RGBA = [220, 30, 30, 255];
export const UNDEFINED_COLOR: RGBA = [120, 120, 120, 255];

/** Per-triangle scalar color through `tri_cell`; cells below `badBelow` or above `badAbove` are red. */
export function scalarColor(
  values: Float32Array,
  cell: number,
  lo: number,
  hi: number,
  badBelow: number | null,
  badAbove: number | null,
): RGBA {
  const v = values[cell]!;
  if (!Number.isFinite(v)) return UNDEFINED_COLOR;
  if ((badBelow !== null && v < badBelow) || (badAbove !== null && v > badAbove)) return BAD_COLOR;
  return ramp(hi > lo ? (v - lo) / (hi - lo) : 0.5);
}

/** Skin triangles colored by owner or scalar. */
export function skinBatch(s: Skin, color: (tri: number) => RGBA): TriBatch {
  return expandTriangles(s.positions, s.indices, color);
}

/** Face id of a model triangle by range (`null` outside every range). */
export function faceOfTriangle(t: Tess, tri: number): string | null {
  const first = tri * 3;
  for (const f of t.faces) if (first >= f.first_index && first < f.first_index + f.index_count) return f.face;
  return null;
}
