// Base64 buffers to typed arrays, with typed errors. No DOM, no atob.
import type { ScalarReply, Skin, SkinReply, Tess, TessReply } from "./types.js";

/** Raised for every malformed buffer: bad base64, wrong length, out-of-range index. */
export class CodecError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CodecError";
  }
}

const ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const LUT = new Int16Array(128).fill(-1);
for (let i = 0; i < 64; i++) LUT[ALPHABET.charCodeAt(i)] = i;

/** Decode padded RFC 4648 base64. */
export function b64ToBytes(s: string): Uint8Array {
  if (s.length % 4 !== 0) throw new CodecError(`base64 length ${s.length} is not a multiple of 4`);
  if (s.length === 0) return new Uint8Array(0);
  let pad = 0;
  if (s.endsWith("==")) pad = 2;
  else if (s.endsWith("=")) pad = 1;
  const out = new Uint8Array((s.length / 4) * 3 - pad);
  let o = 0;
  for (let i = 0; i < s.length; i += 4) {
    let n = 0;
    for (let k = 0; k < 4; k++) {
      const c = s.charCodeAt(i + k);
      const last = i + 4 === s.length;
      if (c === 61 /* = */) {
        if (!last || k < 4 - pad) throw new CodecError("misplaced base64 padding");
        n <<= 6;
        continue;
      }
      const v = c < 128 ? LUT[c] : -1;
      if (v < 0) throw new CodecError(`invalid base64 character at ${i + k}`);
      n = (n << 6) | v;
    }
    if (o < out.length) out[o++] = (n >> 16) & 255;
    if (o < out.length) out[o++] = (n >> 8) & 255;
    if (o < out.length) out[o++] = n & 255;
  }
  return out;
}

/** True on little-endian hosts (the wire is little-endian). */
export function hostIsLittleEndian(): boolean {
  return new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;
}

function check(bytes: Uint8Array, what: string, expected: number | undefined): number {
  if (bytes.length % 4 !== 0) throw new CodecError(`${what}: ${bytes.length} bytes is not a multiple of 4`);
  const n = bytes.length / 4;
  if (expected !== undefined && n !== expected) throw new CodecError(`${what}: expected ${expected} values, got ${n}`);
  return n;
}

/** Decode little-endian f32 values; `expected` is the required count. */
export function decodeF32(s: string, what: string, expected?: number): Float32Array {
  const bytes = b64ToBytes(s);
  const n = check(bytes, what, expected);
  const out = new Float32Array(n);
  if (hostIsLittleEndian()) {
    new Uint8Array(out.buffer).set(bytes);
  } else {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    for (let i = 0; i < n; i++) out[i] = dv.getFloat32(i * 4, true);
  }
  return out;
}

/** Decode little-endian u32 values; `expected` is the required count. */
export function decodeU32(s: string, what: string, expected?: number): Uint32Array {
  const bytes = b64ToBytes(s);
  const n = check(bytes, what, expected);
  const out = new Uint32Array(n);
  if (hostIsLittleEndian()) {
    new Uint8Array(out.buffer).set(bytes);
  } else {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    for (let i = 0; i < n; i++) out[i] = dv.getUint32(i * 4, true);
  }
  return out;
}

function checkIndices(idx: Uint32Array, limit: number, what: string): void {
  for (let i = 0; i < idx.length; i++) {
    if (idx[i]! >= limit) throw new CodecError(`${what}[${i}] = ${idx[i]} is out of range (${limit} vertices)`);
  }
}

/** Decode and validate a `view.mesh_skin` reply. */
export function decodeSkin(r: SkinReply): Skin {
  const positions = decodeF32(r.positions, "positions", r.vertex_count * 3);
  const indices = decodeU32(r.indices, "indices", r.tri_count * 3);
  const triCell = decodeU32(r.tri_cell, "tri_cell", r.tri_count);
  const triOwner = decodeU32(r.tri_owner, "tri_owner", r.tri_count);
  const outline = decodeU32(r.outline_indices, "outline_indices");
  if (outline.length % 2 !== 0) throw new CodecError("outline_indices: odd length");
  checkIndices(indices, r.vertex_count, "indices");
  checkIndices(outline, r.vertex_count, "outline_indices");
  for (let i = 0; i < triOwner.length; i++) {
    if (triOwner[i]! > r.owners.length) throw new CodecError(`tri_owner[${i}] = ${triOwner[i]} has no owners entry`);
  }
  return { triCount: r.tri_count, vertexCount: r.vertex_count, positions, indices, triCell, triOwner, owners: r.owners, outline, bbox: r.bbox };
}

/** Decode and validate a `view.tessellate` reply. */
export function decodeTess(r: TessReply): Tess {
  const positions = decodeF32(r.positions, "positions", r.vertex_count * 3);
  const indices = decodeU32(r.indices, "indices", r.triangle_count * 3);
  const edgeIndices = decodeU32(r.edge_indices, "edge_indices");
  checkIndices(indices, r.vertex_count, "indices");
  checkIndices(edgeIndices, r.vertex_count, "edge_indices");
  for (const f of r.faces) {
    if (f.first_index + f.index_count > indices.length) throw new CodecError(`face ${f.face}: range past the index buffer`);
  }
  for (const e of r.edges) {
    if (e.first + e.count > edgeIndices.length) throw new CodecError(`edge ${e.edge}: range past the index buffer`);
  }
  return { positions, indices, faces: r.faces, edgeIndices, edges: r.edges };
}

/** Decode the per-cell values of a `view.mesh_scalar` reply. */
export function decodeScalar(r: ScalarReply): Float32Array {
  return decodeF32(r.values, "values", r.count);
}
