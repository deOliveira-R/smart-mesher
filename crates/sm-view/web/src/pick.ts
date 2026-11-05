// CPU ray / triangle picking over a decoded skin.
import type { Skin, Vec3 } from "./types.js";
import { vcross, vdot, vsub } from "./camera.js";

export interface Hit {
  /** Triangle index. */
  tri: number;
  /** Ray parameter. */
  t: number;
  point: Vec3;
}

/** Moeller-Trumbore, double sided. Returns the parameter or null. */
export function rayTriangle(o: Vec3, d: Vec3, a: Vec3, b: Vec3, c: Vec3): number | null {
  const e1 = vsub(b, a);
  const e2 = vsub(c, a);
  const p = vcross(d, e2);
  const det = vdot(e1, p);
  if (Math.abs(det) < 1e-18) return null;
  const inv = 1 / det;
  const s = vsub(o, a);
  const u = vdot(s, p) * inv;
  if (u < 0 || u > 1) return null;
  const q = vcross(s, e1);
  const v = vdot(d, q) * inv;
  if (v < 0 || u + v > 1) return null;
  const t = vdot(e2, q) * inv;
  return t > 0 ? t : null;
}

function vertex(s: Skin, i: number): Vec3 {
  return [s.positions[i * 3]!, s.positions[i * 3 + 1]!, s.positions[i * 3 + 2]!];
}

/** Nearest triangle hit by the ray, or null. Ties go to the lower triangle index. */
export function pickTriangle(s: Skin, origin: Vec3, dir: Vec3): Hit | null {
  let best: Hit | null = null;
  for (let t = 0; t < s.triCount; t++) {
    const h = rayTriangle(origin, dir, vertex(s, s.indices[t * 3]!), vertex(s, s.indices[t * 3 + 1]!), vertex(s, s.indices[t * 3 + 2]!));
    if (h !== null && (best === null || h < best.t)) {
      best = { tri: t, t: h, point: [origin[0] + dir[0] * h, origin[1] + dir[1] * h, origin[2] + dir[2] * h] };
    }
  }
  return best;
}

/** Topo entity id owning a triangle (`null` when none). */
export function ownerOf(s: Skin, tri: number): string | null {
  const k = s.triOwner[tri]!;
  return k === 0 ? null : (s.owners[k - 1] ?? null);
}
