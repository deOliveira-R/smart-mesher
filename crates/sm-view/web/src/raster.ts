// CPU ID rasterizer: the reference the WebGL ID pass is compared with in the browser e2e.
import { type Mat4, project } from "./camera.js";
import type { Skin } from "./types.js";

/**
 * Rasterize triangle ids (index + 1, 0 = background) into a `w x h` buffer with a depth test.
 * Pixel centres are sampled at +0.5; a pixel is covered when its centre lies inside or on the
 * triangle (edge functions >= 0 for either winding). Triangles crossing the near plane (w <= 0
 * at any vertex) are skipped.
 */
export function rasterIds(s: Skin, vp: Mat4, w: number, h: number): Uint32Array {
  const ids = new Uint32Array(w * h);
  const depth = new Float64Array(w * h).fill(Infinity);
  const sx = new Float64Array(3), sy = new Float64Array(3), sz = new Float64Array(3);
  for (let t = 0; t < s.triCount; t++) {
    let ok = true;
    for (let k = 0; k < 3; k++) {
      const v = s.indices[t * 3 + k]!;
      const [x, y, z, ww] = project(vp, [s.positions[v * 3]!, s.positions[v * 3 + 1]!, s.positions[v * 3 + 2]!]);
      if (!(ww > 0)) {
        ok = false;
        break;
      }
      sx[k] = ((x / ww) * 0.5 + 0.5) * w;
      sy[k] = (1 - ((y / ww) * 0.5 + 0.5)) * h;
      sz[k] = z / ww;
    }
    if (!ok) continue;
    const area = (sx[1]! - sx[0]!) * (sy[2]! - sy[0]!) - (sx[2]! - sx[0]!) * (sy[1]! - sy[0]!);
    if (area === 0) continue;
    const x0 = Math.max(0, Math.floor(Math.min(sx[0]!, sx[1]!, sx[2]!)));
    const x1 = Math.min(w - 1, Math.ceil(Math.max(sx[0]!, sx[1]!, sx[2]!)));
    const y0 = Math.max(0, Math.floor(Math.min(sy[0]!, sy[1]!, sy[2]!)));
    const y1 = Math.min(h - 1, Math.ceil(Math.max(sy[0]!, sy[1]!, sy[2]!)));
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const px = x + 0.5, py = y + 0.5;
        const w0 = ((sx[1]! - px) * (sy[2]! - py) - (sx[2]! - px) * (sy[1]! - py)) / area;
        const w1 = ((sx[2]! - px) * (sy[0]! - py) - (sx[0]! - px) * (sy[2]! - py)) / area;
        const w2 = 1 - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
        const z = w0 * sz[0]! + w1 * sz[1]! + w2 * sz[2]!;
        if (z < -1 || z > 1) continue;
        const i = y * w + x;
        if (z < depth[i]! || (z === depth[i]! && t + 1 < ids[i]!)) {
          depth[i] = z;
          ids[i] = t + 1;
        }
      }
    }
  }
  return ids;
}
