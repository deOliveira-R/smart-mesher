// Orbit camera math (z-up, right-handed, column-major 4x4). No DOM.
import type { BBox, Vec3 } from "./types.js";

export type Mat4 = number[];

export interface Camera {
  target: Vec3;
  distance: number;
  /** Radians about +z, 0 looks from +x toward the target. */
  yaw: number;
  /** Radians above the xy plane, clamped to +-(pi/2 - 1e-3). */
  pitch: number;
  fovY: number;
  aspect: number;
  near: number;
  far: number;
}

const PITCH_MAX = Math.PI / 2 - 1e-3;

const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a: Vec3, k: number): Vec3 => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]);
  return l > 0 ? scale(a, 1 / l) : [0, 0, 0];
};

export { add as vadd, sub as vsub, scale as vscale, dot as vdot, cross as vcross, norm as vnorm };

/** Eye position. */
export function eye(c: Camera): Vec3 {
  const cp = Math.cos(c.pitch);
  return add(c.target, scale([cp * Math.cos(c.yaw), cp * Math.sin(c.yaw), Math.sin(c.pitch)], c.distance));
}

/** Orthonormal camera basis: forward (toward the target), right, up. */
export function basis(c: Camera): { f: Vec3; r: Vec3; u: Vec3 } {
  const f = norm(sub(c.target, eye(c)));
  const r = norm(cross(f, [0, 0, 1]));
  const u = cross(r, f);
  return { f, r, u };
}

/** Multiply column-major matrices: a * b. */
export function mul(a: Mat4, b: Mat4): Mat4 {
  const o = new Array<number>(16).fill(0);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r]! * b[c * 4 + k]!;
      o[c * 4 + r] = s;
    }
  }
  return o;
}

/** World to camera. */
export function viewMatrix(c: Camera): Mat4 {
  const e = eye(c);
  const { f, r, u } = basis(c);
  return [r[0], u[0], -f[0], 0, r[1], u[1], -f[1], 0, r[2], u[2], -f[2], 0, -dot(r, e), -dot(u, e), dot(f, e), 1];
}

/** Perspective projection with clip z in [-1, 1]. */
export function projMatrix(c: Camera): Mat4 {
  const t = 1 / Math.tan(c.fovY / 2);
  const nf = 1 / (c.near - c.far);
  return [t / c.aspect, 0, 0, 0, 0, t, 0, 0, 0, 0, (c.far + c.near) * nf, -1, 0, 0, 2 * c.far * c.near * nf, 0];
}

/** Projection times view. */
export function viewProj(c: Camera): Mat4 {
  return mul(projMatrix(c), viewMatrix(c));
}

/** Clip-space `[x, y, z, w]` of a world point. */
export function project(m: Mat4, p: Vec3): [number, number, number, number] {
  const x = p[0], y = p[1], z = p[2];
  return [
    m[0]! * x + m[4]! * y + m[8]! * z + m[12]!,
    m[1]! * x + m[5]! * y + m[9]! * z + m[13]!,
    m[2]! * x + m[6]! * y + m[10]! * z + m[14]!,
    m[3]! * x + m[7]! * y + m[11]! * z + m[15]!,
  ];
}

/** True if the point is strictly inside the frustum (w > 0 and |ndc| <= 1 on every axis). */
export function inFrustum(m: Mat4, p: Vec3, slack = 1e-9): boolean {
  const [x, y, z, w] = project(m, p);
  return w > 0 && Math.abs(x) <= w + slack && Math.abs(y) <= w + slack && Math.abs(z) <= w + slack;
}

/** The 8 corners of a bounding box. */
export function corners(b: BBox): Vec3[] {
  const out: Vec3[] = [];
  for (let i = 0; i < 8; i++) out.push([b[i & 1][0], b[(i >> 1) & 1][1], b[(i >> 2) & 1][2]]);
  return out;
}

/** A camera that sees the whole box from the default direction. */
export function fitToBBox(b: BBox, fovY = Math.PI / 4, aspect = 1): Camera {
  const c: Vec3 = [(b[0][0] + b[1][0]) / 2, (b[0][1] + b[1][1]) / 2, (b[0][2] + b[1][2]) / 2];
  let radius = Math.hypot(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]) / 2;
  if (!(radius > 0)) radius = 1;
  const half = Math.min(fovY / 2, Math.atan(Math.tan(fovY / 2) * aspect));
  const distance = (radius / Math.sin(half)) * 1.05;
  return { target: c, distance, yaw: 0.7, pitch: 0.5, fovY, aspect, near: Math.max(distance - radius * 1.2, distance * 0.01), far: distance + radius * 1.2 };
}

/** Rotate about the target. */
export function orbit(c: Camera, dyaw: number, dpitch: number): Camera {
  return { ...c, yaw: c.yaw + dyaw, pitch: Math.max(-PITCH_MAX, Math.min(PITCH_MAX, c.pitch + dpitch)) };
}

/** Dolly by a factor (>1 moves away). */
export function zoom(c: Camera, k: number): Camera {
  return { ...c, distance: c.distance * k };
}

/** Pan in the image plane by fractions of the visible height. */
export function pan(c: Camera, dx: number, dy: number): Camera {
  const { r, u } = basis(c);
  const h = 2 * c.distance * Math.tan(c.fovY / 2);
  return { ...c, target: add(c.target, add(scale(r, -dx * h * c.aspect), scale(u, dy * h))) };
}

/** Look along an axis-aligned canonical direction: "+x", "-x", "+y", "-y", "+z", "-z". */
export function lookFrom(c: Camera, dir: string): Camera {
  const yaw: Record<string, number> = { "+x": 0, "-x": Math.PI, "+y": Math.PI / 2, "-y": -Math.PI / 2, "+z": 0, "-z": 0 };
  const pitch = dir === "+z" ? PITCH_MAX : dir === "-z" ? -PITCH_MAX : 0;
  return { ...c, yaw: yaw[dir] ?? c.yaw, pitch };
}

/** Move the target to `p`, keeping the distance (fly-to). */
export function flyTo(c: Camera, p: Vec3): Camera {
  return { ...c, target: p };
}

/** World ray through pixel `(px, py)` of a `w x h` viewport (pixel centres at +0.5). */
export function rayFromPixel(c: Camera, px: number, py: number, w: number, h: number): { origin: Vec3; dir: Vec3 } {
  const { f, r, u } = basis(c);
  const nx = (2 * (px + 0.5)) / w - 1;
  const ny = 1 - (2 * (py + 0.5)) / h;
  const t = Math.tan(c.fovY / 2);
  const dir = norm(add(f, add(scale(r, nx * t * (w / h)), scale(u, ny * t))));
  return { origin: eye(c), dir };
}
