import assert from "node:assert/strict";
import { test } from "node:test";
import { corners, eye, fitToBBox, flyTo, inFrustum, lookFrom, orbit, project, rayFromPixel, vdot, viewProj, vnorm, vsub } from "../src/camera.js";
import type { BBox } from "../src/types.js";

const boxes: BBox[] = [
  [[0, 0, 0], [4, 2, 2]],
  [[-1, -1, -1], [1, 1, 1]],
  [[10, 20, 30], [10.5, 80, 30.1]],
  [[0, 0, 0], [0, 0, 0]],
];

test("fit-to-bbox puts all 8 corners inside the frustum, any aspect", () => {
  for (const b of boxes) {
    for (const aspect of [0.4, 1, 1.78, 3]) {
      const cam = fitToBBox(b, Math.PI / 4, aspect);
      const vp = viewProj(cam);
      for (const c of corners(b)) assert.ok(inFrustum(vp, c), `corner ${c} of ${JSON.stringify(b)} at aspect ${aspect}`);
    }
  }
});

test("the box centre projects to the middle of the screen", () => {
  const cam = fitToBBox(boxes[0]!, Math.PI / 4, 1.5);
  const [x, y, , w] = project(viewProj(cam), cam.target);
  assert.ok(Math.abs(x / w) < 1e-12 && Math.abs(y / w) < 1e-12);
});

test("orbit clamps the pitch and keeps the distance", () => {
  let cam = fitToBBox(boxes[0]!);
  const d = cam.distance;
  cam = orbit(cam, 1, 10);
  assert.ok(cam.pitch < Math.PI / 2);
  cam = orbit(cam, 0, -100);
  assert.ok(cam.pitch > -Math.PI / 2);
  assert.equal(cam.distance, d);
});

test("the centre pixel's ray points at the target", () => {
  const cam = lookFrom(fitToBBox(boxes[0]!), "+y");
  const { origin, dir } = rayFromPixel(cam, 99.5, 49.5, 200, 100);
  const to = vnorm(vsub(cam.target, origin));
  assert.ok(vdot(to, dir) > 1 - 1e-12);
  assert.ok(eye(cam)[1]! > cam.target[1]);
});

test("flyTo moves the target exactly", () => {
  const cam = flyTo(fitToBBox(boxes[0]!), [1, 2, 3]);
  assert.deepEqual(cam.target, [1, 2, 3]);
});
