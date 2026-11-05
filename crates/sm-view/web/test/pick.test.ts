import assert from "node:assert/strict";
import { test } from "node:test";
import { fitToBBox, lookFrom, rayFromPixel, viewProj } from "../src/camera.js";
import { ownerOf, pickTriangle, rayTriangle } from "../src/pick.js";
import { rasterIds } from "../src/raster.js";
import { boxSkin } from "./util.js";

// hex_block classifies -x,+x,-y,+y,-z,+z as TopoFace#1..6.
const VIEWS: [string, string][] = [["+x", "TopoFace#2"], ["+y", "TopoFace#4"], ["+z", "TopoFace#6"], ["-x", "TopoFace#1"], ["-y", "TopoFace#3"], ["-z", "TopoFace#5"]];

test("CPU pick of the box from the canonical views returns the analytic face", () => {
  const s = boxSkin();
  for (const [dir, face] of VIEWS) {
    const cam = lookFrom(fitToBBox(s.bbox, Math.PI / 4, 1), dir);
    const { origin, dir: d } = rayFromPixel(cam, 99.5, 99.5, 200, 200);
    const hit = pickTriangle(s, origin, d);
    assert.ok(hit, dir);
    assert.equal(ownerOf(s, hit.tri), face, `view ${dir}`);
  }
});

test("a ray that misses the box picks nothing", () => {
  const s = boxSkin();
  assert.equal(pickTriangle(s, [100, 100, 100], [0, 0, 1]), null);
  assert.equal(rayTriangle([0, 0, 1], [0, 0, 1], [0, 0, 0], [1, 0, 0], [0, 1, 0]), null);
});

test("CPU raster: centre pixel agrees with the pick, background stays 0", () => {
  const s = boxSkin();
  const W = 120, H = 90;
  for (const [dir, face] of VIEWS) {
    const cam = lookFrom(fitToBBox(s.bbox, Math.PI / 4, W / H), dir);
    const ids = rasterIds(s, viewProj(cam), W, H);
    const covered = ids.reduce((n, v) => n + (v > 0 ? 1 : 0), 0);
    assert.ok(covered > 500 && covered < W * H, `${dir}: ${covered} covered`);
    assert.equal(ids[0], 0, "corner pixel is background");
    // Off-centre: the exact centre can lie on a quad's diagonal, where both triangles are valid.
    const px = Math.floor(W / 2) + 3, py = Math.floor(H / 2) + 2;
    const centre = ids[py * W + px]!;
    assert.ok(centre > 0 && centre <= s.triCount);
    assert.equal(ownerOf(s, centre - 1), face, dir);
    const { origin, dir: d } = rayFromPixel(cam, px, py, W, H);
    const hit = pickTriangle(s, origin, d)!;
    assert.equal(hit.tri + 1, centre, `${dir}: raster and ray pick agree`);
  }
});

test("raster is deterministic", () => {
  const s = boxSkin();
  const vp = viewProj(fitToBBox(s.bbox, Math.PI / 4, 1.3));
  assert.deepEqual(rasterIds(s, vp, 64, 48), rasterIds(s, vp, 64, 48));
});
