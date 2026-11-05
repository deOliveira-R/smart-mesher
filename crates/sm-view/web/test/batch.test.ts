import assert from "node:assert/strict";
import { test } from "node:test";
import { BAD_COLOR, UNDEFINED_COLOR, edgeLines, expandTriangles, faceOfTriangle, ownerColor, ramp, scalarColor, serialOf, skinBatch } from "../src/batch.js";
import { decodeTess } from "../src/codec.js";
import type { TessReply } from "../src/types.js";
import { boxSkin, loadFixture } from "./util.js";

test("expandTriangles: per-corner buffers and ids", () => {
  const s = boxSkin();
  const b = skinBatch(s, () => [1, 2, 3, 255]);
  assert.equal(b.vertexCount, 240);
  assert.equal(b.positions.length, 720);
  assert.equal(b.colors.length, 960);
  for (let k = 0; k < 3; k++) assert.deepEqual([...b.ids.slice((5 * 3 + k) * 4, (5 * 3 + k) * 4 + 4)], [6, 0, 0, 255]);
  const big = expandTriangles(new Float32Array(9), Uint32Array.from([0, 1, 2]), () => [0, 0, 0, 255], () => true, () => 0x0a0b0c);
  assert.deepEqual([...big.ids.slice(0, 4)], [0x0c, 0x0b, 0x0a, 255]);
  assert.deepEqual([...big.hatch], [255, 255, 255]);
});

test("colors: ramp ends, owner palette, scalar rule", () => {
  assert.deepEqual(ramp(0), [40, 30, 120, 255]);
  assert.deepEqual(ramp(1), [250, 230, 60, 255]);
  assert.deepEqual(ramp(Number.NaN), ramp(0));
  assert.equal(serialOf("TopoFace#14"), 14);
  assert.deepEqual(ownerColor("TopoFace#3"), ownerColor("TopoVolume#3"));
  assert.notDeepEqual(ownerColor("TopoFace#3"), ownerColor("TopoFace#4"));
  const v = Float32Array.from([1, 0.5, Number.NaN]);
  assert.deepEqual(scalarColor(v, 1, 0, 1, 0.9, null), BAD_COLOR);
  assert.deepEqual(scalarColor(v, 0, 0, 1, 0.9, null), ramp(1));
  assert.deepEqual(scalarColor(v, 2, 0, 1, 0.9, null), UNDEFINED_COLOR);
});

test("model tessellation: face lookup and edge lines", () => {
  const t = decodeTess(loadFixture().replies["view.tessellate"]![0]!.reply as TessReply);
  assert.equal(faceOfTriangle(t, 0), "TopoFace#1");
  assert.equal(faceOfTriangle(t, 11), "TopoFace#6");
  assert.equal(faceOfTriangle(t, 12), null);
  assert.equal(edgeLines(t).length, 12 * 2 * 3);
});
