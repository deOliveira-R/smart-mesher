import assert from "node:assert/strict";
import { test } from "node:test";
import { CodecError, b64ToBytes, decodeF32, decodeScalar, decodeSkin, decodeTess, decodeU32 } from "../src/codec.js";
import type { ScalarReply, SkinReply, TessReply } from "../src/types.js";
import { boxSkin, boxSkinReply, loadFixture, triArea } from "./util.js";

test("base64 round trip against node's encoder (all lengths)", () => {
  for (let n = 0; n < 40; n++) {
    const bytes = Uint8Array.from({ length: n }, (_, i) => (i * 37 + 11) & 255);
    const enc = Buffer.from(bytes).toString("base64");
    assert.deepEqual([...b64ToBytes(enc)], [...bytes]);
  }
});

test("f32 and u32 decode agree with Buffer", () => {
  const f = new Float32Array([1.5, -2, 3.25e7]);
  const u = new Uint32Array([0, 1, 4294967295]);
  assert.deepEqual([...decodeF32(Buffer.from(f.buffer).toString("base64"), "f", 3)], [...f]);
  assert.deepEqual([...decodeU32(Buffer.from(u.buffer).toString("base64"), "u", 3)], [...u]);
});

test("Rust-written box skin decodes: 80 triangles, area 40, every triangle owned", () => {
  const s = boxSkin();
  assert.equal(s.triCount, 80);
  assert.equal(s.indices.length, 240);
  assert.equal(s.triCell.length, 80);
  assert.ok(Math.abs(triArea(s) - 40) < 1e-9);
  assert.ok([...s.triOwner].every((o) => o > 0 && o <= s.owners.length));
  assert.deepEqual(s.owners, ["TopoFace#1", "TopoFace#2", "TopoFace#3", "TopoFace#4", "TopoFace#5", "TopoFace#6"]);
  assert.ok(s.bbox[0].every((v) => v >= 0) && s.bbox[1][0] === 4 && s.bbox[1][1] === 2 && s.bbox[1][2] === 2);
  assert.ok(s.outline.length > 0 && s.outline.length % 2 === 0);
});

test("clipped replies from the fixture match the analytic counts", () => {
  const entries = loadFixture().replies["view.mesh_skin"]!;
  // x clip at 2: the 2x2x2 block, 24 quads = 48 triangles, area 24, 8 cells clipped.
  const e = entries.find((x) => JSON.stringify((x.match as { clip?: { origin: number[] } }).clip?.origin) === JSON.stringify([2, 0, 0]))!;
  const s = decodeSkin(e.reply as SkinReply);
  assert.equal(s.triCount, 48);
  // (the fixture's displaced node lies on the cut, so the area is checked in the Rust tests)
  assert.ok(triArea(s) > 20 && triArea(s) < 28);
  assert.equal((e.reply as SkinReply).clipped_cells, 8);
});

test("a truncated buffer throws CodecError", () => {
  const r = boxSkinReply();
  assert.throws(() => decodeSkin({ ...r, positions: r.positions.slice(0, -4) }), CodecError);
  assert.throws(() => decodeSkin({ ...r, indices: r.indices.slice(0, r.indices.length - 8) }), CodecError);
  assert.throws(() => decodeSkin({ ...r, tri_cell: "" }), CodecError);
});

test("malformed base64 throws CodecError", () => {
  assert.throws(() => b64ToBytes("abc"), CodecError);
  assert.throws(() => b64ToBytes("ab!d"), CodecError);
  assert.throws(() => b64ToBytes("a=bc"), CodecError);
  assert.throws(() => b64ToBytes("AA==AA=="), CodecError);
  assert.throws(() => decodeF32("AAAAAAA=", "x"), CodecError); // 5 bytes is not a multiple of 4
});

test("out-of-range indices and owners are rejected", () => {
  const r = boxSkinReply();
  const bad = new Uint32Array(r.tri_count * 3);
  bad[0] = r.vertex_count;
  assert.throws(() => decodeSkin({ ...r, indices: Buffer.from(bad.buffer).toString("base64") }), CodecError);
  const owner = new Uint32Array(r.tri_count).fill(99);
  assert.throws(() => decodeSkin({ ...r, tri_owner: Buffer.from(owner.buffer).toString("base64") }), CodecError);
});

test("tessellation and scalar fixtures decode", () => {
  const fx = loadFixture();
  const t = decodeTess(fx.replies["view.tessellate"]![0]!.reply as TessReply);
  assert.equal(t.faces.length, 6);
  assert.equal(t.indices.length, 36);
  assert.equal(t.edges.length, 12);
  const sc = fx.replies["view.mesh_scalar"]![0]!.reply as ScalarReply;
  const v = decodeScalar(sc);
  assert.equal(v.length, 16);
  assert.ok(Math.min(...v) < 0.9, "the displaced node makes bad cells");
  assert.ok(sc.bad_cells > 0);
  assert.ok(Math.abs(Math.min(...v) - sc.min!) < 1e-6);
});
