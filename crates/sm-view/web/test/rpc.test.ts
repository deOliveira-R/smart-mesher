import assert from "node:assert/strict";
import { test } from "node:test";
import { transportFromUrl } from "../src/main.js";
import { Client, FixtureTransport, RpcError, WsTransport, type WsLike, canonical } from "../src/rpc.js";
import type { SceneReply } from "../src/types.js";
import { fixtureClient, loadFixture } from "./util.js";

test("fixture transport answers queries, hash, log and status", async () => {
  const { client } = fixtureClient();
  const scene = await client.query<SceneReply>("view.scene");
  assert.equal(scene.meshes.length, 1);
  assert.equal(scene.meshes[0]!.cells, 16);
  const h = await client.hash();
  assert.equal(h.state.length, 64);
  const log = await client.log(10);
  assert.equal(log.length, 2);
  assert.equal(log[1]!.parent, null);
  const st = await client.status();
  assert.equal(st.branches.main, log[0]!.node);
});

test("`at` selects the earlier state: no mesh at the first node", async () => {
  const { client } = fixtureClient();
  const log = await client.log(10);
  const first = log[1]!.node;
  const scene = await client.query<SceneReply>("view.scene", {}, first);
  assert.equal(scene.meshes.length, 0);
  const h0 = await client.hash(first);
  const h1 = await client.hash();
  assert.notEqual(h0.state, h1.state);
});

test("clip replies match regardless of key order; missing is an RpcError", async () => {
  const { client, transport } = fixtureClient();
  const clip = { normal: [1, 0, 0], origin: [2, 0, 0] };
  const r = (await client.query("view.mesh_skin", { mesh: "Mesh#1", clip })) as { tri_count: number };
  assert.equal(r.tri_count, 48);
  const all = (await client.query("view.mesh_skin", { mesh: "Mesh#1" })) as { tri_count: number };
  assert.equal(all.tri_count, 80);
  await assert.rejects(client.query("mesh.nothing", {}), (e: unknown) => e instanceof RpcError && e.kind === "fixture");
  assert.equal(transport.log.length, 3);
  assert.equal(canonical({ b: 1, a: [2, { d: 1, c: 2 }] }), canonical({ a: [2, { c: 2, d: 1 }], b: 1 }));
});

test("the console is the only call that mutates: it goes through script.run", async () => {
  const { client, transport } = fixtureClient();
  await client.scriptRun("mesh.smooth iterations=3");
  assert.deepEqual(transport.log.at(-1), { method: "script.run", params: { text: "mesh.smooth iterations=3" } });
});

class FakeWs implements WsLike {
  readyState = 1;
  sent: string[] = [];
  onopen: ((ev: unknown) => void) | null = null;
  onmessage: ((ev: { data: unknown }) => void) | null = null;
  onerror: ((ev: unknown) => void) | null = null;
  onclose: ((ev: unknown) => void) | null = null;
  send(d: string) {
    this.sent.push(d);
  }
  close() {
    this.onclose?.({});
  }
}

const tick = () => new Promise((r) => setTimeout(r, 0));

test("WebSocket transport frames JSON-RPC 2.0 and matches replies by id", async () => {
  const ws = new FakeWs();
  const client = new Client(new WsTransport("ws://x", () => ws));
  ws.onopen?.({});
  const p1 = client.query("view.scene", {});
  const p2 = client.hash();
  await tick();
  assert.equal(ws.sent.length, 2);
  assert.deepEqual(JSON.parse(ws.sent[0]!), { jsonrpc: "2.0", id: 1, method: "query", params: { name: "view.scene", input: {} } });
  assert.equal(JSON.parse(ws.sent[1]!).method, "session.hash");
  // Answer out of order.
  ws.onmessage?.({ data: JSON.stringify({ jsonrpc: "2.0", id: 2, result: { state: "h", layers: {} } }) });
  ws.onmessage?.({ data: JSON.stringify({ jsonrpc: "2.0", id: 1, result: { meshes: [] } }) });
  assert.deepEqual(await p2, { state: "h", layers: {} });
  assert.deepEqual(await p1, { meshes: [] });
  const p3 = client.query("x", {}, "abc");
  await tick();
  assert.deepEqual(JSON.parse(ws.sent[2]!).params.at, { node: "abc" });
  ws.onmessage?.({ data: JSON.stringify({ jsonrpc: "2.0", id: 3, error: { code: -32000, message: "boom", data: { kind: "ambiguous-mesh" } } }) });
  await assert.rejects(p3, (e: unknown) => e instanceof RpcError && e.kind === "ambiguous-mesh" && e.message === "boom");
  const p4 = client.hash();
  await tick();
  ws.close();
  await assert.rejects(p4, RpcError);
});

test("transportFromUrl picks fixture or ws", () => {
  const f = transportFromUrl("?fixture=hexbox", "h");
  assert.equal(f.label, "fixture://hexbox");
  assert.ok(f.client.transport instanceof FixtureTransport);
  assert.throws(() => transportFromUrl("?fixture=nope", "h"), /unknown fixture/);
  assert.equal(loadFixture().name, "hexbox");
});
