// W3b viewer live smoke (plan section 5.3): the real page, served by `smesh serve --ws`, in
// headless Chromium on SwiftShader, against the live session.
//
//   node tools/w3b_viewer_smoke.mjs <http://host:port> <out-dir>
//
// Always (Node >= 22, no dependencies): the protocol path. A WebSocket client asks the live
// server for `session.hash`, `view.scene` and `view.mesh_skin` and prints what the viewer will
// draw (volumes, mesh cells, skin triangles).
// When playwright-core (W3B_PLAYWRIGHT_DIR, a directory holding node_modules/playwright-core) and
// a Chromium or Chrome binary (CHROMIUM_PATH or the usual install paths) exist: the browser path.
// The page must reach `__smview.ready`, draw as many skin triangles as the protocol path
// counted, follow a head change within the poll interval (undo, then redo), report no page
// error, and leave a screenshot in <out-dir>/viewer.png (an artifact, never compared).
// Without them it prints `SKIP browser: <reason>` and exits 0 after the protocol path.
//
// Output: `key value` lines, `SKIP ...`, and a final `OK`. Exit 1 on the first failure.
import { existsSync, mkdirSync } from "node:fs";
import { join } from "node:path";
import { pathToFileURL } from "node:url";

const [base, outDir] = process.argv.slice(2);
if (!base || !outDir) {
  console.error("usage: w3b_viewer_smoke.mjs <http://host:port> <out-dir>");
  process.exit(2);
}
mkdirSync(outDir, { recursive: true });
const fail = (msg) => {
  console.log(`FAIL ${msg}`);
  process.exit(1);
};
const wsUrl = base.replace(/^http/, "ws").replace(/\/$/, "");

// ---------------------------------------------------------------- protocol path
let nextId = 1;
const ws = new WebSocket(wsUrl);
await new Promise((resolve, reject) => {
  ws.onopen = resolve;
  ws.onerror = () => reject(new Error(`cannot open ${wsUrl}`));
});
const pending = new Map();
ws.onmessage = (ev) => {
  const m = JSON.parse(String(ev.data));
  const p = pending.get(m.id);
  if (p) {
    pending.delete(m.id);
    m.error ? p.reject(new Error(JSON.stringify(m.error))) : p.resolve(m.result);
  }
};
const call = (method, params) =>
  new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    ws.send(JSON.stringify({ jsonrpc: "2.0", id, method, params }));
  });
const query = (name, input) => call("query", { name, input });

const hash0 = (await call("session.hash", {})).state;
const scene = await query("view.scene", {});
if (scene.meshes.length < 1) fail("the live session has no mesh");
const mesh = scene.meshes[0];
const skin = await query("view.mesh_skin", { mesh: mesh.id });
const skinTris = Math.floor(atob(skin.indices).length / 12);
console.log(`state ${hash0}`);
console.log(`volumes ${scene.volumes.length}`);
console.log(`mesh ${mesh.id} cells ${mesh.cells} nodes ${mesh.nodes}`);
console.log(`skin_triangles ${skinTris}`);
if (skinTris <= 0) fail("empty skin");

// ---------------------------------------------------------------- browser path
const candidates = [
  process.env.CHROMIUM_PATH,
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  "/Applications/Chromium.app/Contents/MacOS/Chromium",
  "/usr/bin/chromium",
  "/usr/bin/chromium-browser",
  "/usr/bin/google-chrome",
].filter(Boolean);
const exe = candidates.find((p) => existsSync(p));
const pwDir = process.env.W3B_PLAYWRIGHT_DIR;
let chromium = null;
let skipReason = null;
if (!exe) skipReason = "no Chromium or Chrome binary found (set CHROMIUM_PATH)";
else if (!pwDir || !existsSync(join(pwDir, "node_modules/playwright-core"))) skipReason = "playwright-core is not installed (set W3B_PLAYWRIGHT_DIR to a directory with node_modules/playwright-core)";
else {
  try {
    ({ chromium } = await import(pathToFileURL(join(pwDir, "node_modules/playwright-core/index.mjs")).href));
  } catch (e) {
    skipReason = `playwright-core cannot be loaded: ${e}`;
  }
}
if (skipReason) {
  console.log(`SKIP browser: ${skipReason}`);
  ws.close();
  console.log("OK");
  process.exit(0);
}

const browser = await chromium.launch({
  executablePath: exe,
  headless: true,
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist", "--use-gl=angle"],
});
try {
  const page = await browser.newPage({ viewport: { width: 900, height: 560 }, deviceScaleFactor: 1 });
  const errors = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  await page.goto(`${base.replace(/\/$/, "")}/`);
  await page.waitForFunction(() => globalThis.__smview?.ready === true, null, { timeout: 30000 });
  await page.waitForFunction(() => globalThis.__smview.triCount() > 0, null, { timeout: 30000 });
  const tris = await page.evaluate(() => globalThis.__smview.triCount());
  console.log(`page_triangles ${tris}`);
  if (tris !== skinTris) fail(`the page draws ${tris} triangles, the server's skin has ${skinTris}`);
  const short = (h) => h.slice(0, 8);
  const shows = (h) => page.waitForFunction((s) => document.body.innerText.includes(s), short(h), { timeout: 15000 });
  await shows(hash0);
  await page.screenshot({ path: join(outDir, "viewer.png") });

  // A head change on the server reaches the page by polling `session.hash`.
  await call("timeline.undo", {});
  const hash1 = (await call("session.hash", {})).state;
  if (hash1 === hash0) fail("undo did not change the state hash");
  await shows(hash1);
  console.log(`followed_undo ${short(hash1)}`);
  await call("timeline.redo", {});
  const hash2 = (await call("session.hash", {})).state;
  if (hash2 !== hash0) fail(`redo did not restore the state hash (${hash2})`);
  await shows(hash0);
  console.log(`followed_redo ${short(hash0)}`);
  if (errors.length > 0) fail(`page errors: ${errors.join("; ")}`);
  console.log("page_errors 0");
} finally {
  await browser.close();
  ws.close();
}
console.log("OK");
