// Browser e2e (opt in with SM_VIEWER_E2E=1): headless Chromium on SwiftShader, `fixture://` mode.
//   - renders the ID buffer (never colors) and compares it with src/raster.ts: at most 0.5% mismatches
//   - a real click returns the owner the CPU raster predicts
//   - the clip slider changes the triangle count to the analytic value (48 at x = 2)
//   - a findings row centers the camera on its location within 1e-3 of the bbox diagonal
// A color screenshot is saved to e2e/out/viewer.png as an artifact, never compared.
// Skips with a printed reason when playwright-core or a Chromium binary is missing.
import { build } from "esbuild";
import { existsSync, mkdirSync, readFileSync } from "node:fs";
import { createServer } from "node:http";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const web = join(here, "..");
const out = join(here, "out");

function skip(reason) {
  console.log(`e2e skipped: ${reason}`);
  process.exit(0);
}

if (process.env.SM_VIEWER_E2E !== "1") skip("set SM_VIEWER_E2E=1 to run the browser tests");

let chromium;
try {
  ({ chromium } = await import("playwright-core"));
} catch {
  skip("playwright-core is not installed (npm ci)");
}

const candidates = [
  process.env.CHROMIUM_PATH,
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  "/Applications/Chromium.app/Contents/MacOS/Chromium",
  "/usr/bin/chromium",
  "/usr/bin/chromium-browser",
  "/usr/bin/google-chrome",
].filter(Boolean);
const exe = candidates.find((p) => existsSync(p));
if (!exe) skip("no Chromium or Chrome binary found (set CHROMIUM_PATH)");

mkdirSync(out, { recursive: true });
// The CPU reference: bundle the TypeScript modules the test needs.
await build({
  entryPoints: { ref: join(web, "e2e/ref.ts") },
  bundle: true,
  format: "esm",
  outdir: out,
  outExtension: { ".js": ".mjs" },
  logLevel: "warning",
});
const ref = await import(pathToFileURL(join(out, "ref.mjs")).href);

const server = createServer((req, res) => {
  const path = (req.url ?? "/").split("?")[0];
  const file = join(web, "dist", path === "/" ? "index.html" : path.slice(1));
  if (!file.startsWith(join(web, "dist")) || !existsSync(file)) {
    res.writeHead(404).end();
    return;
  }
  const type = file.endsWith(".js") ? "text/javascript" : file.endsWith(".html") ? "text/html" : "application/json";
  res.writeHead(200, { "content-type": type }).end(readFileSync(file));
});
await new Promise((r) => server.listen(0, "127.0.0.1", r));
const url = `http://127.0.0.1:${server.address().port}/index.html?fixture=hexbox`;

let failures = 0;
const check = (ok, msg) => {
  console.log(`${ok ? "ok  " : "FAIL"} ${msg}`);
  if (!ok) failures++;
};

const browser = await chromium.launch({
  executablePath: exe,
  headless: true,
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist", "--use-gl=angle"],
});
try {
  const page = await browser.newPage({ viewport: { width: 900, height: 520 }, deviceScaleFactor: 1 });
  const errors = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  await page.goto(url);
  await page.waitForFunction(() => globalThis.__smview?.ready === true, null, { timeout: 15000 });
  await page.waitForFunction(() => globalThis.__smview.triCount() > 0, null, { timeout: 15000 });

  const fixture = JSON.parse(readFileSync(join(web, "test/fixtures/hexbox.json"), "utf8"));
  const skinEntry = fixture.replies["view.mesh_skin"].find((e) => Object.keys(e.match).length === 0);
  const skin = ref.decodeSkin(skinEntry.reply);
  check((await page.evaluate(() => globalThis.__smview.triCount())) === 80, "full skin has 80 triangles");

  // 1. ID buffer vs CPU raster, three canonical views.
  for (const dir of ["+x", "+y", "+z"]) {
    const got = await page.evaluate((d) => {
      const s = globalThis.__smview;
      const cam = s.camera();
      s.setCamera(s.lookFrom(cam, d));
      const r = s.readIds();
      return { w: r.width, h: r.height, ids: Array.from(r.ids), cam: s.camera() };
    }, dir);
    const want = ref.rasterIds(skin, ref.viewProj(got.cam), got.w, got.h);
    let bad = 0;
    for (let i = 0; i < want.length; i++) if (want[i] !== got.ids[i]) bad++;
    const frac = bad / want.length;
    check(frac <= 0.005, `ID buffer ${dir}: ${(frac * 100).toFixed(3)}% of ${want.length} pixels differ (limit 0.5%)`);
  }

  // 2. Click returns the predicted owner.
  await page.evaluate(() => {
    const s = globalThis.__smview;
    s.setCamera(s.lookFrom(s.camera(), "+x"));
  });
  const geo = await page.evaluate(() => {
    const c = document.querySelector("canvas");
    const r = c.getBoundingClientRect();
    return { left: r.left, top: r.top, w: c.width, h: c.height, cssW: r.width, cssH: r.height, cam: globalThis.__smview.camera() };
  });
  const px = Math.floor(geo.w / 2) + 5, py = Math.floor(geo.h / 2) + 3;
  const cam = { ...geo.cam, aspect: geo.w / geo.h };
  const ids = ref.rasterIds(skin, ref.viewProj(cam), geo.w, geo.h);
  const tri = ids[py * geo.w + px] - 1;
  const owner = skin.owners[skin.triOwner[tri] - 1];
  await page.mouse.click(geo.left + (px + 0.5) * (geo.cssW / geo.w), geo.top + (py + 0.5) * (geo.cssH / geo.h));
  const info = await page.textContent(".info");
  check(info.startsWith(owner), `click on the +x view returns ${owner} (info: "${info}")`);

  // 3. Clip slider: x at half the box keeps the 2x2x2 half: 24 quads = 48 triangles.
  await page.selectOption("aside select >> nth=0", "x");
  await page.$eval("input[type=range]", (el) => {
    el.value = "2";
    el.dispatchEvent(new Event("change"));
  });
  await page.waitForFunction(() => globalThis.__smview.triCount() !== 80, null, { timeout: 5000 });
  check((await page.evaluate(() => globalThis.__smview.triCount())) === 48, "clip at x=2 gives 48 triangles (analytic)");

  // 4. Findings row centers the camera.
  const f = await page.evaluate(() => globalThis.__smview.findings().map((x) => x.location));
  check(f.some((x) => x !== null), "findings carry locations");
  await page.click(".list .row.warning >> nth=0");
  const after = await page.evaluate(() => globalThis.__smview.camera().target);
  const diag = Math.hypot(4, 2, 2);
  const loc = f.find((x) => x !== null);
  const err = Math.hypot(after[0] - loc[0], after[1] - loc[1], after[2] - loc[2]);
  check(err <= 1e-3 * diag, `camera centered on the finding (error ${err.toExponential(2)})`);

  await page.screenshot({ path: join(out, "viewer.png") });
  check(errors.length === 0, `no page errors${errors.length ? `: ${errors.join("; ")}` : ""}`);
} finally {
  await browser.close();
  server.close();
}
if (failures > 0) {
  console.log(`${failures} e2e check(s) failed`);
  process.exit(1);
}
console.log("e2e passed");
