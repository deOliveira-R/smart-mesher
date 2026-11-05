// Bundle src/main.ts into dist/ (viewer.js, viewer.js.map) and copy index.html.
// Usage: node build.mjs [outdir]   (default: dist)
// Writes dist.sha256 (sha256sum format) only when building into the default dist/.
import { build } from "esbuild";
import { createHash } from "node:crypto";
import { copyFileSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const out = process.argv[2] ?? join(here, "dist");
mkdirSync(out, { recursive: true });

await build({
  entryPoints: [join(here, "src/main.ts")],
  bundle: true,
  format: "esm",
  target: "es2022",
  sourcemap: "linked",
  outfile: join(out, "viewer.js"),
  legalComments: "none",
  logLevel: "warning",
  // Reproducible: no absolute paths in the map.
  absWorkingDir: here,
  sourceRoot: "",
});
copyFileSync(join(here, "src/index.html"), join(out, "index.html"));

if (!process.argv[2]) {
  const lines = readdirSync(out)
    .sort()
    .map((f) => `${createHash("sha256").update(readFileSync(join(out, f))).digest("hex")}  ${f}`);
  writeFileSync(join(here, "dist.sha256"), lines.join("\n") + "\n");
}
