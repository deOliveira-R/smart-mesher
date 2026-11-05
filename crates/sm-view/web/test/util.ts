// Shared test helpers: load the Rust-written golden fixture.
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { decodeSkin } from "../src/codec.js";
import { Client, type Fixture, FixtureTransport } from "../src/rpc.js";
import type { SkinReply } from "../src/types.js";

const here = dirname(fileURLToPath(import.meta.url));

/** Committed fixture path (works from `.build/test/` and from `test/`). */
function fixturePath(name: string): string {
  for (const base of [join(here, "fixtures"), join(here, "../../test/fixtures")]) {
    try {
      readFileSync(join(base, name));
      return join(base, name);
    } catch {
      /* try next */
    }
  }
  throw new Error(`fixture ${name} not found`);
}

export function loadFixture(name = "hexbox.json"): Fixture {
  return JSON.parse(readFileSync(fixturePath(name), "utf8")) as Fixture;
}

export function fixtureClient(): { client: Client; transport: FixtureTransport } {
  const transport = new FixtureTransport(loadFixture());
  return { client: new Client(transport), transport };
}

/** The unclipped skin reply of the fixture (the entry with an empty match). */
export function boxSkinReply(): SkinReply {
  const e = loadFixture().replies["view.mesh_skin"]!.find((x) => Object.keys(x.match).length === 0)!;
  return e.reply as SkinReply;
}

export function boxSkin() {
  return decodeSkin(boxSkinReply());
}

export function triArea(s: { positions: Float32Array; indices: Uint32Array; triCount: number }): number {
  let a = 0;
  for (let t = 0; t < s.triCount; t++) {
    const p = [0, 1, 2].map((k) => {
      const v = s.indices[t * 3 + k]!;
      return [s.positions[v * 3]!, s.positions[v * 3 + 1]!, s.positions[v * 3 + 2]!];
    });
    const u = [p[1]![0]! - p[0]![0]!, p[1]![1]! - p[0]![1]!, p[1]![2]! - p[0]![2]!];
    const w = [p[2]![0]! - p[0]![0]!, p[2]![1]! - p[0]![1]!, p[2]![2]! - p[0]![2]!];
    const n = [u[1]! * w[2]! - u[2]! * w[1]!, u[2]! * w[0]! - u[0]! * w[2]!, u[0]! * w[1]! - u[1]! * w[0]!];
    a += 0.5 * Math.hypot(n[0]!, n[1]!, n[2]!);
  }
  return a;
}
