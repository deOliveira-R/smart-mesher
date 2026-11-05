// Entry point: choose a transport from the URL and start the app.
//   index.html?fixture=hexbox     offline replay of a recorded session (fixture://hexbox)
//   index.html?ws=ws://host:port  live server (default: ws://<page host>)
import { App } from "./app.js";
import { Client, type Fixture, FixtureTransport, WsTransport } from "./rpc.js";
import hexbox from "../test/fixtures/hexbox.json" with { type: "json" };

const FIXTURES: Record<string, Fixture> = { hexbox: hexbox as unknown as Fixture };

/** Resolve `fixture://name` or `ws://...` (also `?fixture=` / `?ws=` query forms). */
export function transportFromUrl(search: string, host: string): { client: Client; label: string; steps: number } {
  const q = new URLSearchParams(search);
  const fx = q.get("fixture");
  if (fx !== null) {
    const f = FIXTURES[fx.replace(/^fixture:\/\//, "")];
    if (!f) throw new Error(`unknown fixture "${fx}"; known: ${Object.keys(FIXTURES).join(", ")}`);
    return { client: new Client(new FixtureTransport(f)), label: `fixture://${f.name}`, steps: 4 };
  }
  const url = q.get("ws") ?? `ws://${host}`;
  return { client: new Client(new WsTransport(url)), label: url, steps: 32 };
}

async function main(): Promise<void> {
  const root = document.getElementById("app");
  if (!root) throw new Error("missing #app");
  try {
    const t = transportFromUrl(location.search, location.host);
    // Fixtures never change: no polling.
    const app = new App({ root, client: t.client, clipSteps: t.steps, transport: t.label, pollMs: t.label.startsWith("fixture://") ? 0 : 2000 });
    await app.start();
  } catch (e) {
    root.textContent = e instanceof Error ? e.message : String(e);
  }
}

if (typeof document !== "undefined") void main();
