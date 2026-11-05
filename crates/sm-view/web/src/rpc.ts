// JSON-RPC client over WebSocket or recorded fixtures. Same decode path for both.
import type { HashReply, LogEntry, ScriptReply, StatusReply } from "./types.js";

/** An error answer (or transport failure) with the server's code and kind. */
export class RpcError extends Error {
  constructor(
    message: string,
    public code: number,
    public kind?: string,
    public data?: unknown,
  ) {
    super(message);
    this.name = "RpcError";
  }
}

export interface Transport {
  call(method: string, params: unknown): Promise<unknown>;
  close(): void;
}

/** Revision selector for `at`: a node hash. */
export type At = string | undefined;

function rev(at: At): unknown {
  return at === undefined ? undefined : { node: at };
}

/** Typed wrappers over a transport. Everything here is read-only except `scriptRun`. */
export class Client {
  constructor(public readonly transport: Transport) {}

  async query<T>(name: string, input: unknown = {}, at?: At): Promise<T> {
    const params: Record<string, unknown> = { name, input };
    if (at !== undefined) params.at = rev(at);
    return (await this.transport.call("query", params)) as T;
  }
  async hash(at?: At): Promise<HashReply> {
    return (await this.transport.call("session.hash", at === undefined ? {} : { at: rev(at) })) as HashReply;
  }
  async status(): Promise<StatusReply> {
    return (await this.transport.call("timeline.status", {})) as StatusReply;
  }
  async log(limit: number): Promise<LogEntry[]> {
    return (await this.transport.call("timeline.log", { limit })) as LogEntry[];
  }
  /** The only mutating call: command text typed in the console. */
  async scriptRun(text: string): Promise<ScriptReply> {
    return (await this.transport.call("script.run", { text })) as ScriptReply;
  }
}

// ----------------------------------------------------------------------------- WebSocket

export interface WsLike {
  readyState: number;
  send(data: string): void;
  close(): void;
  onopen: ((ev: unknown) => void) | null;
  onmessage: ((ev: { data: unknown }) => void) | null;
  onerror: ((ev: unknown) => void) | null;
  onclose: ((ev: unknown) => void) | null;
}

interface Pending {
  resolve: (v: unknown) => void;
  reject: (e: Error) => void;
}

/** Text JSON-RPC 2.0 over one WebSocket; replies are matched by id. */
export class WsTransport implements Transport {
  private ws: WsLike;
  private next = 1;
  private pending = new Map<number, Pending>();
  private opened: Promise<void>;

  constructor(url: string, factory: (url: string) => WsLike = (u) => new WebSocket(u) as unknown as WsLike) {
    this.ws = factory(url);
    this.opened = new Promise((resolve, reject) => {
      this.ws.onopen = () => resolve();
      this.ws.onerror = () => reject(new RpcError(`cannot connect to ${url}`, -1, "transport"));
    });
    this.ws.onmessage = (ev) => this.onMessage(String(ev.data));
    this.ws.onclose = () => {
      for (const p of this.pending.values()) p.reject(new RpcError("connection closed", -1, "transport"));
      this.pending.clear();
    };
  }

  private onMessage(text: string): void {
    let msg: { id?: unknown; result?: unknown; error?: { code: number; message: string; data?: unknown } };
    try {
      msg = JSON.parse(text);
    } catch {
      return;
    }
    if (typeof msg.id !== "number") return;
    const p = this.pending.get(msg.id);
    if (!p) return;
    this.pending.delete(msg.id);
    if (msg.error) {
      const kind = (msg.error.data as { kind?: string } | undefined)?.kind;
      p.reject(new RpcError(msg.error.message, msg.error.code, kind, msg.error.data));
    } else {
      p.resolve(msg.result);
    }
  }

  async call(method: string, params: unknown): Promise<unknown> {
    await this.opened;
    const id = this.next++;
    const text = JSON.stringify({ jsonrpc: "2.0", id, method, params });
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(text);
    });
  }

  close(): void {
    this.ws.close();
  }
}

// ----------------------------------------------------------------------------- fixtures

export interface FixtureEntry {
  /** Fields that must equal the request's (subset match); `{}` matches anything. */
  match: Record<string, unknown>;
  reply: unknown;
}
export interface Fixture {
  format: "smview-fixture/1";
  name: string;
  /** Keyed by query name (for `query`) or method name. */
  replies: Record<string, FixtureEntry[]>;
}

/** JSON with object keys sorted, so equality ignores key order. */
export function canonical(v: unknown): string {
  if (Array.isArray(v)) return `[${v.map(canonical).join(",")}]`;
  if (v !== null && typeof v === "object") {
    const o = v as Record<string, unknown>;
    return `{${Object.keys(o).sort().map((k) => `${JSON.stringify(k)}:${canonical(o[k])}`).join(",")}}`;
  }
  return JSON.stringify(v) ?? "null";
}

function same(a: unknown, b: unknown): boolean {
  return canonical(a) === canonical(b);
}

/** Replays recorded replies. The first entry whose `match` is a subset of the request wins. */
export class FixtureTransport implements Transport {
  /** Requests seen, for tests. */
  readonly log: { method: string; params: unknown }[] = [];
  constructor(public readonly fixture: Fixture) {}

  async call(method: string, params: unknown): Promise<unknown> {
    this.log.push({ method, params });
    const p = (params ?? {}) as Record<string, unknown>;
    let key = method;
    let probe: Record<string, unknown> = p;
    if (method === "query") {
      key = String(p.name);
      probe = { ...((p.input as Record<string, unknown> | null) ?? {}) };
      if (p.at !== undefined) probe["@at"] = p.at;
    }
    const entries = this.fixture.replies[key];
    if (entries) {
      for (const e of entries) {
        if (Object.entries(e.match).every(([k, v]) => same(probe[k], v))) return structuredClone(e.reply);
      }
    }
    throw new RpcError(`no fixture reply for ${key} ${JSON.stringify(probe)}`, -32601, "fixture");
  }

  close(): void {}
}
