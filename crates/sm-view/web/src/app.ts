// The viewer application: read-only views plus a one-line `script.run` console.
// Nothing here judges validity or auto-runs an op: it shows what the server answers.
import { type RGBA, BAD_COLOR, edgeLines, expandLines, expandTriangles, faceOfTriangle, ownerColor, scalarColor, skinBatch } from "./batch.js";
import { type Camera, fitToBBox, flyTo, lookFrom, orbit, pan, zoom } from "./camera.js";
import { decodeScalar, decodeSkin, decodeTess } from "./codec.js";
import { Renderer } from "./gl.js";
import { type Client, RpcError } from "./rpc.js";
import type {
  BBox,
  LogEntry,
  QualityReply,
  ScalarReply,
  SceneReply,
  Skin,
  SkinReply,
  StatusReply,
  Tess,
  TessReply,
  ValidateReply,
  Vec3,
} from "./types.js";

export interface AppOptions {
  root: HTMLElement;
  client: Client;
  /** Clip slider resolution (positions 0..steps; `steps` means no clip). */
  clipSteps?: number;
  /** `session.hash` poll interval in ms (0 disables). */
  pollMs?: number;
  /** Label of the transport shown in the header. */
  transport?: string;
}

interface Finding {
  key: string;
  title: string;
  detail: string;
  severity: "error" | "warning" | "info";
  location: Vec3 | null;
  owner: string | null;
  opText: string | null;
}

const METRICS = ["scaled_jacobian", "edge_ratio", "aspect_ratio", "condition", "skew", "shape", "volume"];

function el<K extends keyof HTMLElementTagNameMap>(tag: K, props: Record<string, string> = {}, ...kids: (Node | string)[]): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (k === "class") e.className = v;
    else if (k === "text") e.textContent = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) e.append(k);
  return e;
}

function short(h: string | null | undefined): string {
  return h ? h.slice(0, 10) : "(empty)";
}

function diag(b: BBox): number {
  return Math.hypot(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]);
}

export class App {
  readonly client: Client;
  private root: HTMLElement;
  private canvas!: HTMLCanvasElement;
  private renderer!: Renderer;
  cam: Camera = fitToBBox([[0, 0, 0], [1, 1, 1]]);
  private dirty = true;
  private clipSteps: number;
  private pollMs: number;
  private transportName: string;

  // data
  scene: SceneReply | null = null;
  skin: Skin | null = null;
  skinReply: SkinReply | null = null;
  tess: Tess | null = null;
  scalar: ScalarReply | null = null;
  scalarValues: Float32Array | null = null;
  findings: Finding[] = [];
  log: LogEntry[] = [];
  status: StatusReply | null = null;
  at: string | undefined;
  headHash = "";
  private cache = new Map<string, unknown>();

  // ui state
  private mesh: string | null = null;
  private hidden = new Set<string>();
  private clipAxis = "none";
  private clipPos = 0;
  private metric = "owner";
  private badBelow: number | null = 0.9;
  private highlight: string | null = null;
  private selected: string | null = null;

  // ui elements
  private ui: Record<string, HTMLElement> = {};

  constructor(o: AppOptions) {
    this.client = o.client;
    this.root = o.root;
    this.clipSteps = o.clipSteps ?? 32;
    this.pollMs = o.pollMs ?? 2000;
    this.transportName = o.transport ?? "";
  }

  // ------------------------------------------------------------------ setup

  async start(): Promise<void> {
    this.buildUi();
    this.renderer = new Renderer(this.canvas);
    this.bindCanvas();
    await this.refresh();
    const loop = () => {
      if (this.dirty) {
        this.dirty = false;
        this.renderer.render(this.cam);
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
    if (this.pollMs > 0) setInterval(() => void this.poll(), this.pollMs);
    (globalThis as unknown as { __smview: unknown }).__smview = this.probe();
  }

  /** Test hooks used by the browser e2e (not an API). */
  private probe() {
    return {
      app: this,
      triCount: () => this.skin?.triCount ?? 0,
      skin: () => this.skin,
      camera: () => this.cam,
      lookFrom,
      setCamera: (c: Camera) => {
        this.cam = c;
        this.dirty = true;
      },
      readIds: () => {
        this.cam = { ...this.cam, aspect: this.canvas.width / this.canvas.height };
        return this.renderer.readIds(this.cam, "skin");
      },
      pickAt: (x: number, y: number) => this.pickAt(x, y),
      setClip: (axis: string, pos: number) => this.setClip(axis, pos),
      findings: () => this.findings,
      selectFinding: (i: number) => this.selectFinding(this.findings[i]!),
      ready: true,
    };
  }

  private buildUi(): void {
    const u = this.ui;
    u.header = el("header", {}, el("b", { text: "smart-mesh viewer" }));
    u.state = el("span", { class: "mono", text: "" });
    u.digest = el("span", { class: "mono", text: "" });
    u.mode = el("span", { class: "pill", text: "" });
    u.header.append(" ", u.state, " ", u.digest, " ", u.mode);
    this.canvas = el("canvas", { id: "view" });
    u.info = el("div", { class: "info mono", text: "click a surface to pick" });
    const view = el("div", { class: "viewport" }, this.canvas, u.info);

    // layers
    const layers = el("section", {}, el("h3", { text: "Layers" }));
    for (const name of ["model", "skin", "outline", "edges", "markers"]) {
      const cb = el("input", { type: "checkbox" });
      cb.checked = true;
      cb.addEventListener("change", () => {
        if (cb.checked) this.renderer.visible.add(name);
        else this.renderer.visible.delete(name);
        this.dirty = true;
      });
      layers.append(el("label", {}, cb, ` ${name}`));
    }
    u.volumes = el("div", { class: "volumes" });
    layers.append(u.volumes);

    // clip
    const clip = el("section", {}, el("h3", { text: "Clip plane" }));
    const axis = el("select");
    for (const a of ["none", "x", "y", "z"]) axis.append(el("option", { value: a, text: a }));
    const slider = el("input", { type: "range", min: "0", max: String(this.clipSteps), step: "1", value: String(this.clipSteps) });
    u.clipInfo = el("div", { class: "mono", text: "" });
    const apply = () => void this.setClip(axis.value, Number(slider.value));
    axis.addEventListener("change", apply);
    slider.addEventListener("change", apply);
    clip.append(axis, slider, u.clipInfo);
    u.clipAxis = axis;
    u.clipSlider = slider;

    // quality
    const q = el("section", {}, el("h3", { text: "Color by" }));
    const sel = el("select");
    sel.append(el("option", { value: "owner", text: "model owner" }));
    for (const m of METRICS) sel.append(el("option", { value: m, text: m }));
    const bad = el("input", { type: "number", step: "0.05", value: String(this.badBelow), title: "bad_below (red)" });
    sel.addEventListener("change", () => {
      this.metric = sel.value;
      void this.reloadMesh();
    });
    bad.addEventListener("change", () => {
      this.badBelow = bad.value === "" ? null : Number(bad.value);
      void this.reloadMesh();
    });
    u.scalarInfo = el("div", { class: "mono", text: "" });
    q.append(sel, " bad below ", bad, u.scalarInfo);

    // findings
    const f = el("section", {}, el("h3", { text: "Findings" }));
    u.findings = el("div", { class: "list" });
    u.suggest = el("div", { class: "suggest" });
    f.append(u.findings, u.suggest);

    // timeline
    const t = el("section", {}, el("h3", { text: "Timeline" }));
    u.timeline = el("div", { class: "list" });
    t.append(u.timeline);

    // console
    const c = el("section", {}, el("h3", { text: "Console (script.run)" }));
    const input = el("input", { type: "text", placeholder: "command text, Enter to run", class: "mono" });
    u.consoleOut = el("pre", { class: "mono console" });
    input.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter" && input.value.trim() !== "") {
        const text = input.value;
        input.value = "";
        void this.runScript(text);
      }
    });
    c.append(input, u.consoleOut);

    u.status = el("div", { class: "status mono", text: "" });
    const side = el("aside", {}, layers, clip, q, f, t, c, u.status);
    this.root.append(u.header, el("main", {}, view, side));
  }

  private bindCanvas(): void {
    let drag: { x: number; y: number; moved: number; pan: boolean } | null = null;
    const c = this.canvas;
    c.addEventListener("contextmenu", (e) => e.preventDefault());
    c.addEventListener("pointerdown", (e) => {
      drag = { x: e.clientX, y: e.clientY, moved: 0, pan: e.shiftKey || e.button === 2 };
      c.setPointerCapture(e.pointerId);
    });
    c.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX;
      drag.y = e.clientY;
      drag.moved += Math.abs(dx) + Math.abs(dy);
      if (drag.pan) this.cam = pan(this.cam, dx / c.clientHeight, dy / c.clientHeight);
      else this.cam = orbit(this.cam, -dx * 0.01, dy * 0.01);
      this.dirty = true;
    });
    c.addEventListener("pointerup", (e) => {
      if (drag && drag.moved < 4) {
        const r = c.getBoundingClientRect();
        const s = c.width / r.width;
        this.pickAt((e.clientX - r.left) * s, (e.clientY - r.top) * s);
      }
      drag = null;
    });
    c.addEventListener("wheel", (e) => {
      e.preventDefault();
      this.cam = zoom(this.cam, Math.exp(e.deltaY * 0.001));
      this.dirty = true;
    }, { passive: false });
    new ResizeObserver(() => {
      this.cam = { ...this.cam, aspect: c.clientWidth / Math.max(1, c.clientHeight) };
      this.dirty = true;
    }).observe(c);
  }

  // ------------------------------------------------------------------ data flow

  private async cached<T>(key: string, load: () => Promise<T>): Promise<T> {
    if (this.cache.has(key)) return this.cache.get(key) as T;
    const v = await load();
    this.cache.set(key, v);
    if (this.cache.size > 64) this.cache.delete(this.cache.keys().next().value as string);
    return v;
  }

  private setStatus(msg: string): void {
    this.ui.status!.textContent = msg;
  }

  /** Refetch everything for the current revision (`at`, or HEAD). */
  async refresh(): Promise<void> {
    const at = this.at;
    try {
      const h = await this.client.hash(at);
      if (at === undefined) this.headHash = h.state;
      this.ui.state!.textContent = `state ${short(h.state)}`;
      const scene = await this.cached(`scene|${h.state}`, () => this.client.query<SceneReply>("view.scene", {}, at));
      this.scene = scene;
      if (!this.mesh || !scene.meshes.some((m) => m.id === this.mesh)) this.mesh = scene.meshes[0]?.id ?? null;
      this.ui.digest!.textContent = this.mesh ? `mesh ${short(scene.meshes.find((m) => m.id === this.mesh)?.digest)}` : "no mesh";
      this.ui.mode!.textContent = at === undefined ? `HEAD  ${this.transportName}` : `viewing ${short(at)} (read-only)`;
      this.renderVolumes();
      this.tess = null;
      const vols = scene.volumes.map((v) => v.id);
      if (vols.length > 0) {
        const t = await this.cached(`tess|${h.state}`, () => this.client.query<TessReply>("view.tessellate", { entities: vols }, at));
        this.tess = decodeTess(t);
      }
      this.fitCamera(scene.bbox);
      await this.reloadMesh();
      await this.reloadTimeline();
      this.setStatus("");
    } catch (e) {
      this.setStatus(e instanceof Error ? e.message : String(e));
    }
    this.rebuildModel();
    this.dirty = true;
  }

  private fitCamera(b: BBox): void {
    if (this.fitted) return;
    if (diag(b) > 0) {
      this.fitted = true;
      this.cam = fitToBBox(b, Math.PI / 4, this.canvas.clientWidth / Math.max(1, this.canvas.clientHeight));
    }
  }
  private fitted = false;

  private skinParams(): Record<string, unknown> {
    const p: Record<string, unknown> = { mesh: this.mesh };
    const all = this.scene?.volumes.map((v) => v.id) ?? [];
    if (this.hidden.size > 0 && all.length > 0) p.volumes = all.filter((v) => !this.hidden.has(v));
    if (this.clipAxis !== "none" && this.clipPos < this.clipSteps && this.scene) {
      const b = this.scene.bbox;
      const k = this.clipAxis === "x" ? 0 : this.clipAxis === "y" ? 1 : 2;
      const origin: Vec3 = [b[0][0], b[0][1], b[0][2]];
      origin[k] = b[0][k] + ((b[1][k] - b[0][k]) * this.clipPos) / this.clipSteps;
      const normal: Vec3 = [0, 0, 0];
      normal[k] = 1;
      p.clip = { origin, normal };
    }
    return p;
  }

  /** Reload skin, scalar overlay and findings of the selected mesh. */
  async reloadMesh(): Promise<void> {
    const at = this.at;
    this.skin = null;
    this.skinReply = null;
    this.scalar = null;
    this.scalarValues = null;
    this.findings = [];
    const m = this.scene?.meshes.find((x) => x.id === this.mesh);
    if (!m) {
      this.rebuildSkin();
      this.renderFindings();
      return;
    }
    try {
      const params = this.skinParams();
      const state = this.headForAt();
      const reply = await this.cached(`skin|${m.digest}|${JSON.stringify(params)}`, () => this.client.query<SkinReply>("view.mesh_skin", params, at));
      this.skinReply = reply;
      this.skin = decodeSkin(reply);
      this.ui.clipInfo!.textContent = reply.clipped_cells > 0 ? `${reply.clipped_cells} cells clipped, ${reply.tri_count} triangles` : `${reply.tri_count} triangles`;
      if (this.metric !== "owner") {
        const sp = { mesh: this.mesh, metric: this.metric, ...(this.badBelow !== null ? { bad_below: this.badBelow } : {}) };
        const s = await this.cached(`scalar|${m.digest}|${JSON.stringify(sp)}`, () => this.client.query<ScalarReply>("view.mesh_scalar", sp, at));
        this.scalar = s;
        this.scalarValues = decodeScalar(s);
        this.ui.scalarInfo!.textContent = `${s.metric}: min ${fmt(s.min)} max ${fmt(s.max)} bad ${s.bad_cells}`;
      } else {
        this.ui.scalarInfo!.textContent = "";
      }
      void state;
      await this.loadFindings(m.digest);
    } catch (e) {
      this.setStatus(e instanceof Error ? e.message : String(e));
    }
    this.rebuildSkin();
    this.renderFindings();
    this.dirty = true;
  }

  private headForAt(): string {
    return this.at ?? this.headHash;
  }

  private async loadFindings(digest: string): Promise<void> {
    const at = this.at;
    const out: Finding[] = [];
    const v = await this.cached(`validate|${digest}`, () => this.client.query<ValidateReply>("mesh.validate", { mesh: this.mesh }, at)).catch(() => null);
    for (const x of v?.violations ?? []) {
      out.push({
        key: `v:${x.code}:${out.length}`,
        title: x.code,
        detail: x.detail,
        severity: x.severity,
        location: x.location,
        owner: x.owner,
        opText: null,
      });
    }
    const q = await this.cached(`quality|${digest}`, () => this.client.query<QualityReply>("mesh.quality", { mesh: this.mesh }, at)).catch(() => null);
    if (q) {
      for (const m of q.metrics) {
        for (const w of m.worst.slice(0, 5)) {
          out.push({
            key: `q:${m.metric}:${out.length}`,
            title: `${m.metric} ${w.value.toPrecision(4)}`,
            detail: `grade ${w.grade ?? "-"}`,
            severity: w.grade === "fail" ? "error" : w.grade === "warn" ? "warning" : "info",
            location: w.location,
            owner: w.owner,
            opText: null,
          });
        }
      }
      for (const s of q.suggestions) {
        const where = q.metrics.find((m) => m.metric === s.metric)?.worst[0];
        out.push({
          key: `s:${s.id}`,
          title: `suggestion: ${s.kind}`,
          detail: s.reason,
          severity: s.severity,
          location: where?.location ?? null,
          owner: where?.owner ?? null,
          opText: s.op ? `${s.op} ${JSON.stringify(s.args ?? {})}` : null,
        });
      }
    }
    this.findings = out;
  }

  private async reloadTimeline(): Promise<void> {
    try {
      this.status = await this.client.status();
      this.log = await this.client.log(50);
    } catch {
      this.log = [];
    }
    this.renderTimeline();
  }

  private async poll(): Promise<void> {
    try {
      const h = await this.client.hash();
      if (h.state !== this.headHash) {
        if (this.at === undefined) await this.refresh();
        else {
          this.headHash = h.state;
          await this.reloadTimeline();
        }
      }
    } catch {
      /* transient: try again next tick */
    }
  }

  // ------------------------------------------------------------------ actions

  async setClip(axis: string, pos: number): Promise<void> {
    this.clipAxis = axis;
    this.clipPos = pos;
    (this.ui.clipAxis as HTMLSelectElement).value = axis;
    (this.ui.clipSlider as HTMLInputElement).value = String(pos);
    await this.reloadMesh();
  }

  private async runScript(text: string): Promise<void> {
    const out = this.ui.consoleOut!;
    out.textContent += `> ${text}\n`;
    try {
      const r = await this.client.scriptRun(text);
      out.textContent += `${r.output === null || r.output === undefined ? "ok" : JSON.stringify(r.output)}\n`;
      await this.refresh();
    } catch (e) {
      out.textContent += `${e instanceof RpcError ? `${e.kind ?? e.code}: ${e.message}` : String(e)}\n`;
    }
    out.scrollTop = out.scrollHeight;
  }

  async selectNode(node: string | undefined): Promise<void> {
    this.at = node;
    await this.refresh();
  }

  selectFinding(f: Finding): void {
    this.selected = f.key;
    this.highlight = f.owner;
    if (f.location) this.cam = flyTo(this.cam, f.location);
    const s = this.ui.suggest!;
    s.textContent = "";
    if (f.opText) {
      const pre = el("pre", { class: "mono", text: f.opText });
      const copy = el("button", { text: "copy" });
      copy.addEventListener("click", () => {
        try {
          void navigator.clipboard.writeText(f.opText!);
        } catch {
          /* clipboard unavailable */
        }
      });
      s.append(el("div", { text: f.detail }), pre, copy, el("span", { class: "dim", text: " (shown only; never run automatically)" }));
    } else {
      s.append(el("div", { text: f.detail }));
    }
    this.rebuildSkin();
    this.rebuildModel();
    this.renderFindings();
    this.dirty = true;
  }

  /** Pick at canvas pixel (backing-store coordinates). Returns the topo entity id, if any. */
  pickAt(x: number, y: number): string | null {
    this.cam = { ...this.cam, aspect: this.canvas.width / this.canvas.height };
    const layer = this.skin && this.renderer.visible.has("skin") ? "skin" : "model";
    const { ids, width } = this.renderer.readIds(this.cam, layer);
    const id = ids[Math.floor(y) * width + Math.floor(x)] ?? 0;
    this.dirty = true;
    if (id === 0) {
      this.ui.info!.textContent = "nothing";
      return null;
    }
    const tri = id - 1;
    let owner: string | null = null;
    let text = "";
    if (layer === "skin" && this.skin) {
      const k = this.skin.triOwner[tri]!;
      owner = k === 0 ? null : (this.skin.owners[k - 1] ?? null);
      text = `${owner ?? "no owner"}  skin triangle ${tri}  cell ${this.skin.triCell[tri]}`;
    } else if (this.tess) {
      owner = faceOfTriangle(this.tess, tri);
      text = `${owner ?? "?"}  model triangle ${tri}`;
    }
    this.ui.info!.textContent = text;
    return owner;
  }

  // ------------------------------------------------------------------ scene building

  private skinColor(): (tri: number) => RGBA {
    const s = this.skin!;
    if (this.scalar && this.scalarValues) {
      const sc = this.scalar;
      const lo = sc.min ?? 0, hi = sc.max ?? 1;
      const values = this.scalarValues;
      return (t) => scalarColor(values, s.triCell[t]!, lo, hi, this.badBelow, null);
    }
    const hl = this.highlight;
    return (t) => {
      const k = s.triOwner[t]!;
      const owner = k === 0 ? null : (s.owners[k - 1] ?? null);
      return owner !== null && owner === hl ? [255, 170, 40, 255] : ownerColor(owner);
    };
  }

  private rebuildSkin(): void {
    if (!this.renderer) return;
    const s = this.skin;
    if (!s) {
      this.renderer.setTriangles("skin", null);
      this.renderer.setLines("outline", null, [0, 0, 0, 1]);
      this.rebuildMarkers();
      return;
    }
    this.renderer.setTriangles("skin", skinBatch(s, this.skinColor()));
    this.renderer.setLines("outline", expandLines(s.positions, s.outline), [0.05, 0.05, 0.05, 1]);
    this.rebuildMarkers();
  }

  private rebuildModel(): void {
    if (!this.renderer) return;
    const t = this.tess;
    if (!t) {
      this.renderer.setTriangles("model", null);
      this.renderer.setLines("edges", null, [1, 1, 1, 1]);
      return;
    }
    const hl = this.highlight;
    const body = this.scene?.volumes.find((v) => v.color)?.color ?? null;
    const base: RGBA = body ? [body[0], body[1], body[2], 255] : [150, 160, 175, 255];
    const hatch = new Set(t.faces.filter((f) => f.boundary_only).map((f) => f.face));
    const owner = (tri: number) => faceOfTriangle(t, tri);
    const batch = expandTriangles(
      t.positions,
      t.indices,
      (tri) => (owner(tri) === hl && hl !== null ? [255, 170, 40, 255] : base),
      (tri) => hatch.has(owner(tri) ?? ""),
    );
    this.renderer.setTriangles("model", batch);
    this.renderer.setLines("edges", edgeLines(t), [0.9, 0.9, 0.95, 1]);
    // Model faces sit behind the mesh skin: show them only while the skin is hidden or absent.
    if (this.skin) this.renderer.visible.delete("model");
    else this.renderer.visible.add("model");
  }

  private rebuildMarkers(): void {
    const pts: number[] = [];
    for (const f of this.findings) if (f.location) pts.push(...f.location);
    this.renderer.setLines("markers", pts.length ? Float32Array.from(pts) : null, [1, 0.25, 0.2, 1], true, 11);
  }

  // ------------------------------------------------------------------ panels

  private renderVolumes(): void {
    const box = this.ui.volumes!;
    box.textContent = "";
    for (const v of this.scene?.volumes ?? []) {
      const cb = el("input", { type: "checkbox" });
      cb.checked = !this.hidden.has(v.id);
      cb.addEventListener("change", () => {
        if (cb.checked) this.hidden.delete(v.id);
        else this.hidden.add(v.id);
        void this.reloadMesh();
      });
      const sw = el("span", { class: "swatch" });
      if (v.color) sw.style.background = `rgb(${v.color.join(",")})`;
      box.append(el("label", {}, cb, sw, ` ${v.id}`));
    }
  }

  private renderFindings(): void {
    const box = this.ui.findings!;
    box.textContent = "";
    if (this.findings.length === 0) box.append(el("div", { class: "dim", text: "none" }));
    for (const f of this.findings) {
      const row = el("div", { class: `row ${f.severity}${f.key === this.selected ? " sel" : ""}`, "data-key": f.key });
      row.append(el("b", { text: f.title }), " ", el("span", { class: "dim", text: f.owner ?? "" }));
      row.addEventListener("click", () => this.selectFinding(f));
      box.append(row);
    }
    if (this.scalar) {
      for (const w of this.scalar.worst.slice(0, 5)) {
        const row = el("div", { class: "row info", text: `worst ${w.cell.id}  ${w.value.toPrecision(4)}` });
        row.addEventListener("click", () => {
          this.cam = flyTo(this.cam, w.location);
          this.dirty = true;
        });
        box.append(row);
      }
    }
    void BAD_COLOR;
  }

  private renderTimeline(): void {
    const box = this.ui.timeline!;
    box.textContent = "";
    const tips = new Map<string, string[]>();
    for (const [name, h] of Object.entries(this.status?.branches ?? {})) tips.set(h, [...(tips.get(h) ?? []), name]);
    const head = el("div", { class: `row${this.at === undefined ? " sel" : ""}`, text: "HEAD (live)" });
    head.addEventListener("click", () => void this.selectNode(undefined));
    box.append(head);
    for (const n of this.log) {
      const names = tips.get(n.node)?.join(",") ?? "";
      const row = el("div", { class: `row${this.at === n.node ? " sel" : ""}`, "data-node": n.node });
      row.append(el("span", { class: "mono", text: short(n.node) }), ` ${n.op} `, el("span", { class: "dim", text: names }));
      row.addEventListener("click", () => void this.selectNode(n.node));
      box.append(row);
    }
  }
}

function fmt(x: number | null): string {
  return x === null ? "-" : x.toPrecision(4);
}
