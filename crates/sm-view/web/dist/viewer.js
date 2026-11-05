// src/batch.ts
function expandTriangles(positions, indices, color, hatch = () => false, idOf = (t) => t + 1) {
  const n = indices.length / 3;
  const out = {
    vertexCount: n * 3,
    positions: new Float32Array(n * 9),
    colors: new Uint8Array(n * 12),
    ids: new Uint8Array(n * 12),
    hatch: new Uint8Array(n * 3)
  };
  for (let t = 0; t < n; t++) {
    const c = color(t);
    const id = idOf(t);
    const h = hatch(t) ? 255 : 0;
    for (let k = 0; k < 3; k++) {
      const v = indices[t * 3 + k];
      const o = t * 3 + k;
      out.positions.set(positions.subarray(v * 3, v * 3 + 3), o * 3);
      out.colors.set(c, o * 4);
      out.ids[o * 4] = id & 255;
      out.ids[o * 4 + 1] = id >> 8 & 255;
      out.ids[o * 4 + 2] = id >> 16 & 255;
      out.ids[o * 4 + 3] = 255;
      out.hatch[o] = h;
    }
  }
  return out;
}
function expandLines(positions, pairs) {
  const out = new Float32Array(pairs.length * 3);
  for (let i = 0; i < pairs.length; i++) out.set(positions.subarray(pairs[i] * 3, pairs[i] * 3 + 3), i * 3);
  return out;
}
function edgeLines(t) {
  const segs = [];
  for (const e of t.edges) {
    for (let i = 0; i + 1 < e.count; i++) segs.push(t.edgeIndices[e.first + i], t.edgeIndices[e.first + i + 1]);
  }
  return expandLines(t.positions, Uint32Array.from(segs));
}
function serialOf(id) {
  const n = Number(id.slice(id.indexOf("#") + 1));
  return Number.isFinite(n) ? n : 0;
}
var PALETTE = [
  [76, 120, 168],
  [245, 133, 24],
  [82, 162, 59],
  [228, 87, 86],
  [178, 121, 162],
  [157, 117, 93],
  [242, 142, 171],
  [118, 183, 178],
  [237, 201, 72],
  [186, 176, 172],
  [95, 90, 166],
  [160, 203, 232]
];
function ownerColor(id) {
  if (id === null) return [180, 180, 180, 255];
  const c = PALETTE[serialOf(id) % PALETTE.length];
  return [c[0], c[1], c[2], 255];
}
function ramp(t) {
  const x = Math.max(0, Math.min(1, Number.isFinite(t) ? t : 0));
  const stops = [[40, 30, 120], [30, 130, 160], [60, 180, 110], [250, 230, 60]];
  const s = x * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(s));
  const f = s - i;
  const a = stops[i], b = stops[i + 1];
  return [Math.round(a[0] + (b[0] - a[0]) * f), Math.round(a[1] + (b[1] - a[1]) * f), Math.round(a[2] + (b[2] - a[2]) * f), 255];
}
var BAD_COLOR = [220, 30, 30, 255];
var UNDEFINED_COLOR = [120, 120, 120, 255];
function scalarColor(values, cell, lo, hi, badBelow, badAbove) {
  const v = values[cell];
  if (!Number.isFinite(v)) return UNDEFINED_COLOR;
  if (badBelow !== null && v < badBelow || badAbove !== null && v > badAbove) return BAD_COLOR;
  return ramp(hi > lo ? (v - lo) / (hi - lo) : 0.5);
}
function skinBatch(s, color) {
  return expandTriangles(s.positions, s.indices, color);
}
function faceOfTriangle(t, tri) {
  const first = tri * 3;
  for (const f of t.faces) if (first >= f.first_index && first < f.first_index + f.index_count) return f.face;
  return null;
}

// src/camera.ts
var PITCH_MAX = Math.PI / 2 - 1e-3;
var sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
var add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
var scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
var dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
var cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
var norm = (a) => {
  const l = Math.hypot(a[0], a[1], a[2]);
  return l > 0 ? scale(a, 1 / l) : [0, 0, 0];
};
function eye(c) {
  const cp = Math.cos(c.pitch);
  return add(c.target, scale([cp * Math.cos(c.yaw), cp * Math.sin(c.yaw), Math.sin(c.pitch)], c.distance));
}
function basis(c) {
  const f = norm(sub(c.target, eye(c)));
  const r = norm(cross(f, [0, 0, 1]));
  const u = cross(r, f);
  return { f, r, u };
}
function mul(a, b) {
  const o = new Array(16).fill(0);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
  }
  return o;
}
function viewMatrix(c) {
  const e = eye(c);
  const { f, r, u } = basis(c);
  return [r[0], u[0], -f[0], 0, r[1], u[1], -f[1], 0, r[2], u[2], -f[2], 0, -dot(r, e), -dot(u, e), dot(f, e), 1];
}
function projMatrix(c) {
  const t = 1 / Math.tan(c.fovY / 2);
  const nf = 1 / (c.near - c.far);
  return [t / c.aspect, 0, 0, 0, 0, t, 0, 0, 0, 0, (c.far + c.near) * nf, -1, 0, 0, 2 * c.far * c.near * nf, 0];
}
function viewProj(c) {
  return mul(projMatrix(c), viewMatrix(c));
}
function fitToBBox(b, fovY = Math.PI / 4, aspect = 1) {
  const c = [(b[0][0] + b[1][0]) / 2, (b[0][1] + b[1][1]) / 2, (b[0][2] + b[1][2]) / 2];
  let radius = Math.hypot(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]) / 2;
  if (!(radius > 0)) radius = 1;
  const half = Math.min(fovY / 2, Math.atan(Math.tan(fovY / 2) * aspect));
  const distance = radius / Math.sin(half) * 1.05;
  return { target: c, distance, yaw: 0.7, pitch: 0.5, fovY, aspect, near: Math.max(distance - radius * 1.2, distance * 0.01), far: distance + radius * 1.2 };
}
function orbit(c, dyaw, dpitch) {
  return { ...c, yaw: c.yaw + dyaw, pitch: Math.max(-PITCH_MAX, Math.min(PITCH_MAX, c.pitch + dpitch)) };
}
function zoom(c, k) {
  return { ...c, distance: c.distance * k };
}
function pan(c, dx, dy) {
  const { r, u } = basis(c);
  const h = 2 * c.distance * Math.tan(c.fovY / 2);
  return { ...c, target: add(c.target, add(scale(r, -dx * h * c.aspect), scale(u, dy * h))) };
}
function lookFrom(c, dir) {
  const yaw = { "+x": 0, "-x": Math.PI, "+y": Math.PI / 2, "-y": -Math.PI / 2, "+z": 0, "-z": 0 };
  const pitch = dir === "+z" ? PITCH_MAX : dir === "-z" ? -PITCH_MAX : 0;
  return { ...c, yaw: yaw[dir] ?? c.yaw, pitch };
}
function flyTo(c, p) {
  return { ...c, target: p };
}

// src/codec.ts
var CodecError = class extends Error {
  constructor(message) {
    super(message);
    this.name = "CodecError";
  }
};
var ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
var LUT = new Int16Array(128).fill(-1);
for (let i = 0; i < 64; i++) LUT[ALPHABET.charCodeAt(i)] = i;
function b64ToBytes(s) {
  if (s.length % 4 !== 0) throw new CodecError(`base64 length ${s.length} is not a multiple of 4`);
  if (s.length === 0) return new Uint8Array(0);
  let pad = 0;
  if (s.endsWith("==")) pad = 2;
  else if (s.endsWith("=")) pad = 1;
  const out = new Uint8Array(s.length / 4 * 3 - pad);
  let o = 0;
  for (let i = 0; i < s.length; i += 4) {
    let n = 0;
    for (let k = 0; k < 4; k++) {
      const c = s.charCodeAt(i + k);
      const last = i + 4 === s.length;
      if (c === 61) {
        if (!last || k < 4 - pad) throw new CodecError("misplaced base64 padding");
        n <<= 6;
        continue;
      }
      const v = c < 128 ? LUT[c] : -1;
      if (v < 0) throw new CodecError(`invalid base64 character at ${i + k}`);
      n = n << 6 | v;
    }
    if (o < out.length) out[o++] = n >> 16 & 255;
    if (o < out.length) out[o++] = n >> 8 & 255;
    if (o < out.length) out[o++] = n & 255;
  }
  return out;
}
function hostIsLittleEndian() {
  return new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;
}
function check(bytes, what, expected) {
  if (bytes.length % 4 !== 0) throw new CodecError(`${what}: ${bytes.length} bytes is not a multiple of 4`);
  const n = bytes.length / 4;
  if (expected !== void 0 && n !== expected) throw new CodecError(`${what}: expected ${expected} values, got ${n}`);
  return n;
}
function decodeF32(s, what, expected) {
  const bytes = b64ToBytes(s);
  const n = check(bytes, what, expected);
  const out = new Float32Array(n);
  if (hostIsLittleEndian()) {
    new Uint8Array(out.buffer).set(bytes);
  } else {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    for (let i = 0; i < n; i++) out[i] = dv.getFloat32(i * 4, true);
  }
  return out;
}
function decodeU32(s, what, expected) {
  const bytes = b64ToBytes(s);
  const n = check(bytes, what, expected);
  const out = new Uint32Array(n);
  if (hostIsLittleEndian()) {
    new Uint8Array(out.buffer).set(bytes);
  } else {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    for (let i = 0; i < n; i++) out[i] = dv.getUint32(i * 4, true);
  }
  return out;
}
function checkIndices(idx, limit, what) {
  for (let i = 0; i < idx.length; i++) {
    if (idx[i] >= limit) throw new CodecError(`${what}[${i}] = ${idx[i]} is out of range (${limit} vertices)`);
  }
}
function decodeSkin(r) {
  const positions = decodeF32(r.positions, "positions", r.vertex_count * 3);
  const indices = decodeU32(r.indices, "indices", r.tri_count * 3);
  const triCell = decodeU32(r.tri_cell, "tri_cell", r.tri_count);
  const triOwner = decodeU32(r.tri_owner, "tri_owner", r.tri_count);
  const outline = decodeU32(r.outline_indices, "outline_indices");
  if (outline.length % 2 !== 0) throw new CodecError("outline_indices: odd length");
  checkIndices(indices, r.vertex_count, "indices");
  checkIndices(outline, r.vertex_count, "outline_indices");
  for (let i = 0; i < triOwner.length; i++) {
    if (triOwner[i] > r.owners.length) throw new CodecError(`tri_owner[${i}] = ${triOwner[i]} has no owners entry`);
  }
  return { triCount: r.tri_count, vertexCount: r.vertex_count, positions, indices, triCell, triOwner, owners: r.owners, outline, bbox: r.bbox };
}
function decodeTess(r) {
  const positions = decodeF32(r.positions, "positions", r.vertex_count * 3);
  const indices = decodeU32(r.indices, "indices", r.triangle_count * 3);
  const edgeIndices = decodeU32(r.edge_indices, "edge_indices");
  checkIndices(indices, r.vertex_count, "indices");
  checkIndices(edgeIndices, r.vertex_count, "edge_indices");
  for (const f of r.faces) {
    if (f.first_index + f.index_count > indices.length) throw new CodecError(`face ${f.face}: range past the index buffer`);
  }
  for (const e of r.edges) {
    if (e.first + e.count > edgeIndices.length) throw new CodecError(`edge ${e.edge}: range past the index buffer`);
  }
  return { positions, indices, faces: r.faces, edgeIndices, edges: r.edges };
}
function decodeScalar(r) {
  return decodeF32(r.values, "values", r.count);
}

// src/gl.ts
var TRI_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
layout(location=2) in vec4 aId;
layout(location=3) in float aHatch;
uniform mat4 uVP;
out vec3 vPos; out vec4 vColor; out vec4 vId; out float vHatch;
void main(){ vPos=aPos; vColor=aColor; vId=aId; vHatch=aHatch; gl_Position=uVP*vec4(aPos,1.0); }`;
var TRI_FS = `#version 300 es
precision highp float;
in vec3 vPos; in vec4 vColor; in vec4 vId; in float vHatch;
uniform vec3 uEye; uniform int uIdPass;
out vec4 outColor;
void main(){
  if(uIdPass==1){ outColor=vId; return; }
  vec3 n=normalize(cross(dFdx(vPos),dFdy(vPos)));
  float s=0.35+0.65*abs(dot(n,normalize(uEye-vPos)));
  vec3 c=vColor.rgb*s;
  if(vHatch>0.5 && mod(floor(gl_FragCoord.x+gl_FragCoord.y),8.0)<3.0) c*=0.55;
  outColor=vec4(c,vColor.a);
}`;
var LINE_VS = `#version 300 es
layout(location=0) in vec3 aPos;
uniform mat4 uVP; uniform float uSize;
void main(){ gl_Position=uVP*vec4(aPos,1.0); gl_PointSize=uSize; }`;
var LINE_FS = `#version 300 es
precision highp float;
uniform vec4 uColor; uniform int uRound;
out vec4 outColor;
void main(){
  if(uRound==1){ vec2 d=gl_PointCoord-vec2(0.5); if(dot(d,d)>0.25) discard; }
  outColor=uColor;
}`;
var Renderer = class {
  constructor(canvas) {
    this.canvas = canvas;
    const gl = canvas.getContext("webgl2", { antialias: false, preserveDrawingBuffer: true, alpha: false });
    if (!gl) throw new Error("WebGL2 is not available in this browser");
    this.gl = gl;
    this.tri = this.program(TRI_VS, TRI_FS);
    this.line = this.program(LINE_VS, LINE_FS);
  }
  gl;
  tri;
  line;
  tris = /* @__PURE__ */ new Map();
  lines = /* @__PURE__ */ new Map();
  visible = /* @__PURE__ */ new Set(["skin", "model", "outline", "edges", "markers", "highlight"]);
  program(vs, fs) {
    const gl = this.gl;
    const mk = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(`shader: ${gl.getShaderInfoLog(s)}`);
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, mk(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(`link: ${gl.getProgramInfoLog(p)}`);
    return p;
  }
  buf(data, loc, size, type, normalized) {
    const gl = this.gl;
    const b = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, b);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, size, type, normalized, 0, 0);
    return b;
  }
  /** Replace a triangle layer (`null` clears it). */
  setTriangles(name, b) {
    const gl = this.gl;
    const old = this.tris.get(name);
    if (old) {
      old.buffers.forEach((x) => gl.deleteBuffer(x));
      gl.deleteVertexArray(old.vao);
      this.tris.delete(name);
    }
    if (!b || b.vertexCount === 0) return;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const buffers = [
      this.buf(b.positions, 0, 3, gl.FLOAT, false),
      this.buf(b.colors, 1, 4, gl.UNSIGNED_BYTE, true),
      this.buf(b.ids, 2, 4, gl.UNSIGNED_BYTE, true),
      this.buf(b.hatch, 3, 1, gl.UNSIGNED_BYTE, true)
    ];
    gl.bindVertexArray(null);
    this.tris.set(name, { vao, buffers, count: b.vertexCount });
  }
  /** Replace a line list (`points = false`) or point list (`points = true`); `null` clears it. */
  setLines(name, positions, color, points = false, size = 8) {
    const gl = this.gl;
    const old = this.lines.get(name);
    if (old) {
      gl.deleteBuffer(old.buffer);
      gl.deleteVertexArray(old.vao);
      this.lines.delete(name);
    }
    if (!positions || positions.length === 0) return;
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const buffer = this.buf(positions, 0, 3, gl.FLOAT, false);
    gl.bindVertexArray(null);
    this.lines.set(name, { vao, buffer, count: positions.length / 3, color, mode: points ? gl.POINTS : gl.LINES, size });
  }
  /** Match the canvas backing store to its CSS size. */
  resize() {
    const dpr = Math.min(2, globalThis.devicePixelRatio || 1);
    const w = Math.max(1, Math.floor(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
  }
  drawTris(cam, idPass, only) {
    const gl = this.gl;
    gl.useProgram(this.tri);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.tri, "uVP"), false, new Float32Array(viewProj(cam)));
    gl.uniform3fv(gl.getUniformLocation(this.tri, "uEye"), new Float32Array(eye(cam)));
    gl.uniform1i(gl.getUniformLocation(this.tri, "uIdPass"), idPass ? 1 : 0);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1, 1);
    for (const [name, l] of this.tris) {
      if (only && name !== only || !this.visible.has(name)) continue;
      gl.bindVertexArray(l.vao);
      gl.drawArrays(gl.TRIANGLES, 0, l.count);
    }
    gl.disable(gl.POLYGON_OFFSET_FILL);
  }
  /** Draw the scene. */
  render(cam) {
    const gl = this.gl;
    this.resize();
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0.1, 0.11, 0.13, 1);
    gl.enable(gl.DEPTH_TEST);
    gl.disable(gl.BLEND);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    this.drawTris(cam, false);
    gl.useProgram(this.line);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.line, "uVP"), false, new Float32Array(viewProj(cam)));
    for (const [name, l] of this.lines) {
      if (!this.visible.has(name)) continue;
      gl.uniform4fv(gl.getUniformLocation(this.line, "uColor"), new Float32Array(l.color));
      gl.uniform1f(gl.getUniformLocation(this.line, "uSize"), l.size);
      gl.uniform1i(gl.getUniformLocation(this.line, "uRound"), l.mode === gl.POINTS ? 1 : 0);
      if (l.mode === gl.POINTS) gl.disable(gl.DEPTH_TEST);
      gl.bindVertexArray(l.vao);
      gl.drawArrays(l.mode, 0, l.count);
      gl.enable(gl.DEPTH_TEST);
    }
    gl.bindVertexArray(null);
  }
  /** Render triangle ids (`index + 1`, 0 = background) of one layer and read them back, top row first. */
  readIds(cam, layer) {
    const gl = this.gl;
    this.resize();
    const w = this.canvas.width, h = this.canvas.height;
    gl.viewport(0, 0, w, h);
    gl.clearColor(0, 0, 0, 0);
    gl.enable(gl.DEPTH_TEST);
    gl.disable(gl.BLEND);
    gl.disable(gl.DITHER);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    this.drawTris(cam, true, layer);
    gl.bindVertexArray(null);
    const px = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
    const ids = new Uint32Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const s = ((h - 1 - y) * w + x) * 4;
        ids[y * w + x] = px[s] | px[s + 1] << 8 | px[s + 2] << 16;
      }
    }
    return { ids, width: w, height: h };
  }
};

// src/rpc.ts
var RpcError = class extends Error {
  constructor(message, code, kind, data) {
    super(message);
    this.code = code;
    this.kind = kind;
    this.data = data;
    this.name = "RpcError";
  }
};
function rev(at) {
  return at === void 0 ? void 0 : { node: at };
}
var Client = class {
  constructor(transport) {
    this.transport = transport;
  }
  async query(name, input = {}, at) {
    const params = { name, input };
    if (at !== void 0) params.at = rev(at);
    return await this.transport.call("query", params);
  }
  async hash(at) {
    return await this.transport.call("session.hash", at === void 0 ? {} : { at: rev(at) });
  }
  async status() {
    return await this.transport.call("timeline.status", {});
  }
  async log(limit) {
    return await this.transport.call("timeline.log", { limit });
  }
  /** The only mutating call: command text typed in the console. */
  async scriptRun(text) {
    return await this.transport.call("script.run", { text });
  }
};
var WsTransport = class {
  ws;
  next = 1;
  pending = /* @__PURE__ */ new Map();
  opened;
  constructor(url, factory = (u) => new WebSocket(u)) {
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
  onMessage(text) {
    let msg;
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
      const kind = msg.error.data?.kind;
      p.reject(new RpcError(msg.error.message, msg.error.code, kind, msg.error.data));
    } else {
      p.resolve(msg.result);
    }
  }
  async call(method, params) {
    await this.opened;
    const id = this.next++;
    const text = JSON.stringify({ jsonrpc: "2.0", id, method, params });
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(text);
    });
  }
  close() {
    this.ws.close();
  }
};
function canonical(v) {
  if (Array.isArray(v)) return `[${v.map(canonical).join(",")}]`;
  if (v !== null && typeof v === "object") {
    const o = v;
    return `{${Object.keys(o).sort().map((k) => `${JSON.stringify(k)}:${canonical(o[k])}`).join(",")}}`;
  }
  return JSON.stringify(v) ?? "null";
}
function same(a, b) {
  return canonical(a) === canonical(b);
}
var FixtureTransport = class {
  constructor(fixture) {
    this.fixture = fixture;
  }
  /** Requests seen, for tests. */
  log = [];
  async call(method, params) {
    this.log.push({ method, params });
    const p = params ?? {};
    let key = method;
    let probe = p;
    if (method === "query") {
      key = String(p.name);
      probe = { ...p.input ?? {} };
      if (p.at !== void 0) probe["@at"] = p.at;
    }
    const entries = this.fixture.replies[key];
    if (entries) {
      for (const e of entries) {
        if (Object.entries(e.match).every(([k, v]) => same(probe[k], v))) return structuredClone(e.reply);
      }
    }
    throw new RpcError(`no fixture reply for ${key} ${JSON.stringify(probe)}`, -32601, "fixture");
  }
  close() {
  }
};

// src/app.ts
var METRICS = ["scaled_jacobian", "edge_ratio", "aspect_ratio", "condition", "skew", "shape", "volume"];
function el(tag, props = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (k === "class") e.className = v;
    else if (k === "text") e.textContent = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) e.append(k);
  return e;
}
function short(h) {
  return h ? h.slice(0, 10) : "(empty)";
}
function diag(b) {
  return Math.hypot(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]);
}
var App = class {
  client;
  root;
  canvas;
  renderer;
  cam = fitToBBox([[0, 0, 0], [1, 1, 1]]);
  dirty = true;
  clipSteps;
  pollMs;
  transportName;
  // data
  scene = null;
  skin = null;
  skinReply = null;
  tess = null;
  scalar = null;
  scalarValues = null;
  findings = [];
  log = [];
  status = null;
  at;
  headHash = "";
  cache = /* @__PURE__ */ new Map();
  // ui state
  mesh = null;
  hidden = /* @__PURE__ */ new Set();
  clipAxis = "none";
  clipPos = 0;
  metric = "owner";
  badBelow = 0.9;
  highlight = null;
  selected = null;
  // ui elements
  ui = {};
  constructor(o) {
    this.client = o.client;
    this.root = o.root;
    this.clipSteps = o.clipSteps ?? 32;
    this.pollMs = o.pollMs ?? 2e3;
    this.transportName = o.transport ?? "";
  }
  // ------------------------------------------------------------------ setup
  async start() {
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
    globalThis.__smview = this.probe();
  }
  /** Test hooks used by the browser e2e (not an API). */
  probe() {
    return {
      app: this,
      triCount: () => this.skin?.triCount ?? 0,
      skin: () => this.skin,
      camera: () => this.cam,
      lookFrom,
      setCamera: (c) => {
        this.cam = c;
        this.dirty = true;
      },
      readIds: () => {
        this.cam = { ...this.cam, aspect: this.canvas.width / this.canvas.height };
        return this.renderer.readIds(this.cam, "skin");
      },
      pickAt: (x, y) => this.pickAt(x, y),
      setClip: (axis, pos) => this.setClip(axis, pos),
      findings: () => this.findings,
      selectFinding: (i) => this.selectFinding(this.findings[i]),
      ready: true
    };
  }
  buildUi() {
    const u = this.ui;
    u.header = el("header", {}, el("b", { text: "smart-mesh viewer" }));
    u.state = el("span", { class: "mono", text: "" });
    u.digest = el("span", { class: "mono", text: "" });
    u.mode = el("span", { class: "pill", text: "" });
    u.header.append(" ", u.state, " ", u.digest, " ", u.mode);
    this.canvas = el("canvas", { id: "view" });
    u.info = el("div", { class: "info mono", text: "click a surface to pick" });
    const view = el("div", { class: "viewport" }, this.canvas, u.info);
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
    const f = el("section", {}, el("h3", { text: "Findings" }));
    u.findings = el("div", { class: "list" });
    u.suggest = el("div", { class: "suggest" });
    f.append(u.findings, u.suggest);
    const t = el("section", {}, el("h3", { text: "Timeline" }));
    u.timeline = el("div", { class: "list" });
    t.append(u.timeline);
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
  bindCanvas() {
    let drag = null;
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
      this.cam = zoom(this.cam, Math.exp(e.deltaY * 1e-3));
      this.dirty = true;
    }, { passive: false });
    new ResizeObserver(() => {
      this.cam = { ...this.cam, aspect: c.clientWidth / Math.max(1, c.clientHeight) };
      this.dirty = true;
    }).observe(c);
  }
  // ------------------------------------------------------------------ data flow
  async cached(key, load) {
    if (this.cache.has(key)) return this.cache.get(key);
    const v = await load();
    this.cache.set(key, v);
    if (this.cache.size > 64) this.cache.delete(this.cache.keys().next().value);
    return v;
  }
  setStatus(msg) {
    this.ui.status.textContent = msg;
  }
  /** Refetch everything for the current revision (`at`, or HEAD). */
  async refresh() {
    const at = this.at;
    try {
      const h = await this.client.hash(at);
      if (at === void 0) this.headHash = h.state;
      this.ui.state.textContent = `state ${short(h.state)}`;
      const scene = await this.cached(`scene|${h.state}`, () => this.client.query("view.scene", {}, at));
      this.scene = scene;
      if (!this.mesh || !scene.meshes.some((m) => m.id === this.mesh)) this.mesh = scene.meshes[0]?.id ?? null;
      this.ui.digest.textContent = this.mesh ? `mesh ${short(scene.meshes.find((m) => m.id === this.mesh)?.digest)}` : "no mesh";
      this.ui.mode.textContent = at === void 0 ? `HEAD  ${this.transportName}` : `viewing ${short(at)} (read-only)`;
      this.renderVolumes();
      this.tess = null;
      const vols = scene.volumes.map((v) => v.id);
      if (vols.length > 0) {
        const t = await this.cached(`tess|${h.state}`, () => this.client.query("view.tessellate", { entities: vols }, at));
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
  fitCamera(b) {
    if (this.fitted) return;
    if (diag(b) > 0) {
      this.fitted = true;
      this.cam = fitToBBox(b, Math.PI / 4, this.canvas.clientWidth / Math.max(1, this.canvas.clientHeight));
    }
  }
  fitted = false;
  skinParams() {
    const p = { mesh: this.mesh };
    const all = this.scene?.volumes.map((v) => v.id) ?? [];
    if (this.hidden.size > 0 && all.length > 0) p.volumes = all.filter((v) => !this.hidden.has(v));
    if (this.clipAxis !== "none" && this.clipPos < this.clipSteps && this.scene) {
      const b = this.scene.bbox;
      const k = this.clipAxis === "x" ? 0 : this.clipAxis === "y" ? 1 : 2;
      const origin = [b[0][0], b[0][1], b[0][2]];
      origin[k] = b[0][k] + (b[1][k] - b[0][k]) * this.clipPos / this.clipSteps;
      const normal = [0, 0, 0];
      normal[k] = 1;
      p.clip = { origin, normal };
    }
    return p;
  }
  /** Reload skin, scalar overlay and findings of the selected mesh. */
  async reloadMesh() {
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
      const reply = await this.cached(`skin|${m.digest}|${JSON.stringify(params)}`, () => this.client.query("view.mesh_skin", params, at));
      this.skinReply = reply;
      this.skin = decodeSkin(reply);
      this.ui.clipInfo.textContent = reply.clipped_cells > 0 ? `${reply.clipped_cells} cells clipped, ${reply.tri_count} triangles` : `${reply.tri_count} triangles`;
      if (this.metric !== "owner") {
        const sp = { mesh: this.mesh, metric: this.metric, ...this.badBelow !== null ? { bad_below: this.badBelow } : {} };
        const s = await this.cached(`scalar|${m.digest}|${JSON.stringify(sp)}`, () => this.client.query("view.mesh_scalar", sp, at));
        this.scalar = s;
        this.scalarValues = decodeScalar(s);
        this.ui.scalarInfo.textContent = `${s.metric}: min ${fmt(s.min)} max ${fmt(s.max)} bad ${s.bad_cells}`;
      } else {
        this.ui.scalarInfo.textContent = "";
      }
      await this.loadFindings(m.digest);
    } catch (e) {
      this.setStatus(e instanceof Error ? e.message : String(e));
    }
    this.rebuildSkin();
    this.renderFindings();
    this.dirty = true;
  }
  headForAt() {
    return this.at ?? this.headHash;
  }
  async loadFindings(digest) {
    const at = this.at;
    const out = [];
    const v = await this.cached(`validate|${digest}`, () => this.client.query("mesh.validate", { mesh: this.mesh }, at)).catch(() => null);
    for (const x of v?.violations ?? []) {
      out.push({
        key: `v:${x.code}:${out.length}`,
        title: x.code,
        detail: x.detail,
        severity: x.severity,
        location: x.location,
        owner: x.owner,
        opText: null
      });
    }
    const q = await this.cached(`quality|${digest}`, () => this.client.query("mesh.quality", { mesh: this.mesh }, at)).catch(() => null);
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
            opText: null
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
          opText: s.op ? `${s.op} ${JSON.stringify(s.args ?? {})}` : null
        });
      }
    }
    this.findings = out;
  }
  async reloadTimeline() {
    try {
      this.status = await this.client.status();
      this.log = await this.client.log(50);
    } catch {
      this.log = [];
    }
    this.renderTimeline();
  }
  async poll() {
    try {
      const h = await this.client.hash();
      if (h.state !== this.headHash) {
        if (this.at === void 0) await this.refresh();
        else {
          this.headHash = h.state;
          await this.reloadTimeline();
        }
      }
    } catch {
    }
  }
  // ------------------------------------------------------------------ actions
  async setClip(axis, pos) {
    this.clipAxis = axis;
    this.clipPos = pos;
    this.ui.clipAxis.value = axis;
    this.ui.clipSlider.value = String(pos);
    await this.reloadMesh();
  }
  async runScript(text) {
    const out = this.ui.consoleOut;
    out.textContent += `> ${text}
`;
    try {
      const r = await this.client.scriptRun(text);
      out.textContent += `${r.output === null || r.output === void 0 ? "ok" : JSON.stringify(r.output)}
`;
      await this.refresh();
    } catch (e) {
      out.textContent += `${e instanceof RpcError ? `${e.kind ?? e.code}: ${e.message}` : String(e)}
`;
    }
    out.scrollTop = out.scrollHeight;
  }
  async selectNode(node) {
    this.at = node;
    await this.refresh();
  }
  selectFinding(f) {
    this.selected = f.key;
    this.highlight = f.owner;
    if (f.location) this.cam = flyTo(this.cam, f.location);
    const s = this.ui.suggest;
    s.textContent = "";
    if (f.opText) {
      const pre = el("pre", { class: "mono", text: f.opText });
      const copy = el("button", { text: "copy" });
      copy.addEventListener("click", () => {
        try {
          void navigator.clipboard.writeText(f.opText);
        } catch {
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
  pickAt(x, y) {
    this.cam = { ...this.cam, aspect: this.canvas.width / this.canvas.height };
    const layer = this.skin && this.renderer.visible.has("skin") ? "skin" : "model";
    const { ids, width } = this.renderer.readIds(this.cam, layer);
    const id = ids[Math.floor(y) * width + Math.floor(x)] ?? 0;
    this.dirty = true;
    if (id === 0) {
      this.ui.info.textContent = "nothing";
      return null;
    }
    const tri = id - 1;
    let owner = null;
    let text = "";
    if (layer === "skin" && this.skin) {
      const k = this.skin.triOwner[tri];
      owner = k === 0 ? null : this.skin.owners[k - 1] ?? null;
      text = `${owner ?? "no owner"}  skin triangle ${tri}  cell ${this.skin.triCell[tri]}`;
    } else if (this.tess) {
      owner = faceOfTriangle(this.tess, tri);
      text = `${owner ?? "?"}  model triangle ${tri}`;
    }
    this.ui.info.textContent = text;
    return owner;
  }
  // ------------------------------------------------------------------ scene building
  skinColor() {
    const s = this.skin;
    if (this.scalar && this.scalarValues) {
      const sc = this.scalar;
      const lo = sc.min ?? 0, hi = sc.max ?? 1;
      const values = this.scalarValues;
      return (t) => scalarColor(values, s.triCell[t], lo, hi, this.badBelow, null);
    }
    const hl = this.highlight;
    return (t) => {
      const k = s.triOwner[t];
      const owner = k === 0 ? null : s.owners[k - 1] ?? null;
      return owner !== null && owner === hl ? [255, 170, 40, 255] : ownerColor(owner);
    };
  }
  rebuildSkin() {
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
  rebuildModel() {
    if (!this.renderer) return;
    const t = this.tess;
    if (!t) {
      this.renderer.setTriangles("model", null);
      this.renderer.setLines("edges", null, [1, 1, 1, 1]);
      return;
    }
    const hl = this.highlight;
    const body = this.scene?.volumes.find((v) => v.color)?.color ?? null;
    const base = body ? [body[0], body[1], body[2], 255] : [150, 160, 175, 255];
    const hatch = new Set(t.faces.filter((f) => f.boundary_only).map((f) => f.face));
    const owner = (tri) => faceOfTriangle(t, tri);
    const batch = expandTriangles(
      t.positions,
      t.indices,
      (tri) => owner(tri) === hl && hl !== null ? [255, 170, 40, 255] : base,
      (tri) => hatch.has(owner(tri) ?? "")
    );
    this.renderer.setTriangles("model", batch);
    this.renderer.setLines("edges", edgeLines(t), [0.9, 0.9, 0.95, 1]);
    if (this.skin) this.renderer.visible.delete("model");
    else this.renderer.visible.add("model");
  }
  rebuildMarkers() {
    const pts = [];
    for (const f of this.findings) if (f.location) pts.push(...f.location);
    this.renderer.setLines("markers", pts.length ? Float32Array.from(pts) : null, [1, 0.25, 0.2, 1], true, 11);
  }
  // ------------------------------------------------------------------ panels
  renderVolumes() {
    const box = this.ui.volumes;
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
  renderFindings() {
    const box = this.ui.findings;
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
  }
  renderTimeline() {
    const box = this.ui.timeline;
    box.textContent = "";
    const tips = /* @__PURE__ */ new Map();
    for (const [name, h] of Object.entries(this.status?.branches ?? {})) tips.set(h, [...tips.get(h) ?? [], name]);
    const head = el("div", { class: `row${this.at === void 0 ? " sel" : ""}`, text: "HEAD (live)" });
    head.addEventListener("click", () => void this.selectNode(void 0));
    box.append(head);
    for (const n of this.log) {
      const names = tips.get(n.node)?.join(",") ?? "";
      const row = el("div", { class: `row${this.at === n.node ? " sel" : ""}`, "data-node": n.node });
      row.append(el("span", { class: "mono", text: short(n.node) }), ` ${n.op} `, el("span", { class: "dim", text: names }));
      row.addEventListener("click", () => void this.selectNode(n.node));
      box.append(row);
    }
  }
};
function fmt(x) {
  return x === null ? "-" : x.toPrecision(4);
}

// test/fixtures/hexbox.json
var hexbox_default = {
  format: "smview-fixture/1",
  name: "hexbox",
  replies: {
    "mesh.quality": [
      {
        match: {},
        reply: {
          cells: 16,
          dim: 3,
          mesh: "Mesh#1",
          metrics: [
            {
              entity: "cell",
              metric: "scaled_jacobian",
              worst: [
                {
                  entity: {
                    id: "MeshCell#15",
                    index: 14
                  },
                  grade: "warn",
                  location: [
                    2.546691176470589,
                    1.54093137254902,
                    1.535171568627451
                  ],
                  near: [],
                  owner: "TopoVolume#1",
                  value: 0.6526544819170276
                }
              ]
            }
          ],
          profile: "generic",
          schema_version: 1,
          suggestions: [
            {
              args: {
                iterations: 10,
                mesh: "Mesh#1"
              },
              available: true,
              id: "smooth:Mesh#1:scaled_jacobian",
              kind: "smooth",
              metric: "scaled_jacobian",
              op: "mesh.smooth",
              params: {},
              reason: "bad cells cluster around one displaced node; smoothing relaxes it",
              severity: "warning",
              targets: [
                "Mesh#1"
              ]
            }
          ],
          verdict: "warn"
        }
      }
    ],
    "mesh.validate": [
      {
        match: {},
        reply: {
          geometry_checked: true,
          mesh: "Mesh#1",
          schema_version: 1,
          total: 1,
          truncated: false,
          valid: true,
          violations: [
            {
              code: "geom.face_nonplanar",
              detail: "a face of the cell next to the displaced node is not planar",
              entity: {
                id: "MeshCell#15",
                index: 14
              },
              location: [
                2.546691176470589,
                1.54093137254902,
                1.535171568627451
              ],
              near: [],
              owner: "TopoVolume#1",
              severity: "warning"
            }
          ]
        }
      }
    ],
    "script.run": [
      {
        match: {},
        reply: {
          diagnostics: [],
          output: null,
          state: "dd3d57eefa00b912086c13cbc61d349e28e0511ee83be8b83507408a6357dd81"
        }
      }
    ],
    "session.hash": [
      {
        match: {
          at: {
            node: "7d61586ade3f3adae4c99da825fdb0acf91ea64519f88d0473ce605e62dcde0f"
          }
        },
        reply: {
          layers: {},
          state: "e6142fd7d6660bd4e7e60b67348f388abfee092b778cf1b9d656e97ab8183bb4"
        }
      },
      {
        match: {},
        reply: {
          layers: {},
          state: "dd3d57eefa00b912086c13cbc61d349e28e0511ee83be8b83507408a6357dd81"
        }
      }
    ],
    "timeline.log": [
      {
        match: {},
        reply: [
          {
            author: "test",
            depth: 2,
            message: "",
            node: "a88db6d889b3e52c9ae5256c02a8f3e45d2a0b20c97d124a2b6bb5ebb7c9909f",
            op: "test.insert_mesh",
            parent: "7d61586ade3f3adae4c99da825fdb0acf91ea64519f88d0473ce605e62dcde0f",
            time: ""
          },
          {
            author: "test",
            depth: 1,
            message: "",
            node: "7d61586ade3f3adae4c99da825fdb0acf91ea64519f88d0473ce605e62dcde0f",
            op: "test.create_brick",
            parent: null,
            time: ""
          }
        ]
      }
    ],
    "timeline.status": [
      {
        match: {},
        reply: {
          branches: {
            main: "a88db6d889b3e52c9ae5256c02a8f3e45d2a0b20c97d124a2b6bb5ebb7c9909f"
          },
          depth: 2,
          dirty: false,
          head: {
            ref: "main"
          },
          nodes: 2,
          position: "a88db6d889b3e52c9ae5256c02a8f3e45d2a0b20c97d124a2b6bb5ebb7c9909f",
          project: null,
          redo: 0,
          state: "dd3d57eefa00b912086c13cbc61d349e28e0511ee83be8b83507408a6357dd81"
        }
      }
    ],
    "view.mesh_scalar": [
      {
        match: {
          metric: "scaled_jacobian"
        },
        reply: {
          bad_cells: 4,
          count: 16,
          max: 1,
          mean: 0.9296099416620648,
          mesh: "Mesh#1",
          metric: "scaled_jacobian",
          min: 0.6526544819170276,
          schema_version: 1,
          undefined_cells: 0,
          values: "AACAP5soWD+ognI/AACAPwAAgD8/32k/FuhRPwAAgD8AAIA/z9JhPzCjZz8AAIA/AACAP7exaD9dFCc/AACAPw==",
          worst: [
            {
              cell: {
                id: "MeshCell#15",
                index: 14
              },
              location: [
                2.546691176470589,
                1.54093137254902,
                1.535171568627451
              ],
              value: 0.6526544819170276
            },
            {
              cell: {
                id: "MeshCell#7",
                index: 6
              },
              location: [
                2.538657407407407,
                1.5319444444444439,
                0.4881944444444443
              ],
              value: 0.8199476234186709
            },
            {
              cell: {
                id: "MeshCell#2",
                index: 1
              },
              location: [
                1.5524456521739132,
                0.5422101449275363,
                0.5319746376811595
              ],
              value: 0.8443696194114069
            },
            {
              cell: {
                id: "MeshCell#10",
                index: 9
              },
              location: [
                1.5441287878787877,
                0.534469696969697,
                1.4945075757575754
              ],
              value: 0.8821229563969747
            },
            {
              cell: {
                id: "MeshCell#11",
                index: 10
              },
              location: [
                2.531469298245614,
                0.5063596491228071,
                1.5163377192982455
              ],
              value: 0.9048338095913208
            },
            {
              cell: {
                id: "MeshCell#14",
                index: 13
              },
              location: [
                1.5249999999999995,
                1.516666666666666,
                1.5083333333333329
              ],
              value: 0.9089617104926324
            },
            {
              cell: {
                id: "MeshCell#6",
                index: 5
              },
              location: [
                1.5350198412698413,
                1.5101190476190474,
                0.5169642857142857
              ],
              value: 0.9135627073092001
            },
            {
              cell: {
                id: "MeshCell#3",
                index: 2
              },
              location: [
                2.5250000000000004,
                0.5166666666666668,
                0.5083333333333335
              ],
              value: 0.947306158055805
            },
            {
              cell: {
                id: "MeshCell#1",
                index: 0
              },
              location: [
                0.49999999999999983,
                0.5,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#4",
                index: 3
              },
              location: [
                3.5,
                0.5,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#5",
                index: 4
              },
              location: [
                0.5,
                1.4999999999999998,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#8",
                index: 7
              },
              location: [
                3.5,
                1.4999999999999998,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#9",
                index: 8
              },
              location: [
                0.5,
                0.5,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#12",
                index: 11
              },
              location: [
                3.5,
                0.5,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#13",
                index: 12
              },
              location: [
                0.5,
                1.4999999999999998,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#16",
                index: 15
              },
              location: [
                3.5,
                1.4999999999999998,
                1.4999999999999998
              ],
              value: 1
            }
          ],
          worst_end: "low"
        }
      },
      {
        match: {
          metric: "edge_ratio"
        },
        reply: {
          bad_cells: 0,
          count: 16,
          max: 1.6887426837300732,
          mean: 1.2537523050589354,
          mesh: "Mesh#1",
          metric: "edge_ratio",
          min: 1,
          schema_version: 1,
          undefined_cells: 0,
          values: "AACAP/bXqD+4KNg/AACAPwAAgD/VRsQ/gqLJPwAAgD8AAIA/HSauP7go2D8AAIA/AACAP9VGxD+aL64/AACAPw==",
          worst: [
            {
              cell: {
                id: "MeshCell#1",
                index: 0
              },
              location: [
                0.49999999999999983,
                0.5,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#4",
                index: 3
              },
              location: [
                3.5,
                0.5,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#5",
                index: 4
              },
              location: [
                0.5,
                1.4999999999999998,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#8",
                index: 7
              },
              location: [
                3.5,
                1.4999999999999998,
                0.5
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#9",
                index: 8
              },
              location: [
                0.5,
                0.5,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#12",
                index: 11
              },
              location: [
                3.5,
                0.5,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#13",
                index: 12
              },
              location: [
                0.5,
                1.4999999999999998,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#16",
                index: 15
              },
              location: [
                3.5,
                1.4999999999999998,
                1.4999999999999998
              ],
              value: 1
            },
            {
              cell: {
                id: "MeshCell#2",
                index: 1
              },
              location: [
                1.5524456521739132,
                0.5422101449275363,
                0.5319746376811595
              ],
              value: 1.3190905958272918
            },
            {
              cell: {
                id: "MeshCell#10",
                index: 9
              },
              location: [
                1.5441287878787877,
                0.534469696969697,
                1.4945075757575754
              ],
              value: 1.36053806627644
            },
            {
              cell: {
                id: "MeshCell#15",
                index: 14
              },
              location: [
                2.546691176470589,
                1.54093137254902,
                1.535171568627451
              ],
              value: 1.3608276348795432
            },
            {
              cell: {
                id: "MeshCell#6",
                index: 5
              },
              location: [
                1.5350198412698413,
                1.5101190476190474,
                0.5169642857142857
              ],
              value: 1.5334116705410035
            },
            {
              cell: {
                id: "MeshCell#14",
                index: 13
              },
              location: [
                1.5249999999999995,
                1.516666666666666,
                1.5083333333333329
              ],
              value: 1.5334116705410035
            },
            {
              cell: {
                id: "MeshCell#7",
                index: 6
              },
              location: [
                2.538657407407407,
                1.5319444444444439,
                0.4881944444444443
              ],
              value: 1.5752718754175359
            },
            {
              cell: {
                id: "MeshCell#3",
                index: 2
              },
              location: [
                2.5250000000000004,
                0.5166666666666668,
                0.5083333333333335
              ],
              value: 1.6887426837300732
            },
            {
              cell: {
                id: "MeshCell#11",
                index: 10
              },
              location: [
                2.531469298245614,
                0.5063596491228071,
                1.5163377192982455
              ],
              value: 1.6887426837300732
            }
          ],
          worst_end: "low"
        }
      }
    ],
    "view.mesh_skin": [
      {
        match: {
          clip: {
            normal: [
              1,
              0,
              0
            ],
            origin: [
              0,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              0,
              0,
              0
            ]
          ],
          clipped_cells: 16,
          indices: "",
          kept_cells: 0,
          mesh: "Mesh#1",
          node_vertices: 0,
          outline_indices: "",
          owners: [],
          positions: "",
          schema_version: 1,
          tri_cell: "",
          tri_count: 0,
          tri_owner: "",
          vertex_count: 0
        }
      },
      {
        match: {
          clip: {
            normal: [
              1,
              0,
              0
            ],
            origin: [
              1,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              1,
              2,
              2
            ]
          ],
          clipped_cells: 12,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAABAAAAAAAAAADAAAABAAAAAMAAAAFAAAABAAAAAYAAAABAAAABAAAAAEAAAAAAAAABAAAAAUAAAAHAAAABAAAAAcAAAAGAAAAAQAAAAgAAAAJAAAAAQAAAAkAAAACAAAABgAAAAoAAAAIAAAABgAAAAgAAAABAAAABgAAAAcAAAALAAAABgAAAAsAAAAKAAAACgAAAAsAAAAJAAAACgAAAAkAAAAIAAAAAwAAAAIAAAAMAAAAAwAAAAwAAAANAAAABQAAAAMAAAANAAAABQAAAA0AAAAOAAAABQAAAA4AAAAPAAAABQAAAA8AAAAHAAAADgAAAA0AAAAMAAAADgAAAAwAAAAPAAAAAgAAAAkAAAAQAAAAAgAAABAAAAAMAAAABwAAAA8AAAARAAAABwAAABEAAAALAAAACwAAABEAAAAQAAAACwAAABAAAAAJAAAADwAAAAwAAAAQAAAADwAAABAAAAARAAAA",
          kept_cells: 4,
          mesh: "Mesh#1",
          node_vertices: 18,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABgAAAAEAAAAIAAAAAgAAAAMAAAACAAAACQAAAAIAAAAMAAAAAwAAAAUAAAADAAAADQAAAAQAAAAFAAAABAAAAAYAAAAFAAAABwAAAAUAAAAOAAAABgAAAAcAAAAGAAAACgAAAAcAAAALAAAABwAAAA8AAAAIAAAACQAAAAgAAAAKAAAACQAAAAsAAAAJAAAAEAAAAAoAAAALAAAACwAAABEAAAAMAAAADQAAAAwAAAAPAAAADAAAABAAAAANAAAADgAAAA4AAAAPAAAADwAAABEAAAAQAAAAEQAAAA==",
          owners: [
            "TopoFace#1",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AACAPwAAAAAAAAAAAACAPwAAgD8AAAAAAACAPwAAgD8AAIA/AACAPwAAAAAAAIA/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAAAAAAAAgD8AAIA/AACAPwAAAEAAAAAAAACAPwAAAEAAAIA/AAAAAAAAAEAAAAAAAAAAAAAAAEAAAIA/AACAPwAAgD8AAABAAACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAAEAAAABAAAAAAAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAA=",
          tri_count: 32,
          tri_owner: "BgAAAAYAAAACAAAAAgAAAAQAAAAEAAAAAQAAAAEAAAAGAAAABgAAAAQAAAAEAAAAAQAAAAEAAAADAAAAAwAAAAYAAAAGAAAAAgAAAAIAAAABAAAAAQAAAAUAAAAFAAAABgAAAAYAAAABAAAAAQAAAAMAAAADAAAABQAAAAUAAAA=",
          vertex_count: 18
        }
      },
      {
        match: {
          clip: {
            normal: [
              1,
              0,
              0
            ],
            origin: [
              2,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              2.3,
              2,
              2
            ]
          ],
          clipped_cells: 8,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAAAAAAAAQAAAAFAAAAAAAAAAUAAAABAAAAAAAAAAMAAAAGAAAAAAAAAAYAAAAEAAAABwAAAAgAAAAJAAAABwAAAAkAAAAKAAAAAQAAAAcAAAAKAAAAAQAAAAoAAAACAAAAAQAAAAUAAAAIAAAAAQAAAAgAAAAHAAAABAAAAAsAAAAMAAAABAAAAAwAAAAFAAAABAAAAAYAAAANAAAABAAAAA0AAAALAAAACwAAAA0AAAAOAAAACwAAAA4AAAAMAAAACAAAAA8AAAAQAAAACAAAABAAAAAJAAAABQAAAAwAAAAPAAAABQAAAA8AAAAIAAAADAAAAA4AAAAQAAAADAAAABAAAAAPAAAAAwAAAAIAAAARAAAAAwAAABEAAAASAAAAAwAAABIAAAATAAAAAwAAABMAAAAGAAAAEgAAABEAAAAUAAAAEgAAABQAAAATAAAACgAAAAkAAAAVAAAACgAAABUAAAAWAAAAAgAAAAoAAAAWAAAAAgAAABYAAAARAAAAEQAAABYAAAAVAAAAEQAAABUAAAAUAAAABgAAABMAAAAXAAAABgAAABcAAAANAAAADQAAABcAAAAYAAAADQAAABgAAAAOAAAAEwAAABQAAAAYAAAAEwAAABgAAAAXAAAACQAAABAAAAAZAAAACQAAABkAAAAVAAAADgAAABgAAAAZAAAADgAAABkAAAAQAAAAFAAAABUAAAAZAAAAFAAAABkAAAAYAAAA",
          kept_cells: 8,
          mesh: "Mesh#1",
          node_vertices: 26,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAHAAAAAgAAAAMAAAACAAAACgAAAAIAAAARAAAAAwAAAAYAAAADAAAAEgAAAAQAAAAFAAAABAAAAAYAAAAEAAAACwAAAAUAAAAIAAAABQAAAAwAAAAGAAAADQAAAAYAAAATAAAABwAAAAgAAAAHAAAACgAAAAgAAAAJAAAACAAAAA8AAAAJAAAACgAAAAkAAAAQAAAACQAAABUAAAAKAAAAFgAAAAsAAAAMAAAACwAAAA0AAAAMAAAADgAAAAwAAAAPAAAADQAAAA4AAAANAAAAFwAAAA4AAAAQAAAADgAAABgAAAAPAAAAEAAAABAAAAAZAAAAEQAAABIAAAARAAAAFAAAABEAAAAWAAAAEgAAABMAAAATAAAAFAAAABMAAAAXAAAAFAAAABUAAAAUAAAAGAAAABUAAAAWAAAAFQAAABkAAAAXAAAAGAAAABgAAAAZAAAA",
          owners: [
            "TopoFace#1",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAgD8AAIA/AAAAQAAAAAAAAAAAAAAAQAAAgD8AAAAAMzMTQJqZmT/NzIw/AAAAQAAAAAAAAIA/AAAAAAAAAEAAAAAAAACAPwAAAEAAAAAAAAAAAAAAAEAAAIA/AACAPwAAAEAAAIA/AAAAQAAAAEAAAAAAAAAAQAAAAEAAAIA/AACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAgD8AAABAAAAAQAAAgD8AAABAAAAAQAAAAAAAAABAAAAAAAAAAEAAAABAAACAPwAAAEAAAABAAAAAQAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABQAAAAUAAAAFAAAABQAAAAUAAAAFAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACQAAAAkAAAAJAAAACQAAAAkAAAAJAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADQAAAA0AAAANAAAADQAAAA0AAAANAAAA",
          tri_count: 48,
          tri_owner: "AgAAAAIAAAAEAAAABAAAAAEAAAABAAAABgAAAAYAAAACAAAAAgAAAAQAAAAEAAAABAAAAAQAAAABAAAAAQAAAAMAAAADAAAABgAAAAYAAAAEAAAABAAAAAMAAAADAAAAAgAAAAIAAAABAAAAAQAAAAUAAAAFAAAABgAAAAYAAAACAAAAAgAAAAUAAAAFAAAAAQAAAAEAAAADAAAAAwAAAAUAAAAFAAAABgAAAAYAAAADAAAAAwAAAAUAAAAFAAAA",
          vertex_count: 26
        }
      },
      {
        match: {
          clip: {
            normal: [
              1,
              0,
              0
            ],
            origin: [
              3,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              3,
              2,
              2
            ]
          ],
          clipped_cells: 4,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAAAAAAAAQAAAAFAAAAAAAAAAUAAAABAAAAAAAAAAMAAAAGAAAAAAAAAAYAAAAEAAAAAQAAAAcAAAAIAAAAAQAAAAgAAAACAAAAAQAAAAUAAAAJAAAAAQAAAAkAAAAHAAAACgAAAAsAAAAMAAAACgAAAAwAAAANAAAABwAAAAoAAAANAAAABwAAAA0AAAAIAAAABwAAAAkAAAALAAAABwAAAAsAAAAKAAAABAAAAA4AAAAPAAAABAAAAA8AAAAFAAAABAAAAAYAAAAQAAAABAAAABAAAAAOAAAADgAAABAAAAARAAAADgAAABEAAAAPAAAABQAAAA8AAAASAAAABQAAABIAAAAJAAAADwAAABEAAAATAAAADwAAABMAAAASAAAACwAAABQAAAAVAAAACwAAABUAAAAMAAAACQAAABIAAAAUAAAACQAAABQAAAALAAAAEgAAABMAAAAVAAAAEgAAABUAAAAUAAAAAwAAAAIAAAAWAAAAAwAAABYAAAAXAAAAAwAAABcAAAAYAAAAAwAAABgAAAAGAAAAFwAAABYAAAAZAAAAFwAAABkAAAAYAAAAAgAAAAgAAAAaAAAAAgAAABoAAAAWAAAAFgAAABoAAAAbAAAAFgAAABsAAAAZAAAADQAAAAwAAAAcAAAADQAAABwAAAAdAAAACAAAAA0AAAAdAAAACAAAAB0AAAAaAAAAGgAAAB0AAAAcAAAAGgAAABwAAAAbAAAABgAAABgAAAAeAAAABgAAAB4AAAAQAAAAEAAAAB4AAAAfAAAAEAAAAB8AAAARAAAAGAAAABkAAAAfAAAAGAAAAB8AAAAeAAAAEQAAAB8AAAAgAAAAEQAAACAAAAATAAAAGQAAABsAAAAgAAAAGQAAACAAAAAfAAAADAAAABUAAAAhAAAADAAAACEAAAAcAAAAEwAAACAAAAAhAAAAEwAAACEAAAAVAAAAGwAAABwAAAAhAAAAGwAAACEAAAAgAAAA",
          kept_cells: 12,
          mesh: "Mesh#1",
          node_vertices: 34,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAHAAAAAgAAAAMAAAACAAAACAAAAAIAAAAWAAAAAwAAAAYAAAADAAAAFwAAAAQAAAAFAAAABAAAAAYAAAAEAAAADgAAAAUAAAAJAAAABQAAAA8AAAAGAAAAEAAAAAYAAAAYAAAABwAAAAgAAAAHAAAACQAAAAcAAAAKAAAACAAAAA0AAAAIAAAAGgAAAAkAAAALAAAACQAAABIAAAAKAAAACwAAAAoAAAANAAAACwAAAAwAAAALAAAAFAAAAAwAAAANAAAADAAAABUAAAAMAAAAHAAAAA0AAAAdAAAADgAAAA8AAAAOAAAAEAAAAA8AAAARAAAADwAAABIAAAAQAAAAEQAAABAAAAAeAAAAEQAAABMAAAARAAAAHwAAABIAAAATAAAAEgAAABQAAAATAAAAFQAAABMAAAAgAAAAFAAAABUAAAAVAAAAIQAAABYAAAAXAAAAFgAAABkAAAAWAAAAGgAAABcAAAAYAAAAGAAAABkAAAAYAAAAHgAAABkAAAAbAAAAGQAAAB8AAAAaAAAAGwAAABoAAAAdAAAAGwAAABwAAAAbAAAAIAAAABwAAAAdAAAAHAAAACEAAAAeAAAAHwAAAB8AAAAgAAAAIAAAACEAAAA=",
          owners: [
            "TopoFace#1",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAgD8AAIA/AAAAQAAAAAAAAAAAAAAAQAAAAAAAAIA/AAAAQAAAgD8AAAAAAABAQAAAAAAAAAAAAABAQAAAgD8AAAAAAABAQAAAgD8AAIA/AABAQAAAAAAAAIA/AAAAAAAAAEAAAAAAAACAPwAAAEAAAAAAAAAAAAAAAEAAAIA/AACAPwAAAEAAAIA/AAAAQAAAAEAAAAAAAAAAQAAAAEAAAIA/AABAQAAAAEAAAAAAAABAQAAAAEAAAIA/AACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAgD8AAABAAAAAQAAAAAAAAABAAAAAQAAAgD8AAABAAABAQAAAgD8AAABAAABAQAAAAAAAAABAAAAAAAAAAEAAAABAAACAPwAAAEAAAABAAAAAQAAAAEAAAABAAABAQAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAAAEAAAABAAAAAQAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAUAAAAFAAAABQAAAAUAAAAGAAAABgAAAAYAAAAGAAAABgAAAAYAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAJAAAACQAAAAkAAAAJAAAACgAAAAoAAAAKAAAACgAAAAoAAAAKAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADQAAAA0AAAANAAAADQAAAA4AAAAOAAAADgAAAA4AAAAOAAAADgAAAA==",
          tri_count: 64,
          tri_owner: "AgAAAAIAAAAEAAAABAAAAAEAAAABAAAAAgAAAAIAAAAEAAAABAAAAAYAAAAGAAAAAgAAAAIAAAAEAAAABAAAAAQAAAAEAAAAAQAAAAEAAAADAAAAAwAAAAQAAAAEAAAAAwAAAAMAAAAGAAAABgAAAAQAAAAEAAAAAwAAAAMAAAACAAAAAgAAAAEAAAABAAAABQAAAAUAAAACAAAAAgAAAAUAAAAFAAAABgAAAAYAAAACAAAAAgAAAAUAAAAFAAAAAQAAAAEAAAADAAAAAwAAAAUAAAAFAAAAAwAAAAMAAAAFAAAABQAAAAYAAAAGAAAAAwAAAAMAAAAFAAAABQAAAA==",
          vertex_count: 34
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              1,
              0
            ],
            origin: [
              0,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              0,
              0,
              0
            ]
          ],
          clipped_cells: 16,
          indices: "",
          kept_cells: 0,
          mesh: "Mesh#1",
          node_vertices: 0,
          outline_indices: "",
          owners: [],
          positions: "",
          schema_version: 1,
          tri_cell: "",
          tri_count: 0,
          tri_owner: "",
          vertex_count: 0
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              1,
              0
            ],
            origin: [
              0,
              0.5,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              1,
              2
            ]
          ],
          clipped_cells: 12,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAABAAAAAUAAAACAAAABAAAAAIAAAABAAAABgAAAAAAAAADAAAABgAAAAMAAAAHAAAABgAAAAQAAAABAAAABgAAAAEAAAAAAAAABgAAAAcAAAAFAAAABgAAAAUAAAAEAAAACAAAAAkAAAAKAAAACAAAAAoAAAALAAAACgAAAAkAAAAMAAAACgAAAAwAAAANAAAACwAAAA4AAAAPAAAACwAAAA8AAAAIAAAACwAAAAoAAAANAAAACwAAAA0AAAAOAAAADgAAAA0AAAAMAAAADgAAAAwAAAAPAAAAAwAAAAIAAAAQAAAAAwAAABAAAAARAAAABQAAABIAAAAQAAAABQAAABAAAAACAAAABwAAAAMAAAARAAAABwAAABEAAAATAAAABwAAABMAAAASAAAABwAAABIAAAAFAAAAEwAAABEAAAAQAAAAEwAAABAAAAASAAAAFAAAABUAAAAJAAAAFAAAAAkAAAAIAAAACQAAABUAAAAWAAAACQAAABYAAAAMAAAACAAAAA8AAAAXAAAACAAAABcAAAAUAAAADwAAAAwAAAAWAAAADwAAABYAAAAXAAAAFAAAABcAAAAWAAAAFAAAABYAAAAVAAAA",
          kept_cells: 4,
          mesh: "Mesh#1",
          node_vertices: 24,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAGAAAAAQAAAAIAAAABAAAABAAAAAIAAAADAAAAAgAAAAUAAAACAAAAEAAAAAMAAAAHAAAAAwAAABEAAAAEAAAABQAAAAQAAAAGAAAABQAAAAcAAAAFAAAAEgAAAAYAAAAHAAAABwAAABMAAAAIAAAACQAAAAgAAAALAAAACAAAAA8AAAAIAAAAFAAAAAkAAAAKAAAACQAAAAwAAAAJAAAAFQAAAAoAAAALAAAACgAAAA0AAAALAAAADgAAAAwAAAANAAAADAAAAA8AAAAMAAAAFgAAAA0AAAAOAAAADgAAAA8AAAAPAAAAFwAAABAAAAARAAAAEAAAABIAAAARAAAAEwAAABIAAAATAAAAFAAAABUAAAAUAAAAFwAAABUAAAAWAAAAFgAAABcAAAA=",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AACAPwAAAAAAAAAAAACAPwAAgD8AAAAAAACAPwAAgD8AAIA/AACAPwAAAAAAAIA/AAAAAAAAgD8AAAAAAAAAAAAAgD8AAIA/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAIA/AABAQAAAAAAAAIA/AABAQAAAgD8AAIA/AABAQAAAgD8AAAAAAABAQAAAAAAAAAAAAACAQAAAgD8AAIA/AACAQAAAgD8AAAAAAACAQAAAAAAAAAAAAACAQAAAAAAAAIA/AACAPwAAgD8AAABAAACAPwAAAAAAAABAAAAAAAAAgD8AAABAAAAAAAAAAAAAAABAAABAQAAAAAAAAABAAABAQAAAgD8AAABAAACAQAAAgD8AAABAAACAQAAAAAAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACwAAAAsAAAALAAAACwAAAAsAAAALAAAACwAAAAsAAAALAAAACwAAAA==",
          tri_count: 40,
          tri_owner: "BgAAAAYAAAAGAAAABgAAAAMAAAADAAAABAAAAAQAAAABAAAAAQAAAAYAAAAGAAAABgAAAAYAAAADAAAAAwAAAAQAAAAEAAAAAgAAAAIAAAAGAAAABgAAAAYAAAAGAAAAAwAAAAMAAAABAAAAAQAAAAUAAAAFAAAABgAAAAYAAAAGAAAABgAAAAMAAAADAAAAAgAAAAIAAAAFAAAABQAAAA==",
          vertex_count: 24
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              1,
              0
            ],
            origin: [
              0,
              1,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              1.2,
              2
            ]
          ],
          clipped_cells: 8,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAABAAAAAUAAAAGAAAABAAAAAYAAAAHAAAABAAAAAAAAAADAAAABAAAAAMAAAAFAAAABAAAAAcAAAABAAAABAAAAAEAAAAAAAAAAwAAAAIAAAAIAAAAAwAAAAgAAAAJAAAABQAAAAoAAAALAAAABQAAAAsAAAAGAAAABQAAAAMAAAAJAAAABQAAAAkAAAAKAAAACQAAAAgAAAAMAAAACQAAAAwAAAANAAAACgAAAA4AAAAPAAAACgAAAA8AAAALAAAACgAAAAkAAAANAAAACgAAAA0AAAAOAAAADQAAAAwAAAAQAAAADQAAABAAAAARAAAADgAAABIAAAATAAAADgAAABMAAAAPAAAADgAAAA0AAAARAAAADgAAABEAAAASAAAAEgAAABEAAAAQAAAAEgAAABAAAAATAAAAAQAAABQAAAAVAAAAAQAAABUAAAACAAAABwAAAAYAAAAWAAAABwAAABYAAAAXAAAABwAAABcAAAAUAAAABwAAABQAAAABAAAAFwAAABYAAAAVAAAAFwAAABUAAAAUAAAAAgAAABUAAAAYAAAAAgAAABgAAAAIAAAABgAAAAsAAAAZAAAABgAAABkAAAAWAAAAFgAAABkAAAAYAAAAFgAAABgAAAAVAAAACAAAABgAAAAaAAAACAAAABoAAAAMAAAACwAAAA8AAAAbAAAACwAAABsAAAAZAAAAGQAAABsAAAAaAAAAGQAAABoAAAAYAAAADAAAABoAAAAcAAAADAAAABwAAAAQAAAADwAAABMAAAAdAAAADwAAAB0AAAAbAAAAEwAAABAAAAAcAAAAEwAAABwAAAAdAAAAGwAAAB0AAAAcAAAAGwAAABwAAAAaAAAA",
          kept_cells: 8,
          mesh: "Mesh#1",
          node_vertices: 30,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABwAAAAEAAAAUAAAAAgAAAAMAAAACAAAACAAAAAIAAAAVAAAAAwAAAAUAAAADAAAACQAAAAQAAAAFAAAABAAAAAcAAAAFAAAABgAAAAUAAAAKAAAABgAAAAcAAAAGAAAACwAAAAYAAAAWAAAABwAAABcAAAAIAAAACQAAAAgAAAAMAAAACAAAABgAAAAJAAAACgAAAAkAAAANAAAACgAAAAsAAAAKAAAADgAAAAsAAAAPAAAACwAAABkAAAAMAAAADQAAAAwAAAAQAAAADAAAABoAAAANAAAADgAAAA0AAAARAAAADgAAAA8AAAAOAAAAEgAAAA8AAAATAAAADwAAABsAAAAQAAAAEQAAABAAAAATAAAAEAAAABwAAAARAAAAEgAAABIAAAATAAAAEwAAAB0AAAAUAAAAFQAAABQAAAAXAAAAFQAAABYAAAAVAAAAGAAAABYAAAAXAAAAFgAAABkAAAAYAAAAGQAAABgAAAAaAAAAGQAAABsAAAAaAAAAGwAAABoAAAAcAAAAGwAAAB0AAAAcAAAAHQAAAA==",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAgD8AAAAAAAAAAAAAgD8AAIA/AACAPwAAgD8AAIA/AACAPwAAgD8AAAAAAAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/MzMTQJqZmT/NzIw/AAAAQAAAgD8AAAAAAAAAQAAAAAAAAAAAAAAAQAAAAAAAAIA/AABAQAAAgD8AAIA/AABAQAAAgD8AAAAAAABAQAAAAAAAAAAAAABAQAAAAAAAAIA/AACAQAAAgD8AAIA/AACAQAAAgD8AAAAAAACAQAAAAAAAAAAAAACAQAAAAAAAAIA/AAAAAAAAgD8AAABAAACAPwAAgD8AAABAAACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAQAAAgD8AAABAAAAAQAAAAAAAAABAAABAQAAAgD8AAABAAABAQAAAAAAAAABAAACAQAAAgD8AAABAAACAQAAAAAAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACQAAAAkAAAAJAAAACQAAAAkAAAAJAAAACgAAAAoAAAAKAAAACgAAAAoAAAAKAAAACwAAAAsAAAALAAAACwAAAAsAAAALAAAACwAAAAsAAAA=",
          tri_count: 56,
          tri_owner: "BgAAAAYAAAADAAAAAwAAAAQAAAAEAAAAAQAAAAEAAAAGAAAABgAAAAMAAAADAAAABAAAAAQAAAAGAAAABgAAAAMAAAADAAAABAAAAAQAAAAGAAAABgAAAAMAAAADAAAABAAAAAQAAAACAAAAAgAAAAYAAAAGAAAAAwAAAAMAAAABAAAAAQAAAAUAAAAFAAAABgAAAAYAAAADAAAAAwAAAAUAAAAFAAAABgAAAAYAAAADAAAAAwAAAAUAAAAFAAAABgAAAAYAAAADAAAAAwAAAAIAAAACAAAABQAAAAUAAAA=",
          vertex_count: 30
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              1,
              0
            ],
            origin: [
              0,
              1.5,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              2
            ]
          ],
          clipped_cells: 4,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAAAAAAAAQAAAAFAAAAAAAAAAUAAAABAAAAAAAAAAMAAAAGAAAAAAAAAAYAAAAEAAAABQAAAAcAAAAIAAAABQAAAAgAAAAJAAAAAQAAAAoAAAALAAAAAQAAAAsAAAACAAAAAQAAAAUAAAAJAAAAAQAAAAkAAAAKAAAACQAAAAgAAAAMAAAACQAAAAwAAAANAAAACgAAAA4AAAAPAAAACgAAAA8AAAALAAAACgAAAAkAAAANAAAACgAAAA0AAAAOAAAADgAAABAAAAARAAAADgAAABEAAAAPAAAADgAAAA0AAAASAAAADgAAABIAAAAQAAAAEAAAABIAAAATAAAAEAAAABMAAAARAAAABQAAABQAAAAVAAAABQAAABUAAAAHAAAABAAAABYAAAAUAAAABAAAABQAAAAFAAAABAAAAAYAAAAXAAAABAAAABcAAAAWAAAAFgAAABcAAAAVAAAAFgAAABUAAAAUAAAADAAAABgAAAAZAAAADAAAABkAAAANAAAADQAAABkAAAAaAAAADQAAABoAAAASAAAAEgAAABoAAAAbAAAAEgAAABsAAAATAAAAGQAAABgAAAAbAAAAGQAAABsAAAAaAAAAAwAAAAIAAAAcAAAAAwAAABwAAAAdAAAAAwAAAB0AAAAeAAAAAwAAAB4AAAAGAAAAHQAAABwAAAAfAAAAHQAAAB8AAAAeAAAABwAAAB8AAAAgAAAABwAAACAAAAAIAAAAAgAAAAsAAAAhAAAAAgAAACEAAAAcAAAAHAAAACEAAAAgAAAAHAAAACAAAAAfAAAACAAAACAAAAAiAAAACAAAACIAAAAMAAAACwAAAA8AAAAjAAAACwAAACMAAAAhAAAAIQAAACMAAAAiAAAAIQAAACIAAAAgAAAADwAAABEAAAAkAAAADwAAACQAAAAjAAAAEQAAABMAAAAlAAAAEQAAACUAAAAkAAAAIwAAACQAAAAlAAAAIwAAACUAAAAiAAAABwAAABUAAAAmAAAABwAAACYAAAAfAAAABgAAAB4AAAAnAAAABgAAACcAAAAXAAAAFwAAACcAAAAmAAAAFwAAACYAAAAVAAAAHgAAAB8AAAAmAAAAHgAAACYAAAAnAAAAIgAAACgAAAAYAAAAIgAAABgAAAAMAAAAEwAAABsAAAApAAAAEwAAACkAAAAlAAAAGAAAACgAAAApAAAAGAAAACkAAAAbAAAAIgAAACUAAAApAAAAIgAAACkAAAAoAAAA",
          kept_cells: 12,
          mesh: "Mesh#1",
          node_vertices: 42,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAKAAAAAgAAAAMAAAACAAAACwAAAAIAAAAcAAAAAwAAAAYAAAADAAAAHQAAAAQAAAAFAAAABAAAAAYAAAAEAAAAFgAAAAUAAAAHAAAABQAAAAkAAAAFAAAAFAAAAAYAAAAXAAAABgAAAB4AAAAHAAAACAAAAAcAAAAVAAAABwAAAB8AAAAIAAAACQAAAAgAAAAMAAAACAAAACAAAAAJAAAACgAAAAkAAAANAAAACgAAAAsAAAAKAAAADgAAAAsAAAAPAAAACwAAACEAAAAMAAAADQAAAAwAAAAYAAAADAAAACIAAAANAAAADgAAAA0AAAASAAAADQAAABkAAAAOAAAADwAAAA4AAAAQAAAADwAAABEAAAAPAAAAIwAAABAAAAARAAAAEAAAABIAAAARAAAAEwAAABEAAAAkAAAAEgAAABMAAAASAAAAGgAAABMAAAAbAAAAEwAAACUAAAAUAAAAFQAAABQAAAAWAAAAFQAAABcAAAAVAAAAJgAAABYAAAAXAAAAFwAAACcAAAAYAAAAGQAAABgAAAAbAAAAGAAAACgAAAAZAAAAGgAAABoAAAAbAAAAGwAAACkAAAAcAAAAHQAAABwAAAAfAAAAHAAAACEAAAAdAAAAHgAAAB4AAAAfAAAAHgAAACcAAAAfAAAAIAAAAB8AAAAmAAAAIAAAACEAAAAgAAAAIgAAACEAAAAjAAAAIgAAACMAAAAiAAAAJQAAACIAAAAoAAAAIwAAACQAAAAkAAAAJQAAACUAAAApAAAAJgAAACcAAAAoAAAAKQAAAA==",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAgD8AAIA/AACAPwAAgD8AAIA/MzMTQJqZmT/NzIw/AAAAQAAAgD8AAAAAAAAAQAAAAAAAAAAAAAAAQAAAAAAAAIA/AABAQAAAgD8AAIA/AABAQAAAgD8AAAAAAABAQAAAAAAAAAAAAABAQAAAAAAAAIA/AACAQAAAAAAAAAAAAACAQAAAAAAAAIA/AACAQAAAgD8AAAAAAACAQAAAgD8AAIA/AACAPwAAAEAAAAAAAACAPwAAAEAAAIA/AAAAAAAAAEAAAAAAAAAAAAAAAEAAAIA/AABAQAAAAEAAAIA/AABAQAAAAEAAAAAAAACAQAAAAEAAAAAAAACAQAAAAEAAAIA/AACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAgD8AAABAAAAAQAAAgD8AAABAAAAAQAAAAAAAAABAAABAQAAAgD8AAABAAABAQAAAAAAAAABAAACAQAAAAAAAAABAAACAQAAAgD8AAABAAACAPwAAAEAAAABAAAAAAAAAAEAAAABAAABAQAAAAEAAAABAAACAQAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAHAAAABwAAAAcAAAAHAAAABwAAAAcAAAAHAAAABwAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAkAAAAJAAAACQAAAAkAAAAJAAAACQAAAAoAAAAKAAAACgAAAAoAAAAKAAAACgAAAAsAAAALAAAACwAAAAsAAAALAAAACwAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADwAAAA8AAAAPAAAADwAAAA8AAAAPAAAADwAAAA8AAAA=",
          tri_count: 80,
          tri_owner: "AwAAAAMAAAAFAAAABQAAAAEAAAABAAAABwAAAAcAAAADAAAAAwAAAAUAAAAFAAAABwAAAAcAAAADAAAAAwAAAAUAAAAFAAAAAwAAAAMAAAAFAAAABQAAAAIAAAACAAAABwAAAAcAAAAFAAAABQAAAAEAAAABAAAABAAAAAQAAAAHAAAABwAAAAUAAAAFAAAAAgAAAAIAAAAEAAAABAAAAAMAAAADAAAAAQAAAAEAAAAGAAAABgAAAAcAAAAHAAAAAwAAAAMAAAAGAAAABgAAAAcAAAAHAAAAAwAAAAMAAAAGAAAABgAAAAMAAAADAAAAAgAAAAIAAAAGAAAABgAAAAcAAAAHAAAAAQAAAAEAAAAEAAAABAAAAAYAAAAGAAAABwAAAAcAAAACAAAAAgAAAAQAAAAEAAAABgAAAAYAAAA=",
          vertex_count: 42
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              0,
              1
            ],
            origin: [
              0,
              0,
              0
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              0,
              0,
              0
            ]
          ],
          clipped_cells: 16,
          indices: "",
          kept_cells: 0,
          mesh: "Mesh#1",
          node_vertices: 0,
          outline_indices: "",
          owners: [],
          positions: "",
          schema_version: 1,
          tri_cell: "",
          tri_count: 0,
          tri_owner: "",
          vertex_count: 0
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              0,
              1
            ],
            origin: [
              0,
              0,
              0.5
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              1.1
            ]
          ],
          clipped_cells: 11,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAABAAAAAMAAAACAAAABAAAAAIAAAAFAAAABgAAAAAAAAADAAAABgAAAAMAAAAEAAAABgAAAAcAAAABAAAABgAAAAEAAAAAAAAABgAAAAQAAAAFAAAABgAAAAUAAAAHAAAACAAAAAkAAAAKAAAACAAAAAoAAAALAAAACgAAAAkAAAAMAAAACgAAAAwAAAANAAAACAAAAA4AAAAPAAAACAAAAA8AAAAJAAAACwAAABAAAAAOAAAACwAAAA4AAAAIAAAACwAAAAoAAAARAAAACwAAABEAAAAQAAAAEAAAABEAAAAPAAAAEAAAAA8AAAAOAAAAAQAAABIAAAATAAAAAQAAABMAAAACAAAABQAAAAIAAAATAAAABQAAABMAAAAUAAAABwAAABUAAAASAAAABwAAABIAAAABAAAABwAAAAUAAAAUAAAABwAAABQAAAAVAAAAFQAAABQAAAATAAAAFQAAABMAAAASAAAADAAAABYAAAAXAAAADAAAABcAAAANAAAADAAAAAkAAAAYAAAADAAAABgAAAAWAAAADQAAABcAAAAZAAAADQAAABkAAAAKAAAAFwAAABYAAAAYAAAAFwAAABgAAAAZAAAACQAAAA8AAAAaAAAACQAAABoAAAAYAAAACgAAABkAAAAbAAAACgAAABsAAAARAAAAEQAAABsAAAAaAAAAEQAAABoAAAAPAAAAGQAAABgAAAAaAAAAGQAAABoAAAAbAAAA",
          kept_cells: 5,
          mesh: "Mesh#1",
          node_vertices: 28,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAGAAAAAQAAAAIAAAABAAAABwAAAAEAAAASAAAAAgAAAAMAAAACAAAABQAAAAIAAAATAAAAAwAAAAQAAAAEAAAABQAAAAQAAAAGAAAABQAAAAcAAAAFAAAAFAAAAAYAAAAHAAAABwAAABUAAAAIAAAACQAAAAgAAAALAAAACAAAAA4AAAAJAAAACgAAAAkAAAAMAAAACQAAAA8AAAAJAAAAGAAAAAoAAAALAAAACgAAAA0AAAAKAAAAEQAAAAoAAAAZAAAACwAAABAAAAAMAAAADQAAAAwAAAAWAAAADQAAABcAAAAOAAAADwAAAA4AAAAQAAAADwAAABEAAAAPAAAAGgAAABAAAAARAAAAEQAAABsAAAASAAAAEwAAABIAAAAVAAAAEwAAABQAAAAUAAAAFQAAABYAAAAXAAAAFgAAABgAAAAXAAAAGQAAABgAAAAZAAAAGAAAABoAAAAZAAAAGwAAABoAAAAbAAAA",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoVolume#1"
          ],
          positions: "AACAPwAAAAAAAAAAAACAPwAAgD8AAAAAAACAPwAAgD8AAIA/AACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAIA/AAAAAAAAAAAAAAAAAAAAAAAAgD8AAAAAAABAQAAAAAAAAIA/AABAQAAAgD8AAIA/AABAQAAAgD8AAAAAAABAQAAAAAAAAAAAMzMTQJqZmT/NzIw/AAAAQAAAgD8AAAAAAACAQAAAAAAAAIA/AACAQAAAgD8AAIA/AACAQAAAAAAAAAAAAACAQAAAgD8AAAAAAACAPwAAAEAAAAAAAACAPwAAAEAAAIA/AAAAAAAAAEAAAIA/AAAAAAAAAEAAAAAAAAAAQAAAAEAAAIA/AAAAQAAAAEAAAAAAAABAQAAAAEAAAIA/AABAQAAAAEAAAAAAAACAQAAAAEAAAIA/AACAQAAAAEAAAAAA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAMAAAADAAAABgAAAAYAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAGAAAABgAAAAYAAAAGAAAABgAAAAYAAAAGAAAABgAAAAcAAAAHAAAABwAAAAcAAAAHAAAABwAAAAcAAAAHAAAA",
          tri_count: 48,
          tri_owner: "BgAAAAYAAAAGAAAABgAAAAMAAAADAAAABQAAAAUAAAABAAAAAQAAAAYAAAAGAAAABgAAAAYAAAAGAAAABgAAAAMAAAADAAAABQAAAAUAAAACAAAAAgAAAAYAAAAGAAAABgAAAAYAAAAFAAAABQAAAAEAAAABAAAABAAAAAQAAAAGAAAABgAAAAYAAAAGAAAABQAAAAUAAAAEAAAABAAAAAYAAAAGAAAABQAAAAUAAAACAAAAAgAAAAQAAAAEAAAA",
          vertex_count: 28
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              0,
              1
            ],
            origin: [
              0,
              0,
              1
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              1.1
            ]
          ],
          clipped_cells: 8,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAABAAAAAUAAAABAAAABAAAAAEAAAAAAAAABAAAAAYAAAAHAAAABAAAAAcAAAAFAAAABAAAAAAAAAADAAAABAAAAAMAAAAGAAAAAQAAAAgAAAAJAAAAAQAAAAkAAAACAAAABQAAAAoAAAAIAAAABQAAAAgAAAABAAAABQAAAAcAAAALAAAABQAAAAsAAAAKAAAACAAAAAwAAAANAAAACAAAAA0AAAAJAAAACgAAAA4AAAAMAAAACgAAAAwAAAAIAAAACgAAAAsAAAAPAAAACgAAAA8AAAAOAAAADAAAABAAAAARAAAADAAAABEAAAANAAAADgAAABIAAAAQAAAADgAAABAAAAAMAAAADgAAAA8AAAATAAAADgAAABMAAAASAAAAEgAAABMAAAARAAAAEgAAABEAAAAQAAAAAwAAAAIAAAAUAAAAAwAAABQAAAAVAAAABgAAABYAAAAXAAAABgAAABcAAAAHAAAABgAAAAMAAAAVAAAABgAAABUAAAAWAAAAFgAAABUAAAAUAAAAFgAAABQAAAAXAAAAAgAAAAkAAAAYAAAAAgAAABgAAAAUAAAABwAAABcAAAAZAAAABwAAABkAAAALAAAAFwAAABQAAAAYAAAAFwAAABgAAAAZAAAACQAAAA0AAAAaAAAACQAAABoAAAAYAAAACwAAABkAAAAbAAAACwAAABsAAAAPAAAAGQAAABgAAAAaAAAAGQAAABoAAAAbAAAADQAAABEAAAAcAAAADQAAABwAAAAaAAAADwAAABsAAAAdAAAADwAAAB0AAAATAAAAEwAAAB0AAAAcAAAAEwAAABwAAAARAAAAGwAAABoAAAAcAAAAGwAAABwAAAAdAAAA",
          kept_cells: 8,
          mesh: "Mesh#1",
          node_vertices: 30,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAIAAAAAgAAAAMAAAACAAAACQAAAAIAAAAUAAAAAwAAAAYAAAADAAAAFQAAAAQAAAAFAAAABAAAAAYAAAAFAAAABwAAAAUAAAAKAAAABgAAAAcAAAAGAAAAFgAAAAcAAAALAAAABwAAABcAAAAIAAAACQAAAAgAAAAKAAAACAAAAAwAAAAJAAAADQAAAAkAAAAYAAAACgAAAAsAAAAKAAAADgAAAAsAAAAPAAAACwAAABkAAAAMAAAADQAAAAwAAAAOAAAADAAAABAAAAANAAAAEQAAAA0AAAAaAAAADgAAAA8AAAAOAAAAEgAAAA8AAAATAAAADwAAABsAAAAQAAAAEQAAABAAAAASAAAAEQAAABMAAAARAAAAHAAAABIAAAATAAAAEwAAAB0AAAAUAAAAFQAAABQAAAAXAAAAFAAAABgAAAAVAAAAFgAAABYAAAAXAAAAFwAAABkAAAAYAAAAGQAAABgAAAAaAAAAGQAAABsAAAAaAAAAGwAAABoAAAAcAAAAGwAAAB0AAAAcAAAAHQAAAA==",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAAAAAAIA/AACAPwAAAAAAAIA/AACAPwAAgD8AAIA/AAAAAAAAgD8AAIA/AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAQAAAAAAAAIA/MzMTQJqZmT/NzIw/AAAAQAAAAAAAAAAAAAAAQAAAgD8AAAAAAABAQAAAAAAAAIA/AABAQAAAgD8AAIA/AABAQAAAAAAAAAAAAABAQAAAgD8AAAAAAACAQAAAAAAAAIA/AACAQAAAgD8AAIA/AACAQAAAAAAAAAAAAACAQAAAgD8AAAAAAACAPwAAAEAAAIA/AAAAAAAAAEAAAIA/AAAAAAAAAEAAAAAAAACAPwAAAEAAAAAAAAAAQAAAAEAAAIA/AAAAQAAAAEAAAAAAAABAQAAAAEAAAIA/AABAQAAAAEAAAAAAAACAQAAAAEAAAIA/AACAQAAAAEAAAAAA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAACAAAAAgAAAAIAAAACAAAAAgAAAAIAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABQAAAAUAAAAFAAAABQAAAAUAAAAFAAAABgAAAAYAAAAGAAAABgAAAAYAAAAGAAAABwAAAAcAAAAHAAAABwAAAAcAAAAHAAAABwAAAAcAAAA=",
          tri_count: 56,
          tri_owner: "BgAAAAYAAAADAAAAAwAAAAUAAAAFAAAAAQAAAAEAAAAGAAAABgAAAAMAAAADAAAABQAAAAUAAAAGAAAABgAAAAMAAAADAAAABQAAAAUAAAAGAAAABgAAAAMAAAADAAAABQAAAAUAAAACAAAAAgAAAAYAAAAGAAAABQAAAAUAAAABAAAAAQAAAAQAAAAEAAAABgAAAAYAAAAFAAAABQAAAAQAAAAEAAAABgAAAAYAAAAFAAAABQAAAAQAAAAEAAAABgAAAAYAAAAFAAAABQAAAAIAAAACAAAABAAAAAQAAAA=",
          vertex_count: 30
        }
      },
      {
        match: {
          clip: {
            normal: [
              0,
              0,
              1
            ],
            origin: [
              0,
              0,
              1.5
            ]
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              2
            ]
          ],
          clipped_cells: 3,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAAAAAAAAQAAAAFAAAAAAAAAAUAAAABAAAAAAAAAAMAAAAGAAAAAAAAAAYAAAAEAAAAAQAAAAcAAAAIAAAAAQAAAAgAAAACAAAAAQAAAAUAAAAJAAAAAQAAAAkAAAAHAAAACAAAAAoAAAALAAAACAAAAAsAAAAMAAAABwAAAA0AAAAKAAAABwAAAAoAAAAIAAAABwAAAAkAAAAOAAAABwAAAA4AAAANAAAADQAAAA8AAAAQAAAADQAAABAAAAAKAAAADQAAAA4AAAARAAAADQAAABEAAAAPAAAADwAAABEAAAASAAAADwAAABIAAAAQAAAABAAAABMAAAAUAAAABAAAABQAAAAFAAAABAAAAAYAAAAVAAAABAAAABUAAAATAAAAEwAAABUAAAAWAAAAEwAAABYAAAAUAAAAFwAAAAwAAAAYAAAAFwAAABgAAAAWAAAABQAAABQAAAAZAAAABQAAABkAAAAJAAAAFAAAABYAAAAYAAAAFAAAABgAAAAZAAAADAAAAAsAAAAaAAAADAAAABoAAAAYAAAACQAAABkAAAAbAAAACQAAABsAAAAOAAAAGQAAABgAAAAaAAAAGQAAABoAAAAbAAAADgAAABsAAAAcAAAADgAAABwAAAARAAAAEQAAABwAAAAdAAAAEQAAAB0AAAASAAAAGwAAABoAAAAdAAAAGwAAAB0AAAAcAAAAAwAAAAIAAAAeAAAAAwAAAB4AAAAfAAAAAwAAAB8AAAAgAAAAAwAAACAAAAAGAAAAHwAAAB4AAAAhAAAAHwAAACEAAAAgAAAACAAAAAwAAAAiAAAACAAAACIAAAAjAAAAFwAAACEAAAAiAAAAFwAAACIAAAAMAAAAAgAAAAgAAAAjAAAAAgAAACMAAAAeAAAAHgAAACMAAAAiAAAAHgAAACIAAAAhAAAAJAAAACUAAAALAAAAJAAAAAsAAAAKAAAACgAAABAAAAAmAAAACgAAACYAAAAkAAAAEAAAABIAAAAnAAAAEAAAACcAAAAmAAAAJAAAACYAAAAnAAAAJAAAACcAAAAlAAAAFwAAABYAAAAoAAAAFwAAACgAAAAhAAAABgAAACAAAAApAAAABgAAACkAAAAVAAAAFQAAACkAAAAoAAAAFQAAACgAAAAWAAAAIAAAACEAAAAoAAAAIAAAACgAAAApAAAAJQAAACoAAAAaAAAAJQAAABoAAAALAAAAEgAAAB0AAAArAAAAEgAAACsAAAAnAAAAGgAAACoAAAArAAAAGgAAACsAAAAdAAAAJQAAACcAAAArAAAAJQAAACsAAAAqAAAA",
          kept_cells: 13,
          mesh: "Mesh#1",
          node_vertices: 44,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAHAAAAAgAAAAMAAAACAAAACAAAAAIAAAAeAAAAAwAAAAYAAAADAAAAHwAAAAQAAAAFAAAABAAAAAYAAAAEAAAAEwAAAAUAAAAJAAAABQAAABQAAAAGAAAAFQAAAAYAAAAgAAAABwAAAAgAAAAHAAAACQAAAAcAAAANAAAACAAAAAoAAAAIAAAADAAAAAgAAAAjAAAACQAAAA4AAAAJAAAAGQAAAAoAAAALAAAACgAAAA0AAAAKAAAAEAAAAAoAAAAkAAAACwAAAAwAAAALAAAAGgAAAAsAAAAlAAAADAAAABcAAAAMAAAAGAAAAAwAAAAiAAAADQAAAA4AAAANAAAADwAAAA4AAAARAAAADgAAABsAAAAPAAAAEAAAAA8AAAARAAAAEAAAABIAAAAQAAAAJgAAABEAAAASAAAAEQAAABwAAAASAAAAHQAAABIAAAAnAAAAEwAAABQAAAATAAAAFQAAABQAAAAWAAAAFAAAABkAAAAVAAAAFgAAABUAAAApAAAAFgAAABcAAAAWAAAAGAAAABYAAAAoAAAAFwAAACEAAAAYAAAAGQAAABgAAAAaAAAAGQAAABsAAAAaAAAAGwAAABoAAAAdAAAAGgAAACoAAAAbAAAAHAAAABwAAAAdAAAAHQAAACsAAAAeAAAAHwAAAB4AAAAhAAAAHgAAACMAAAAfAAAAIAAAACAAAAAhAAAAIAAAACkAAAAhAAAAIgAAACEAAAAoAAAAIgAAACMAAAAkAAAAJQAAACQAAAAmAAAAJQAAACcAAAAlAAAAKgAAACYAAAAnAAAAJwAAACsAAAAoAAAAKQAAACoAAAArAAAA",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
            "TopoVolume#1"
          ],
          positions: "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAgD8AAIA/AAAAQAAAAAAAAAAAAAAAQAAAAAAAAIA/AAAAQAAAgD8AAAAAAABAQAAAAAAAAIA/AABAQAAAgD8AAIA/MzMTQJqZmT/NzIw/AABAQAAAAAAAAAAAAABAQAAAgD8AAAAAAACAQAAAAAAAAAAAAACAQAAAAAAAAIA/AACAQAAAgD8AAAAAAACAQAAAgD8AAIA/AAAAAAAAAEAAAAAAAACAPwAAAEAAAAAAAAAAAAAAAEAAAIA/AACAPwAAAEAAAIA/AACAPwAAgD8AAIA/AAAAQAAAAEAAAIA/AAAAQAAAAEAAAAAAAABAQAAAAEAAAIA/AABAQAAAAEAAAAAAAACAQAAAAEAAAAAAAACAQAAAAEAAAIA/AACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAgD8AAABAAAAAQAAAgD8AAABAAAAAQAAAAAAAAABAAABAQAAAAAAAAABAAABAQAAAgD8AAABAAACAQAAAAAAAAABAAACAQAAAgD8AAABAAACAPwAAAEAAAABAAAAAAAAAAEAAAABAAABAQAAAAEAAAABAAACAQAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAAAEAAAABAAAAAQAAAAIAAAACAAAAAgAAAAIAAAACAAAAAgAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAUAAAAFAAAABQAAAAUAAAAFAAAABQAAAAYAAAAGAAAABgAAAAYAAAAGAAAABgAAAAcAAAAHAAAABwAAAAcAAAAHAAAABwAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAkAAAAJAAAACQAAAAkAAAAJAAAACQAAAAkAAAAJAAAACwAAAAsAAAALAAAACwAAAAsAAAALAAAACwAAAAsAAAAMAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADAAAAA8AAAAPAAAADwAAAA8AAAAPAAAADwAAAA8AAAAPAAAA",
          tri_count: 84,
          tri_owner: "AwAAAAMAAAAFAAAABQAAAAEAAAABAAAAAwAAAAMAAAAFAAAABQAAAAcAAAAHAAAAAwAAAAMAAAAFAAAABQAAAAMAAAADAAAABQAAAAUAAAACAAAAAgAAAAUAAAAFAAAAAQAAAAEAAAAEAAAABAAAAAcAAAAHAAAABQAAAAUAAAAEAAAABAAAAAcAAAAHAAAABQAAAAUAAAAEAAAABAAAAAUAAAAFAAAAAgAAAAIAAAAEAAAABAAAAAMAAAADAAAAAQAAAAEAAAAGAAAABgAAAAcAAAAHAAAABwAAAAcAAAADAAAAAwAAAAYAAAAGAAAABwAAAAcAAAADAAAAAwAAAAIAAAACAAAABgAAAAYAAAAHAAAABwAAAAEAAAABAAAABAAAAAQAAAAGAAAABgAAAAcAAAAHAAAAAgAAAAIAAAAEAAAABAAAAAYAAAAGAAAA",
          vertex_count: 44
        }
      },
      {
        match: {},
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              2
            ]
          ],
          clipped_cells: 0,
          indices: "AAAAAAEAAAACAAAAAAAAAAIAAAADAAAAAAAAAAQAAAAFAAAAAAAAAAUAAAABAAAAAAAAAAMAAAAGAAAAAAAAAAYAAAAEAAAAAQAAAAcAAAAIAAAAAQAAAAgAAAACAAAAAQAAAAUAAAAJAAAAAQAAAAkAAAAHAAAABwAAAAoAAAALAAAABwAAAAsAAAAIAAAABwAAAAkAAAAMAAAABwAAAAwAAAAKAAAACgAAAA0AAAAOAAAACgAAAA4AAAALAAAACgAAAAwAAAAPAAAACgAAAA8AAAANAAAADQAAAA8AAAAQAAAADQAAABAAAAAOAAAABAAAABEAAAASAAAABAAAABIAAAAFAAAABAAAAAYAAAATAAAABAAAABMAAAARAAAAEQAAABMAAAAUAAAAEQAAABQAAAASAAAABQAAABIAAAAVAAAABQAAABUAAAAJAAAAEgAAABQAAAAWAAAAEgAAABYAAAAVAAAACQAAABUAAAAXAAAACQAAABcAAAAMAAAAFQAAABYAAAAYAAAAFQAAABgAAAAXAAAADAAAABcAAAAZAAAADAAAABkAAAAPAAAADwAAABkAAAAaAAAADwAAABoAAAAQAAAAFwAAABgAAAAaAAAAFwAAABoAAAAZAAAAAwAAAAIAAAAbAAAAAwAAABsAAAAcAAAAAwAAABwAAAAdAAAAAwAAAB0AAAAGAAAAHAAAABsAAAAeAAAAHAAAAB4AAAAdAAAAAgAAAAgAAAAfAAAAAgAAAB8AAAAbAAAAGwAAAB8AAAAgAAAAGwAAACAAAAAeAAAACAAAAAsAAAAhAAAACAAAACEAAAAfAAAAHwAAACEAAAAiAAAAHwAAACIAAAAgAAAACwAAAA4AAAAjAAAACwAAACMAAAAhAAAADgAAABAAAAAkAAAADgAAACQAAAAjAAAAIQAAACMAAAAkAAAAIQAAACQAAAAiAAAABgAAAB0AAAAlAAAABgAAACUAAAATAAAAEwAAACUAAAAmAAAAEwAAACYAAAAUAAAAHQAAAB4AAAAmAAAAHQAAACYAAAAlAAAAFAAAACYAAAAnAAAAFAAAACcAAAAWAAAAHgAAACAAAAAnAAAAHgAAACcAAAAmAAAAFgAAACcAAAAoAAAAFgAAACgAAAAYAAAAIAAAACIAAAAoAAAAIAAAACgAAAAnAAAAEAAAABoAAAApAAAAEAAAACkAAAAkAAAAGAAAACgAAAApAAAAGAAAACkAAAAaAAAAIgAAACQAAAApAAAAIgAAACkAAAAoAAAA",
          kept_cells: 16,
          mesh: "Mesh#1",
          node_vertices: 42,
          outline_indices: "AAAAAAEAAAAAAAAAAwAAAAAAAAAEAAAAAQAAAAIAAAABAAAABQAAAAEAAAAHAAAAAgAAAAMAAAACAAAACAAAAAIAAAAbAAAAAwAAAAYAAAADAAAAHAAAAAQAAAAFAAAABAAAAAYAAAAEAAAAEQAAAAUAAAAJAAAABQAAABIAAAAGAAAAEwAAAAYAAAAdAAAABwAAAAgAAAAHAAAACQAAAAcAAAAKAAAACAAAAAsAAAAIAAAAHwAAAAkAAAAMAAAACQAAABUAAAAKAAAACwAAAAoAAAAMAAAACgAAAA0AAAALAAAADgAAAAsAAAAhAAAADAAAAA8AAAAMAAAAFwAAAA0AAAAOAAAADQAAAA8AAAAOAAAAEAAAAA4AAAAjAAAADwAAABAAAAAPAAAAGQAAABAAAAAaAAAAEAAAACQAAAARAAAAEgAAABEAAAATAAAAEgAAABQAAAASAAAAFQAAABMAAAAUAAAAEwAAACUAAAAUAAAAFgAAABQAAAAmAAAAFQAAABYAAAAVAAAAFwAAABYAAAAYAAAAFgAAACcAAAAXAAAAGAAAABcAAAAZAAAAGAAAABoAAAAYAAAAKAAAABkAAAAaAAAAGgAAACkAAAAbAAAAHAAAABsAAAAeAAAAGwAAAB8AAAAcAAAAHQAAAB0AAAAeAAAAHQAAACUAAAAeAAAAIAAAAB4AAAAmAAAAHwAAACAAAAAfAAAAIQAAACAAAAAiAAAAIAAAACcAAAAhAAAAIgAAACEAAAAjAAAAIgAAACQAAAAiAAAAKAAAACMAAAAkAAAAJAAAACkAAAAlAAAAJgAAACYAAAAnAAAAJwAAACgAAAAoAAAAKQAAAA==",
          owners: [
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6"
          ],
          positions: "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAgD8AAIA/AAAAQAAAAAAAAAAAAAAAQAAAAAAAAIA/AAAAQAAAgD8AAAAAAABAQAAAAAAAAAAAAABAQAAAAAAAAIA/AABAQAAAgD8AAAAAAACAQAAAAAAAAAAAAACAQAAAAAAAAIA/AACAQAAAgD8AAAAAAACAQAAAgD8AAIA/AAAAAAAAAEAAAAAAAACAPwAAAEAAAAAAAAAAAAAAAEAAAIA/AACAPwAAAEAAAIA/AAAAQAAAAEAAAAAAAAAAQAAAAEAAAIA/AABAQAAAAEAAAAAAAABAQAAAAEAAAIA/AACAQAAAAEAAAAAAAACAQAAAAEAAAIA/AACAPwAAAAAAAABAAAAAAAAAAAAAAABAAAAAAAAAgD8AAABAAACAPwAAgD8AAABAAAAAQAAAAAAAAABAAAAAQAAAgD8AAABAAABAQAAAAAAAAABAAABAQAAAgD8AAABAAACAQAAAAAAAAABAAACAQAAAgD8AAABAAAAAAAAAAEAAAABAAACAPwAAAEAAAABAAAAAQAAAAEAAAABAAABAQAAAAEAAAABAAACAQAAAAEAAAABA",
          schema_version: 1,
          tri_cell: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAAAEAAAABAAAAAQAAAAIAAAACAAAAAgAAAAIAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAFAAAABQAAAAUAAAAFAAAABgAAAAYAAAAGAAAABgAAAAcAAAAHAAAABwAAAAcAAAAHAAAABwAAAAgAAAAIAAAACAAAAAgAAAAIAAAACAAAAAkAAAAJAAAACQAAAAkAAAAKAAAACgAAAAoAAAAKAAAACwAAAAsAAAALAAAACwAAAAsAAAALAAAADAAAAAwAAAAMAAAADAAAAAwAAAAMAAAADQAAAA0AAAANAAAADQAAAA4AAAAOAAAADgAAAA4AAAAPAAAADwAAAA8AAAAPAAAADwAAAA8AAAA=",
          tri_count: 80,
          tri_owner: "AwAAAAMAAAAFAAAABQAAAAEAAAABAAAAAwAAAAMAAAAFAAAABQAAAAMAAAADAAAABQAAAAUAAAADAAAAAwAAAAUAAAAFAAAAAgAAAAIAAAAFAAAABQAAAAEAAAABAAAABAAAAAQAAAAFAAAABQAAAAQAAAAEAAAABQAAAAUAAAAEAAAABAAAAAUAAAAFAAAAAgAAAAIAAAAEAAAABAAAAAMAAAADAAAAAQAAAAEAAAAGAAAABgAAAAMAAAADAAAABgAAAAYAAAADAAAAAwAAAAYAAAAGAAAAAwAAAAMAAAACAAAAAgAAAAYAAAAGAAAAAQAAAAEAAAAEAAAABAAAAAYAAAAGAAAABAAAAAQAAAAGAAAABgAAAAQAAAAEAAAABgAAAAYAAAACAAAAAgAAAAQAAAAEAAAABgAAAAYAAAA=",
          vertex_count: 42
        }
      }
    ],
    "view.scene": [
      {
        match: {
          "@at": {
            node: "7d61586ade3f3adae4c99da825fdb0acf91ea64519f88d0473ce605e62dcde0f"
          }
        },
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              2
            ]
          ],
          bodies: [
            {
              id: "TopoBody#1",
              volumes: [
                "TopoVolume#1"
              ]
            }
          ],
          meshes: [],
          schema_version: 1,
          volumes: [
            {
              id: "TopoVolume#1"
            }
          ]
        }
      },
      {
        match: {},
        reply: {
          bbox: [
            [
              0,
              0,
              0
            ],
            [
              4,
              2,
              2
            ]
          ],
          bodies: [
            {
              id: "TopoBody#1",
              volumes: [
                "TopoVolume#1"
              ]
            }
          ],
          meshes: [
            {
              bbox: [
                [
                  0,
                  0,
                  0
                ],
                [
                  4,
                  2,
                  2
                ]
              ],
              cells: 16,
              derived_from: [],
              digest: "76e51cdbbe370e70b64415d1f9b120ad4426625d1b85a64d724a036b669eb160",
              dim: 3,
              facets: 68,
              id: "Mesh#1",
              name: "Mesh#1",
              nodes: 45
            }
          ],
          schema_version: 1,
          volumes: [
            {
              id: "TopoVolume#1"
            }
          ]
        }
      }
    ],
    "view.tessellate": [
      {
        match: {},
        reply: {
          angle_tol: 0.3,
          chord_tol: 0,
          edge_indices: "AAAAAAEAAAACAAAAAwAAAAQAAAAFAAAABgAAAAcAAAAAAAAAAgAAAAEAAAADAAAABAAAAAYAAAAFAAAABwAAAAAAAAAEAAAAAQAAAAUAAAACAAAABgAAAAMAAAAHAAAA",
          edges: [
            {
              count: 2,
              edge: "TopoEdge#1",
              first: 0
            },
            {
              count: 2,
              edge: "TopoEdge#2",
              first: 2
            },
            {
              count: 2,
              edge: "TopoEdge#3",
              first: 4
            },
            {
              count: 2,
              edge: "TopoEdge#4",
              first: 6
            },
            {
              count: 2,
              edge: "TopoEdge#5",
              first: 8
            },
            {
              count: 2,
              edge: "TopoEdge#6",
              first: 10
            },
            {
              count: 2,
              edge: "TopoEdge#7",
              first: 12
            },
            {
              count: 2,
              edge: "TopoEdge#8",
              first: 14
            },
            {
              count: 2,
              edge: "TopoEdge#9",
              first: 16
            },
            {
              count: 2,
              edge: "TopoEdge#10",
              first: 18
            },
            {
              count: 2,
              edge: "TopoEdge#11",
              first: 20
            },
            {
              count: 2,
              edge: "TopoEdge#12",
              first: 22
            }
          ],
          faces: [
            {
              boundary_only: false,
              face: "TopoFace#1",
              first_index: 0,
              index_count: 6
            },
            {
              boundary_only: false,
              face: "TopoFace#2",
              first_index: 6,
              index_count: 6
            },
            {
              boundary_only: false,
              face: "TopoFace#3",
              first_index: 12,
              index_count: 6
            },
            {
              boundary_only: false,
              face: "TopoFace#4",
              first_index: 18,
              index_count: 6
            },
            {
              boundary_only: false,
              face: "TopoFace#5",
              first_index: 24,
              index_count: 6
            },
            {
              boundary_only: false,
              face: "TopoFace#6",
              first_index: 30,
              index_count: 6
            }
          ],
          indices: "AAAAAAQAAAAGAAAAAAAAAAYAAAACAAAAAQAAAAMAAAAHAAAAAQAAAAcAAAAFAAAAAAAAAAEAAAAFAAAAAAAAAAUAAAAEAAAAAgAAAAYAAAAHAAAAAgAAAAcAAAADAAAAAAAAAAIAAAADAAAAAAAAAAMAAAABAAAABAAAAAUAAAAHAAAABAAAAAcAAAAGAAAA",
          max_edge: null,
          positions: "AAAAAAAAAAAAAAAAAACAQAAAAAAAAAAAAAAAAAAAAEAAAAAAAACAQAAAAEAAAAAAAAAAAAAAAAAAAABAAACAQAAAAAAAAABAAAAAAAAAAEAAAABAAACAQAAAAEAAAABA",
          triangle_count: 12,
          vertex_count: 8
        }
      }
    ]
  }
};

// src/main.ts
var FIXTURES = { hexbox: hexbox_default };
function transportFromUrl(search, host) {
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
async function main() {
  const root = document.getElementById("app");
  if (!root) throw new Error("missing #app");
  try {
    const t = transportFromUrl(location.search, location.host);
    const app = new App({ root, client: t.client, clipSteps: t.steps, transport: t.label, pollMs: t.label.startsWith("fixture://") ? 0 : 2e3 });
    await app.start();
  } catch (e) {
    root.textContent = e instanceof Error ? e.message : String(e);
  }
}
if (typeof document !== "undefined") void main();
export {
  transportFromUrl
};
//# sourceMappingURL=viewer.js.map
