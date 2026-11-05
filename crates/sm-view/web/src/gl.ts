// Raw WebGL2 renderer: flat-shaded triangles, an ID pass for picking, lines and point sprites.
import type { TriBatch } from "./batch.js";
import { type Camera, eye, viewProj } from "./camera.js";

const TRI_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
layout(location=2) in vec4 aId;
layout(location=3) in float aHatch;
uniform mat4 uVP;
out vec3 vPos; out vec4 vColor; out vec4 vId; out float vHatch;
void main(){ vPos=aPos; vColor=aColor; vId=aId; vHatch=aHatch; gl_Position=uVP*vec4(aPos,1.0); }`;

const TRI_FS = `#version 300 es
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

const LINE_VS = `#version 300 es
layout(location=0) in vec3 aPos;
uniform mat4 uVP; uniform float uSize;
void main(){ gl_Position=uVP*vec4(aPos,1.0); gl_PointSize=uSize; }`;

const LINE_FS = `#version 300 es
precision highp float;
uniform vec4 uColor; uniform int uRound;
out vec4 outColor;
void main(){
  if(uRound==1){ vec2 d=gl_PointCoord-vec2(0.5); if(dot(d,d)>0.25) discard; }
  outColor=uColor;
}`;

interface TriLayer {
  vao: WebGLVertexArrayObject;
  buffers: WebGLBuffer[];
  count: number;
}
interface LineLayer {
  vao: WebGLVertexArrayObject;
  buffer: WebGLBuffer;
  count: number;
  color: [number, number, number, number];
  mode: number;
  size: number;
}

export type Layer = "skin" | "model";

/** Owns a WebGL2 context; layers are replaced wholesale by the app. */
export class Renderer {
  readonly gl: WebGL2RenderingContext;
  private tri: WebGLProgram;
  private line: WebGLProgram;
  private tris = new Map<Layer, TriLayer>();
  private lines = new Map<string, LineLayer>();
  visible = new Set<string>(["skin", "model", "outline", "edges", "markers", "highlight"]);

  constructor(readonly canvas: HTMLCanvasElement) {
    const gl = canvas.getContext("webgl2", { antialias: false, preserveDrawingBuffer: true, alpha: false });
    if (!gl) throw new Error("WebGL2 is not available in this browser");
    this.gl = gl;
    this.tri = this.program(TRI_VS, TRI_FS);
    this.line = this.program(LINE_VS, LINE_FS);
  }

  private program(vs: string, fs: string): WebGLProgram {
    const gl = this.gl;
    const mk = (type: number, src: string) => {
      const s = gl.createShader(type)!;
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(`shader: ${gl.getShaderInfoLog(s)}`);
      return s;
    };
    const p = gl.createProgram()!;
    gl.attachShader(p, mk(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(`link: ${gl.getProgramInfoLog(p)}`);
    return p;
  }

  private buf(data: ArrayBufferView, loc: number, size: number, type: number, normalized: boolean): WebGLBuffer {
    const gl = this.gl;
    const b = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, b);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, size, type, normalized, 0, 0);
    return b;
  }

  /** Replace a triangle layer (`null` clears it). */
  setTriangles(name: Layer, b: TriBatch | null): void {
    const gl = this.gl;
    const old = this.tris.get(name);
    if (old) {
      old.buffers.forEach((x) => gl.deleteBuffer(x));
      gl.deleteVertexArray(old.vao);
      this.tris.delete(name);
    }
    if (!b || b.vertexCount === 0) return;
    const vao = gl.createVertexArray()!;
    gl.bindVertexArray(vao);
    const buffers = [
      this.buf(b.positions, 0, 3, gl.FLOAT, false),
      this.buf(b.colors, 1, 4, gl.UNSIGNED_BYTE, true),
      this.buf(b.ids, 2, 4, gl.UNSIGNED_BYTE, true),
      this.buf(b.hatch, 3, 1, gl.UNSIGNED_BYTE, true),
    ];
    gl.bindVertexArray(null);
    this.tris.set(name, { vao, buffers, count: b.vertexCount });
  }

  /** Replace a line list (`points = false`) or point list (`points = true`); `null` clears it. */
  setLines(name: string, positions: Float32Array | null, color: [number, number, number, number], points = false, size = 8): void {
    const gl = this.gl;
    const old = this.lines.get(name);
    if (old) {
      gl.deleteBuffer(old.buffer);
      gl.deleteVertexArray(old.vao);
      this.lines.delete(name);
    }
    if (!positions || positions.length === 0) return;
    const vao = gl.createVertexArray()!;
    gl.bindVertexArray(vao);
    const buffer = this.buf(positions, 0, 3, gl.FLOAT, false);
    gl.bindVertexArray(null);
    this.lines.set(name, { vao, buffer, count: positions.length / 3, color, mode: points ? gl.POINTS : gl.LINES, size });
  }

  /** Match the canvas backing store to its CSS size. */
  resize(): void {
    const dpr = Math.min(2, globalThis.devicePixelRatio || 1);
    const w = Math.max(1, Math.floor(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
  }

  private drawTris(cam: Camera, idPass: boolean, only?: Layer): void {
    const gl = this.gl;
    gl.useProgram(this.tri);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.tri, "uVP"), false, new Float32Array(viewProj(cam)));
    gl.uniform3fv(gl.getUniformLocation(this.tri, "uEye"), new Float32Array(eye(cam)));
    gl.uniform1i(gl.getUniformLocation(this.tri, "uIdPass"), idPass ? 1 : 0);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1, 1);
    for (const [name, l] of this.tris) {
      if ((only && name !== only) || !this.visible.has(name)) continue;
      gl.bindVertexArray(l.vao);
      gl.drawArrays(gl.TRIANGLES, 0, l.count);
    }
    gl.disable(gl.POLYGON_OFFSET_FILL);
  }

  /** Draw the scene. */
  render(cam: Camera): void {
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
  readIds(cam: Camera, layer: Layer): { ids: Uint32Array; width: number; height: number } {
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
        ids[y * w + x] = px[s]! | (px[s + 1]! << 8) | (px[s + 2]! << 16);
      }
    }
    return { ids, width: w, height: h };
  }
}
