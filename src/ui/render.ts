// Textbook-figure renderer. Everything is drawn in screen pixels from world
// coordinates (y up) so line widths stay crisp at any zoom.

import { RopeC } from '../engine/constraints';
import type { PointForce } from '../engine/forces';
import { System } from '../engine/system';
import type { Attach, El, GroundEl, Scene, TSpringEl } from '../model/types';
import { si } from './format';
import { localPolygon, type Outline } from '../model/shapes';

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface Preview {
  /** World points of an in-progress connector (rubber band to the cursor). */
  points: [number, number][];
  kind: string;
}

export interface DrawOpts {
  /** Translucent elements not in the scene yet (placement previews). */
  ghosts?: El[];
  /** Snap target for connector tools. */
  marker?: { p: [number, number]; label: string } | null;
  /** Drag handles (screen squares) at world points. */
  handles?: [number, number][];
  /** Measurement label near a point, e.g. "1.20 m · 90°". */
  measure?: { p: [number, number]; text: string } | null;
  selection: Set<string>;
  hover: string | null;
  labels: boolean;
  grid: boolean;
  forces: PointForce[] | null;
  preview: Preview | null;
  cursor: [number, number] | null;
}

const C = {
  bg: '#ffffff',
  gridMinor: '#f0f1f3',
  gridMajor: '#e1e3e8',
  fill: '#dcdcdc',
  fillDark: '#c4c4c4',
  ink: '#111111',
  sel: '#2563eb',
  hover: '#60a5fa',
  rope: '#3b2f22',
  label: '#333',
  track: '#9aa0a6',
  gravity: '#15803d',
  spring: '#1d4ed8',
  constraint: '#c2410c',
  friction: '#a16207',
  tspring: '#7e22ce',
  contact: '#be123c',
};

export class Renderer {
  ctx: CanvasRenderingContext2D;
  view = { cx: 0, cy: -0.3, scale: 150 };
  w = 0;
  h = 0;
  dpr = 1;
  /** Fixed FBD scale (px per N) so arrows stay comparable over time. */
  forceScale: number | null = null;

  constructor(public canvas: HTMLCanvasElement) {
    this.ctx = canvas.getContext('2d')!;
  }

  resize(): void {
    const r = this.canvas.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.w = r.width;
    this.h = r.height;
    this.canvas.width = Math.round(r.width * this.dpr);
    this.canvas.height = Math.round(r.height * this.dpr);
  }

  sx(x: number): number { return this.w / 2 + (x - this.view.cx) * this.view.scale; }
  sy(y: number): number { return this.h / 2 - (y - this.view.cy) * this.view.scale; }
  toWorld(px: number, py: number): [number, number] {
    return [(px - this.w / 2) / this.view.scale + this.view.cx, -(py - this.h / 2) / this.view.scale + this.view.cy];
  }

  zoomAt(px: number, py: number, factor: number): void {
    const [wx, wy] = this.toWorld(px, py);
    this.view.scale = Math.min(2000, Math.max(20, this.view.scale * factor));
    const [nx, ny] = this.toWorld(px, py);
    this.view.cx += wx - nx;
    this.view.cy += wy - ny;
  }

  // ------------------------------------------------------------- geometry

  static pose(scene: Scene, sys: System, q: ArrayLike<number>, id: string): Pose | null {
    const b = sys.byId.get(id);
    if (b) return { x: q[b.idx], y: q[b.idx + 1], theta: b.dof === 3 ? q[b.idx + 2] : 0 };
    const g = sys.grounds.get(id) ?? (scene.elements.find((e) => e.id === id && e.kind === 'ground') as GroundEl | undefined);
    return g ? { x: g.x, y: g.y, theta: g.theta } : null;
  }

  static attachWorld(scene: Scene, sys: System, q: ArrayLike<number>, a: Attach): [number, number] {
    if (a.body === null) return [a.p[0], a.p[1]];
    const p = Renderer.pose(scene, sys, q, a.body);
    if (!p) return [a.p[0], a.p[1]];
    const b = sys.byId.get(a.body);
    if (b && b.dof === 2) return [p.x, p.y];
    const c = Math.cos(p.theta), s = Math.sin(p.theta);
    return [p.x + c * a.p[0] - s * a.p[1], p.y + s * a.p[0] + c * a.p[1]];
  }

  // ----------------------------------------------------------------- draw

  draw(scene: Scene, sys: System, q: ArrayLike<number>, o: DrawOpts): void {
    const { ctx } = this;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.fillStyle = C.bg;
    ctx.fillRect(0, 0, this.w, this.h);
    if (o.grid) this.drawGrid();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    const els = scene.elements;
    const strokeFor = (e: El) => (o.selection.has(e.id) ? C.sel : o.hover === e.id ? C.hover : C.ink);
    const widthFor = (e: El, w: number) => (o.selection.has(e.id) ? w + 1.2 : w);

    for (const e of els) if (e.kind === 'ground') this.drawGround(e, strokeFor(e), widthFor(e, 2));
    for (const e of els) {
      if (e.kind !== 'slider') continue;
      const t: [number, number] = [Math.cos(e.angle), Math.sin(e.angle)];
      ctx.save();
      ctx.setLineDash([6, 4]);
      ctx.strokeStyle = o.selection.has(e.id) ? C.sel : C.track;
      ctx.lineWidth = o.selection.has(e.id) ? 2 : 1;
      this.line(e.origin[0] - t[0] * e.extent, e.origin[1] - t[1] * e.extent, e.origin[0] + t[0] * e.extent, e.origin[1] + t[1] * e.extent);
      ctx.restore();
      // rollers between the body and the track
      const p = Renderer.attachWorld(scene, sys, q, e.a);
      const n: [number, number] = [-t[1], t[0]];
      const body = els.find((b) => b.id === e.a.body);
      const rr = 5 / this.view.scale;
      const spread = body && body.kind === 'rect' ? body.w / 3 : 0;
      const offsets = spread ? [-spread, spread] : [0];
      for (const s of offsets) {
        const cx = p[0] + t[0] * s + n[0] * rr, cy = p[1] + t[1] * s + n[1] * rr;
        ctx.beginPath();
        ctx.arc(this.sx(cx), this.sy(cy), 5, 0, Math.PI * 2);
        ctx.fillStyle = '#fff';
        ctx.fill();
        ctx.strokeStyle = strokeFor(e);
        ctx.lineWidth = 1.3;
        ctx.stroke();
      }
    }

    for (const e of els) {
      if (e.kind === 'rod') {
        const a = Renderer.attachWorld(scene, sys, q, e.a), b = Renderer.attachWorld(scene, sys, q, e.b);
        ctx.strokeStyle = strokeFor(e);
        ctx.lineWidth = widthFor(e, 3.2);
        this.line(a[0], a[1], b[0], b[1]);
      }
    }
    for (const e of els) {
      if (e.kind === 'spring') {
        const a = Renderer.attachWorld(scene, sys, q, e.a), b = Renderer.attachWorld(scene, sys, q, e.b);
        this.drawSpring(a, b, strokeFor(e), widthFor(e, 1.4), e.c > 0);
      }
    }
    for (const e of els) {
      if (e.kind !== 'rope') continue;
      const con = sys.cons.find((c) => c.el === e.id) as RopeC | undefined;
      ctx.strokeStyle = o.selection.has(e.id) ? C.sel : o.hover === e.id ? C.hover : C.rope;
      ctx.lineWidth = widthFor(e, 1.6);
      if (!con) {
        const a = Renderer.attachWorld(scene, sys, q, e.a), b = Renderer.attachWorld(scene, sys, q, e.b);
        this.line(a[0], a[1], b[0], b[1]);
        continue;
      }
      const g = con.geometry(q);
      for (const pc of g.pieces) this.line(pc.t0[0], pc.t0[1], pc.t1[0], pc.t1[1]);
      for (const a of g.arcs) {
        ctx.beginPath();
        // screen y is flipped, so a CCW world arc is clockwise on screen
        ctx.arc(this.sx(a.cx), this.sy(a.cy), a.r * this.view.scale + 0.8, -a.a0, -a.a1, a.sigma > 0);
        ctx.stroke();
      }
    }

    for (const e of els) {
      if (e.kind === 'rect') this.drawRect(Renderer.pose(scene, sys, q, e.id)!, e, strokeFor(e), widthFor(e, 1.6));
    }
    for (const e of els) {
      if (e.kind === 'pulley') {
        const p = Renderer.pose(scene, sys, q, e.id)!;
        this.drawPulley(p, e.r, e.fixedAxle, strokeFor(e), widthFor(e, 1.6));
      }
    }
    for (const e of els) {
      if (e.kind === 'point') {
        const p = Renderer.pose(scene, sys, q, e.id)!;
        ctx.beginPath();
        ctx.arc(this.sx(p.x), this.sy(p.y), Math.max(4, e.radius * this.view.scale), 0, Math.PI * 2);
        ctx.fillStyle = C.fillDark;
        ctx.fill();
        ctx.strokeStyle = strokeFor(e);
        ctx.lineWidth = widthFor(e, 1.6);
        ctx.stroke();
      }
    }
    for (const e of els) {
      if (e.kind !== 'pin') continue;
      const a = Renderer.attachWorld(scene, sys, q, e.a);
      const toWorld = e.b.body === null || e.a.body === null;
      if (toWorld) this.drawSupport(a, strokeFor(e));
      this.drawPinHead(a, strokeFor(e), o.selection.has(e.id));
    }
    for (const e of els) {
      if (e.kind === 'tspring') this.drawTSpring(e, scene, sys, q, strokeFor(e), widthFor(e, 1.3));
    }
    for (const e of els) {
      if (e.kind === 'rod' || e.kind === 'spring' || e.kind === 'rope') {
        const pa = Renderer.attachWorld(scene, sys, q, e.a), pb = Renderer.attachWorld(scene, sys, q, e.b);
        let na = pb, nb = pa;
        if (e.kind === 'rope') {
          const con = sys.cons.find((c) => c.el === e.id) as RopeC | undefined;
          const g = con?.geometry(q);
          if (g && g.pieces.length) { na = g.pieces[0].t1; nb = g.pieces[g.pieces.length - 1].t0; }
        }
        if (e.a.body === null) this.drawAnchor(pa, na);
        if (e.b.body === null) this.drawAnchor(pb, nb);
        for (const at of [e.a, e.b]) {
          const p = Renderer.attachWorld(scene, sys, q, at);
          ctx.beginPath();
          ctx.arc(this.sx(p[0]), this.sy(p[1]), 2.2, 0, Math.PI * 2);
          ctx.fillStyle = C.ink;
          ctx.fill();
        }
      }
    }

    if (o.labels) this.drawLabels(scene, sys, q);
    if (o.forces) this.drawForces(o.forces);
    if (o.ghosts?.length) this.drawGhosts(o.ghosts);
    if (o.preview) this.drawPreview(o.preview, o.cursor);
    if (o.handles) for (const h of o.handles) this.drawHandle(h);
    if (o.marker) this.drawMarker(o.marker.p, o.marker.label);
    if (o.measure) this.drawMeasure(o.measure.p, o.measure.text);
  }

  private drawGhosts(ghosts: El[]): void {
    const { ctx } = this;
    ctx.save();
    ctx.globalAlpha = 0.5;
    for (const e of ghosts) {
      if (e.kind === 'ground') this.drawGround(e, C.sel, 2);
      else if (e.kind === 'rect') this.drawRect({ x: e.x, y: e.y, theta: e.theta }, e, C.sel, 1.6);
      else if (e.kind === 'pulley') this.drawPulley({ x: e.x, y: e.y, theta: e.theta }, e.r, e.fixedAxle, C.sel, 1.6);
      else if (e.kind === 'point') {
        ctx.beginPath();
        ctx.arc(this.sx(e.x), this.sy(e.y), Math.max(4, e.radius * this.view.scale), 0, Math.PI * 2);
        ctx.fillStyle = C.fillDark;
        ctx.fill();
        ctx.strokeStyle = C.sel;
        ctx.lineWidth = 1.6;
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  private drawHandle(p: [number, number]): void {
    const { ctx } = this;
    const X = this.sx(p[0]), Y = this.sy(p[1]);
    ctx.fillStyle = '#fff';
    ctx.strokeStyle = C.sel;
    ctx.lineWidth = 1.5;
    ctx.fillRect(X - 5, Y - 5, 10, 10);
    ctx.strokeRect(X - 5, Y - 5, 10, 10);
  }

  private drawMarker(p: [number, number], label: string): void {
    const { ctx } = this;
    const X = this.sx(p[0]), Y = this.sy(p[1]);
    ctx.beginPath();
    ctx.arc(X, Y, 7, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(37, 99, 235, 0.15)';
    ctx.fill();
    ctx.strokeStyle = C.sel;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(X, Y, 2.2, 0, Math.PI * 2);
    ctx.fillStyle = C.sel;
    ctx.fill();
    if (label) this.tag(X + 10, Y - 12, label);
  }

  private drawMeasure(p: [number, number], text: string): void {
    this.tag(this.sx(p[0]) + 12, this.sy(p[1]) + 16, text);
  }

  /** Small rounded label box in screen space. */
  private tag(x: number, y: number, text: string): void {
    const { ctx } = this;
    ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
    const w = ctx.measureText(text).width + 10;
    ctx.fillStyle = 'rgba(17, 24, 39, 0.85)';
    ctx.beginPath();
    ctx.roundRect(x, y - 9, w, 18, 4);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    ctx.fillText(text, x + 5, y);
  }

  private line(x0: number, y0: number, x1: number, y1: number): void {
    const { ctx } = this;
    ctx.beginPath();
    ctx.moveTo(this.sx(x0), this.sy(y0));
    ctx.lineTo(this.sx(x1), this.sy(y1));
    ctx.stroke();
  }

  private drawGrid(): void {
    const { ctx } = this;
    const [x0, y1] = this.toWorld(0, 0), [x1, y0] = this.toWorld(this.w, this.h);
    const minor = this.view.scale > 60 ? 0.1 : 0.5;
    for (const [step, color] of [[minor, C.gridMinor], [1, C.gridMajor]] as const) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
        const X = Math.round(this.sx(x)) + 0.5;
        ctx.moveTo(X, 0); ctx.lineTo(X, this.h);
      }
      for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
        const Y = Math.round(this.sy(y)) + 0.5;
        ctx.moveTo(0, Y); ctx.lineTo(this.w, Y);
      }
      ctx.stroke();
    }
  }

  private local(p: Pose, lx: number, ly: number): [number, number] {
    const c = Math.cos(p.theta), s = Math.sin(p.theta);
    return [this.sx(p.x + c * lx - s * ly), this.sy(p.y + s * lx + c * ly)];
  }

  drawGround(g: GroundEl, stroke: string, lw: number): void {
    if (g.shape === 'wedge') return this.drawRamp(g, stroke, lw);
    const { ctx } = this;
    const p = { x: g.x, y: g.y, theta: g.theta };
    const hw = g.w / 2, hh = g.h / 2;
    const corners = [this.local(p, -hw, hh), this.local(p, hw, hh), this.local(p, hw, -hh), this.local(p, -hw, -hh)];
    ctx.save();
    ctx.beginPath();
    corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.clip();
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    const spacing = 9;
    const len = (g.w + g.h) * this.view.scale;
    const steps = Math.ceil(len / spacing) + 2;
    const [ox, oy] = this.local(p, -hw, -hh);
    const ux = Math.cos(-g.theta), uy = Math.sin(-g.theta);
    ctx.beginPath();
    for (let i = -steps; i <= steps; i++) {
      // 45° hatching in the ground's own frame
      const bx = ox + ux * i * spacing, by = oy + uy * i * spacing;
      const dx = Math.cos(-g.theta - Math.PI / 4) * len * 2, dy = Math.sin(-g.theta - Math.PI / 4) * len * 2;
      ctx.moveTo(bx - dx, by - dy);
      ctx.lineTo(bx + dx, by + dy);
    }
    ctx.stroke();
    ctx.restore();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lw + 0.5;
    ctx.beginPath();
    ctx.moveTo(...corners[0]);
    ctx.lineTo(...corners[1]);
    ctx.stroke();
  }

  /** Fixed ramp: hatched triangle with a bold slope line and base. */
  private drawRamp(g: GroundEl, stroke: string, lw: number): void {
    const { ctx } = this;
    const p = { x: g.x, y: g.y, theta: g.theta };
    const pts = localPolygon(g).map(([lx, ly]) => this.local(p, lx, ly));
    ctx.save();
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.fillStyle = '#f3f4f6';
    ctx.fill();
    ctx.clip();
    ctx.strokeStyle = '#8a8f98';
    ctx.lineWidth = 1;
    const xs = pts.map((q) => q[0]), ys = pts.map((q) => q[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    ctx.beginPath();
    for (let x = x0 - (y1 - y0); x < x1; x += 9) {
      ctx.moveTo(x, y1);
      ctx.lineTo(x + (y1 - y0), y0);
    }
    ctx.stroke();
    ctx.restore();
    // vertices are base-left, base-right, top (CCW); for a rising-left wedge they are reversed
    ctx.strokeStyle = stroke;
    ctx.lineJoin = 'round';
    ctx.lineWidth = lw * 0.6;
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.stroke();
    const [a, , c] = g.flip ? [pts[2], pts[1], pts[0]] : pts;
    ctx.lineWidth = lw + 0.5;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(c[0], c[1]);
    ctx.stroke();
  }

  private drawRect(p: Pose, o: Outline, stroke: string, lw: number): void {
    const { ctx } = this;
    const pts = localPolygon(o).map(([lx, ly]) => this.local(p, lx, ly));
    const w = o.w, h = o.h;
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.fillStyle = C.fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lw;
    ctx.stroke();
    this.drawCOM(this.sx(p.x), this.sy(p.y), Math.min(6, Math.min(w, h) * this.view.scale * (o.shape === 'wedge' ? 0.18 : 0.35)), -p.theta);
  }

  /** Quartered centre-of-mass symbol. */
  private drawCOM(x: number, y: number, r: number, rot: number): void {
    if (r < 2.5) return;
    const { ctx } = this;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.fillStyle = C.ink;
    for (const a of [0, Math.PI]) {
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.arc(x, y, r, rot + a, rot + a + Math.PI / 2);
      ctx.closePath();
      ctx.fill();
    }
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  private drawPulley(p: Pose, r: number, fixed: boolean, stroke: string, lw: number): void {
    const { ctx } = this;
    const X = this.sx(p.x), Y = this.sy(p.y), R = r * this.view.scale;
    if (fixed) this.drawHanger([p.x, p.y], r, stroke);
    ctx.beginPath();
    ctx.arc(X, Y, R, 0, Math.PI * 2);
    ctx.fillStyle = C.fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lw;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(X, Y, R * 0.82, 0, Math.PI * 2);
    ctx.lineWidth = 0.8;
    ctx.stroke();
    // spokes show rotation
    ctx.beginPath();
    for (let k = 0; k < 3; k++) {
      const a = -p.theta - (k * 2 * Math.PI) / 3;
      ctx.moveTo(X + Math.cos(a) * R * 0.18, Y + Math.sin(a) * R * 0.18);
      ctx.lineTo(X + Math.cos(a) * R * 0.82, Y + Math.sin(a) * R * 0.82);
    }
    ctx.lineWidth = 1.2;
    ctx.stroke();
    // reference mark on the rim
    const a0 = -p.theta;
    ctx.beginPath();
    ctx.arc(X + Math.cos(a0) * R * 0.91, Y + Math.sin(a0) * R * 0.91, Math.max(1.5, R * 0.06), 0, Math.PI * 2);
    ctx.fillStyle = C.ink;
    ctx.fill();
    this.drawPinHead([p.x, p.y], stroke, false);
  }

  /** Bracket from a fixed pulley's axle up to a small hatched ceiling block. */
  private drawHanger(c: [number, number], r: number, stroke: string): void {
    const { ctx } = this;
    const X = this.sx(c[0]), Y = this.sy(c[1]);
    const top = Y - r * this.view.scale - 16;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(X, Y);
    ctx.lineTo(X, top);
    ctx.stroke();
    this.hatchBlock(X - 12, top - 6, 24, 6, true);
  }

  private hatchBlock(x: number, y: number, w: number, h: number, surfaceBottom: boolean): void {
    const { ctx } = this;
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = -h; i < w + h; i += 5) {
      ctx.moveTo(x + i, y + h);
      ctx.lineTo(x + i + h, y);
    }
    ctx.stroke();
    ctx.restore();
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    const yy = surfaceBottom ? y + h : y;
    ctx.moveTo(x, yy);
    ctx.lineTo(x + w, yy);
    ctx.stroke();
  }

  /** Classic pinned-support triangle with hatched base. */
  private drawSupport(p: [number, number], stroke: string): void {
    const { ctx } = this;
    const X = this.sx(p[0]), Y = this.sy(p[1]);
    ctx.beginPath();
    ctx.moveTo(X, Y);
    ctx.lineTo(X - 9, Y + 14);
    ctx.lineTo(X + 9, Y + 14);
    ctx.closePath();
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.4;
    ctx.stroke();
    this.hatchBlock(X - 13, Y + 14, 26, 6, false);
  }

  /** Small hatched wall behind a connector end fixed to the world. */
  private drawAnchor(p: [number, number], toward: [number, number]): void {
    const { ctx } = this;
    const X = this.sx(p[0]), Y = this.sy(p[1]);
    const dx = this.sx(toward[0]) - X, dy = this.sy(toward[1]) - Y;
    const ang = Math.atan2(dy, dx) + Math.PI; // points away from the connector
    ctx.save();
    ctx.translate(X, Y);
    ctx.rotate(ang);
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(0, -11);
    ctx.lineTo(0, 11);
    ctx.stroke();
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#555';
    ctx.beginPath();
    for (let i = -11; i < 11; i += 4.5) {
      ctx.moveTo(0, i);
      ctx.lineTo(6, i + 5);
    }
    ctx.stroke();
    ctx.restore();
  }

  private drawPinHead(p: [number, number], stroke: string, selected: boolean): void {
    const { ctx } = this;
    ctx.beginPath();
    ctx.arc(this.sx(p[0]), this.sy(p[1]), selected ? 5 : 4, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  private drawSpring(a: [number, number], b: [number, number], stroke: string, lw: number, damper: boolean): void {
    const { ctx } = this;
    const A = [this.sx(a[0]), this.sy(a[1])], B = [this.sx(b[0]), this.sy(b[1])];
    const dx = B[0] - A[0], dy = B[1] - A[1];
    const L = Math.hypot(dx, dy);
    if (L < 1) return;
    const ux = dx / L, uy = dy / L, nx = -uy, ny = ux;
    const off = damper ? 7 : 0;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lw;
    const lead = Math.min(14, L * 0.15);
    const coils = 8;
    const amp = 7;
    const P = (s: number, t: number, o = 0) => [A[0] + ux * s + nx * (t + o), A[1] + uy * s + ny * (t + o)];
    ctx.beginPath();
    ctx.moveTo(A[0], A[1]);
    if (damper) { const p = P(lead * 0.5, 0, off); ctx.lineTo(p[0], p[1]); }
    let p = P(lead, 0, off);
    ctx.lineTo(p[0], p[1]);
    // side view of a helical coil: a prolate cycloid s = vθ + d(cos θ − 1), t = amp·sin θ.
    // With d > v each turn draws a small loop; a stretched spring smooths into a wave.
    const body = L - 2 * lead;
    const turns = coils * 2 * Math.PI;
    const v = body / turns, d = amp * 0.55;
    const steps = coils * 24;
    for (let i = 1; i <= steps; i++) {
      const th = (i / steps) * turns;
      p = P(lead + v * th + d * (Math.cos(th) - 1), amp * Math.sin(th), off);
      ctx.lineTo(p[0], p[1]);
    }
    p = P(L - lead, 0, off);
    ctx.lineTo(p[0], p[1]);
    if (damper) { const pp = P(L - lead * 0.5, 0, off); ctx.lineTo(pp[0], pp[1]); }
    ctx.lineTo(B[0], B[1]);
    ctx.stroke();
    if (!damper) return;
    // dashpot in parallel
    const o2 = -off;
    const m = L / 2;
    ctx.beginPath();
    let s = P(lead * 0.5, 0, 0);
    ctx.moveTo(A[0], A[1]);
    ctx.lineTo(s[0], s[1]);
    s = P(lead * 0.5, 0, o2); ctx.lineTo(s[0], s[1]);
    s = P(m - 6, 0, o2); ctx.lineTo(s[0], s[1]);
    // piston
    const pa = P(m - 6, -4, o2), pb = P(m - 6, 4, o2);
    ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]);
    // cylinder
    const c1 = P(m - 10, -6, o2), c2 = P(m + 8, -6, o2), c3 = P(m + 8, 6, o2), c4 = P(m - 10, 6, o2);
    ctx.moveTo(c1[0], c1[1]); ctx.lineTo(c2[0], c2[1]); ctx.lineTo(c3[0], c3[1]); ctx.lineTo(c4[0], c4[1]);
    s = P(m + 8, 0, o2); ctx.moveTo(s[0], s[1]);
    s = P(L - lead * 0.5, 0, o2); ctx.lineTo(s[0], s[1]);
    s = P(L - lead * 0.5, 0, 0); ctx.lineTo(s[0], s[1]);
    ctx.lineTo(B[0], B[1]);
    ctx.stroke();
  }

  private drawTSpring(e: TSpringEl, scene: Scene, sys: System, q: ArrayLike<number>, stroke: string, lw: number): void {
    const pinEl = scene.elements.find((p) => p.id === e.pin);
    if (!pinEl || pinEl.kind !== 'pin') return;
    const c = Renderer.attachWorld(scene, sys, q, pinEl.a);
    const angA = pinEl.a.body ? Renderer.pose(scene, sys, q, pinEl.a.body)?.theta ?? 0 : 0;
    const angB = pinEl.b.body ? Renderer.pose(scene, sys, q, pinEl.b.body)?.theta ?? 0 : 0;
    const { ctx } = this;
    const X = this.sx(c[0]), Y = this.sy(c[1]);
    const turns = 2.25;
    const rel = angA - angB - e.theta0; // deflection winds/unwinds the coil
    const total = turns * 2 * Math.PI + rel;
    const start = angB + Math.PI / 2;
    const r0 = 7, r1 = 17;
    ctx.beginPath();
    const N = 80;
    for (let i = 0; i <= N; i++) {
      const t = i / N;
      const a = start + t * total;
      const r = r0 + (r1 - r0) * t;
      const px = X + Math.cos(a) * r, py = Y - Math.sin(a) * r;
      i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
    }
    // tab into body A
    const aEnd = start + total;
    ctx.lineTo(X + Math.cos(aEnd) * (r1 + 6), Y - Math.sin(aEnd) * (r1 + 6));
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lw;
    ctx.stroke();
  }

  private drawLabels(scene: Scene, sys: System, q: ArrayLike<number>): void {
    const { ctx } = this;
    ctx.font = 'italic 12px "Times New Roman", serif';
    ctx.fillStyle = C.label;
    ctx.textBaseline = 'middle';
    for (const e of scene.elements) {
      let x: number, y: number;
      if (e.kind === 'point' || e.kind === 'rect' || e.kind === 'pulley') {
        const p = Renderer.pose(scene, sys, q, e.id)!;
        const off = e.kind === 'point' ? e.radius : e.kind === 'pulley' ? e.r : Math.min(e.w, e.h) / 2;
        x = this.sx(p.x) + off * this.view.scale + 6;
        y = this.sy(p.y) - off * this.view.scale - 4;
      } else if (e.kind === 'spring') {
        const a = Renderer.attachWorld(scene, sys, q, e.a), b = Renderer.attachWorld(scene, sys, q, e.b);
        x = this.sx((a[0] + b[0]) / 2) + 6;
        y = this.sy((a[1] + b[1]) / 2) - 12;
      } else if (e.kind === 'tspring') {
        const pinEl = scene.elements.find((p) => p.id === e.pin);
        if (!pinEl || pinEl.kind !== 'pin') continue;
        const c = Renderer.attachWorld(scene, sys, q, pinEl.a);
        x = this.sx(c[0]) + 18;
        y = this.sy(c[1]) + 16;
      } else continue;
      ctx.fillText(e.name, x, y);
    }
  }

  private drawForces(forces: PointForce[]): void {
    const { ctx } = this;
    let fmax = 0;
    for (const f of forces) fmax = Math.max(fmax, Math.hypot(f.fx, f.fy));
    if (fmax < 1e-9) return;
    const scale = this.forceScale ?? 70 / fmax;
    const color: Record<PointForce['kind'], string> = {
      gravity: C.gravity, spring: C.spring, constraint: C.constraint, friction: C.friction, tspring: C.tspring, contact: C.contact,
    };
    ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
    for (const f of forces) {
      const X = this.sx(f.x), Y = this.sy(f.y);
      ctx.strokeStyle = color[f.kind];
      ctx.fillStyle = color[f.kind];
      if (f.torque !== undefined) {
        if (Math.abs(f.torque) < 1e-9) continue;
        const r = 24;
        const ccw = f.torque > 0;
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        const a0 = Math.PI * 0.15, a1 = Math.PI * 1.35;
        ctx.arc(X, Y, r, -a0, -a1, ccw);
        ctx.stroke();
        const end = -a1;
        const ex = X + Math.cos(end) * r, ey = Y + Math.sin(end) * r;
        const tang = end + (ccw ? -Math.PI / 2 : Math.PI / 2);
        this.arrowHead(ex, ey, Math.cos(tang), Math.sin(tang));
        ctx.fillText(si(Math.abs(f.torque), 'N·m'), X + r + 4, Y - r);
        continue;
      }
      const mag = Math.hypot(f.fx, f.fy);
      if (mag < fmax * 1e-3) continue;
      const len = mag * scale;
      const ux = f.fx / mag, uy = -f.fy / mag;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(X, Y);
      ctx.lineTo(X + ux * len, Y + uy * len);
      ctx.stroke();
      this.arrowHead(X + ux * len, Y + uy * len, ux, uy);
      if (len > 28) ctx.fillText(si(mag, 'N'), X + ux * len + 4, Y + uy * len - 4);
    }
  }

  private arrowHead(x: number, y: number, ux: number, uy: number): void {
    const { ctx } = this;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x - ux * 9 - uy * 4, y - uy * 9 + ux * 4);
    ctx.lineTo(x - ux * 9 + uy * 4, y - uy * 9 - ux * 4);
    ctx.closePath();
    ctx.fill();
  }

  private drawPreview(p: Preview, cursor: [number, number] | null): void {
    const { ctx } = this;
    const pts = cursor ? [...p.points, cursor] : p.points;
    ctx.save();
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = C.sel;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(this.sx(x), this.sy(y)) : ctx.moveTo(this.sx(x), this.sy(y))));
    ctx.stroke();
    ctx.restore();
    for (const [x, y] of p.points) {
      ctx.beginPath();
      ctx.arc(this.sx(x), this.sy(y), 4, 0, Math.PI * 2);
      ctx.fillStyle = C.sel;
      ctx.fill();
    }
  }
}
