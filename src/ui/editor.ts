// Canvas interaction: tools, hit testing, drag-to-move, palette drop, keys.

import { RopeC } from '../engine/constraints';
import { autoWrapDir, ropeGeometry } from '../engine/rope';
import * as F from '../model/factory';
import { defaultSide, groundEnds, groundFromEnds, snapSegment } from '../model/ground';
import { insidePolygon, localPolygon, rampFromSlope, snapPoints } from '../model/shapes';
import type { Attach, El, GroundEl, PinEl, RopeRoute, Scene } from '../model/types';
import { App } from './app';
import { TOOL_KEYS, Tool, TOOL_HELP, TOOL_LABEL, buildPalette } from './palette';
import { Renderer } from './render';

type V = [number, number];

interface Pending {
  tool: 'rod' | 'spring' | 'rope' | 'ground' | 'ramp';
  a: Attach;
  aWorld: V;
  route: string[];
}

interface Drag {
  kind: 'body' | 'ground' | 'worldpin' | 'pan' | 'gend';
  id?: string;
  /** For 'gend': which surface end (0 = a, 1 = b). */
  end?: 0 | 1;
  startWorld: V;
  startScreen: V;
  orig: V;
  moved: boolean;
}

const dist = (a: V, b: V) => Math.hypot(a[0] - b[0], a[1] - b[1]);

function segDist(p: V, a: V, b: V): number {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L2 = dx * dx + dy * dy;
  const t = L2 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2)) : 0;
  return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}

export class Editor {
  tool: Tool = 'select';
  pending: Pending | null = null;
  cursor: V | null = null;
  drag: Drag | null = null;
  private setActive: (t: Tool) => void;
  private hintEl = document.getElementById('tool-hint')!;
  private coordsEl = document.getElementById('coords')!;
  private shiftDown = false;
  /** Called whenever the active tool changes. */
  onToolChange: (() => void) | null = null;

  constructor(private app: App, private canvas: HTMLCanvasElement) {
    this.setActive = buildPalette(document.getElementById('palette')!, (t) => this.setTool(t));
    this.setTool('select');
    app.editorOverlay = () => this.overlay();
    app.onState(() => this.updateHint());
    this.bind();
  }

  /** Previews drawn on top of the scene for the active tool. */
  private overlay(): Partial<import('./render').DrawOpts> {
    const out: Partial<import('./render').DrawOpts> = { preview: null, cursor: null, ghosts: [], marker: null, measure: null, handles: [] };
    const cur = this.cursor;
    const editing = this.app.editing && !this.app.modeAnim;
    // endpoint handles of a selected ground piece
    if (this.tool === 'select' && editing && this.app.selection.size === 1) {
      const g = this.selectedGround();
      if (g) { const e = groundEnds(g); out.handles = [e.a, e.b]; }
    }
    if (!cur || this.tool === 'select') return out;
    const pend = this.pending;
    if (this.tool === 'ground') {
      if (pend) {
        const b = this.groundEnd(pend.aWorld, cur);
        out.ghosts = [this.makeGround(pend.aWorld, b)];
        out.measure = { p: b, text: this.segText(pend.aWorld, b) };
      } else {
        out.marker = { p: this.snapV(cur), label: '' };
      }
      return out;
    }
    if (this.tool === 'ramp') {
      if (pend) {
        const b = this.slopeEnd(pend.aWorld, cur);
        const ramp = this.makeRamp(pend.aWorld, b);
        if (ramp) out.ghosts = [ramp];
        out.measure = { p: b, text: this.slopeText(pend.aWorld, b) };
      } else {
        out.marker = { p: this.snapV(cur), label: '' };
      }
      return out;
    }
    if (this.tool === 'point' || this.tool === 'rect' || this.tool === 'pulley') {
      out.ghosts = [this.makeBody(this.tool, this.snapV(cur))];
      return out;
    }
    if (this.tool === 'tspring') {
      const pin = this.pinNear(cur);
      if (pin) out.marker = { p: Renderer.attachWorld(this.scene, this.app.sys, this.q(), pin.a), label: pin.name };
      return out;
    }
    // connector tools: show the snap target
    const ropeOverPulley = pend?.tool === 'rope' ? this.pulleyUnder(cur, pend) : null;
    if (ropeOverPulley) {
      out.marker = { p: this.poseXY(ropeOverPulley.id), label: `over ${ropeOverPulley.name}` };
    } else if (this.tool === 'pin' || this.tool === 'slider') {
      const body = this.bodyAt(cur, false);
      if (body) { const t = this.attachAt(cur); out.marker = { p: t.world, label: t.label }; }
    } else {
      const t = this.attachAt(cur, pend?.a.body ?? undefined);
      out.marker = { p: t.world, label: t.label };
    }
    if (pend) {
      out.preview = { kind: pend.tool, points: [pend.aWorld, ...pend.route.map((id) => this.poseXY(id))] };
      const end = out.marker?.p ?? cur;
      out.cursor = end;
      if (pend.tool !== 'rope') out.measure = { p: end, text: this.segText(pend.aWorld, end) };
    }
    return out;
  }

  private segText(a: V, b: V): string {
    const L = dist(a, b);
    const ang = (Math.atan2(b[1] - a[1], b[0] - a[0]) * 180) / Math.PI;
    return `${L.toFixed(2)} m · ${Math.round(ang)}°`;
  }

  /** The selected flat ground piece (ramps have no end handles). */
  private selectedGround(): GroundEl | null {
    const id = [...this.app.selection][0];
    const e = this.scene.elements.find((x) => x.id === id);
    return e && e.kind === 'ground' && e.shape !== 'wedge' ? e : null;
  }

  /** Ramp slope end: angle snapped to 5° steps and length to 5 cm. */
  private slopeEnd(a: V, p: V): V {
    if (!this.app.snap || this.shiftDown) return p;
    const dx = p[0] - a[0], dy = p[1] - a[1];
    const L = Math.round(Math.hypot(dx, dy) / 0.05) * 0.05;
    const inc = Math.PI / 36;
    const ang = Math.round(Math.atan2(dy, dx) / inc) * inc;
    return [a[0] + L * Math.cos(ang), a[1] + L * Math.sin(ang)];
  }

  private slopeText(a: V, b: V): string {
    const ang = Math.abs((Math.atan2(b[1] - a[1], Math.abs(b[0] - a[0])) * 180) / Math.PI);
    return `slope ${Math.round(ang)}° · ${dist(a, b).toFixed(2)} m`;
  }

  private makeRamp(a: V, b: V): GroundEl | null {
    const r = rampFromSlope(a, b);
    if (!r) return null;
    return F.ground(F.nextName(this.scene, 'ground'), r.x, r.y, { shape: 'wedge', w: r.w, h: r.h, flip: r.flip });
  }

  /** Second end of a ground segment being drawn, with angle/length snapping. */
  private groundEnd(a: V, p: V): V {
    return this.app.snap && !this.shiftDown ? snapSegment(a, p) : p;
  }

  private bodyCentroid(): V | null {
    const pts = this.scene.elements.filter((e) => e.kind === 'point' || e.kind === 'rect' || e.kind === 'pulley').map((e) => this.poseXY(e.id));
    if (!pts.length) return null;
    return [pts.reduce((s, p) => s + p[0], 0) / pts.length, pts.reduce((s, p) => s + p[1], 0) / pts.length];
  }

  private makeGround(a: V, b: V, id?: string): GroundEl {
    const ref = this.bodyCentroid() ?? [this.r.view.cx, this.r.view.cy];
    const box = groundFromEnds(a, b, defaultSide(a, b, ref), 0.12);
    const g = F.ground(F.nextName(this.scene, 'ground'), 0, 0, box);
    if (id) g.id = id;
    return g;
  }

  private makeBody(tool: 'point' | 'rect' | 'pulley', p: V): El {
    if (tool === 'point') return F.point(F.nextName(this.scene, 'point'), p[0], p[1], 1);
    if (tool === 'rect') return F.rect(F.nextName(this.scene, 'rect'), p[0], p[1], 2, 0.6, 0.2);
    return F.pulley(F.nextName(this.scene, 'pulley'), p[0], p[1], 1, 0.12);
  }

  private pinNear(p: V): PinEl | null {
    const tol = this.pxToWorld(14);
    let best: PinEl | null = null, bd = tol;
    for (const e of this.scene.elements) {
      if (e.kind !== 'pin') continue;
      const d = dist(p, Renderer.attachWorld(this.scene, this.app.sys, this.q(), e.a));
      if (d < bd) { bd = d; best = e; }
    }
    return best;
  }

  private pulleyUnder(p: V, pend: Pending): El | null {
    const body = this.bodyAt(p, false);
    if (body && body.kind === 'pulley' && pend.route[pend.route.length - 1] !== body.id && dist(p, this.poseXY(body.id)) < body.r) return body;
    return null;
  }

  get r(): Renderer { return this.app.renderer; }
  get scene(): Scene { return this.app.scene; }

  setTool(t: Tool): void {
    this.tool = t;
    this.pending = null;
    this.setActive(t);
    this.updateHint();
    this.canvas.style.cursor = t === 'select' ? 'default' : 'crosshair';
    this.onToolChange?.();
    this.app.redraw();
  }

  /** Step-by-step instruction pill at the top of the canvas. */
  updateHint(): void {
    const esc = '<kbd>Esc</kbd> cancel';
    let html = '';
    if (this.tool !== 'select') {
      const name = `<b>${TOOL_LABEL[this.tool]}</b>`;
      const p = this.pending;
      if (this.tool === 'ground') {
        html = p ? `${name} · step 2/2 · click the other end (snaps to 15°, hold <kbd>Shift</kbd> for free angle) · ${esc}`
          : `${name} · step 1/2 · click where the surface starts · ${esc}`;
      } else if (this.tool === 'ramp') {
        html = p ? `${name} · step 2/2 · click the top of the slope (snaps to 5°, hold <kbd>Shift</kbd> for free angle) · ${esc}`
          : `${name} · step 1/2 · click the bottom of the slope · ${esc}`;
      } else if (this.tool === 'rope') {
        html = !p ? `${name} · step 1 · click the first end (a body, or empty space for a fixed point) · ${esc}`
          : `${name} · click pulleys in order (${p.route.length} so far), then the other end · ${esc}`;
      } else if (this.tool === 'rod' || this.tool === 'spring') {
        html = p ? `${name} · step 2/2 · click the second point · ${esc}` : `${name} · step 1/2 · click the first point · ${esc}`;
      } else {
        html = `${name} · ${TOOL_HELP[this.tool]} · <kbd>Shift</kbd>-click to place several · ${esc}`;
      }
    } else if (!this.app.editing && !this.app.modeAnim) {
      html = this.app.running ? 'Running · editing anything resets to t = 0' : 'Paused · editing anything resets to t = 0';
    }
    this.hintEl.innerHTML = html;
    this.hintEl.hidden = !html;
  }

  // ------------------------------------------------------------ geometry

  private q(): ArrayLike<number> { return this.app.displayQ ?? this.app.sim.q; }

  private poseXY(id: string): V {
    const p = Renderer.pose(this.scene, this.app.sys, this.q(), id);
    return p ? [p.x, p.y] : [0, 0];
  }

  private snapV(p: V, step = 0.05): V {
    if (!this.app.snap) return p;
    return [Math.round(p[0] / step) * step, Math.round(p[1] / step) * step];
  }

  private pxToWorld(px: number): number { return px / this.r.view.scale; }

  /** Body (dynamic first, topmost first) whose shape contains p. */
  bodyAt(p: V, includeGround = true): El | null {
    const els = this.scene.elements;
    const tol = this.pxToWorld(5);
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind === 'point') {
        const c = this.poseXY(e.id);
        if (dist(p, c) <= Math.max(e.radius, this.pxToWorld(5)) + tol) return e;
      }
    }
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind === 'pulley' && dist(p, this.poseXY(e.id)) <= e.r + tol) return e;
      if (e.kind === 'rect' && insidePolygon(localPolygon(e), this.toLocal(e.id, p), tol)) return e;
    }
    if (includeGround) {
      for (let i = els.length - 1; i >= 0; i--) {
        const e = els[i];
        if (e.kind === 'ground' && insidePolygon(localPolygon(e), this.toLocal(e.id, p), tol)) return e;
      }
    }
    return null;
  }

  private toLocal(id: string, p: V): V {
    const ps = Renderer.pose(this.scene, this.app.sys, this.q(), id)!;
    const c = Math.cos(ps.theta), s = Math.sin(ps.theta);
    const dx = p[0] - ps.x, dy = p[1] - ps.y;
    return [c * dx + s * dy, -s * dx + c * dy];
  }

  /** Attachment for a click at p, snapping to features of the body. */
  attachAt(p: V, exclude?: string): { a: Attach; world: V; label: string } {
    let body = this.bodyAt(p);
    if (body && body.id === exclude) body = null;
    if (!body) {
      const w = this.snapV(p);
      return { a: { body: null, p: w }, world: w, label: 'fixed point' };
    }
    if (body.kind === 'point') return { a: { body: body.id, p: [0, 0] }, world: this.poseXY(body.id), label: body.name };
    if (body.kind === 'pulley') {
      const c = this.poseXY(body.id);
      if (dist(p, c) < body.r * 0.6) return { a: { body: body.id, p: [0, 0] }, world: c, label: `${body.name} axle` };
    }
    let local = this.toLocal(body.id, p);
    let feature = '';
    if ((body.kind === 'rect' || (body.kind === 'ground' && body.shape === 'wedge')) && this.app.snap) {
      // snap to corners, edge midpoints, quarter points and the centre
      const cands = snapPoints(localPolygon(body));
      cands.push({ p: [0, 0], label: 'centre' });
      if (body.shape !== 'wedge') {
        cands.push({ p: [-body.w / 2, 0], label: 'end' }, { p: [body.w / 2, 0], label: 'end' });
      }
      let best = this.pxToWorld(10), pick: V | null = null;
      for (const c of cands) {
        const d = dist(c.p, local);
        if (d < best) { best = d; pick = c.p; feature = c.label; }
      }
      local = pick ?? local;
    } else if (body.kind === 'ground' && this.app.snap) {
      const w = this.snapV(p);
      local = this.toLocal(body.id, w);
    }
    const ps = Renderer.pose(this.scene, this.app.sys, this.q(), body.id)!;
    const c = Math.cos(ps.theta), s = Math.sin(ps.theta);
    const world: V = [ps.x + c * local[0] - s * local[1], ps.y + s * local[0] + c * local[1]];
    return { a: { body: body.id, p: [local[0], local[1]] }, world, label: feature ? `${body.name} ${feature}` : body.name };
  }

  /** Element under the cursor for selection. */
  hit(p: V): string | null {
    const els = this.scene.elements;
    const tol = this.pxToWorld(7);
    const q = this.q(), sys = this.app.sys;
    const aw = (a: Attach) => Renderer.attachWorld(this.scene, sys, q, a);
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind === 'pin' && dist(p, aw(e.a)) < tol) return e.id;
    }
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind !== 'tspring') continue;
      const pin = els.find((x) => x.id === e.pin) as PinEl | undefined;
      if (!pin) continue;
      const d = dist(p, aw(pin.a));
      if (d >= tol && d < this.pxToWorld(22)) return e.id;
    }
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind === 'point' && dist(p, this.poseXY(e.id)) <= Math.max(e.radius, this.pxToWorld(5)) + this.pxToWorld(2)) return e.id;
    }
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if ((e.kind === 'rod' || e.kind === 'spring') && segDist(p, aw(e.a), aw(e.b)) < tol) return e.id;
      if (e.kind === 'rope') {
        const con = sys.cons.find((c) => c.el === e.id) as RopeC | undefined;
        if (con) {
          const g = con.geometry(q);
          if (g.pieces.some((pc) => segDist(p, pc.t0, pc.t1) < tol)) return e.id;
        } else if (segDist(p, aw(e.a), aw(e.b)) < tol) return e.id;
      }
    }
    const body = this.bodyAt(p, false);
    if (body) return body.id;
    for (let i = els.length - 1; i >= 0; i--) {
      const e = els[i];
      if (e.kind !== 'slider') continue;
      const t: V = [Math.cos(e.angle), Math.sin(e.angle)];
      const a: V = [e.origin[0] - t[0] * e.extent, e.origin[1] - t[1] * e.extent];
      const b: V = [e.origin[0] + t[0] * e.extent, e.origin[1] + t[1] * e.extent];
      if (segDist(p, a, b) < tol) return e.id;
    }
    const g = this.bodyAt(p, true);
    return g ? g.id : null;
  }

  // -------------------------------------------------------------- actions

  /** Perform the current tool at world point p. Returns true if consumed. */
  apply(tool: Tool, p: V): void {
    const app = this.app;
    const sp = this.snapV(p);
    const keep = this.shiftDown;
    const add = (e: El) => {
      app.commit((s) => s.elements.push(e));
      app.select([e.id]);
      if (keep) { this.pending = null; this.updateHint(); } else this.setTool('select');
    };
    switch (tool) {
      case 'ground': {
        if (!this.pending) {
          this.pending = { tool: 'ground', a: { body: null, p: sp }, aWorld: sp, route: [] };
          this.updateHint();
          app.redraw();
          return;
        }
        const a = this.pending.aWorld;
        const b = this.groundEnd(a, p);
        if (dist(a, b) < 0.05) { this.flash('Make the surface at least 5 cm long.'); return; }
        this.pending = null;
        add(this.makeGround(a, b));
        return;
      }
      case 'ramp': {
        if (!this.pending) {
          this.pending = { tool: 'ramp', a: { body: null, p: sp }, aWorld: sp, route: [] };
          this.updateHint();
          app.redraw();
          return;
        }
        const ramp = this.makeRamp(this.pending.aWorld, this.slopeEnd(this.pending.aWorld, p));
        if (!ramp) { this.flash('Give the slope some height and length.'); return; }
        this.pending = null;
        add(ramp);
        return;
      }
      case 'point':
      case 'rect':
      case 'pulley':
        add(this.makeBody(tool, sp));
        return;
      case 'pin': {
        const first = this.bodyAt(p, false);
        if (!first) { this.flash('Click on a body to pin it.'); return; }
        const at1 = this.attachAt(p);
        // a second body under the same point, else ground under it, else the world
        const others = this.scene.elements.filter((e) => e.id !== first.id);
        const saved = this.scene.elements;
        let second: El | null = null;
        try {
          this.scene.elements = others;
          second = this.bodyAt(at1.world, true);
        } finally {
          this.scene.elements = saved;
        }
        let b: Attach;
        if (second) {
          const local = this.toLocal(second.id, at1.world);
          b = second.kind === 'point' ? { body: second.id, p: [0, 0] } : { body: second.id, p: local };
        } else {
          b = { body: null, p: at1.world };
        }
        add(F.pin(F.nextName(this.scene, 'pin'), at1.a, b));
        return;
      }
      case 'slider': {
        const body = this.bodyAt(p, false);
        if (!body) { this.flash('Click on a body to put it on a track.'); return; }
        const at1 = this.attachAt(p);
        add(F.slider(F.nextName(this.scene, 'slider'), at1.a, at1.world, 0, { lockRotation: body.kind === 'rect' }));
        return;
      }
      case 'tspring': {
        const pin = this.pinNear(p);
        if (!pin) { this.flash('Click on a pin joint (the small white circle).'); return; }
        const sys = app.sys, q = this.q();
        const ang = (a: Attach) => {
          if (!a.body) return 0;
          return Renderer.pose(this.scene, sys, q, a.body)?.theta ?? 0;
        };
        add(F.tspring(F.nextName(this.scene, 'tspring'), pin.id, 5, ang(pin.a) - ang(pin.b)));
        return;
      }
      case 'rod':
      case 'spring':
      case 'rope': {
        if (!this.pending) {
          const at1 = this.attachAt(p);
          this.pending = { tool, a: at1.a, aWorld: at1.world, route: [] };
          this.updateHint();
          app.redraw();
          return;
        }
        const pend = this.pending;
        if (tool === 'rope') {
          const body = this.pulleyUnder(p, pend);
          if (body) {
            pend.route.push(body.id);
            this.updateHint();
            app.redraw();
            return;
          }
        }
        const at2 = this.attachAt(p, pend.a.body ?? undefined);
        const L = dist(pend.aWorld, at2.world);
        if (tool !== 'rope' && L < 1e-6) { this.flash('Pick two different points.'); return; }
        this.pending = null;
        if (tool === 'rod') add(F.rod(F.nextName(this.scene, 'rod'), pend.a, at2.a, L));
        else if (tool === 'spring') add(F.spring(F.nextName(this.scene, 'spring'), pend.a, at2.a, 50, L));
        else add(this.makeRope(pend, at2.a, at2.world));
        return;
      }
      default:
        return;
    }
  }

  private makeRope(pend: Pending, b: Attach, bWorld: V): El {
    const pts: V[] = [pend.aWorld, ...pend.route.map((id) => this.poseXY(id)), bWorld];
    const route: RopeRoute[] = pend.route.map((id, i) => ({ pulley: id, dir: autoWrapDir(pts[i], pts[i + 1], pts[i + 2]) }));
    const rope = F.rope(F.nextName(this.scene, 'rope'), pend.a, b, route, 1);
    rope.length = this.ropeLength(rope);
    return rope;
  }

  /** Current geometric length of a rope (for "make taut"). */
  ropeLength(rope: El): number {
    if (rope.kind !== 'rope') return 0;
    const sys = this.app.sys;
    const nodes = [
      { type: 'end' as const, a: sys.resolve(rope.a) },
      ...rope.route.map((r) => {
        const p = this.scene.elements.find((e) => e.id === r.pulley);
        const b = sys.byId.get(r.pulley);
        return { type: 'pulley' as const, b: b!, r: p && p.kind === 'pulley' ? p.r : 0.1, sigma: r.dir };
      }),
      { type: 'end' as const, a: sys.resolve(rope.b) },
    ];
    return ropeGeometry(nodes, this.q()).total;
  }

  private flash(msg: string): void {
    this.hintEl.innerHTML = `<span class="warn-text">${msg}</span>`;
    this.hintEl.hidden = false;
    setTimeout(() => this.updateHint(), 1800);
  }

  // --------------------------------------------------------------- events

  private eventWorld(ev: { clientX: number; clientY: number }): V {
    const rect = this.canvas.getBoundingClientRect();
    return this.r.toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
  }

  private bind(): void {
    const c = this.canvas;
    c.addEventListener('pointerdown', (ev) => this.onDown(ev));
    c.addEventListener('pointermove', (ev) => this.onMove(ev));
    c.addEventListener('pointerup', (ev) => this.onUp(ev));
    c.addEventListener('pointerleave', () => { this.cursor = null; this.app.hover = null; this.coordsEl.textContent = ''; this.app.redraw(); });
    c.addEventListener('contextmenu', (ev) => ev.preventDefault());
    window.addEventListener('keyup', (ev) => { if (ev.key === 'Shift') { this.shiftDown = false; this.app.redraw(); } });
    window.addEventListener('blur', () => (this.shiftDown = false));
    c.addEventListener('wheel', (ev) => {
      ev.preventDefault();
      const rect = c.getBoundingClientRect();
      this.r.zoomAt(ev.clientX - rect.left, ev.clientY - rect.top, Math.exp(-ev.deltaY * 0.0015));
      this.app.redraw();
    }, { passive: false });
    c.addEventListener('dragover', (ev) => { ev.preventDefault(); if (ev.dataTransfer) ev.dataTransfer.dropEffect = 'copy'; });
    c.addEventListener('drop', (ev) => {
      ev.preventDefault();
      const t = (ev.dataTransfer?.getData('text/x-tool') || ev.dataTransfer?.getData('text/plain')) as Tool;
      if (!t || !(t in TOOL_HELP)) return;
      this.app.ensureEditMode();
      this.setTool(t);
      this.apply(t, this.eventWorld(ev));
    });
    window.addEventListener('keydown', (ev) => this.onKey(ev));
  }

  private onDown(ev: PointerEvent): void {
    const p = this.eventWorld(ev);
    this.shiftDown = ev.shiftKey;
    this.canvas.setPointerCapture(ev.pointerId);
    const screen: V = [ev.clientX, ev.clientY];
    if (ev.button === 2 && this.tool !== 'select') {
      // right click backs out of the tool
      if (this.pending) { this.pending = null; this.updateHint(); this.app.redraw(); } else this.setTool('select');
      return;
    }
    if (ev.button === 1 || ev.button === 2 || (ev.button === 0 && ev.altKey)) {
      this.drag = { kind: 'pan', startWorld: p, startScreen: screen, orig: [this.r.view.cx, this.r.view.cy], moved: false };
      return;
    }
    if (this.tool !== 'select') {
      this.app.ensureEditMode();
      this.apply(this.tool, p);
      return;
    }
    // ground end handles take priority
    const g = this.selectedGround();
    if (g && this.app.editing) {
      const e = groundEnds(g);
      const tol = this.pxToWorld(9);
      const end = dist(p, e.a) < tol ? 0 : dist(p, e.b) < tol ? 1 : null;
      if (end !== null) {
        this.drag = { kind: 'gend', id: g.id, end, startWorld: p, startScreen: screen, orig: end ? e.b : e.a, moved: false };
        return;
      }
    }
    const id = this.hit(p);
    if (!id) {
      this.app.select([]);
      this.drag = { kind: 'pan', startWorld: p, startScreen: screen, orig: [this.r.view.cx, this.r.view.cy], moved: false };
      return;
    }
    if (ev.shiftKey) {
      const s = new Set(this.app.selection);
      s.has(id) ? s.delete(id) : s.add(id);
      this.app.select([...s]);
    } else if (!this.app.selection.has(id)) {
      this.app.select([id]);
    }
    const e = this.scene.elements.find((x) => x.id === id)!;
    if (e.kind === 'point' || e.kind === 'rect' || e.kind === 'pulley' || e.kind === 'ground') {
      this.drag = { kind: e.kind === 'ground' ? 'ground' : 'body', id, startWorld: p, startScreen: screen, orig: [e.x, e.y], moved: false };
    } else if (e.kind === 'pin' && (e.b.body === null || e.a.body === null)) {
      const w = e.b.body === null ? e.b.p : e.a.p;
      this.drag = { kind: 'worldpin', id, startWorld: p, startScreen: screen, orig: [w[0], w[1]], moved: false };
    }
  }

  private onMove(ev: PointerEvent): void {
    const p = this.eventWorld(ev);
    this.shiftDown = ev.shiftKey;
    this.cursor = p;
    this.coordsEl.textContent = `x ${p[0].toFixed(2)} m   y ${p[1].toFixed(2)} m`;
    const d = this.drag;
    if (!d) {
      let h: string | null = null;
      if (this.tool === 'select') h = this.hit(p);
      else if (this.tool !== 'ground' && this.tool !== 'point' && this.tool !== 'rect' && this.tool !== 'pulley') h = this.bodyAt(p, true)?.id ?? null;
      if (h !== this.app.hover) this.app.hover = h;
      if (this.tool === 'select') {
        const g = this.selectedGround();
        const ends = g && this.app.editing ? groundEnds(g) : null;
        const onHandle = ends && (dist(p, ends.a) < this.pxToWorld(9) || dist(p, ends.b) < this.pxToWorld(9));
        const e = h ? this.scene.elements.find((x) => x.id === h) : null;
        this.canvas.style.cursor = onHandle ? 'move' : e && ('x' in e || e.kind === 'pin') ? 'grab' : h ? 'pointer' : 'default';
      }
      this.app.redraw();
      return;
    }
    const sdx = ev.clientX - d.startScreen[0], sdy = ev.clientY - d.startScreen[1];
    if (!d.moved && Math.hypot(sdx, sdy) < 3) return;
    if (d.kind === 'pan') {
      d.moved = true;
      this.r.view.cx = d.orig[0] - sdx / this.r.view.scale;
      this.r.view.cy = d.orig[1] + sdy / this.r.view.scale;
      this.app.redraw();
      return;
    }
    if (!d.moved) {
      this.app.ensureEditMode();
      this.app.history.push(this.scene);
      d.moved = true;
    }
    const id = d.id!;
    if (d.kind === 'gend') {
      const g0 = this.scene.elements.find((x) => x.id === id) as GroundEl;
      const e = groundEnds(g0);
      const fixed = d.end ? e.a : e.b;
      let moving: V = [d.orig[0] + p[0] - d.startWorld[0], d.orig[1] + p[1] - d.startWorld[1]];
      moving = this.app.snap && !ev.shiftKey ? snapSegment(fixed, moving) : moving;
      if (dist(fixed, moving) < 0.05) return;
      const [a, b] = d.end ? [fixed, moving] : [moving, fixed];
      this.app.commit((s) => {
        const t = s.elements.find((x) => x.id === id) as GroundEl;
        Object.assign(t, groundFromEnds(a, b, e.n, t.h));
      }, { history: false });
      return;
    }
    const np = this.snapV([d.orig[0] + p[0] - d.startWorld[0], d.orig[1] + p[1] - d.startWorld[1]]);
    if (d.kind === 'worldpin') {
      this.app.commit((s) => {
        const e = s.elements.find((x) => x.id === id) as PinEl;
        if (e.b.body === null) e.b.p = np; else e.a.p = np;
      }, { history: false });
      return;
    }
    this.app.commit((s) => {
      const e = s.elements.find((x) => x.id === id);
      if (!e || !('x' in e)) return;
      e.x = np[0];
      e.y = np[1];
    }, { history: false, lock: { id, keys: ['x', 'y'] } });
  }

  private onUp(ev: PointerEvent): void {
    this.canvas.releasePointerCapture(ev.pointerId);
    this.drag = null;
  }

  private onKey(ev: KeyboardEvent): void {
    if (ev.key === 'Shift') { this.shiftDown = true; this.app.redraw(); }
    const target = ev.target as HTMLElement;
    if (target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' || target.tagName === 'TEXTAREA')) return;
    const mod = ev.metaKey || ev.ctrlKey;
    const shortcut = !mod && !ev.altKey ? TOOL_KEYS[ev.key.toLowerCase()] : undefined;
    if (shortcut) {
      ev.preventDefault();
      this.setTool(shortcut);
      return;
    }
    if (mod && ev.key.toLowerCase() === 'z') {
      ev.preventDefault();
      ev.shiftKey ? this.app.redo() : this.app.undo();
    } else if (mod && ev.key.toLowerCase() === 'd') {
      ev.preventDefault();
      this.duplicateSelection();
    } else if (mod && ev.key.toLowerCase() === 'y') {
      ev.preventDefault();
      this.app.redo();
    } else if (ev.key === 'Delete' || ev.key === 'Backspace') {
      if (this.app.selection.size) {
        ev.preventDefault();
        this.app.ensureEditMode();
        this.app.deleteElements(new Set(this.app.selection));
      }
    } else if (ev.key === 'Escape') {
      if (this.pending) { this.pending = null; this.updateHint(); this.app.redraw(); }
      else if (this.tool !== 'select') this.setTool('select');
      else this.app.select([]);
    } else if (ev.key === ' ') {
      ev.preventDefault();
      this.app.toggle();
    } else if (ev.key === 'r' || ev.key === 'R') {
      this.app.reset();
    } else if (ev.key === '[' || ev.key === ']') {
      this.rotateSelection(ev.key === ']' ? -15 : 15);
    } else if (ev.key.startsWith('Arrow') && this.app.selection.size) {
      ev.preventDefault();
      const step = ev.shiftKey ? 0.01 : 0.05;
      const d: V = ev.key === 'ArrowLeft' ? [-step, 0] : ev.key === 'ArrowRight' ? [step, 0] : ev.key === 'ArrowUp' ? [0, step] : [0, -step];
      this.nudgeSelection(d);
    }
  }

  /**
   * Copy the selected bodies, shifted right, together with every connector
   * that only touches copied bodies or the world (pins, rods, springs,
   * sliders, torsional springs, ropes).
   */
  duplicateSelection(): void {
    const sel = this.app.selection;
    const els = this.scene.elements;
    const bodies = els.filter((e) => sel.has(e.id) && 'x' in e);
    if (!bodies.length) { this.flash('Select one or more bodies to duplicate (⌘/Ctrl D).'); return; }
    const off: V = [0.3, -0.3];
    const map = new Map<string, string>();
    const copies: El[] = [];
    const clone = <T extends El>(e: T): T => JSON.parse(JSON.stringify(e));
    for (const b of bodies) {
      const c = clone(b);
      c.id = F.newId(b.kind[0]);
      c.name = F.nextName({ ...this.scene, elements: [...els, ...copies] }, b.kind);
      if ('x' in c) { c.x += off[0]; c.y += off[1]; }
      map.set(b.id, c.id);
      copies.push(c);
    }
    const grounds = new Map(els.filter((e): e is GroundEl => e.kind === 'ground').map((g) => [g.id, g]));
    // connectors may hang on the world or on a shared (uncopied) ground piece
    const ok = (id: string | null) => id === null || map.has(id) || grounds.has(id);
    const remap = (a: Attach): Attach => {
      if (a.body === null) return { body: null, p: [a.p[0] + off[0], a.p[1] + off[1]] };
      if (map.has(a.body)) return { body: map.get(a.body)!, p: [...a.p] as V };
      const g = grounds.get(a.body)!;
      const c = Math.cos(g.theta), s = Math.sin(g.theta);
      return { body: a.body, p: [a.p[0] + c * off[0] + s * off[1], a.p[1] - s * off[0] + c * off[1]] };
    };
    for (const e of els) {
      if (map.has(e.id) || !('a' in e)) continue;
      const refs = 'b' in e ? [e.a.body, e.b.body] : [e.a.body];
      if (e.kind === 'rope' && !e.route.every((r) => map.has(r.pulley))) continue;
      if (!refs.every(ok) || !refs.some((r) => r !== null && map.has(r))) continue;
      const c = clone(e);
      c.id = F.newId(e.kind[0]);
      c.name = F.nextName({ ...this.scene, elements: [...els, ...copies] }, e.kind);
      c.a = remap(e.a);
      if ('b' in c && 'b' in e) c.b = remap(e.b);
      if (c.kind === 'slider' && e.kind === 'slider') c.origin = [e.origin[0] + off[0], e.origin[1] + off[1]];
      if (c.kind === 'rope') c.route = c.route.map((r) => ({ ...r, pulley: map.get(r.pulley)! }));
      map.set(e.id, c.id);
      copies.push(c);
    }
    for (const e of els) {
      if (e.kind !== 'tspring' || !map.has(e.pin)) continue;
      const c = clone(e);
      c.id = F.newId('t');
      c.name = F.nextName({ ...this.scene, elements: [...els, ...copies] }, 'tspring');
      c.pin = map.get(e.pin)!;
      copies.push(c);
    }
    this.app.ensureEditMode();
    this.app.commit((s) => s.elements.push(...copies));
    this.app.select(copies.filter((c) => 'x' in c).map((c) => c.id));
  }

  /** Rotate selected bodies about their centres by `deg` degrees. */
  private rotateSelection(deg: number): void {
    const ids = [...this.app.selection].filter((id) => {
      const e = this.scene.elements.find((x) => x.id === id);
      return e && (e.kind === 'rect' || e.kind === 'ground' || e.kind === 'pulley');
    });
    if (!ids.length) return;
    this.app.ensureEditMode();
    this.app.commit((s) => {
      for (const id of ids) {
        const e = s.elements.find((x) => x.id === id);
        if (e && 'theta' in e) e.theta += (deg * Math.PI) / 180;
      }
    }, { lock: ids.map((id) => ({ id, keys: ['x', 'y', 'theta'] as ('x' | 'y' | 'theta')[] })) });
  }

  private nudgeSelection(d: V): void {
    const ids = [...this.app.selection].filter((id) => {
      const e = this.scene.elements.find((x) => x.id === id);
      return e && 'x' in e;
    });
    if (!ids.length) return;
    this.app.ensureEditMode();
    this.app.commit((s) => {
      for (const id of ids) {
        const e = s.elements.find((x) => x.id === id);
        if (e && 'x' in e) { e.x += d[0]; e.y += d[1]; }
      }
    }, { lock: ids.map((id) => ({ id, keys: ['x', 'y'] as ('x' | 'y' | 'theta')[] })) });
  }
}
