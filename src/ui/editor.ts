// Canvas interaction: tools, hit testing, drag-to-move, palette drop, keys.

import { RopeC } from '../engine/constraints';
import { autoWrapDir, ropeGeometry } from '../engine/rope';
import * as F from '../model/factory';
import type { Attach, El, PinEl, RopeRoute, Scene } from '../model/types';
import { App } from './app';
import { Tool, TOOL_HELP, buildPalette } from './palette';
import { Renderer } from './render';

type V = [number, number];

interface Pending {
  tool: 'rod' | 'spring' | 'rope';
  a: Attach;
  aWorld: V;
  route: string[];
}

interface Drag {
  kind: 'body' | 'ground' | 'worldpin' | 'pan';
  id?: string;
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
  private hintEl = document.getElementById('hint')!;

  constructor(private app: App, private canvas: HTMLCanvasElement) {
    this.setActive = buildPalette(document.getElementById('palette')!, (t) => this.setTool(t));
    this.setTool('select');
    app.editorOverlay = () => ({
      preview: this.pending
        ? { kind: this.pending.tool, points: [this.pending.aWorld, ...this.pending.route.map((id) => this.poseXY(id))] }
        : null,
      cursor: this.pending ? this.cursor : null,
    });
    this.bind();
  }

  get r(): Renderer { return this.app.renderer; }
  get scene(): Scene { return this.app.scene; }

  setTool(t: Tool): void {
    this.tool = t;
    this.pending = null;
    this.setActive(t);
    this.updateHint();
    this.canvas.style.cursor = t === 'select' ? 'default' : 'crosshair';
    this.app.redraw();
  }

  private updateHint(): void {
    let h = this.tool === 'select' ? '' : TOOL_HELP[this.tool];
    if (this.pending) {
      h = this.pending.tool === 'rope'
        ? `Rope: click pulleys in order (${this.pending.route.length} so far), then click the other end. Esc cancels.`
        : `Click the second point for the ${this.pending.tool}. Esc cancels.`;
    }
    this.hintEl.textContent = h;
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
      if (e.kind === 'rect') {
        const [lx, ly] = this.toLocal(e.id, p);
        if (Math.abs(lx) <= e.w / 2 + tol && Math.abs(ly) <= e.h / 2 + tol) return e;
      }
    }
    if (includeGround) {
      for (let i = els.length - 1; i >= 0; i--) {
        const e = els[i];
        if (e.kind !== 'ground') continue;
        const [lx, ly] = this.toLocal(e.id, p);
        if (Math.abs(lx) <= e.w / 2 + tol && Math.abs(ly) <= e.h / 2 + tol) return e;
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
  attachAt(p: V, exclude?: string): { a: Attach; world: V } {
    let body = this.bodyAt(p);
    if (body && body.id === exclude) body = null;
    if (!body) {
      const w = this.snapV(p);
      return { a: { body: null, p: w }, world: w };
    }
    if (body.kind === 'point') return { a: { body: body.id, p: [0, 0] }, world: this.poseXY(body.id) };
    if (body.kind === 'pulley') {
      const c = this.poseXY(body.id);
      if (dist(p, c) < body.r * 0.6) return { a: { body: body.id, p: [0, 0] }, world: c };
    }
    let local = this.toLocal(body.id, p);
    if (body.kind === 'rect' && this.app.snap) {
      // snap to corners, edge midpoints, quarter points and centre
      const cands: V[] = [];
      for (const fx of [-0.5, -0.25, 0, 0.25, 0.5]) for (const fy of [-0.5, 0, 0.5]) cands.push([fx * body.w, fy * body.h]);
      let best = this.pxToWorld(10), pick: V | null = null;
      for (const c of cands) { const d = dist(c, local); if (d < best) { best = d; pick = c; } }
      local = pick ?? local;
    } else if (body.kind === 'ground' && this.app.snap) {
      const w = this.snapV(p);
      local = this.toLocal(body.id, w);
    }
    const ps = Renderer.pose(this.scene, this.app.sys, this.q(), body.id)!;
    const c = Math.cos(ps.theta), s = Math.sin(ps.theta);
    const world: V = [ps.x + c * local[0] - s * local[1], ps.y + s * local[0] + c * local[1]];
    return { a: { body: body.id, p: [local[0], local[1]] }, world };
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
    const add = (e: El, lock?: { id: string; keys: ('x' | 'y' | 'theta')[] }) => {
      app.commit((s) => s.elements.push(e), lock ? { lock } : {});
      app.select([e.id]);
      this.setTool('select');
    };
    switch (tool) {
      case 'ground':
        add(F.ground(F.nextName(this.scene, 'ground'), sp[0], sp[1]));
        return;
      case 'point':
        add(F.point(F.nextName(this.scene, 'point'), sp[0], sp[1], 1));
        return;
      case 'rect':
        add(F.rect(F.nextName(this.scene, 'rect'), sp[0], sp[1], 2, 0.6, 0.2));
        return;
      case 'pulley':
        add(F.pulley(F.nextName(this.scene, 'pulley'), sp[0], sp[1], 1, 0.12));
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
        const pinId = this.hit(p);
        const pin = this.scene.elements.find((e) => e.id === pinId && e.kind === 'pin') as PinEl | undefined;
        if (!pin) { this.flash('Click on a pin joint.'); return; }
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
          const body = this.bodyAt(p, false);
          if (body && body.kind === 'pulley' && pend.route[pend.route.length - 1] !== body.id && dist(p, this.poseXY(body.id)) < body.r) {
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
    this.hintEl.textContent = msg;
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
    c.addEventListener('pointerleave', () => { this.cursor = null; this.app.hover = null; this.app.redraw(); });
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
    this.canvas.setPointerCapture(ev.pointerId);
    const screen: V = [ev.clientX, ev.clientY];
    if (ev.button === 1 || ev.button === 2 || (ev.button === 0 && ev.altKey)) {
      this.drag = { kind: 'pan', startWorld: p, startScreen: screen, orig: [this.r.view.cx, this.r.view.cy], moved: false };
      return;
    }
    if (this.tool !== 'select') {
      this.app.ensureEditMode();
      this.apply(this.tool, p);
      return;
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
    this.cursor = this.pending ? this.snapV(p) : p;
    const d = this.drag;
    if (!d) {
      const h = this.tool === 'select' ? this.hit(p) : null;
      if (h !== this.app.hover) this.app.hover = h;
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
    const np = this.snapV([d.orig[0] + p[0] - d.startWorld[0], d.orig[1] + p[1] - d.startWorld[1]]);
    const id = d.id!;
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
    const target = ev.target as HTMLElement;
    if (target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' || target.tagName === 'TEXTAREA')) return;
    const mod = ev.metaKey || ev.ctrlKey;
    if (mod && ev.key.toLowerCase() === 'z') {
      ev.preventDefault();
      ev.shiftKey ? this.app.redo() : this.app.undo();
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
    }
  }
}
