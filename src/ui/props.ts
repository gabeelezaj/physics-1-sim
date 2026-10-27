// Properties panel: an editable form for the selected element, or the scene
// settings when nothing is selected. SI units throughout; angles in degrees.

import { ropeGeometry } from '../engine/rope';
import { groundEnds, groundFromEnds } from '../model/ground';
import { wedgeFromLowEnd, wedgeLowEnd } from '../model/shapes';
import type { Attach, El, GroundEl, RectEl, Scene } from '../model/types';
import { App, CoordKey } from './app';
import type { Editor } from './editor';
import { liveRows } from './live';
import { Renderer } from './render';

type FieldKind = 'num' | 'deg' | 'bool' | 'text' | 'auto';

interface Field {
  key: string;
  label: string;
  unit?: string;
  kind?: FieldKind;
  step?: number;
  min?: number;
  lock?: CoordKey[];
  wide?: boolean;
  /** Fields describing the state at t = 0 are grouped separately. */
  init?: boolean;
}

const POSE: Field[] = [
  { key: 'x', label: 'x', unit: 'm', step: 0.05, lock: ['x'], init: true },
  { key: 'y', label: 'y', unit: 'm', step: 0.05, lock: ['y'], init: true },
];
const ANGLE: Field = { key: 'theta', label: 'angle θ', unit: '°', kind: 'deg', step: 1, lock: ['theta'], init: true };
const VEL: Field[] = [
  { key: 'vx', label: 'velocity ẋ', unit: 'm/s', step: 0.1, init: true },
  { key: 'vy', label: 'velocity ẏ', unit: 'm/s', step: 0.1, init: true },
];
/** Absent means true, so it is shown as `value !== false`. */
const COLLIDE: Field = { key: 'collide', label: 'collides', kind: 'bool' };
const OMEGA: Field = { key: 'omega', label: 'spin θ̇', unit: 'rad/s', step: 0.1, init: true };

const FIELDS: Record<El['kind'], Field[]> = {
  ground: [],
  point: [{ key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'radius', label: 'radius', unit: 'm', step: 0.01, min: 0.005 }, COLLIDE, ...POSE, ...VEL],
  rect: [
    { key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'I', label: 'I_G', unit: 'kg·m²', kind: 'auto', step: 0.01 },
    { key: 'w', label: 'length', unit: 'm', step: 0.05, min: 0.01 }, { key: 'h', label: 'height', unit: 'm', step: 0.02, min: 0.01 },
    COLLIDE, ...POSE, ANGLE, ...VEL, OMEGA,
  ],
  pulley: [
    { key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'I', label: 'I_O', unit: 'kg·m²', kind: 'auto', step: 0.001 },
    { key: 'r', label: 'radius', unit: 'm', step: 0.01, min: 0.01 }, { key: 'fixedAxle', label: 'fixed axle', kind: 'bool' }, COLLIDE,
    ...POSE, ANGLE, ...VEL, OMEGA,
  ],
  rod: [{ key: 'length', label: 'length', unit: 'm', step: 0.05, min: 1e-4 }],
  pin: [],
  slider: [
    { key: 'origin.0', label: 'track x', unit: 'm', step: 0.05 }, { key: 'origin.1', label: 'track y', unit: 'm', step: 0.05 },
    { key: 'angle', label: 'track angle', unit: '°', kind: 'deg', step: 5 }, { key: 'mu', label: 'friction μ', step: 0.05, min: 0 },
    { key: 'lockRotation', label: 'lock rotation', kind: 'bool' }, { key: 'extent', label: 'draw half-length', unit: 'm', step: 0.1, min: 0.05 },
  ],
  spring: [
    { key: 'k', label: 'k', unit: 'N/m', step: 10, min: 0 }, { key: 'L0', label: 'rest length L₀', unit: 'm', step: 0.05, min: 0 },
    { key: 'c', label: 'damping c', unit: 'N·s/m', step: 0.1, min: 0 },
  ],
  tspring: [
    { key: 'k', label: 'k_t', unit: 'N·m/rad', step: 1, min: 0 }, { key: 'theta0', label: 'rest angle θ₀', unit: '°', kind: 'deg', step: 1 },
    { key: 'c', label: 'damping c_t', unit: 'N·m·s', step: 0.05, min: 0 },
  ],
  rope: [{ key: 'length', label: 'length', unit: 'm', step: 0.05, min: 1e-3 }],
};

const KIND_LABEL: Record<El['kind'], string> = {
  ground: 'Ground', point: 'Point mass', rect: 'Rigid body', pulley: 'Pulley', rod: 'Massless rod',
  pin: 'Pin joint', slider: 'Slider / track', spring: 'Linear spring', tspring: 'Torsional spring', rope: 'Rope',
};

function getPath(o: any, path: string): any {
  return path.split('.').reduce((v, k) => v?.[k], o);
}
function setPath(o: any, path: string, value: any): void {
  const keys = path.split('.');
  const last = keys.pop()!;
  const parent = keys.reduce((v, k) => v[k], o);
  parent[last] = value;
}

const fmt = (v: number) => (Math.abs(v) < 1e-12 ? '0' : Number(v.toPrecision(6)).toString());

export class PropsPanel {
  private liveEl: HTMLElement | null = null;
  private lastLive = 0;

  constructor(private app: App, private root: HTMLElement, private editor: Editor) {
    app.onSelection(() => this.render());
    app.onSceneChange(() => this.render());
    app.onTick(() => this.updateLive());
    app.onState(() => this.updateLive(true));
    this.render();
  }

  /** Refresh the live readout of the selected element (throttled while running). */
  private updateLive(force = false): void {
    const el = this.liveEl;
    if (!el || !el.isConnected) return;
    const now = performance.now();
    if (!force && now - this.lastLive < 150) return;
    this.lastLive = now;
    const id = [...this.app.selection][0];
    const e = this.scene.elements.find((x) => x.id === id);
    const rows = e ? liveRows(this.app, e) : [];
    el.hidden = !rows.length;
    el.innerHTML = `<div class="props-section">Live · t = ${this.app.sim.t.toFixed(3)} s</div>
      <div class="live-grid">${rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('')}</div>`;
  }

  private get scene(): Scene { return this.app.scene; }

  render(): void {
    const ae = document.activeElement as HTMLElement | null;
    const focusKey = ae && this.root.contains(ae) ? ae.dataset.key : undefined;
    this.root.innerHTML = '';
    const sel = [...this.app.selection];
    if (sel.length === 0) this.renderScene();
    else if (sel.length > 1) this.renderMulti(sel);
    else {
      const e = this.scene.elements.find((x) => x.id === sel[0]);
      if (e) this.renderElement(e);
    }
    if (focusKey) (this.root.querySelector(`[data-key="${focusKey}"]`) as HTMLElement | null)?.focus();
  }

  private renderScene(): void {
    const s = this.scene;
    this.root.innerHTML = `<h3>Scene <span class="kind">${s.elements.length} elements · ${this.app.sys.n} coordinates · ${this.app.sys.m} constraint rows</span></h3>`;
    const grid = document.createElement('div');
    grid.className = 'fields';
    grid.append(
      this.input('Title', 'text', s.title, (v) => this.app.commit((sc) => (sc.title = String(v))), { key: 'title' }),
      this.input('Time step', 'num', s.settings.dt * 1000, (v) => this.app.commit((sc) => (sc.settings.dt = Math.max(1e-5, Number(v) / 1000))), { unit: 'ms', step: 0.1, key: 'dt' }),
      this.input('Baumgarte α', 'num', s.settings.alpha, (v) => this.app.commit((sc) => (sc.settings.alpha = Number(v))), { unit: '1/s', step: 1, key: 'alpha' }),
      this.input('Baumgarte β', 'num', s.settings.beta, (v) => this.app.commit((sc) => (sc.settings.beta = Number(v))), { unit: '1/s', step: 1, key: 'beta' }),
      this.input('Collisions', 'bool', s.settings.contact !== false, (v) => this.app.commit((sc) => (sc.settings.contact = !!v)), { key: 'contact' }),
      this.input('Restitution e', 'num', s.settings.restitution ?? 0.5, (v) => this.app.commit((sc) => (sc.settings.restitution = Math.min(1, Math.max(0, Number(v))))), { step: 0.1, key: 'restitution' }),
    );
    this.root.appendChild(grid);
    const note = document.createElement('div');
    note.className = 'props-note';
    note.textContent = 'Select an element to edit it. Positions you type are held while the rest of the mechanism re-assembles around them. Collisions cover walls, floors and other bodies; restitution 1 = perfectly elastic, 0 = no bounce. Untick “collides” on a body to let it pass through things.';
    this.root.appendChild(note);
  }

  private renderMulti(ids: string[]): void {
    this.root.innerHTML = `<h3>${ids.length} elements selected</h3>`;
    const actions = document.createElement('div');
    actions.className = 'props-actions';
    actions.appendChild(this.button('Delete', () => this.app.deleteElements(new Set(ids))));
    this.root.appendChild(actions);
  }

  private renderElement(e: El): void {
    const h = document.createElement('h3');
    h.innerHTML = `<span class="kind">${KIND_LABEL[e.kind]}</span>`;
    const name = document.createElement('input');
    name.type = 'text';
    name.value = e.name;
    name.dataset.key = 'name';
    name.addEventListener('change', () => this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id); if (t) t.name = name.value.trim() || t.name; }));
    h.prepend(name);
    this.root.appendChild(h);

    if (e.kind === 'ground') {
      if (e.shape === 'wedge') this.renderRamp(e); else this.renderGround(e);
    }
    if (e.kind === 'rect' && e.shape === 'wedge') this.renderRamp(e);
    const own = e.kind === 'rect' && e.shape === 'wedge' ? FIELDS.rect.filter((f) => f.key !== 'w' && f.key !== 'h') : FIELDS[e.kind];
    const sections: [string, Field[]][] = [
      ['Properties', own.filter((f) => !f.init)],
      ['Initial conditions (t = 0)', own.filter((f) => f.init)],
    ];
    for (const [title, fields] of sections) {
      if (!fields.length) continue;
      const head = document.createElement('div');
      head.className = 'props-section';
      head.textContent = title;
      this.root.appendChild(head);
      const grid = document.createElement('div');
      grid.className = 'fields';
      for (const f of fields) {
        const raw = getPath(e, f.key);
        const kind = f.kind ?? 'num';
        const shown = kind === 'deg' ? (raw * 180) / Math.PI : f.key === 'collide' ? raw !== false : raw;
        grid.appendChild(this.input(f.label, kind, shown, (v) => {
          let val: any = v;
          if (kind === 'deg') val = (Number(v) * Math.PI) / 180;
          else if (kind === 'num') val = f.min !== undefined ? Math.max(f.min, Number(v)) : Number(v);
          else if (kind === 'auto') val = v === '' || v === null ? null : Math.max(1e-9, Number(v));
          this.app.ensureEditMode();
          this.app.commit((s) => {
            const t = s.elements.find((x) => x.id === e.id);
            if (t) setPath(t, f.key, val);
          }, f.lock ? { lock: { id: e.id, keys: f.lock } } : {});
        }, { unit: f.unit, step: f.step, key: f.key, placeholder: kind === 'auto' ? this.autoInertia(e) : undefined }));
      }
      this.root.appendChild(grid);
    }

    if ('a' in e) this.renderAttach(e, 'a', e.kind === 'slider' ? 'Body point' : 'End A');
    if ('b' in e) this.renderAttach(e, 'b', 'End B');
    if (e.kind === 'rope') this.renderRoute(e);

    const actions = document.createElement('div');
    actions.className = 'props-actions';
    const q = this.app.sim.q;
    const aw = (a: Attach) => Renderer.attachWorld(this.scene, this.app.sys, q, a);
    if (e.kind === 'rod' || e.kind === 'spring') {
      const cur = Math.hypot(aw(e.b)[0] - aw(e.a)[0], aw(e.b)[1] - aw(e.a)[1]);
      const key = e.kind === 'rod' ? 'length' : 'L0';
      actions.appendChild(this.button(`Set ${e.kind === 'rod' ? 'length' : 'L₀'} = current (${fmt(cur)} m)`, () =>
        this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id) as any; t[key] = cur; })));
    }
    if (e.kind === 'tspring') {
      const pin = this.scene.elements.find((x) => x.id === e.pin);
      if (pin && pin.kind === 'pin') {
        const ang = (a: Attach) => (a.body ? Renderer.pose(this.scene, this.app.sys, q, a.body)?.theta ?? 0 : 0);
        const rel = ang(pin.a) - ang(pin.b);
        actions.appendChild(this.button(`Set θ₀ = current (${fmt((rel * 180) / Math.PI)}°)`, () =>
          this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id) as any; t.theta0 = rel; })));
      }
    }
    if (e.kind === 'rope') {
      actions.appendChild(this.button('Make taut (length = current path)', () => {
        const L = this.editor.ropeLength(e);
        this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id) as any; t.length = L; });
      }));
    }
    if (e.kind === 'ground' && e.shape !== 'wedge') {
      actions.appendChild(this.button('Flip solid side', () => this.app.commit((s) => {
        const t = s.elements.find((x) => x.id === e.id) as GroundEl;
        const g = groundEnds(t);
        Object.assign(t, groundFromEnds(g.a, g.b, [-g.n[0], -g.n[1]], t.h));
      })));
    }
    const del = this.button('Delete', () => this.app.deleteElements(new Set([e.id])));
    del.classList.add('danger');
    actions.appendChild(del);
    this.root.appendChild(actions);

    this.liveEl = document.createElement('div');
    this.liveEl.className = 'live';
    this.root.appendChild(this.liveEl);
    this.updateLive(true);

    const info = this.info(e);
    if (info) {
      const n = document.createElement('div');
      n.className = 'props-note';
      n.innerHTML = info;
      this.root.appendChild(n);
    }
  }

  /**
   * Ramp (fixed) or wedge (movable): slope angle, base and height, which way
   * it rises, and a switch between the two. The low end of the slope stays
   * put while the size or angle changes.
   */
  private renderRamp(g: GroundEl | RectEl): void {
    const head = document.createElement('div');
    head.className = 'props-section';
    head.textContent = g.kind === 'ground' ? 'Ramp (fixed)' : 'Wedge (movable)';
    this.root.appendChild(head);
    const grid = document.createElement('div');
    grid.className = 'fields';
    const reshape = (w: number, h: number, flip: boolean) => this.app.commit((s) => {
      const t = s.elements.find((x) => x.id === g.id) as GroundEl | RectEl;
      const low = wedgeLowEnd(t);
      Object.assign(t, { w, h, flip }, wedgeFromLowEnd(low, w, h, flip, t.theta));
    }, { lock: { id: g.id, keys: ['x', 'y'] } });
    const angle = (Math.atan2(g.h, g.w) * 180) / Math.PI;
    grid.append(
      this.input('slope angle', 'num', angle, (v) => {
        const a = Math.min(85, Math.max(1, Number(v)));
        reshape(g.w, g.w * Math.tan((a * Math.PI) / 180), !!g.flip);
      }, { unit: '°', step: 1, key: 'rampAngle' }),
      this.input('base', 'num', g.w, (v) => reshape(Math.max(0.05, Number(v)), g.h, !!g.flip), { unit: 'm', step: 0.05, key: 'rampW' }),
      this.input('height', 'num', g.h, (v) => reshape(g.w, Math.max(0.02, Number(v)), !!g.flip), { unit: 'm', step: 0.05, key: 'rampH' }),
      this.input('rises to the left', 'bool', !!g.flip, (v) => reshape(g.w, g.h, !!v), { key: 'rampFlip' }),
      this.input('movable (slides)', 'bool', g.kind === 'rect', (v) => this.app.commit((s) => {
        const i = s.elements.findIndex((x) => x.id === g.id);
        const t = s.elements[i] as GroundEl | RectEl;
        const base = { id: t.id, name: t.name, x: t.x, y: t.y, theta: t.theta, w: t.w, h: t.h, shape: 'wedge' as const, flip: t.flip };
        s.elements[i] = v
          ? { ...base, kind: 'rect', vx: 0, vy: 0, omega: 0, m: 3, I: null }
          : { ...base, kind: 'ground' };
      }), { key: 'rampMovable' }),
    );
    if (g.kind === 'ground') {
      grid.append(
        this.input('x (centroid)', 'num', g.x, (v) => this.app.commit((s) => { (s.elements.find((x) => x.id === g.id) as GroundEl).x = Number(v); }), { unit: 'm', step: 0.05, key: 'rampX' }),
        this.input('y (centroid)', 'num', g.y, (v) => this.app.commit((s) => { (s.elements.find((x) => x.id === g.id) as GroundEl).y = Number(v); }), { unit: 'm', step: 0.05, key: 'rampY' }),
      );
    }
    this.root.appendChild(grid);
  }

  /** Ground pieces are edited by their surface end points. */
  private renderGround(g: GroundEl): void {
    const ends = groundEnds(g);
    const head = document.createElement('div');
    head.className = 'props-section';
    head.textContent = 'Surface';
    this.root.appendChild(head);
    const grid = document.createElement('div');
    grid.className = 'fields';
    const set = (which: 0 | 1, axis: 0 | 1) => (v: number) => this.app.commit((s) => {
      const t = s.elements.find((x) => x.id === g.id) as GroundEl;
      const e = groundEnds(t);
      const a: [number, number] = [...e.a], b: [number, number] = [...e.b];
      (which ? b : a)[axis] = Number(v);
      if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.01) return;
      Object.assign(t, groundFromEnds(a, b, e.n, t.h));
    });
    grid.append(
      this.input('start x', 'num', ends.a[0], set(0, 0), { unit: 'm', step: 0.05, key: 'ga0' }),
      this.input('start y', 'num', ends.a[1], set(0, 1), { unit: 'm', step: 0.05, key: 'ga1' }),
      this.input('end x', 'num', ends.b[0], set(1, 0), { unit: 'm', step: 0.05, key: 'gb0' }),
      this.input('end y', 'num', ends.b[1], set(1, 1), { unit: 'm', step: 0.05, key: 'gb1' }),
      this.input('thickness', 'num', g.h, (v) => this.app.commit((s) => {
        const t = s.elements.find((x) => x.id === g.id) as GroundEl;
        const e = groundEnds(t);
        Object.assign(t, groundFromEnds(e.a, e.b, e.n, Math.max(0.01, Number(v))));
      }), { unit: 'm', step: 0.02, key: 'gh' }),
      this.input('length', 'num', g.w, (v) => this.app.commit((s) => {
        const t = s.elements.find((x) => x.id === g.id) as GroundEl;
        const e = groundEnds(t);
        const L = Math.max(0.05, Number(v));
        const ux = (e.b[0] - e.a[0]) / t.w, uy = (e.b[1] - e.a[1]) / t.w;
        Object.assign(t, groundFromEnds(e.a, [e.a[0] + ux * L, e.a[1] + uy * L], e.n, t.h));
      }), { unit: 'm', step: 0.05, key: 'gw' }),
    );
    this.root.appendChild(grid);
  }

  private autoInertia(e: El): string | undefined {
    if (e.kind === 'rect') return `auto ${fmt((e.m * (e.w * e.w + e.h * e.h)) / 12)}`;
    if (e.kind === 'pulley') return `auto ${fmt(0.5 * e.m * e.r * e.r)}`;
    return undefined;
  }

  private info(e: El): string {
    switch (e.kind) {
      case 'rect': return e.shape === 'wedge'
        ? 'A movable wedge (uniform right triangle, I = m(b² + h²)/18 about its centroid). Its position is its centre of mass. Untick “movable” to fix it in place.'
        : 'Angles are measured counter-clockwise from the body\'s long axis. Leave I blank for a uniform plate, m(L²+h²)/12. Bodies joined directly by a pin or rod never collide with each other.';
      case 'pulley': return 'Leave I blank for a uniform disc, ½mr². The rope does not slip, so the pulley\'s inertia enters the dynamics.';
      case 'slider': return 'The attachment point stays on a fixed straight track. With rotation locked the body translates only (a cart).';
      case 'tspring': return 'Torque −k_t(θ_A − θ_B − θ₀) on body A of the pin, opposite on body B.';
      case 'pin': return 'Revolute joint: the two attachment points coincide. Add a torsional spring with the palette tool.';
      case 'rope': return 'Inextensible, always taut (bilateral). Wrap direction per pulley: CCW = rope travels counter-clockwise around it.';
      case 'ground': return e.shape === 'wedge'
        ? 'A fixed, frictionless incline: blocks and balls slide down it at g sin θ. Tick “movable” to turn it into a wedge that slides on the floor when something rides it.'
        : 'Fixed and solid: bodies bounce off it. The hatched side is the inside. Drag the square handles on the canvas to move either end.';
      default: return '';
    }
  }

  private renderAttach(e: El & { a: Attach }, key: 'a' | 'b', label: string): void {
    const at = (e as any)[key] as Attach;
    const row = document.createElement('div');
    row.className = 'fields';
    row.style.marginTop = '8px';
    const sel = document.createElement('select');
    sel.dataset.key = `${key}.body`;
    const opts = [{ id: '', name: 'World (fixed)' }, ...this.scene.elements.filter((x) => x.kind === 'ground' || x.kind === 'point' || x.kind === 'rect' || x.kind === 'pulley').map((x) => ({ id: x.id, name: x.name }))];
    for (const o of opts) {
      const op = document.createElement('option');
      op.value = o.id;
      op.textContent = o.name;
      if ((at.body ?? '') === o.id) op.selected = true;
      sel.appendChild(op);
    }
    sel.addEventListener('change', () => {
      const body = sel.value || null;
      // keep the point where it is in the world
      const w = Renderer.attachWorld(this.scene, this.app.sys, this.app.sim.q, at);
      let p: [number, number] = [w[0], w[1]];
      if (body) {
        const ps = Renderer.pose(this.scene, this.app.sys, this.app.sim.q, body);
        if (ps) {
          const c = Math.cos(ps.theta), s = Math.sin(ps.theta);
          const dx = w[0] - ps.x, dy = w[1] - ps.y;
          p = [c * dx + s * dy, -s * dx + c * dy];
        }
      }
      this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id) as any; t[key] = { body, p }; });
    });
    const wrap = document.createElement('label');
    wrap.className = 'field wide';
    wrap.innerHTML = `<span>${label}</span>`;
    const v = document.createElement('span');
    v.className = 'val';
    v.appendChild(sel);
    wrap.appendChild(v);
    row.appendChild(wrap);
    const frame = at.body ? 'local' : 'world';
    for (const i of [0, 1] as const) {
      row.appendChild(this.input(`${i ? 'y' : 'x'} (${frame})`, 'num', at.p[i], (val) =>
        this.app.commit((s) => { const t = s.elements.find((x) => x.id === e.id) as any; t[key].p[i] = Number(val); }),
      { unit: 'm', step: 0.05, key: `${key}.p.${i}` }));
    }
    this.root.appendChild(row);
  }

  private renderRoute(e: El & { kind: 'rope' }): void {
    if (!e.route.length) return;
    const box = document.createElement('div');
    box.style.marginTop = '8px';
    box.innerHTML = '<div class="muted" style="margin-bottom:4px">Route (end A → end B)</div>';
    e.route.forEach((r, i) => {
      const p = this.scene.elements.find((x) => x.id === r.pulley);
      const row = document.createElement('div');
      row.className = 'route-row';
      row.innerHTML = `<span>${i + 1}. ${p?.name ?? '?'}</span>`;
      const s = document.createElement('select');
      s.innerHTML = '<option value="1">counter-clockwise</option><option value="-1">clockwise</option>';
      s.value = String(r.dir);
      s.addEventListener('change', () => this.app.commit((sc) => {
        const t = sc.elements.find((x) => x.id === e.id);
        if (t && t.kind === 'rope') {
          t.route[i].dir = Number(s.value) as 1 | -1;
          t.length = this.ropeLengthFor(sc, t);
        }
      }));
      row.appendChild(s);
      box.appendChild(row);
    });
    this.root.appendChild(box);
  }

  private ropeLengthFor(_s: Scene, rope: El): number {
    if (rope.kind !== 'rope') return 0;
    const sys = this.app.sys;
    const nodes = [
      { type: 'end' as const, a: sys.resolve(rope.a) },
      ...rope.route.map((r) => {
        const p = this.scene.elements.find((x) => x.id === r.pulley);
        return { type: 'pulley' as const, b: sys.byId.get(r.pulley)!, r: p && p.kind === 'pulley' ? p.r : 0.1, sigma: r.dir };
      }),
      { type: 'end' as const, a: sys.resolve(rope.b) },
    ];
    return ropeGeometry(nodes, this.app.sim.q).total;
  }

  // -------------------------------------------------------------- widgets

  private input(
    label: string, kind: FieldKind, value: any, onChange: (v: any) => void,
    o: { unit?: string; step?: number; key?: string; placeholder?: string } = {},
  ): HTMLElement {
    const wrap = document.createElement('label');
    wrap.className = 'field';
    const l = document.createElement('span');
    l.textContent = label;
    wrap.appendChild(l);
    const v = document.createElement('span');
    v.className = 'val';
    const inp = document.createElement('input');
    if (o.key) inp.dataset.key = o.key;
    if (kind === 'bool') {
      inp.type = 'checkbox';
      inp.checked = !!value;
      inp.addEventListener('change', () => onChange(inp.checked));
    } else if (kind === 'text') {
      inp.type = 'text';
      inp.value = String(value ?? '');
      inp.addEventListener('change', () => onChange(inp.value));
    } else {
      inp.type = 'number';
      if (o.step) inp.step = String(o.step);
      inp.value = value === null || value === undefined ? '' : fmt(Number(value));
      if (o.placeholder) inp.placeholder = o.placeholder;
      inp.addEventListener('change', () => {
        if (kind === 'auto' && inp.value.trim() === '') return onChange(null);
        const n = Number(inp.value);
        if (Number.isFinite(n)) onChange(n);
      });
    }
    v.appendChild(inp);
    if (o.unit) {
      const u = document.createElement('span');
      u.className = 'unit';
      u.textContent = o.unit;
      v.appendChild(u);
    }
    wrap.appendChild(v);
    return wrap;
  }

  private button(label: string, onClick: () => void): HTMLButtonElement {
    const b = document.createElement('button');
    b.className = 'small';
    b.textContent = label;
    b.addEventListener('click', onClick);
    return b;
  }
}
