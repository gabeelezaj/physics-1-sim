// Properties panel: an editable form for the selected element, or the scene
// settings when nothing is selected. SI units throughout; angles in degrees.

import { ropeGeometry } from '../engine/rope';
import type { Attach, El, Scene } from '../model/types';
import { App, CoordKey } from './app';
import type { Editor } from './editor';
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
}

const POSE: Field[] = [
  { key: 'x', label: 'x₀', unit: 'm', step: 0.05, lock: ['x'] },
  { key: 'y', label: 'y₀', unit: 'm', step: 0.05, lock: ['y'] },
];
const ANGLE: Field = { key: 'theta', label: 'θ₀', unit: '°', kind: 'deg', step: 1, lock: ['theta'] };
const VEL: Field[] = [
  { key: 'vx', label: 'ẋ₀', unit: 'm/s', step: 0.1 },
  { key: 'vy', label: 'ẏ₀', unit: 'm/s', step: 0.1 },
];
const OMEGA: Field = { key: 'omega', label: 'θ̇₀', unit: 'rad/s', step: 0.1 };

const FIELDS: Record<El['kind'], Field[]> = {
  ground: [...POSE, ANGLE, { key: 'w', label: 'width', unit: 'm', step: 0.1, min: 0.05 }, { key: 'h', label: 'thickness', unit: 'm', step: 0.02, min: 0.01 }],
  point: [{ key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'radius', label: 'radius (draw)', unit: 'm', step: 0.01, min: 0.005 }, ...POSE, ...VEL],
  rect: [
    { key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'I', label: 'I_G', unit: 'kg·m²', kind: 'auto', step: 0.01 },
    { key: 'w', label: 'length', unit: 'm', step: 0.05, min: 0.01 }, { key: 'h', label: 'height', unit: 'm', step: 0.02, min: 0.01 },
    ...POSE, ANGLE, ...VEL, OMEGA,
  ],
  pulley: [
    { key: 'm', label: 'mass', unit: 'kg', step: 0.1, min: 1e-6 }, { key: 'I', label: 'I_O', unit: 'kg·m²', kind: 'auto', step: 0.001 },
    { key: 'r', label: 'radius', unit: 'm', step: 0.01, min: 0.01 }, { key: 'fixedAxle', label: 'fixed axle', kind: 'bool' },
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
  constructor(private app: App, private root: HTMLElement, private editor: Editor) {
    app.onSelection(() => this.render());
    app.onSceneChange(() => this.render());
    this.render();
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
    );
    this.root.appendChild(grid);
    const note = document.createElement('div');
    note.className = 'props-note';
    note.textContent = 'Select an element to edit it. Initial positions you type are held while the rest of the mechanism re-assembles around them.';
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

    const grid = document.createElement('div');
    grid.className = 'fields';
    for (const f of FIELDS[e.kind]) {
      const raw = getPath(e, f.key);
      const kind = f.kind ?? 'num';
      const shown = kind === 'deg' ? (raw * 180) / Math.PI : raw;
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
    actions.appendChild(this.button('Delete', () => this.app.deleteElements(new Set([e.id]))));
    this.root.appendChild(actions);

    const info = this.info(e);
    if (info) {
      const n = document.createElement('div');
      n.className = 'props-note';
      n.innerHTML = info;
      this.root.appendChild(n);
    }
  }

  private autoInertia(e: El): string | undefined {
    if (e.kind === 'rect') return `auto ${fmt((e.m * (e.w * e.w + e.h * e.h)) / 12)}`;
    if (e.kind === 'pulley') return `auto ${fmt(0.5 * e.m * e.r * e.r)}`;
    return undefined;
  }

  private info(e: El): string {
    switch (e.kind) {
      case 'rect': return 'Angles are measured counter-clockwise from the body\'s long axis. Leave I blank for a uniform plate, m(L²+h²)/12.';
      case 'pulley': return 'Leave I blank for a uniform disc, ½mr². The rope does not slip, so the pulley\'s inertia enters the dynamics.';
      case 'slider': return 'The attachment point stays on a fixed straight track. With rotation locked the body translates only (a cart).';
      case 'tspring': return 'Torque −k_t(θ_A − θ_B − θ₀) on body A of the pin, opposite on body B.';
      case 'pin': return 'Revolute joint: the two attachment points coincide. Add a torsional spring with the palette tool.';
      case 'rope': return 'Inextensible, always taut (bilateral). Wrap direction per pulley: CCW = rope travels counter-clockwise around it.';
      case 'ground': return 'Fixed. The hatched side is the inside; the solid edge is the surface.';
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
