// Application state: the scene being edited, the running simulation, the
// view, selection, and the frame loop. UI modules subscribe to its events.

import type { Linearization } from '../engine/analysis';
import { ptPos } from '../engine/kin';
import { Simulation } from '../engine/sim';
import { System } from '../engine/system';
import type { DynBodyEl, Scene } from '../model/types';
import { emptyScene } from '../model/types';
import { History } from './history';
import { Renderer } from './render';

export type CoordKey = 'x' | 'y' | 'theta';

export interface Lock {
  id: string;
  keys: CoordKey[];
}

/** Time series recorded while the simulation runs (for the plots). */
export class Recorder {
  t: number[] = [];
  /** One array per coordinate, then KE, PE, E, dissipated energy. */
  ch: number[][] = [];
  names: string[] = [];
  maxSamples = 12000;
  lastT = -Infinity;

  reset(sim: Simulation): void {
    this.names = [...sim.sys.coords.map((c) => c.name), 'KE', 'PE', 'E', 'W'];
    this.t = [];
    this.ch = this.names.map(() => []);
    this.lastT = -Infinity;
    this.sample(sim, true);
  }

  sample(sim: Simulation, force = false): void {
    if (!force && sim.t - this.lastT < 1 / 240) return;
    this.lastT = sim.t;
    this.t.push(sim.t);
    const n = sim.sys.n;
    for (let k = 0; k < n; k++) this.ch[k].push(sim.q[k]);
    const e = sim.energy();
    this.ch[n].push(e.kinetic);
    this.ch[n + 1].push(e.potential);
    this.ch[n + 2].push(e.total);
    this.ch[n + 3].push(sim.dissipated);
    if (this.t.length > this.maxSamples) {
      // decimate the older half to keep memory bounded
      const keep = (arr: number[]) => arr.filter((_, i) => i % 2 === 0 || i > arr.length / 2);
      this.t = keep(this.t);
      this.ch = this.ch.map(keep);
    }
  }
}

export interface ModeAnim {
  lin: Linearization;
  index: number;
  start: number;
  amp: number;
}

type Listener = () => void;

const STORAGE_KEY = 'mechanism-sandbox:scene';

export class App {
  scene: Scene;
  sim!: Simulation;
  /** System compiled from the current scene (for drawing and hit tests). */
  history = new History();
  renderer: Renderer;
  recorder = new Recorder();
  running = false;
  timeScale = 1;
  selection = new Set<string>();
  hover: string | null = null;
  showFBD = false;
  showTrails = false;
  showVelocity = false;
  /** Recent body positions (flat x, y) for motion trails. */
  trails = new Map<string, number[]>();
  private lastTrailT = -Infinity;
  showLabels = true;
  snap = true;
  modeAnim: ModeAnim | null = null;
  displayQ: Float64Array | null = null;
  warnings: string[] = [];

  private sceneListeners: Listener[] = [];
  private selectionListeners: Listener[] = [];
  private tickListeners: Listener[] = [];
  private stateListeners: Listener[] = [];
  private acc = 0;
  private lastFrame = 0;
  /** Measured simulated-seconds per real second while running (null when idle). */
  achievedScale: number | null = null;
  private rateWin = { real: 0, sim: 0 };
  private needsDraw = true;
  editorOverlay: (() => Partial<import('./render').DrawOpts>) | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new Renderer(canvas);
    this.scene = this.loadStored() ?? emptyScene();
    this.rebuild();
  }

  // --------------------------------------------------------------- events

  onSceneChange(f: Listener): void { this.sceneListeners.push(f); }
  onSelection(f: Listener): void { this.selectionListeners.push(f); }
  onTick(f: Listener): void { this.tickListeners.push(f); }
  /** Play/pause/reset state changes. */
  onState(f: Listener): void { this.stateListeners.push(f); }

  private emit(list: Listener[]): void {
    for (const f of list) f();
    this.needsDraw = true;
  }

  redraw(): void { this.needsDraw = true; }

  // ---------------------------------------------------------------- scene

  get sys(): System { return this.sim.sys; }

  get editing(): boolean { return !this.running && this.sim.t === 0; }

  /**
   * Re-assemble the scene's initial configuration (projecting onto the
   * constraints, softly holding `lock`), write it back, and reset the
   * simulation to t = 0.
   */
  rebuild(lock?: Lock | Lock[]): void {
    const locks = lock ? (Array.isArray(lock) ? lock : [lock]) : [];
    const sys = new System(this.scene);
    const { q, qd } = sys.initialState();
    const lockSet = new Set<number>();
    for (const l of locks) {
      const b = sys.byId.get(l.id);
      if (!b) continue;
      for (const k of l.keys) {
        if (k === 'x') lockSet.add(b.idx);
        if (k === 'y') lockSet.add(b.idx + 1);
        if (k === 'theta' && b.dof === 3) lockSet.add(b.idx + 2);
      }
    }
    const res = sys.assemble(q, qd, lockSet);
    sys.writeBack(q, qd);
    this.sim = new Simulation(this.scene, { assemble: false });
    this.warnings = [...sys.warnings];
    if (res > 1e-6) this.warnings.push(`Constraints cannot all be satisfied (residual ${res.toExponential(2)} m). Check rod/rope lengths.`);
    this.running = false;
    this.modeAnim = null;
    this.displayQ = null;
    this.recorder.reset(this.sim);
    this.clearTrails();
    this.persist();
    for (const id of [...this.selection]) if (!this.scene.elements.some((e) => e.id === id)) this.selection.delete(id);
    this.emit(this.sceneListeners);
    this.emit(this.stateListeners);
  }

  /** Apply an edit with undo support and re-assemble. */
  commit(mutate: (s: Scene) => void, opts: { lock?: Lock | Lock[]; history?: boolean } = {}): void {
    if (opts.history !== false) this.history.push(this.scene);
    mutate(this.scene);
    this.rebuild(opts.lock);
  }

  setScene(scene: Scene, pushHistory = true): void {
    if (pushHistory) this.history.push(this.scene);
    this.scene = scene;
    this.selection.clear();
    this.rebuild();
    this.emit(this.selectionListeners);
  }

  undo(): void {
    const s = this.history.undo(this.scene);
    if (s) { this.scene = s; this.rebuild(); this.emit(this.selectionListeners); }
  }

  redo(): void {
    const s = this.history.redo(this.scene);
    if (s) { this.scene = s; this.rebuild(); this.emit(this.selectionListeners); }
  }

  select(ids: string[]): void {
    this.selection = new Set(ids);
    this.emit(this.selectionListeners);
  }

  /** Delete elements plus anything that depends on them. */
  deleteElements(ids: Set<string>): void {
    if (!ids.size) return;
    this.commit((s) => {
      const gone = new Set(ids);
      let changed = true;
      while (changed) {
        changed = false;
        for (const e of s.elements) {
          if (gone.has(e.id)) continue;
          const refs: (string | null)[] = [];
          if ('a' in e) refs.push(e.a.body);
          if ('b' in e) refs.push(e.b.body);
          if (e.kind === 'tspring') refs.push(e.pin);
          if (e.kind === 'rope') refs.push(...e.route.map((r) => r.pulley));
          if (refs.some((r) => r !== null && gone.has(r))) { gone.add(e.id); changed = true; }
        }
      }
      s.elements = s.elements.filter((e) => !gone.has(e.id));
    });
    this.selection.clear();
    this.emit(this.selectionListeners);
  }

  // ------------------------------------------------------------ simulation

  play(): void {
    if (this.modeAnim) this.stopModeAnim();
    this.running = true;
    this.lastFrame = performance.now();
    this.acc = 0;
    this.achievedScale = null;
    this.rateWin = { real: 0, sim: 0 };
    this.emit(this.stateListeners);
  }

  pause(): void {
    this.running = false;
    this.emit(this.stateListeners);
  }

  toggle(): void { this.running ? this.pause() : this.play(); }

  clearTrails(): void {
    this.trails.clear();
    this.lastTrailT = -Infinity;
  }

  private recordTrails(): void {
    if (this.sim.t - this.lastTrailT < 1 / 60) return;
    this.lastTrailT = this.sim.t;
    for (const b of this.sys.bodies) {
      let t = this.trails.get(b.id);
      if (!t) this.trails.set(b.id, (t = []));
      t.push(this.sim.q[b.idx], this.sim.q[b.idx + 1]);
      if (t.length > 1200) t.splice(0, t.length - 1200); // last ~10 s
    }
  }

  reset(): void {
    this.clearTrails();
    this.sim = new Simulation(this.scene, { assemble: false });
    this.running = false;
    this.modeAnim = null;
    this.displayQ = null;
    this.recorder.reset(this.sim);
    this.emit(this.stateListeners);
    this.emit(this.tickListeners);
  }

  step(duration = 0.01): void {
    if (this.modeAnim) this.stopModeAnim();
    this.running = false;
    this.sim.run(duration, (s) => { this.recorder.sample(s); this.recordTrails(); });
    this.emit(this.stateListeners);
    this.emit(this.tickListeners);
  }

  /** Start pulling a body point towards the cursor (only while running). */
  grabBody(id: string, p: [number, number]): boolean {
    const b = this.sys.byId.get(id);
    if (!b || !this.running) return false;
    const q = this.sim.q;
    const th = b.dof === 3 ? q[b.idx + 2] : 0;
    const dx = p[0] - q[b.idx], dy = p[1] - q[b.idx + 1];
    const c = Math.cos(th), sn = Math.sin(th);
    this.sys.mouse.a = { b, lx: c * dx + sn * dy, ly: -sn * dx + c * dy, angle: 0 };
    [this.sys.mouse.tx, this.sys.mouse.ty] = p;
    return true;
  }

  moveGrab(p: [number, number]): void {
    [this.sys.mouse.tx, this.sys.mouse.ty] = p;
  }

  releaseGrab(): void {
    this.sys.mouse.a = null;
  }

  get grabbing(): boolean { return this.sys.mouse.a !== null; }

  /** Make sure edits apply to the initial configuration. */
  ensureEditMode(): void {
    if (!this.editing || this.modeAnim) this.reset();
  }

  startModeAnim(lin: Linearization, index: number): void {
    this.running = false;
    let span = 0;
    for (const row of lin.modesQ) span = Math.max(span, Math.abs(row[index]));
    // aim for ~0.12 m or ~0.25 rad of peak motion
    this.modeAnim = { lin, index, start: performance.now(), amp: span > 0 ? 0.12 / span : 0 };
    this.emit(this.stateListeners);
  }

  stopModeAnim(): void {
    this.modeAnim = null;
    this.displayQ = null;
    this.emit(this.stateListeners);
  }

  // ----------------------------------------------------------------- loop

  start(): void {
    const frame = (now: number) => {
      this.tick(now);
      requestAnimationFrame(frame);
    };
    requestAnimationFrame(frame);
  }

  private tick(now: number): void {
    if (this.running) {
      const real = Math.min(0.1, (now - this.lastFrame) / 1000);
      this.acc += real * this.timeScale;
      const dt = this.sim.dt;
      const simBefore = this.sim.t;
      const t0 = performance.now();
      let steps = 0;
      while (this.acc >= dt) {
        this.sim.step();
        this.recorder.sample(this.sim);
        this.acc -= dt;
        steps++;
        // stay interactive if the system is too heavy for real time
        if ((steps & 63) === 0 && performance.now() - t0 > 30) { this.acc = 0; break; }
      }
      this.rateWin.real += real;
      this.rateWin.sim += this.sim.t - simBefore;
      if (this.rateWin.real > 0.5) {
        this.achievedScale = this.rateWin.sim / this.rateWin.real;
        this.rateWin = { real: 0, sim: 0 };
      }
      if (!this.sim.q.every(Number.isFinite)) {
        this.running = false;
        this.warnings = ['Simulation diverged (non-finite state). Reset and check the parameters.'];
        this.emit(this.sceneListeners);
      }
      this.recordTrails();
      this.emit(this.tickListeners);
      this.needsDraw = true;
    }
    this.lastFrame = now;
    if (this.modeAnim) {
      const m = this.modeAnim;
      const w = Math.max(m.lin.omega[m.index], 0);
      // play modes at a watchable rate: one cycle every ~1.6 s regardless of ω
      const tt = (now - m.start) / 1000;
      const phase = w > 0 ? (2 * Math.PI * tt) / 1.6 : 0;
      const q = Float64Array.from(m.lin.q);
      for (let k = 0; k < q.length; k++) q[k] += m.amp * Math.sin(phase) * m.lin.modesQ[k][m.index];
      this.displayQ = q;
      this.needsDraw = true;
    }
    if (this.needsDraw) {
      this.needsDraw = false;
      this.draw();
    }
  }

  draw(): void {
    const q = this.displayQ ?? this.sim.q;
    let forces = null;
    if (this.showFBD && !this.modeAnim) {
      try { forces = this.sys.forceReport(this.sim.q, this.sim.qd); } catch { forces = null; }
    }
    const overlay = this.editorOverlay?.() ?? {};
    this.renderer.draw(this.scene, this.sys, q, {
      selection: this.selection,
      hover: this.hover,
      labels: this.showLabels,
      grid: true,
      forces,
      trails: this.showTrails && !this.modeAnim ? this.trails : null,
      velocities: this.showVelocity && !this.modeAnim ? this.sim.qd : null,
      grab: this.grabbing ? { from: ptPos(this.sys.mouse.a!, this.sim.q), to: [this.sys.mouse.tx, this.sys.mouse.ty] } : null,
      preview: null,
      cursor: null,
      ...overlay,
    });
  }

  // ------------------------------------------------------------ persistence

  persist(): void {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(this.scene)); } catch { /* storage unavailable */ }
  }

  private loadStored(): Scene | null {
    try {
      const s = localStorage.getItem(STORAGE_KEY);
      return s ? (JSON.parse(s) as Scene) : null;
    } catch {
      return null;
    }
  }

  /** Element lookup helper. */
  el<T = DynBodyEl>(id: string): T | undefined {
    return this.scene.elements.find((e) => e.id === id) as T | undefined;
  }
}
