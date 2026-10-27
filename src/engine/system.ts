// Compiles a Scene into a System in maximal coordinates and evaluates the
// constrained equations of motion
//   M q̈ = F + Jᵀλ,   J q̈ = −J̇q̇ − 2α J q̇ − β² g(q)
// by solving (J M⁻¹ Jᵀ) λ = rhs − J M⁻¹ F.

import { solveFullPivotInPlace } from '../math/linalg';
import type { Attach, DynBodyEl, El, GroundEl, PinEl, Scene } from '../model/types';
import { Constraint, PinC, RodC, RopeC, SliderC } from './constraints';
import { ForceEl, FrictionF, PointForce, SpringF, TSpringF } from './forces';
import { APt, SBody, addPtForce, ptPos } from './kin';
import { RopeNode } from './rope';

export type CoordKind = 'x' | 'y' | 'theta';

export interface Coord {
  body: SBody;
  kind: CoordKind;
  name: string;
  tex: string;
}

export class System {
  n = 0;
  m = 0;
  bodies: SBody[] = [];
  byId = new Map<string, SBody>();
  grounds = new Map<string, GroundEl>();
  coords: Coord[] = [];
  mass!: Float64Array;
  invMass!: Float64Array;
  cons: Constraint[] = [];
  rowStart: number[] = [];
  forces: ForceEl[] = [];
  g: number;
  alpha: number;
  beta: number;
  warnings: string[] = [];
  /** Multipliers from the most recent solve (used for friction and the FBD). */
  lambda!: Float64Array;

  // scratch
  private F!: Float64Array;
  private J!: Float64Array;
  private c!: Float64Array;
  private gam!: Float64Array;
  private A!: Float64Array;
  private rhs!: Float64Array;
  private perm!: Int32Array;

  constructor(public scene: Scene) {
    this.g = scene.gravity.on ? scene.gravity.g : 0;
    this.alpha = scene.settings.alpha;
    this.beta = scene.settings.beta;
    this.compile();
  }

  // ---------------------------------------------------------------- compile

  private compile(): void {
    const els = this.scene.elements;
    for (const e of els) if (e.kind === 'ground') this.grounds.set(e.id, e);
    let idx = 0;
    for (const e of els) {
      if (e.kind !== 'point' && e.kind !== 'rect' && e.kind !== 'pulley') continue;
      const dof = e.kind === 'point' ? 2 : 3;
      const I = e.kind === 'rect' ? (e.I ?? (e.m * (e.w * e.w + e.h * e.h)) / 12)
        : e.kind === 'pulley' ? (e.I ?? 0.5 * e.m * e.r * e.r) : 0;
      const b: SBody = { id: e.id, name: e.name, kind: e.kind, m: e.m, I, idx, dof };
      this.bodies.push(b);
      this.byId.set(e.id, b);
      const t = (s: string) => `${s}_{\\mathrm{${e.name.replace(/_/g, '\\_')}}}`;
      this.coords.push({ body: b, kind: 'x', name: `x_${e.name}`, tex: t('x') });
      this.coords.push({ body: b, kind: 'y', name: `y_${e.name}`, tex: t('y') });
      if (dof === 3) this.coords.push({ body: b, kind: 'theta', name: `θ_${e.name}`, tex: t('\\theta') });
      idx += dof;
    }
    this.n = idx;
    this.mass = new Float64Array(idx);
    for (const b of this.bodies) {
      this.mass[b.idx] = b.m;
      this.mass[b.idx + 1] = b.m;
      if (b.dof === 3) this.mass[b.idx + 2] = b.I;
    }
    this.invMass = this.mass.map((v) => (v > 0 ? 1 / v : 0));
    for (const b of this.bodies) {
      if (!(b.m > 0) || (b.dof === 3 && !(b.I > 0))) this.warnings.push(`${b.name} needs positive mass and inertia`);
    }

    const byEl = new Map(els.map((e) => [e.id, e] as const));
    for (const e of els) {
      try {
        this.compileElement(e, byEl);
      } catch (err) {
        this.warnings.push(`${e.name}: ${(err as Error).message}`);
      }
    }
    let r = 0;
    for (const c of this.cons) {
      this.rowStart.push(r);
      r += c.rows;
    }
    this.m = r;
    this.lambda = new Float64Array(r);
    this.F = new Float64Array(this.n);
    this.J = new Float64Array(r * this.n);
    this.c = new Float64Array(r);
    this.gam = new Float64Array(r);
    this.A = new Float64Array(r * r);
    this.rhs = new Float64Array(r);
    this.perm = new Int32Array(r);
  }

  resolve(a: Attach): APt {
    if (a.body === null) return { b: null, lx: a.p[0], ly: a.p[1], angle: 0 };
    const g = this.grounds.get(a.body);
    if (g) {
      const c = Math.cos(g.theta), s = Math.sin(g.theta);
      return { b: null, lx: g.x + c * a.p[0] - s * a.p[1], ly: g.y + s * a.p[0] + c * a.p[1], angle: g.theta };
    }
    const b = this.byId.get(a.body);
    if (!b) throw new Error('attached to a missing body');
    return b.dof === 2 ? { b, lx: 0, ly: 0, angle: 0 } : { b, lx: a.p[0], ly: a.p[1], angle: 0 };
  }

  private compileElement(e: El, byEl: Map<string, El>): void {
    switch (e.kind) {
      case 'pulley':
        if (e.fixedAxle) {
          const b = this.byId.get(e.id)!;
          this.cons.push(new PinC(`${e.id}:axle`, { b, lx: 0, ly: 0, angle: 0 }, { b: null, lx: e.x, ly: e.y, angle: 0 }, 'axle'));
        }
        break;
      case 'pin':
        this.cons.push(new PinC(e.id, this.resolve(e.a), this.resolve(e.b)));
        break;
      case 'rod':
        this.cons.push(new RodC(e.id, this.resolve(e.a), this.resolve(e.b), e.length));
        break;
      case 'slider': {
        const a = this.resolve(e.a);
        if (!a.b) throw new Error('slider must be attached to a moving body');
        const lock = e.lockRotation && a.b.dof === 3 ? a.b : null;
        const body = byEl.get(a.b.id) as DynBodyEl;
        const theta0 = 'theta' in body ? body.theta : 0;
        const sc = new SliderC(e.id, a, e.origin[0], e.origin[1], e.angle, lock, theta0);
        this.cons.push(sc);
        if (e.mu > 0) {
          const ci = this.cons.length - 1;
          this.forces.push(new FrictionF(e.id, e.name, a, e.angle, e.mu, () => this.rowStart[ci]));
        }
        break;
      }
      case 'rope': {
        const nodes: RopeNode[] = [{ type: 'end', a: this.resolve(e.a) }];
        for (const rt of e.route) {
          const p = byEl.get(rt.pulley);
          const b = this.byId.get(rt.pulley);
          if (!p || p.kind !== 'pulley' || !b) throw new Error('rope routed over a missing pulley');
          nodes.push({ type: 'pulley', b, r: p.r, sigma: rt.dir });
        }
        nodes.push({ type: 'end', a: this.resolve(e.b) });
        this.cons.push(new RopeC(e.id, nodes, e.length));
        break;
      }
      case 'spring':
        this.forces.push(new SpringF(e.id, e.name, this.resolve(e.a), this.resolve(e.b), e.k, e.L0, e.c));
        break;
      case 'tspring': {
        const p = byEl.get(e.pin) as PinEl | undefined;
        if (!p || p.kind !== 'pin') throw new Error('torsional spring needs a pin joint');
        this.forces.push(new TSpringF(e.id, e.name, this.resolve(p.a), this.resolve(p.b), e.k, e.theta0, e.c));
        break;
      }
      default:
        break;
    }
  }

  // ----------------------------------------------------------------- state

  initialState(): { q: Float64Array; qd: Float64Array } {
    const q = new Float64Array(this.n), qd = new Float64Array(this.n);
    for (const e of this.scene.elements) {
      const b = this.byId.get(e.id);
      if (!b) continue;
      const d = e as DynBodyEl;
      q[b.idx] = d.x; q[b.idx + 1] = d.y;
      qd[b.idx] = d.vx; qd[b.idx + 1] = d.vy;
      if (b.dof === 3 && d.kind !== 'point') {
        q[b.idx + 2] = d.theta;
        qd[b.idx + 2] = d.omega;
      }
    }
    return { q, qd };
  }

  /** Copy a state into the scene's initial-condition fields. */
  writeBack(q: ArrayLike<number>, qd: ArrayLike<number>, positionsOnly = false): void {
    for (const e of this.scene.elements) {
      const b = this.byId.get(e.id);
      if (!b) continue;
      const d = e as DynBodyEl;
      d.x = q[b.idx]; d.y = q[b.idx + 1];
      if (!positionsOnly) { d.vx = qd[b.idx]; d.vy = qd[b.idx + 1]; }
      if (b.dof === 3 && d.kind !== 'point') {
        d.theta = q[b.idx + 2];
        if (!positionsOnly) d.omega = qd[b.idx + 2];
      }
    }
  }

  // --------------------------------------------------------------- dynamics

  /** Generalized applied force (gravity + force elements). */
  appliedForce(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array, withFriction = true): void {
    F.fill(0);
    if (this.g !== 0) for (const b of this.bodies) F[b.idx + 1] -= b.m * this.g;
    for (const f of this.forces) {
      if (!withFriction && f instanceof FrictionF) continue;
      f.apply(q, qd, F, this.lambda);
    }
  }

  /** Evaluate all constraint rows; returns the internal buffers (valid until the next call). */
  evalConstraints(q: ArrayLike<number>, qd: ArrayLike<number>, wantGamma = true) {
    const { J, c, gam, n } = this;
    J.fill(0);
    c.fill(0);
    gam.fill(0);
    for (let i = 0; i < this.cons.length; i++) {
      this.cons[i].eval(q, qd, n, this.rowStart[i], J, c, gam, wantGamma);
    }
    return { J, c, gam };
  }

  /** Rows that carry a position-level error (everything except no-slip rows). */
  holoMask(): boolean[] {
    const mask: boolean[] = [];
    for (const c of this.cons) mask.push(...c.holo);
    return mask;
  }

  /**
   * q̈ from (q, q̇). Also stores λ in this.lambda. With `stab = false` the
   * Baumgarte terms are omitted (used to report the "physical" multipliers).
   */
  accel(q: ArrayLike<number>, qd: ArrayLike<number>, qdd: Float64Array, stab = true): void {
    const { n, m, F, invMass } = this;
    this.appliedForce(q, qd, F);
    if (m === 0) {
      for (let k = 0; k < n; k++) qdd[k] = F[k] * invMass[k];
      return;
    }
    const { J, c, gam } = this.evalConstraints(q, qd, true);
    const { A, rhs } = this;
    const a2 = stab ? 2 * this.alpha : 0, b2 = stab ? this.beta * this.beta : 0;
    for (let i = 0; i < m; i++) {
      const oi = i * n;
      let jqd = 0, jwf = 0;
      for (let k = 0; k < n; k++) {
        const v = J[oi + k];
        if (v === 0) continue;
        jqd += v * qd[k];
        jwf += v * invMass[k] * F[k];
      }
      rhs[i] = gam[i] - a2 * jqd - b2 * c[i] - jwf;
      for (let j = 0; j <= i; j++) {
        const oj = j * n;
        let s = 0;
        for (let k = 0; k < n; k++) {
          const v = J[oi + k];
          if (v !== 0) s += v * invMass[k] * J[oj + k];
        }
        A[i * m + j] = s;
        A[j * m + i] = s;
      }
    }
    solveFullPivotInPlace(A, rhs, m, this.lambda, 1e-11, this.perm);
    const lam = this.lambda;
    for (let k = 0; k < n; k++) qdd[k] = F[k];
    for (let i = 0; i < m; i++) {
      const l = lam[i];
      if (l === 0) continue;
      const oi = i * n;
      for (let k = 0; k < n; k++) qdd[k] += J[oi + k] * l;
    }
    for (let k = 0; k < n; k++) qdd[k] *= invMass[k];
  }

  kinetic(qd: ArrayLike<number>): number {
    let s = 0;
    for (let k = 0; k < this.n; k++) s += this.mass[k] * qd[k] * qd[k];
    return 0.5 * s;
  }

  potential(q: ArrayLike<number>): number {
    let v = 0;
    if (this.g !== 0) for (const b of this.bodies) v += b.m * this.g * q[b.idx + 1];
    for (const f of this.forces) v += f.energy(q);
    return v;
  }

  // --------------------------------------------------------------- assembly

  /**
   * Project (q, q̇) onto the constraint manifold with a mass-weighted
   * minimum-norm Gauss–Newton. Locked DOFs move only as a last resort. Velocities whose
   * initial value is non-zero are strongly preferred to stay as given.
   * Returns the final position residual.
   */
  assemble(q: Float64Array, qd: Float64Array, lockPos: Set<number> = new Set(), lockVel?: Set<number>): number {
    const { n, m } = this;
    if (m === 0) return 0;
    const holo = this.holoMask();
    const W = new Float64Array(n);
    // soft locks: a locked DOF only moves if nothing else can satisfy g(q) = 0
    for (let k = 0; k < n; k++) W[k] = this.invMass[k] * (lockPos.has(k) ? 1e-6 : 1);
    const zero = new Float64Array(n);
    const A = new Float64Array(m * m), rhs = new Float64Array(m), mu = new Float64Array(m);
    const resid = () => {
      const { c } = this.evalConstraints(q, zero, false);
      let r = 0;
      for (let i = 0; i < m; i++) if (holo[i]) r = Math.max(r, Math.abs(c[i]));
      return r;
    };
    let res = resid();
    for (let it = 0; it < 60 && res > 1e-12; it++) {
      const { J, c } = this.evalConstraints(q, zero, false);
      this.buildA(J, W, A, holo);
      for (let i = 0; i < m; i++) rhs[i] = holo[i] ? c[i] : 0;
      solveFullPivotInPlace(A, rhs, m, mu, 1e-12);
      const dq = new Float64Array(n);
      for (let i = 0; i < m; i++) {
        if (!holo[i] || mu[i] === 0) continue;
        for (let k = 0; k < n; k++) dq[k] -= W[k] * J[i * n + k] * mu[i];
      }
      const q0 = Float64Array.from(q);
      let step = 1, newRes = Infinity;
      for (let ls = 0; ls < 8; ls++) {
        for (let k = 0; k < n; k++) q[k] = q0[k] + step * dq[k];
        newRes = resid();
        if (newRes < res) break;
        step *= 0.5;
      }
      if (!(newRes < res)) { q.set(q0); break; }
      res = newRes;
    }
    // velocity projection (all rows, including no-slip rows)
    const Wv = new Float64Array(n);
    for (let k = 0; k < n; k++) {
      const pinned = lockVel ? lockVel.has(k) : qd[k] !== 0;
      Wv[k] = this.invMass[k] * (pinned ? 1e-8 : 1);
    }
    const all = holo.map(() => true);
    const { J } = this.evalConstraints(q, zero, false);
    this.buildA(J, Wv, A, all);
    for (let i = 0; i < m; i++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += J[i * n + k] * qd[k];
      rhs[i] = s;
    }
    solveFullPivotInPlace(A, rhs, m, mu, 1e-14);
    for (let i = 0; i < m; i++) {
      if (mu[i] === 0) continue;
      for (let k = 0; k < n; k++) qd[k] -= Wv[k] * J[i * n + k] * mu[i];
    }
    return res;
  }

  private buildA(J: Float64Array, W: Float64Array, A: Float64Array, use: boolean[]): void {
    const { n, m } = this;
    for (let i = 0; i < m; i++) {
      for (let j = 0; j <= i; j++) {
        let s = 0;
        if (use[i] && use[j]) for (let k = 0; k < n; k++) s += J[i * n + k] * W[k] * J[j * n + k];
        A[i * m + j] = s;
        A[j * m + i] = s;
      }
    }
  }

  // ------------------------------------------------------------- reporting

  /**
   * World-space forces on each body for the free body diagram: gravity,
   * force elements, and constraint reactions recovered from λ.
   */
  forceReport(q: ArrayLike<number>, qd: ArrayLike<number>): PointForce[] {
    const out: PointForce[] = [];
    const tmp = new Float64Array(this.n);
    this.accel(q, qd, tmp, false);
    const lam = this.lambda;
    for (const b of this.bodies) {
      if (this.g !== 0) out.push({ body: b, x: q[b.idx], y: q[b.idx + 1], fx: 0, fy: -b.m * this.g, label: `${b.name} g`, kind: 'gravity' });
    }
    for (const f of this.forces) f.report(q, qd, lam, out);
    this.cons.forEach((con, ci) => {
      const r0 = this.rowStart[ci];
      if (con instanceof PinC) {
        const pa = ptPos(con.a, q), pb = ptPos(con.b, q);
        const fx = lam[r0], fy = lam[r0 + 1];
        if (con.a.b) out.push({ body: con.a.b, x: pa.x, y: pa.y, fx, fy, label: 'R', kind: 'constraint' });
        if (con.b.b) out.push({ body: con.b.b, x: pb.x, y: pb.y, fx: -fx, fy: -fy, label: 'R', kind: 'constraint' });
      } else if (con instanceof RodC) {
        const pa = ptPos(con.a, q), pb = ptPos(con.b, q);
        const dx = pb.x - pa.x, dy = pb.y - pa.y, d = Math.hypot(dx, dy) || 1;
        const l = lam[r0];
        if (con.b.b) out.push({ body: con.b.b, x: pb.x, y: pb.y, fx: (l * dx) / d, fy: (l * dy) / d, label: 'rod', kind: 'constraint' });
        if (con.a.b) out.push({ body: con.a.b, x: pa.x, y: pa.y, fx: (-l * dx) / d, fy: (-l * dy) / d, label: 'rod', kind: 'constraint' });
      } else if (con instanceof SliderC) {
        const p = ptPos(con.a, q);
        const l = lam[r0];
        if (con.a.b) out.push({ body: con.a.b, x: p.x, y: p.y, fx: l * con.nx, fy: l * con.ny, label: 'N', kind: 'constraint' });
        if (con.lockBody && Math.abs(lam[r0 + 1]) > 1e-12) {
          out.push({ body: con.lockBody, x: q[con.lockBody.idx], y: q[con.lockBody.idx + 1], fx: 0, fy: 0, torque: lam[r0 + 1], label: 'M', kind: 'constraint' });
        }
      } else if (con instanceof RopeC) {
        const g = con.geometry(q);
        const tensions = this.ropeTensions(ci);
        g.pieces.forEach((pc, j) => {
          const T = tensions[j];
          const A = con.nodes[j], B = con.nodes[j + 1];
          const ba = A.type === 'end' ? A.a.b : A.b;
          const bb = B.type === 'end' ? B.a.b : B.b;
          // tension pulls each end toward the other
          if (ba) out.push({ body: ba, x: pc.t0[0], y: pc.t0[1], fx: T * pc.ux, fy: T * pc.uy, label: 'T', kind: 'constraint' });
          if (bb) out.push({ body: bb, x: pc.t1[0], y: pc.t1[1], fx: -T * pc.ux, fy: -T * pc.uy, label: 'T', kind: 'constraint' });
        });
      }
    });
    return out;
  }

  /**
   * Tension in each straight piece of a rope from the current λ. The total
   * length row equals the sum of all piece rows, so piece j carries
   * −(λ_L + λ_j) for j < np and −λ_L for the last piece.
   */
  ropeTensions(ci: number): number[] {
    const con = this.cons[ci];
    const r0 = this.rowStart[ci];
    const np = con.rows - 1;
    const out: number[] = [];
    for (let j = 0; j <= np; j++) out.push(-(this.lambda[r0] + (j < np ? this.lambda[r0 + 1 + j] : 0)));
    return out;
  }

  /** Add a world-space force at a body point (used by UI drag forces, tests). */
  static addForce(F: Float64Array, b: SBody, rx: number, ry: number, fx: number, fy: number): void {
    addPtForce(F, b, rx, ry, fx, fy);
  }
}
