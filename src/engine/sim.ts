// Fixed-step RK4 integration of the constrained system.

import type { Scene } from '../model/types';
import { System } from './system';

export class Simulation {
  sys: System;
  q: Float64Array;
  qd: Float64Array;
  t = 0;
  dt: number;
  /** Total energy at t = 0 and the largest kinetic energy seen (drift reference). */
  E0 = 0;
  keMax = 0;
  /** Energy removed so far by damping, friction and impacts (≥ 0 when dissipating). */
  dissipated = 0;
  assemblyResidual = 0;

  private k: Float64Array[];
  private tmpQ: Float64Array;
  private tmpV: Float64Array;
  private acc: Float64Array;

  constructor(scene: Scene, opts: { assemble?: boolean } = {}) {
    this.sys = new System(scene);
    const { q, qd } = this.sys.initialState();
    if (opts.assemble !== false) this.assemblyResidual = this.sys.assemble(q, qd);
    this.q = q;
    this.qd = qd;
    this.dt = scene.settings.dt;
    const n = this.sys.n;
    this.k = Array.from({ length: 8 }, () => new Float64Array(n));
    this.tmpQ = new Float64Array(n);
    this.tmpV = new Float64Array(n);
    this.acc = new Float64Array(n);
    this.E0 = this.energy().total;
    this.keMax = this.energy().kinetic;
    this.sys.accel(this.q, this.qd, this.acc);
  }

  energy(): { kinetic: number; potential: number; total: number } {
    const kinetic = this.sys.kinetic(this.qd);
    const potential = this.sys.potential(this.q);
    return { kinetic, potential, total: kinetic + potential };
  }

  /**
   * Energy-balance error (T + V + dissipated − E₀) as a percentage of a
   * reference energy: the largest kinetic energy so far, but at least 1% of
   * Σ m g · 1 m, so the first instants (when hardly anything moves yet) are
   * not divided by almost nothing. Zero for exact integration, with or
   * without damping.
   */
  driftPercent(): number {
    const e = this.energy();
    return (100 * (e.total + this.dissipated - this.E0)) / this.energyRef();
  }

  energyRef(): number {
    let floor = 0;
    for (const b of this.sys.bodies) floor += b.m * this.sys.g;
    return Math.max(this.keMax, 0.01 * floor, 1e-12);
  }

  /**
   * Advance one step. While anything is in contact the step is split into
   * four RK4 substeps: penalty contacts are stiff and switch on and off, and
   * the finer steps keep impacts (and their energy bookkeeping) accurate.
   */
  step(h = this.dt): void {
    // in contact now, or about to be by the end of the step (look ahead along q̇)
    const ahead = this.tmpQ;
    for (let i = 0; i < this.q.length; i++) ahead[i] = this.q[i] + 1.5 * h * this.qd[i];
    if (this.sys.inContact(this.q) || this.sys.inContact(ahead)) for (let i = 0; i < 4; i++) this.rk4(h / 4);
    else this.rk4(h);
    // ropes leaving/catching pulleys; a catch is a plastic impulse, so book its loss
    const { sys, q, qd } = this;
    const keBefore = sys.kinetic(qd);
    if (sys.postStep(q, qd)) this.dissipated += keBefore - sys.kinetic(qd);
    const ke = sys.kinetic(qd);
    if (ke > this.keMax) this.keMax = ke;
  }

  private rk4(h: number): void {
    const { sys, q, qd } = this;
    const n = sys.n;
    const [k1q, k1v, k2q, k2v, k3q, k3v, k4q, k4v] = this.k;
    const tq = this.tmpQ, tv = this.tmpV;

    k1q.set(qd);
    sys.accel(q, qd, k1v);
    const p1 = sys.dissipativePower(q, qd);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + 0.5 * h * k1q[i]; tv[i] = qd[i] + 0.5 * h * k1v[i]; }
    k2q.set(tv);
    sys.accel(tq, tv, k2v);
    const p2 = sys.dissipativePower(tq, tv);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + 0.5 * h * k2q[i]; tv[i] = qd[i] + 0.5 * h * k2v[i]; }
    k3q.set(tv);
    sys.accel(tq, tv, k3v);
    const p3 = sys.dissipativePower(tq, tv);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + h * k3q[i]; tv[i] = qd[i] + h * k3v[i]; }
    k4q.set(tv);
    sys.accel(tq, tv, k4v);
    const p4 = sys.dissipativePower(tq, tv);
    this.dissipated -= (h / 6) * (p1 + 2 * p2 + 2 * p3 + p4);

    for (let i = 0; i < n; i++) {
      q[i] += (h / 6) * (k1q[i] + 2 * k2q[i] + 2 * k3q[i] + k4q[i]);
      qd[i] += (h / 6) * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]);
    }
    this.t += h;
  }

  /** Advance by `duration` seconds in fixed steps. */
  run(duration: number, onStep?: (s: Simulation) => void): void {
    const steps = Math.round(duration / this.dt);
    for (let i = 0; i < steps; i++) {
      this.step();
      onStep?.(this);
    }
  }

  /** Current acceleration and multipliers (no Baumgarte terms). */
  currentAccel(): Float64Array {
    this.sys.accel(this.q, this.qd, this.acc, false);
    return this.acc;
  }
}
