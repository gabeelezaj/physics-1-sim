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

  /** Energy drift relative to the largest kinetic energy observed so far. */
  driftPercent(): number {
    const e = this.energy();
    const ref = Math.max(this.keMax, 1e-12);
    return (100 * (e.total - this.E0)) / ref;
  }

  step(h = this.dt): void {
    const { sys, q, qd } = this;
    const n = sys.n;
    const [k1q, k1v, k2q, k2v, k3q, k3v, k4q, k4v] = this.k;
    const tq = this.tmpQ, tv = this.tmpV;

    k1q.set(qd);
    sys.accel(q, qd, k1v);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + 0.5 * h * k1q[i]; tv[i] = qd[i] + 0.5 * h * k1v[i]; }
    k2q.set(tv);
    sys.accel(tq, tv, k2v);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + 0.5 * h * k2q[i]; tv[i] = qd[i] + 0.5 * h * k2v[i]; }
    k3q.set(tv);
    sys.accel(tq, tv, k3v);

    for (let i = 0; i < n; i++) { tq[i] = q[i] + h * k3q[i]; tv[i] = qd[i] + h * k3v[i]; }
    k4q.set(tv);
    sys.accel(tq, tv, k4v);

    for (let i = 0; i < n; i++) {
      q[i] += (h / 6) * (k1q[i] + 2 * k2q[i] + 2 * k3q[i] + k4q[i]);
      qd[i] += (h / 6) * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]);
    }
    this.t += h;
    const ke = sys.kinetic(qd);
    if (ke > this.keMax) this.keMax = ke;
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
