// Applied (non-constraint) forces. Each force element adds its generalized
// force into F and reports its potential energy and the world-space forces it
// exerts (for the free body diagram).

import { APt, addPtForce, angleOf, omegaOf, ptPos, ptVel } from './kin';
import type { SBody } from './kin';

export interface PointForce {
  body: SBody;
  /** Application point (world). */
  x: number;
  y: number;
  fx: number;
  fy: number;
  /** Pure torque (for torsional springs); fx = fy = 0 then. */
  torque?: number;
  label: string;
  kind: 'gravity' | 'spring' | 'tspring' | 'friction' | 'constraint' | 'contact' | 'applied';
}

export interface ForceEl {
  el: string;
  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array, lambda: ArrayLike<number>): void;
  energy(q: ArrayLike<number>): number;
  report(q: ArrayLike<number>, qd: ArrayLike<number>, lambda: ArrayLike<number>, out: PointForce[]): void;
  /** Power of the non-conservative part of the force (≤ 0 when dissipating). */
  power?(q: ArrayLike<number>, qd: ArrayLike<number>, lambda: ArrayLike<number>): number;
}

/** Linear spring-damper between two points: tension f = k(L − L0) + c L̇. */
export class SpringF implements ForceEl {
  constructor(
    public el: string, public name: string, public a: APt, public b: APt,
    public k: number, public L0: number, public c: number,
  ) {}
  private tension(q: ArrayLike<number>, qd: ArrayLike<number>) {
    const pa = ptPos(this.a, q), pb = ptPos(this.b, q);
    const dx = pb.x - pa.x, dy = pb.y - pa.y;
    const L = Math.hypot(dx, dy);
    if (L < 1e-12) return null;
    const ux = dx / L, uy = dy / L;
    let f = this.k * (L - this.L0);
    if (this.c !== 0) {
      const va = ptVel(this.a, pa, qd), vb = ptVel(this.b, pb, qd);
      f += this.c * (ux * (vb[0] - va[0]) + uy * (vb[1] - va[1]));
    }
    return { pa, pb, ux, uy, f, L };
  }
  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    const t = this.tension(q, qd);
    if (!t) return;
    addPtForce(F, this.a.b, t.pa.rx, t.pa.ry, t.f * t.ux, t.f * t.uy);
    addPtForce(F, this.b.b, t.pb.rx, t.pb.ry, -t.f * t.ux, -t.f * t.uy);
  }
  power(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    if (this.c === 0) return 0;
    const pa = ptPos(this.a, q), pb = ptPos(this.b, q);
    const dx = pb.x - pa.x, dy = pb.y - pa.y;
    const L = Math.hypot(dx, dy);
    if (L < 1e-12) return 0;
    const va = ptVel(this.a, pa, qd), vb = ptVel(this.b, pb, qd);
    const Ldot = (dx * (vb[0] - va[0]) + dy * (vb[1] - va[1])) / L;
    return -this.c * Ldot * Ldot;
  }
  energy(q: ArrayLike<number>): number {
    const pa = ptPos(this.a, q), pb = ptPos(this.b, q);
    const e = Math.hypot(pb.x - pa.x, pb.y - pa.y) - this.L0;
    return 0.5 * this.k * e * e;
  }
  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    const t = this.tension(q, qd);
    if (!t) return;
    if (this.a.b) out.push({ body: this.a.b, x: t.pa.x, y: t.pa.y, fx: t.f * t.ux, fy: t.f * t.uy, label: this.name, kind: 'spring' });
    if (this.b.b) out.push({ body: this.b.b, x: t.pb.x, y: t.pb.y, fx: -t.f * t.ux, fy: -t.f * t.uy, label: this.name, kind: 'spring' });
  }
}

/** Torsional spring-damper on the relative angle Δ = θ_a − θ_b − θ0. */
export class TSpringF implements ForceEl {
  constructor(
    public el: string, public name: string, public a: APt, public b: APt,
    public k: number, public theta0: number, public c: number,
  ) {}
  private torque(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const d = angleOf(this.a, q) - angleOf(this.b, q) - this.theta0;
    const dd = omegaOf(this.a, qd) - omegaOf(this.b, qd);
    return -this.k * d - this.c * dd;
  }
  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    const tau = this.torque(q, qd);
    if (this.a.b && this.a.b.dof === 3) F[this.a.b.idx + 2] += tau;
    if (this.b.b && this.b.b.dof === 3) F[this.b.b.idx + 2] -= tau;
  }
  power(_q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const dd = omegaOf(this.a, qd) - omegaOf(this.b, qd);
    return -this.c * dd * dd;
  }
  energy(q: ArrayLike<number>): number {
    const d = angleOf(this.a, q) - angleOf(this.b, q) - this.theta0;
    return 0.5 * this.k * d * d;
  }
  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    const tau = this.torque(q, qd);
    const pa = ptPos(this.a, q);
    if (this.a.b && this.a.b.dof === 3) out.push({ body: this.a.b, x: pa.x, y: pa.y, fx: 0, fy: 0, torque: tau, label: this.name, kind: 'tspring' });
    if (this.b.b && this.b.b.dof === 3) out.push({ body: this.b.b, x: pa.x, y: pa.y, fx: 0, fy: 0, torque: -tau, label: this.name, kind: 'tspring' });
  }
}

/**
 * Coulomb friction on a slider: F = −μ |N| tanh(v_t / v_ε) t̂, with N the
 * slider's normal multiplier from the most recent solve (lagged one call).
 */
export class FrictionF implements ForceEl {
  tx: number;
  ty: number;
  constructor(public el: string, public name: string, public a: APt, angle: number, public mu: number, public row: () => number) {
    this.tx = Math.cos(angle);
    this.ty = Math.sin(angle);
  }
  private force(q: ArrayLike<number>, qd: ArrayLike<number>, lambda: ArrayLike<number>) {
    const p = ptPos(this.a, q);
    const v = ptVel(this.a, p, qd);
    const vt = v[0] * this.tx + v[1] * this.ty;
    const N = Math.abs(lambda[this.row()] ?? 0);
    const f = -this.mu * N * Math.tanh(vt / 1e-3);
    return { p, fx: f * this.tx, fy: f * this.ty };
  }
  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array, lambda: ArrayLike<number>): void {
    if (this.mu === 0) return;
    const r = this.force(q, qd, lambda);
    addPtForce(F, this.a.b, r.p.rx, r.p.ry, r.fx, r.fy);
  }
  energy(): number {
    return 0;
  }
  power(q: ArrayLike<number>, qd: ArrayLike<number>, lambda: ArrayLike<number>): number {
    if (this.mu === 0) return 0;
    const r = this.force(q, qd, lambda);
    const v = ptVel(this.a, r.p, qd);
    return r.fx * v[0] + r.fy * v[1];
  }
  report(q: ArrayLike<number>, qd: ArrayLike<number>, lambda: ArrayLike<number>, out: PointForce[]): void {
    if (this.mu === 0 || !this.a.b) return;
    const r = this.force(q, qd, lambda);
    out.push({ body: this.a.b, x: r.p.x, y: r.p.y, fx: r.fx, fy: r.fy, label: `${this.name} friction`, kind: 'friction' });
  }
}

/**
 * Constant applied load: a force of fixed world direction at a body point
 * plus a torque. Both are conservative: V = −F·p − τθ.
 */
export class AppliedF implements ForceEl {
  constructor(public el: string, public name: string, public a: APt, public fx: number, public fy: number, public torque: number) {}
  apply(q: ArrayLike<number>, _qd: ArrayLike<number>, F: Float64Array): void {
    const p = ptPos(this.a, q);
    addPtForce(F, this.a.b, p.rx, p.ry, this.fx, this.fy);
    if (this.a.b && this.a.b.dof === 3) F[this.a.b.idx + 2] += this.torque;
  }
  energy(q: ArrayLike<number>): number {
    const p = ptPos(this.a, q);
    return -(this.fx * p.x + this.fy * p.y) - this.torque * angleOf(this.a, q);
  }
  report(q: ArrayLike<number>, _qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    if (!this.a.b) return;
    const p = ptPos(this.a, q);
    if (this.fx || this.fy) out.push({ body: this.a.b, x: p.x, y: p.y, fx: this.fx, fy: this.fy, label: this.name, kind: 'applied' });
    if (this.torque && this.a.b.dof === 3) out.push({ body: this.a.b, x: q[this.a.b.idx], y: q[this.a.b.idx + 1], fx: 0, fy: 0, torque: this.torque, label: this.name, kind: 'applied' });
  }
}
