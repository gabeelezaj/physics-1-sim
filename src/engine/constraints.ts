// Constraint rows. Each constraint writes, for its rows r0..r0+rows−1:
//   J[r, :]  the Jacobian row (or velocity-constraint row),
//   c[r]     the position error g(q) (0 for velocity-only rows),
//   gam[r]   −J̇ q̇.
// Sign convention for multipliers: the constraint force is Jᵀλ.

import { APt, addPtJac, ptAcc0, ptPos, ptVel } from './kin';
import { RopeGeom, RopeNode, ropeGeometry, ropeRows } from './rope';
import type { SBody } from './kin';

export interface Constraint {
  /** Scene element id that produced this constraint (or "<id>:axle"). */
  el: string;
  kind: 'pin' | 'rod' | 'slider' | 'axle' | 'rope';
  rows: number;
  holo: boolean[];
  /** Row labels for the equation display, e.g. ["x", "y"]. */
  labels: string[];
  eval(
    q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean,
  ): void;
}

/** Coincident points: p_a − p_b = 0. Also used for fixed pulley axles. */
export class PinC implements Constraint {
  rows = 2;
  holo = [true, true];
  labels = ['x', 'y'];
  constructor(public el: string, public a: APt, public b: APt, public kind: 'pin' | 'axle' = 'pin') {}
  eval(q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean): void {
    const pa = ptPos(this.a, q), pb = ptPos(this.b, q);
    c[r0] = pa.x - pb.x;
    c[r0 + 1] = pa.y - pb.y;
    const o0 = r0 * n, o1 = (r0 + 1) * n;
    addPtJac(J, o0, this.a, pa, 1, 0, 1);
    addPtJac(J, o0, this.b, pb, 1, 0, -1);
    addPtJac(J, o1, this.a, pa, 0, 1, 1);
    addPtJac(J, o1, this.b, pb, 0, 1, -1);
    if (wantGamma) {
      const aa = ptAcc0(this.a, pa, qd), ab = ptAcc0(this.b, pb, qd);
      gam[r0] = -(aa[0] - ab[0]);
      gam[r0 + 1] = -(aa[1] - ab[1]);
    }
  }
}

/** Massless rigid rod: |p_b − p_a| − L = 0. */
export class RodC implements Constraint {
  kind = 'rod' as const;
  rows = 1;
  holo = [true];
  labels = [''];
  constructor(public el: string, public a: APt, public b: APt, public L: number) {}
  eval(q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean): void {
    const pa = ptPos(this.a, q), pb = ptPos(this.b, q);
    const dx = pb.x - pa.x, dy = pb.y - pa.y;
    const d = Math.hypot(dx, dy) || 1e-12;
    const ux = dx / d, uy = dy / d;
    c[r0] = d - this.L;
    const o = r0 * n;
    addPtJac(J, o, this.b, pb, ux, uy, 1);
    addPtJac(J, o, this.a, pa, ux, uy, -1);
    if (wantGamma) {
      const va = ptVel(this.a, pa, qd), vb = ptVel(this.b, pb, qd);
      const ddx = vb[0] - va[0], ddy = vb[1] - va[1];
      const aa = ptAcc0(this.a, pa, qd), ab = ptAcc0(this.b, pb, qd);
      const along = ux * ddx + uy * ddy;
      const jdot = ux * (ab[0] - aa[0]) + uy * (ab[1] - aa[1]) + (ddx * ddx + ddy * ddy - along * along) / d;
      gam[r0] = -jdot;
    }
  }
}

/**
 * Point on a body kept on a world-fixed line: n·(p − o) = 0, optionally with
 * the body's angle locked (prismatic joint).
 */
export class SliderC implements Constraint {
  kind = 'slider' as const;
  rows: number;
  holo: boolean[];
  labels: string[];
  nx: number;
  ny: number;
  constructor(
    public el: string, public a: APt, public ox: number, public oy: number,
    public angle: number, public lockBody: SBody | null, public theta0: number,
  ) {
    this.nx = -Math.sin(angle);
    this.ny = Math.cos(angle);
    this.rows = lockBody ? 2 : 1;
    this.holo = lockBody ? [true, true] : [true];
    this.labels = lockBody ? ['n', '\\theta'] : ['n'];
  }
  eval(q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean): void {
    const p = ptPos(this.a, q);
    c[r0] = this.nx * (p.x - this.ox) + this.ny * (p.y - this.oy);
    addPtJac(J, r0 * n, this.a, p, this.nx, this.ny, 1);
    if (wantGamma) {
      const a0 = ptAcc0(this.a, p, qd);
      gam[r0] = -(this.nx * a0[0] + this.ny * a0[1]);
    }
    if (this.lockBody) {
      const i = this.lockBody.idx + 2;
      c[r0 + 1] = q[i] - this.theta0;
      J[(r0 + 1) * n + i] = 1;
      if (wantGamma) gam[r0 + 1] = 0;
    }
  }
}

/**
 * Rope over pulleys: row 0 holds total length (holonomic), rows 1..np are
 * no-slip rows (velocity level). −J̇q̇ is a central difference of J(q)q̇.
 */
export class RopeC implements Constraint {
  kind = 'rope' as const;
  rows: number;
  holo: boolean[];
  labels: string[];
  private buf: Float64Array = new Float64Array(0);
  private bufP: Float64Array = new Float64Array(0);
  private bufM: Float64Array = new Float64Array(0);
  private qTmp: Float64Array = new Float64Array(0);
  constructor(public el: string, public nodes: RopeNode[], public L: number) {
    const np = nodes.length - 2;
    this.rows = 1 + np;
    this.holo = [true, ...Array(np).fill(false)];
    this.labels = ['L', ...Array.from({ length: np }, (_, j) => `ns${j + 1}`)];
  }
  geometry(q: ArrayLike<number>): RopeGeom {
    return ropeGeometry(this.nodes, q);
  }
  private ensure(n: number): void {
    const size = this.rows * n;
    if (this.buf.length !== size) {
      this.buf = new Float64Array(size);
      this.bufP = new Float64Array(size);
      this.bufM = new Float64Array(size);
      this.qTmp = new Float64Array(n);
    }
  }
  eval(q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean): void {
    this.ensure(n);
    const g = ropeRows(this.nodes, q, n, this.buf);
    J.set(this.buf, r0 * n);
    c[r0] = g.total - this.L;
    for (let r = 1; r < this.rows; r++) c[r0 + r] = 0;
    if (!wantGamma) return;
    let vmax = 0;
    for (let k = 0; k < n; k++) vmax = Math.max(vmax, Math.abs(qd[k]));
    if (vmax < 1e-12) {
      for (let r = 0; r < this.rows; r++) gam[r0 + r] = 0;
      return;
    }
    const h = 1e-6 / vmax;
    const qt = this.qTmp;
    for (let k = 0; k < n; k++) qt[k] = q[k] + h * qd[k];
    ropeRows(this.nodes, qt, n, this.bufP);
    for (let k = 0; k < n; k++) qt[k] = q[k] - h * qd[k];
    ropeRows(this.nodes, qt, n, this.bufM);
    for (let r = 0; r < this.rows; r++) {
      let s = 0;
      const o = r * n;
      for (let k = 0; k < n; k++) s += (this.bufP[o + k] - this.bufM[o + k]) * qd[k];
      gam[r0 + r] = -s / (2 * h);
    }
  }
}
