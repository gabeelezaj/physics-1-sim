// Constraint rows. Each constraint writes, for its rows r0..r0+rows−1:
//   J[r, :]  the Jacobian row (or velocity-constraint row),
//   c[r]     the position error g(q) (0 for velocity-only rows),
//   gam[r]   −J̇ q̇.
// Sign convention for multipliers: the constraint force is Jᵀλ.

import { APt, addPtJac, ptAcc0, ptPos, ptVel } from './kin';
import { RopeGeom, RopeNode, RopeNodeEnd, RopeNodePulley, ropeGeometry, ropeRows } from './rope';
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
 * Rope over pulleys: row 0 holds total length (holonomic), and each pulley in
 * the route owns one no-slip row (velocity level). −J̇q̇ is a central
 * difference of J(q)q̇.
 *
 * Pulleys engage and disengage as the rope moves: a pulley whose wrap angle
 * passes through zero lets go of the rope (instead of the wrap jumping to 2π
 * and knotting the rope around it), and a free pulley catches the rope again
 * when a straight run cuts into it. Inactive pulleys keep their row, zeroed.
 */
export class RopeC implements Constraint {
  kind = 'rope' as const;
  rows: number;
  holo: boolean[];
  labels: string[];
  readonly ends: [RopeNodeEnd, RopeNodeEnd];
  readonly route: RopeNodePulley[];
  /** Whether each route pulley currently carries the rope. */
  active: boolean[];
  /** Continuously tracked wrap angle of each active pulley (rad). */
  private phiRef: number[];
  private activeNodes: RopeNode[] = [];
  private activeIdx: number[] = [];
  private buf: Float64Array = new Float64Array(0);
  private bufP: Float64Array = new Float64Array(0);
  private bufM: Float64Array = new Float64Array(0);
  private qTmp: Float64Array = new Float64Array(0);

  constructor(public el: string, nodes: RopeNode[], public L: number) {
    this.ends = [nodes[0] as RopeNodeEnd, nodes[nodes.length - 1] as RopeNodeEnd];
    // own copies: wrap directions can change when a pulley re-catches the rope
    this.route = nodes.slice(1, -1).map((nd) => ({ ...(nd as RopeNodePulley) }));
    const np = this.route.length;
    this.rows = 1 + np;
    this.holo = [true, ...Array(np).fill(false)];
    this.labels = ['L', ...Array.from({ length: np }, (_, j) => `ns${j + 1}`)];
    this.active = this.route.map(() => true);
    this.phiRef = this.route.map(() => 0);
    this.rebuild();
  }

  /** Nodes of the current path: end A, engaged pulleys, end B. */
  get nodes(): RopeNode[] { return this.activeNodes; }

  /** Route indices of the engaged pulleys, in path order. */
  get activeRoute(): readonly number[] { return this.activeIdx; }

  private rebuild(): void {
    this.activeIdx = this.route.map((_, i) => i).filter((i) => this.active[i]);
    this.activeNodes = [this.ends[0], ...this.activeIdx.map((i) => this.route[i]), this.ends[1]];
  }

  private unwrap(raw: number, i: number): number {
    const ref = this.phiRef[i];
    let d = (raw - ref) % (2 * Math.PI);
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d < -Math.PI) d += 2 * Math.PI;
    return ref + d;
  }

  /** Path geometry with wraps tracked continuously (so a wrap can dip below 0). */
  geometry(q: ArrayLike<number>): RopeGeom {
    const g = ropeGeometry(this.activeNodes, q);
    let total = 0;
    for (const p of g.pieces) total += p.len;
    g.wraps = g.wraps.map((w, j) => this.unwrap(w, this.activeIdx[j]));
    g.wraps.forEach((w, j) => (total += this.route[this.activeIdx[j]].r * w));
    g.total = total;
    return g;
  }

  /** Initial engagement: a routed pulley the rope would wrap almost fully around is left free. */
  init(q: ArrayLike<number>): void {
    for (let pass = 0; pass < this.route.length + 1; pass++) {
      const g = ropeGeometry(this.activeNodes, q);
      let changed = false;
      g.wraps.forEach((w, j) => {
        const i = this.activeIdx[j];
        if (w > 1.5 * Math.PI) { this.active[i] = false; changed = true; } else this.phiRef[i] = w;
      });
      if (!changed) return;
      this.rebuild();
    }
  }

  /**
   * After each accepted step: release pulleys whose wrap went negative and
   * engage free pulleys the rope now cuts into. Returns true if any pulley
   * caught the rope (the caller then applies the no-slip impulse).
   */
  update(q: ArrayLike<number>): boolean {
    const released = new Set<number>();
    const g = ropeGeometry(this.activeNodes, q);
    g.wraps.forEach((w, j) => {
      const i = this.activeIdx[j];
      const phi = this.unwrap(w, i);
      if (phi < 0) released.add(i); else this.phiRef[i] = phi;
    });
    if (released.size) {
      for (const i of released) this.active[i] = false;
      this.rebuild();
    }
    const g2 = released.size ? ropeGeometry(this.activeNodes, q) : g;
    const caught: number[] = [];
    this.route.forEach((nd, i) => {
      if (this.active[i] || released.has(i)) return;
      const k = this.activeIdx.filter((a) => a < i).length;
      const pc = g2.pieces[k];
      const wx = q[nd.b.idx] - pc.t0[0], wy = q[nd.b.idx + 1] - pc.t0[1];
      const along = wx * pc.ux + wy * pc.uy;
      const d = pc.ux * wy - pc.uy * wx; // > 0: centre left of the rope
      if (along > 0 && along < pc.len && Math.abs(d) < nd.r - 1e-9) {
        nd.sigma = d > 0 ? 1 : -1;
        caught.push(i);
      }
    });
    if (!caught.length) return false;
    for (const i of caught) { this.active[i] = true; this.phiRef[i] = NaN; }
    this.rebuild();
    const g3 = ropeGeometry(this.activeNodes, q);
    g3.wraps.forEach((w, j) => {
      const i = this.activeIdx[j];
      if (Number.isNaN(this.phiRef[i])) this.phiRef[i] = w > Math.PI ? w - 2 * Math.PI : w;
    });
    return true;
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

  /** Target row (relative to r0) of local row r of the active path. */
  private rowOf(r: number): number {
    return r === 0 ? 0 : 1 + this.activeIdx[r - 1];
  }

  eval(q: ArrayLike<number>, qd: ArrayLike<number>, n: number, r0: number,
    J: Float64Array, c: Float64Array, gam: Float64Array, wantGamma: boolean): void {
    this.ensure(n);
    ropeRows(this.activeNodes, q, n, this.buf);
    const nr = 1 + this.activeIdx.length;
    for (let r = 0; r < nr; r++) J.set(this.buf.subarray(r * n, (r + 1) * n), (r0 + this.rowOf(r)) * n);
    c[r0] = this.geometry(q).total - this.L;
    for (let r = 1; r < this.rows; r++) { c[r0 + r] = 0; gam[r0 + r] = 0; }
    gam[r0] = 0;
    if (!wantGamma) return;
    let vmax = 0;
    for (let k = 0; k < n; k++) vmax = Math.max(vmax, Math.abs(qd[k]));
    if (vmax < 1e-12) return;
    const h = 1e-6 / vmax;
    const qt = this.qTmp;
    for (let k = 0; k < n; k++) qt[k] = q[k] + h * qd[k];
    ropeRows(this.activeNodes, qt, n, this.bufP);
    for (let k = 0; k < n; k++) qt[k] = q[k] - h * qd[k];
    ropeRows(this.activeNodes, qt, n, this.bufM);
    for (let r = 0; r < nr; r++) {
      let s = 0;
      const o = r * n;
      for (let k = 0; k < n; k++) s += (this.bufP[o + k] - this.bufM[o + k]) * qd[k];
      gam[r0 + this.rowOf(r)] = -s / (2 * h);
    }
  }
}
