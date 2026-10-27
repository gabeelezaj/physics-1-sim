// Kinematics of points fixed on bodies, in maximal coordinates.
// A point mass owns (x, y); rigid bodies and pulleys own (x, y, θ).

export interface SBody {
  id: string;
  name: string;
  kind: 'point' | 'rect' | 'pulley';
  m: number;
  I: number;
  /** Index of x in q; y is idx+1 and θ (if dof === 3) is idx+2. */
  idx: number;
  dof: 2 | 3;
}

/**
 * A resolved attachment point. With `b === null` the point is fixed in the
 * world at (lx, ly); ground elements are folded into this case at compile
 * time, `angle` then holds the fixed frame angle (used by torsional springs).
 */
export interface APt {
  b: SBody | null;
  lx: number;
  ly: number;
  angle: number;
}

/** World position plus the rotated lever arm r = R(θ)·local. */
export interface PtState {
  x: number;
  y: number;
  rx: number;
  ry: number;
}

export function angleOf(a: APt, q: ArrayLike<number>): number {
  if (!a.b) return a.angle;
  return a.b.dof === 3 ? q[a.b.idx + 2] : 0;
}

export function omegaOf(a: APt, qd: ArrayLike<number>): number {
  return a.b && a.b.dof === 3 ? qd[a.b.idx + 2] : 0;
}

export function ptPos(a: APt, q: ArrayLike<number>): PtState {
  const b = a.b;
  if (!b) return { x: a.lx, y: a.ly, rx: 0, ry: 0 };
  const i = b.idx;
  if (b.dof === 2) return { x: q[i], y: q[i + 1], rx: 0, ry: 0 };
  const c = Math.cos(q[i + 2]), s = Math.sin(q[i + 2]);
  const rx = c * a.lx - s * a.ly, ry = s * a.lx + c * a.ly;
  return { x: q[i] + rx, y: q[i + 1] + ry, rx, ry };
}

export function ptVel(a: APt, p: PtState, qd: ArrayLike<number>): [number, number] {
  const b = a.b;
  if (!b) return [0, 0];
  const i = b.idx;
  const w = b.dof === 3 ? qd[i + 2] : 0;
  return [qd[i] - w * p.ry, qd[i + 1] + w * p.rx];
}

/** Acceleration of the point when q̈ = 0 (the centripetal part, −ω² r). */
export function ptAcc0(a: APt, p: PtState, qd: ArrayLike<number>): [number, number] {
  const b = a.b;
  if (!b || b.dof === 2) return [0, 0];
  const w = qd[b.idx + 2];
  return [-w * w * p.rx, -w * w * p.ry];
}

/** row[off + ·] += s · (ux, uy)ᵀ ∂p/∂q for a body-fixed point with lever (rx, ry). */
export function addLeverJac(
  row: Float64Array, off: number, b: SBody | null, rx: number, ry: number,
  ux: number, uy: number, s: number,
): void {
  if (!b) return;
  const i = off + b.idx;
  row[i] += s * ux;
  row[i + 1] += s * uy;
  if (b.dof === 3) row[i + 2] += s * (-ry * ux + rx * uy);
}

export function addPtJac(row: Float64Array, off: number, a: APt, p: PtState, ux: number, uy: number, s: number): void {
  addLeverJac(row, off, a.b, p.rx, p.ry, ux, uy, s);
}

/** Accumulate the generalized force of a world force (fx, fy) applied at a point. */
export function addPtForce(F: Float64Array, b: SBody | null, rx: number, ry: number, fx: number, fy: number): void {
  if (!b) return;
  const i = b.idx;
  F[i] += fx;
  F[i + 1] += fy;
  if (b.dof === 3) F[i + 2] += rx * fy - ry * fx;
}
