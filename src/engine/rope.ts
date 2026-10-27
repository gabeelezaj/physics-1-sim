// Geometry of an inextensible rope routed end A → pulleys → end B.
//
// Each straight piece is a common tangent between two "nodes" (an end point is
// a circle of radius 0). With wrap direction σ (+1 CCW) the pulley centre lies
// to the left of the direction of travel for σ = +1, so the tangent point is
// T = C + ρ n with signed radius ρ = −σ r and n = u rotated +90°.
// For centres C₁, C₂ with d = C₂ − C₁, δ = ρ₁ − ρ₂:
//   ℓ = √(|d|² − δ²),  u = (ℓ d − δ R₉₀ d) / |d|²

import { APt, PtState, SBody, ptPos } from './kin';

export interface RopeNodeEnd {
  type: 'end';
  a: APt;
}
export interface RopeNodePulley {
  type: 'pulley';
  b: SBody;
  r: number;
  sigma: 1 | -1;
}
export type RopeNode = RopeNodeEnd | RopeNodePulley;

export interface RopePiece {
  /** Unit direction of travel from node j to node j+1. */
  ux: number;
  uy: number;
  len: number;
  /** Tangent points (world) at the start and end of the piece. */
  t0: [number, number];
  t1: [number, number];
  /** Lever arms of the tangent points from their body origins. */
  r0: [number, number];
  r1: [number, number];
  degenerate: boolean;
}

export interface RopeGeom {
  pieces: RopePiece[];
  /** Wrap angle (rad, in [0, 2π)) on each pulley, in route order. */
  wraps: number[];
  /** Angles of entry/exit tangent points on each pulley (for drawing). */
  arcs: { cx: number; cy: number; r: number; a0: number; a1: number; sigma: 1 | -1 }[];
  total: number;
}

interface NodeState {
  cx: number;
  cy: number;
  rho: number;
  /** For end nodes, the lever of the attachment point. */
  end?: PtState;
}

function nodeState(nd: RopeNode, q: ArrayLike<number>): NodeState {
  if (nd.type === 'end') {
    const p = ptPos(nd.a, q);
    return { cx: p.x, cy: p.y, rho: 0, end: p };
  }
  const i = nd.b.idx;
  return { cx: q[i], cy: q[i + 1], rho: -nd.sigma * nd.r };
}

const TAU = Math.PI * 2;

export function ropeGeometry(nodes: RopeNode[], q: ArrayLike<number>): RopeGeom {
  const st = nodes.map((nd) => nodeState(nd, q));
  const pieces: RopePiece[] = [];
  for (let j = 0; j + 1 < st.length; j++) {
    const A = st[j], B = st[j + 1];
    const dx = B.cx - A.cx, dy = B.cy - A.cy;
    const D2 = dx * dx + dy * dy;
    const delta = A.rho - B.rho;
    let len2 = D2 - delta * delta;
    let degenerate = false;
    if (len2 <= 1e-18 || D2 < 1e-18) {
      len2 = 0;
      degenerate = true;
    }
    const len = Math.sqrt(len2);
    let ux: number, uy: number;
    if (D2 < 1e-18) {
      ux = 1; uy = 0;
    } else {
      // R90 d = (−dy, dx)
      ux = (len * dx + delta * dy) / D2;
      uy = (len * dy - delta * dx) / D2;
    }
    const nx = -uy, ny = ux;
    const t0: [number, number] = [A.cx + A.rho * nx, A.cy + A.rho * ny];
    const t1: [number, number] = [B.cx + B.rho * nx, B.cy + B.rho * ny];
    const r0: [number, number] = A.end ? [A.end.rx, A.end.ry] : [t0[0] - A.cx, t0[1] - A.cy];
    const r1: [number, number] = B.end ? [B.end.rx, B.end.ry] : [t1[0] - B.cx, t1[1] - B.cy];
    pieces.push({ ux, uy, len, t0, t1, r0, r1, degenerate });
  }
  const wraps: number[] = [];
  const arcs: RopeGeom['arcs'] = [];
  let total = 0;
  for (const p of pieces) total += p.len;
  for (let i = 1; i + 1 < nodes.length; i++) {
    const nd = nodes[i] as RopeNodePulley;
    const s = st[i];
    const tin = pieces[i - 1].t1, tout = pieces[i].t0;
    const a0 = Math.atan2(tin[1] - s.cy, tin[0] - s.cx);
    const a1 = Math.atan2(tout[1] - s.cy, tout[0] - s.cx);
    let w = (nd.sigma * (a1 - a0)) % TAU;
    if (w < 0) w += TAU;
    wraps.push(w);
    arcs.push({ cx: s.cx, cy: s.cy, r: nd.r, a0, a1, sigma: nd.sigma });
    total += nd.r * w;
  }
  return { pieces, wraps, arcs, total };
}

/**
 * Fill the rope's constraint rows at q into `rows` (row-major, nRows × n):
 * row 0 is d/dt(total length) — pulleys contribute only through their
 * centres — and rows 1..np are the no-slip rows u_j·(v(T_{j+1}) − v(T_j)) for
 * pieces j = 0..np−1, with pulley spin entering through the tangent levers.
 */
export function ropeRows(nodes: RopeNode[], q: ArrayLike<number>, n: number, rows: Float64Array): RopeGeom {
  rows.fill(0);
  const g = ropeGeometry(nodes, q);
  const np = nodes.length - 2;
  for (let j = 0; j < g.pieces.length; j++) {
    const pc = g.pieces[j];
    const A = nodes[j], B = nodes[j + 1];
    // total-length row
    addNode(rows, 0, A, pc.r0, pc.ux, pc.uy, -1, false);
    addNode(rows, 0, B, pc.r1, pc.ux, pc.uy, +1, false);
    if (j < np) {
      const off = (1 + j) * n;
      addNode(rows, off, A, pc.r0, pc.ux, pc.uy, -1, true);
      addNode(rows, off, B, pc.r1, pc.ux, pc.uy, +1, true);
    }
  }
  return g;
}

function addNode(
  rows: Float64Array, off: number, nd: RopeNode, r: [number, number],
  ux: number, uy: number, s: number, withSpin: boolean,
): void {
  if (nd.type === 'end') {
    const b = nd.a.b;
    if (!b) return;
    const i = off + b.idx;
    rows[i] += s * ux;
    rows[i + 1] += s * uy;
    if (b.dof === 3) rows[i + 2] += s * (-r[1] * ux + r[0] * uy);
    return;
  }
  const i = off + nd.b.idx;
  rows[i] += s * ux;
  rows[i + 1] += s * uy;
  if (withSpin) rows[i + 2] += s * (-r[1] * ux + r[0] * uy);
}

/** Wrap direction a rope naturally takes around a pulley between two points. */
export function autoWrapDir(prev: [number, number], c: [number, number], next: [number, number]): 1 | -1 {
  const ax = c[0] - prev[0], ay = c[1] - prev[1];
  const bx = next[0] - c[0], by = next[1] - c[1];
  return ax * by - ay * bx > 0 ? 1 : -1;
}
