// Penalty contact between moving bodies and ground pieces (walls, floors,
// ramps), and between pairs of moving bodies.
//
// Shapes are convex polygons (boxes, wedges) or circles (point masses,
// pulleys). A contact is a stiff, one-sided spring-damper along the normal:
//   f = max(0, k δ + c δ̇),   V = ½ k δ²
// with k = m ω_c² (ω_c ≈ 300 rad/s, well inside RK4's stability at 1 ms) and c
// set from the coefficient of restitution e (ζ calibrated numerically for the
// clamped force), faded in over the first millimetre of penetration.
// Polygon pairs test points sampled along each outline (corners plus three
// per edge) against the other polygon, so two flat faces meeting square-on
// register. A corner carries the full k and c; the edge samples 1/6 each, so
// two aligned faces (six edge samples, corners exactly on the boundary) get
// the calibrated values too.

import type { V2 } from '../model/shapes';
import type { ForceEl, PointForce } from './forces';
import { SBody, addPtForce } from './kin';

/**
 * Coefficient of restitution of a one-sided spring-dashpot impact with
 * damping ratio ζ: δ̈ = −max(0, δ + 2ζδ̇), δ(0) = 0, δ̇(0) = 1 (unit ω and
 * speed). The clamp releases the body before δ returns to zero, which is
 * why the textbook e = exp(−ζπ/√(1−ζ²)) overestimates the loss.
 */
function clampedRestitution(zeta: number): number {
  let x = 0, v = 1;
  const h = 2e-3;
  const acc = (x: number, v: number) => -Math.max(0, x + 2 * zeta * v);
  for (let i = 0; i < 20000; i++) {
    const k1x = v, k1v = acc(x, v);
    const k2x = v + 0.5 * h * k1v, k2v = acc(x + 0.5 * h * k1x, v + 0.5 * h * k1v);
    const k3x = v + 0.5 * h * k2v, k3v = acc(x + 0.5 * h * k2x, v + 0.5 * h * k2v);
    const k4x = v + h * k3v, k4v = acc(x + h * k3x, v + h * k3v);
    x += (h / 6) * (k1x + 2 * k2x + 2 * k3x + k4x);
    v += (h / 6) * (k1v + 2 * k2v + 2 * k3v + k4v);
    if (v < 0 && (x <= 0 || x + 2 * zeta * v <= 0)) return -v;
  }
  return 0;
}

const zetaCache = new Map<number, number>();

/** Damping ratio that gives restitution e for the clamped contact model (bisection). */
export function zetaFor(e: number): number {
  const key = Math.round(e * 1000) / 1000;
  const hit = zetaCache.get(key);
  if (hit !== undefined) return hit;
  let lo = 0, hi = 3;
  for (let i = 0; i < 40; i++) {
    const mid = 0.5 * (lo + hi);
    if (clampedRestitution(mid) > key) lo = mid; else hi = mid;
  }
  const z = 0.5 * (lo + hi);
  zetaCache.set(key, z);
  return z;
}

/** Contact stiffness/damping for a body (or pair) of effective mass m. */
export function contactGains(m: number, dt: number, restitution: number): { k: number; c: number } {
  const wc = Math.min(300, 0.3 / dt);
  const k = m * wc * wc;
  const e = Math.min(1, Math.max(1e-3, restitution));
  const zeta = e >= 1 ? 0 : zetaFor(e);
  return { k, c: 2 * zeta * Math.sqrt(k * m) };
}

export type BodyShape = { kind: 'poly'; verts: V2[] } | { kind: 'circle'; r: number };

/** One side of a contact: a moving body, or a fixed piece with a constant pose. */
export interface ContactSide {
  b: SBody | null;
  name: string;
  shape: BodyShape;
  /** Pose of a fixed piece (ignored for moving bodies). */
  fixed?: { x: number; y: number; theta: number };
}

interface Prepared extends ContactSide {
  /** Outline sample points (local) with their stiffness weight. */
  samples: [number, number, number][];
  /** Outward unit normals of the polygon edges (local frame). */
  normals: V2[];
  radius: number;
}

interface PoseS {
  x: number;
  y: number;
  c: number;
  s: number;
}

interface Hit {
  /** Contact point (world). */
  px: number;
  py: number;
  /** Unit normal: direction side A is pushed (B is pushed the opposite way). */
  nx: number;
  ny: number;
  depth: number;
  /** Share of k and c carried by this point. */
  wt: number;
}

function prepare(side: ContactSide): Prepared {
  if (side.shape.kind === 'circle') return { ...side, samples: [], normals: [], radius: side.shape.r };
  const v = side.shape.verts;
  const samples: [number, number, number][] = [];
  const normals: V2[] = [];
  for (let i = 0; i < v.length; i++) {
    const [ax, ay] = v[i], [bx, by] = v[(i + 1) % v.length];
    samples.push([ax, ay, 1]);
    for (const t of [0.25, 0.5, 0.75]) samples.push([ax + (bx - ax) * t, ay + (by - ay) * t, 1 / 6]);
    const L = Math.hypot(bx - ax, by - ay) || 1;
    normals.push([(by - ay) / L, -(bx - ax) / L]);
  }
  return { ...side, samples, normals, radius: Math.max(...v.map(([x, y]) => Math.hypot(x, y))) };
}

/** Penetration of a world point into a convex polygon: depth and the nearest edge's outward normal (world). */
function inPoly(P: Prepared, pose: PoseS, px: number, py: number): { depth: number; nx: number; ny: number } | null {
  const v = (P.shape as { verts: V2[] }).verts;
  const dx = px - pose.x, dy = py - pose.y;
  const u = pose.c * dx + pose.s * dy, w = -pose.s * dx + pose.c * dy;
  let best = Infinity, bi = 0;
  for (let i = 0; i < v.length; i++) {
    const d = (u - v[i][0]) * P.normals[i][0] + (w - v[i][1]) * P.normals[i][1];
    if (d >= 0) return null;
    if (-d < best) { best = -d; bi = i; }
  }
  const [lx, ly] = P.normals[bi];
  return { depth: best, nx: pose.c * lx - pose.s * ly, ny: pose.s * lx + pose.c * ly };
}

/** Circle against a convex polygon: contact point on the polygon and push direction for the circle. */
function circlePoly(P: Prepared, pose: PoseS, cx: number, cy: number, r: number): Hit | null {
  const v = (P.shape as { verts: V2[] }).verts;
  const dx = cx - pose.x, dy = cy - pose.y;
  const u = pose.c * dx + pose.s * dy, w = -pose.s * dx + pose.c * dy;
  const toW = (lx: number, ly: number): V2 => [pose.x + pose.c * lx - pose.s * ly, pose.y + pose.s * lx + pose.c * ly];
  const rot = (lx: number, ly: number): V2 => [pose.c * lx - pose.s * ly, pose.s * lx + pose.c * ly];
  // inside: leave through the nearest edge
  let inside = true, best = Infinity, bi = 0;
  for (let i = 0; i < v.length; i++) {
    const d = (u - v[i][0]) * P.normals[i][0] + (w - v[i][1]) * P.normals[i][1];
    if (d >= 0) inside = false;
    if (-d < best) { best = -d; bi = i; }
  }
  if (inside) {
    const [nx, ny] = P.normals[bi];
    const [px, py] = toW(u + nx * best, w + ny * best);
    const [wx, wy] = rot(nx, ny);
    return { px, py, nx: wx, ny: wy, depth: r + best, wt: 1 };
  }
  let dmin = Infinity, qx = 0, qy = 0;
  for (let i = 0; i < v.length; i++) {
    const [ax, ay] = v[i], [bx, by] = v[(i + 1) % v.length];
    const ex = bx - ax, ey = by - ay;
    const t = Math.max(0, Math.min(1, ((u - ax) * ex + (w - ay) * ey) / (ex * ex + ey * ey || 1)));
    const px = ax + t * ex, py = ay + t * ey;
    const d = Math.hypot(u - px, w - py);
    if (d < dmin) { dmin = d; qx = px; qy = py; }
  }
  if (dmin >= r || dmin < 1e-12) return null;
  const [px, py] = toW(qx, qy);
  const [nx, ny] = rot((u - qx) / dmin, (w - qy) / dmin);
  return { px, py, nx, ny, depth: r - dmin, wt: 1 };
}

/**
 * Contact between side A (always a moving body) and side B (a moving body or
 * a fixed piece). The force acts equally and oppositely on the two sides at
 * the contact point, so momentum is conserved between moving bodies.
 */
export class ContactF implements ForceEl {
  el: string;
  private A: Prepared;
  private B: Prepared;
  private reach: number;

  constructor(a: ContactSide, b: ContactSide, private k: number, private c: number) {
    this.A = prepare(a);
    this.B = prepare(b);
    this.el = `${a.b?.id}:${b.b?.id ?? b.name}:contact`;
    this.reach = this.A.radius + this.B.radius;
  }

  private pose(P: Prepared, q: ArrayLike<number>): PoseS {
    if (!P.b) {
      const f = P.fixed!;
      return { x: f.x, y: f.y, c: Math.cos(f.theta), s: Math.sin(f.theta) };
    }
    const th = P.b.dof === 3 ? q[P.b.idx + 2] : 0;
    return { x: q[P.b.idx], y: q[P.b.idx + 1], c: Math.cos(th), s: Math.sin(th) };
  }

  hits(q: ArrayLike<number>): Hit[] {
    const { A, B } = this;
    const pa = this.pose(A, q), pb = this.pose(B, q);
    const out: Hit[] = [];
    const ca = A.shape.kind === 'circle', cb = B.shape.kind === 'circle';
    if (ca && cb) {
      const dx = pa.x - pb.x, dy = pa.y - pb.y;
      const d = Math.hypot(dx, dy);
      const ra = A.radius, rb = B.radius;
      if (d < ra + rb) {
        const nx = d > 1e-12 ? dx / d : 0, ny = d > 1e-12 ? dy / d : 1;
        out.push({ px: pb.x + nx * rb, py: pb.y + ny * rb, nx, ny, depth: ra + rb - d, wt: 1 });
      }
      return out;
    }
    // broad phase (fixed pieces can be long: their radius covers the whole outline)
    if (Math.hypot(pa.x - pb.x, pa.y - pb.y) >= this.reach) return out;
    if (ca) {
      const h = circlePoly(B, pb, pa.x, pa.y, A.radius);
      if (h) out.push(h);
      return out;
    }
    if (cb) {
      const h = circlePoly(A, pa, pb.x, pb.y, B.radius);
      if (h) out.push({ ...h, nx: -h.nx, ny: -h.ny });
      return out;
    }
    for (const [lx, ly, wt] of A.samples) {
      const px = pa.x + pa.c * lx - pa.s * ly, py = pa.y + pa.s * lx + pa.c * ly;
      const h = inPoly(B, pb, px, py);
      if (h) out.push({ px, py, nx: h.nx, ny: h.ny, depth: h.depth, wt });
    }
    for (const [lx, ly, wt] of B.samples) {
      const px = pb.x + pb.c * lx - pb.s * ly, py = pb.y + pb.s * lx + pb.c * ly;
      const h = inPoly(A, pa, px, py);
      if (h) out.push({ px, py, nx: -h.nx, ny: -h.ny, depth: h.depth, wt });
    }
    return out;
  }

  /** Penetration rate at a contact (positive = going deeper). */
  private rate(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const v = (b: SBody | null): V2 => {
      if (!b) return [0, 0];
      const w = b.dof === 3 ? qd[b.idx + 2] : 0;
      const rx = h.px - q[b.idx], ry = h.py - q[b.idx + 1];
      return [qd[b.idx] - w * ry, qd[b.idx + 1] + w * rx];
    };
    const va = v(this.A.b), vb = v(this.B.b);
    return -((va[0] - vb[0]) * h.nx + (va[1] - vb[1]) * h.ny);
  }

  private forceOf(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    // damping fades in over the first millimetre so the force is continuous at first touch
    const ramp = Math.min(1, h.depth / 1e-3);
    return h.wt * Math.max(0, this.k * h.depth + ramp * this.c * this.rate(h, q, qd));
  }

  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    const a = this.A.b, b = this.B.b;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      if (a) addPtForce(F, a, h.px - q[a.idx], h.py - q[a.idx + 1], f * h.nx, f * h.ny);
      if (b) addPtForce(F, b, h.px - q[b.idx], h.py - q[b.idx + 1], -f * h.nx, -f * h.ny);
    }
  }

  energy(q: ArrayLike<number>): number {
    let e = 0;
    for (const h of this.hits(q)) e += 0.5 * h.wt * this.k * h.depth * h.depth;
    return e;
  }

  /** Power of everything except the conservative ½kδ² spring. */
  power(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let p = 0;
    for (const h of this.hits(q)) p -= (this.forceOf(h, q, qd) - h.wt * this.k * h.depth) * this.rate(h, q, qd);
    return p;
  }

  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    const a = this.A.b, b = this.B.b;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      if (a) out.push({ body: a, x: h.px, y: h.py, fx: f * h.nx, fy: f * h.ny, label: `${this.B.name} contact`, kind: 'contact' });
      if (b) out.push({ body: b, x: h.px, y: h.py, fx: -f * h.nx, fy: -f * h.ny, label: `${this.A.name} contact`, kind: 'contact' });
    }
  }

  /** Deepest penetration at q (for the initial-overlap check). */
  maxDepth(q: ArrayLike<number>): number {
    return this.hits(q).reduce((m, h) => Math.max(m, h.depth), 0);
  }
}
