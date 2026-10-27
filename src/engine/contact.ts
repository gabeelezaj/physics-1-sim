// Penalty contact between moving bodies and ground pieces (walls, floors),
// and between pairs of moving bodies.
//
// Ground pieces are solid oriented boxes. A contact is a stiff, one-sided
// spring-damper along the contact normal:
//   f = max(0, k δ + c δ̇),   V = ½ k δ²
// with k = m ω_c² (ω_c ≈ 300 rad/s, well inside RK4's stability at 1 ms) and c
// set from the coefficient of restitution e (ζ calibrated numerically for the
// clamped force), faded in over the first millimetre of penetration.
// Tested points: rect corners in the ground box, ground corners in a rect,
// and circles (point masses, pulleys) against the box.

import type { ForceEl, PointForce } from './forces';
import { SBody, addPtForce } from './kin';

export interface GroundBox {
  id: string;
  name: string;
  x: number;
  y: number;
  theta: number;
  hw: number;
  hh: number;
}

export type BodyShape = { kind: 'box'; hw: number; hh: number } | { kind: 'circle'; r: number };

interface Hit {
  /** World application point on the body and its lever from the body origin. */
  px: number;
  py: number;
  rx: number;
  ry: number;
  /** Unit normal: direction the body is pushed. */
  nx: number;
  ny: number;
  depth: number;
}

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

export class ContactF implements ForceEl {
  el: string;
  private gc: number;
  private gs: number;
  constructor(
    public body: SBody, public shape: BodyShape, public g: GroundBox,
    public k: number, public c: number,
  ) {
    this.el = `${body.id}:${g.id}:contact`;
    this.gc = Math.cos(g.theta);
    this.gs = Math.sin(g.theta);
  }

  /** All penetrating contact points at q. */
  hits(q: ArrayLike<number>): Hit[] {
    const b = this.body, g = this.g;
    const bx = q[b.idx], by = q[b.idx + 1];
    const th = b.dof === 3 ? q[b.idx + 2] : 0;
    const out: Hit[] = [];
    const { gc, gs } = this;
    const toG = (x: number, y: number): [number, number] => {
      const dx = x - g.x, dy = y - g.y;
      return [gc * dx + gs * dy, -gs * dx + gc * dy];
    };
    const fromG = (u: number, v: number): [number, number] => [gc * u - gs * v, gs * u + gc * v];
    if (this.shape.kind === 'circle') {
      const r = this.shape.r;
      const [u, v] = toG(bx, by);
      const cu = Math.max(-g.hw, Math.min(g.hw, u)), cv = Math.max(-g.hh, Math.min(g.hh, v));
      const du = u - cu, dv = v - cv;
      const d = Math.hypot(du, dv);
      if (d > 1e-12) {
        if (d < r) {
          const [nx, ny] = fromG(du / d, dv / d);
          out.push({ px: bx, py: by, rx: 0, ry: 0, nx, ny, depth: r - d });
        }
      } else {
        // centre inside the box: push out through the nearest face
        const fu = g.hw - Math.abs(u), fv = g.hh - Math.abs(v);
        const [nx, ny] = fu < fv ? fromG(Math.sign(u) || 1, 0) : fromG(0, Math.sign(v) || 1);
        out.push({ px: bx, py: by, rx: 0, ry: 0, nx, ny, depth: r + Math.min(fu, fv) });
      }
      return out;
    }
    const { hw, hh } = this.shape;
    const c = Math.cos(th), s = Math.sin(th);
    // body corners inside the ground box
    for (const [lx, ly] of [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]) {
      const rx = c * lx - s * ly, ry = s * lx + c * ly;
      const px = bx + rx, py = by + ry;
      const [u, v] = toG(px, py);
      const fu = g.hw - Math.abs(u), fv = g.hh - Math.abs(v);
      if (fu <= 0 || fv <= 0) continue;
      const [nx, ny] = fu < fv ? fromG(Math.sign(u) || 1, 0) : fromG(0, Math.sign(v) || 1);
      out.push({ px, py, rx, ry, nx, ny, depth: Math.min(fu, fv) });
    }
    // ground corners inside the body box
    for (const [gu, gv] of [[-g.hw, -g.hh], [g.hw, -g.hh], [g.hw, g.hh], [-g.hw, g.hh]]) {
      const [wx, wy] = fromG(gu, gv);
      const px = g.x + wx, py = g.y + wy;
      const dx = px - bx, dy = py - by;
      const lx = c * dx + s * dy, ly = -s * dx + c * dy;
      const fu = hw - Math.abs(lx), fv = hh - Math.abs(ly);
      if (fu <= 0 || fv <= 0) continue;
      // the corner leaves through the nearest body face; the body is pushed the other way
      const ou = fu < fv ? Math.sign(lx) || 1 : 0, ov = fu < fv ? 0 : Math.sign(ly) || 1;
      const nx = -(c * ou - s * ov), ny = -(s * ou + c * ov);
      out.push({ px, py, rx: dx, ry: dy, nx, ny, depth: Math.min(fu, fv) });
    }
    return out;
  }

  /** Penetration rate of a contact point (positive = going deeper). */
  private rate(h: Hit, qd: ArrayLike<number>): number {
    const b = this.body;
    const w = b.dof === 3 ? qd[b.idx + 2] : 0;
    const vx = qd[b.idx] - w * h.ry, vy = qd[b.idx + 1] + w * h.rx;
    return -(vx * h.nx + vy * h.ny);
  }

  private forceOf(h: Hit, qd: ArrayLike<number>): number {
    // damping fades in over the first millimetre so the force is continuous at first touch
    const ramp = Math.min(1, h.depth / 1e-3);
    return Math.max(0, this.k * h.depth + ramp * this.c * this.rate(h, qd));
  }

  /** Power of everything except the conservative ½kδ² spring. */
  power(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let p = 0;
    for (const h of this.hits(q)) {
      const r = this.rate(h, qd);
      p -= (this.forceOf(h, qd) - this.k * h.depth) * r;
    }
    return p;
  }

  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, qd);
      if (f > 0) addPtForce(F, this.body, h.rx, h.ry, f * h.nx, f * h.ny);
    }
  }

  energy(q: ArrayLike<number>): number {
    let e = 0;
    for (const h of this.hits(q)) e += 0.5 * this.k * h.depth * h.depth;
    return e;
  }

  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, qd);
      if (f > 0) out.push({ body: this.body, x: h.px, y: h.py, fx: f * h.nx, fy: f * h.ny, label: `${this.g.name} contact`, kind: 'contact' });
    }
  }

  /** Deepest penetration at q (for the initial-overlap check). */
  maxDepth(q: ArrayLike<number>): number {
    return this.hits(q).reduce((m, h) => Math.max(m, h.depth), 0);
  }
}

// ---------------------------------------------------------------- body–body

interface PairHit {
  /** Contact point (world). */
  px: number;
  py: number;
  /** Unit normal: direction body A is pushed (B is pushed the opposite way). */
  nx: number;
  ny: number;
  depth: number;
}

interface PoseS {
  x: number;
  y: number;
  c: number;
  s: number;
}

/**
 * Points sampled on a box outline: corners plus three per edge. Faces of two
 * aligned boxes (two carts meeting head-on) put their corners exactly on each
 * other's edges, so corners alone would miss that contact.
 */
function boxSamples(hw: number, hh: number): [number, number][] {
  const out: [number, number][] = [];
  const corners: [number, number][] = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]];
  for (let i = 0; i < 4; i++) {
    const [ax, ay] = corners[i], [bx, by] = corners[(i + 1) % 4];
    out.push([ax, ay]);
    for (const t of [0.25, 0.5, 0.75]) out.push([ax + (bx - ax) * t, ay + (by - ay) * t]);
  }
  return out;
}

/** Penetration of world point (px, py) into a box; normal is the box's nearest outward face. */
function inBox(p: PoseS, hw: number, hh: number, px: number, py: number): { depth: number; nx: number; ny: number } | null {
  const dx = px - p.x, dy = py - p.y;
  const u = p.c * dx + p.s * dy, v = -p.s * dx + p.c * dy;
  const fu = hw - Math.abs(u), fv = hh - Math.abs(v);
  if (fu <= 0 || fv <= 0) return null;
  const ou = fu < fv ? Math.sign(u) || 1 : 0, ov = fu < fv ? 0 : Math.sign(v) || 1;
  return { depth: Math.min(fu, fv), nx: p.c * ou - p.s * ov, ny: p.s * ou + p.c * ov };
}

/** Circle (centre cx, cy, radius r) against a box: contact point on the box and push direction for the circle. */
function circleBox(p: PoseS, hw: number, hh: number, cx: number, cy: number, r: number): PairHit | null {
  const dx = cx - p.x, dy = cy - p.y;
  const u = p.c * dx + p.s * dy, v = -p.s * dx + p.c * dy;
  const cu = Math.max(-hw, Math.min(hw, u)), cv = Math.max(-hh, Math.min(hh, v));
  const du = u - cu, dv = v - cv;
  const d = Math.hypot(du, dv);
  let nu: number, nv: number, depth: number;
  if (d > 1e-12) {
    if (d >= r) return null;
    nu = du / d; nv = dv / d; depth = r - d;
  } else {
    const fu = hw - Math.abs(u), fv = hh - Math.abs(v);
    if (fu < fv) { nu = Math.sign(u) || 1; nv = 0; } else { nu = 0; nv = Math.sign(v) || 1; }
    depth = r + Math.min(fu, fv);
  }
  const pxL = d > 1e-12 ? cu : u, pyL = d > 1e-12 ? cv : v;
  return {
    px: p.x + p.c * pxL - p.s * pyL, py: p.y + p.s * pxL + p.c * pyL,
    nx: p.c * nu - p.s * nv, ny: p.s * nu + p.c * nv, depth,
  };
}

/**
 * Penalty contact between two moving bodies (boxes and circles). The force
 * acts equally and oppositely on both bodies at the contact point, so linear
 * and angular momentum of the pair are conserved. Box–box contacts use many
 * sample points, each carrying a share of the stiffness so that a flat
 * face-to-face contact (about six points) has the calibrated k and c.
 */
export class BodyContactF implements ForceEl {
  el: string;
  private samplesA: [number, number][] = [];
  private samplesB: [number, number][] = [];
  private kPt: number;
  private cPt: number;
  private reach: number;

  constructor(
    public a: SBody, public sa: BodyShape, public b: SBody, public sb: BodyShape,
    k: number, c: number,
  ) {
    this.el = `${a.id}:${b.id}:contact`;
    const boxes = sa.kind === 'box' && sb.kind === 'box';
    this.kPt = boxes ? k / 6 : k;
    this.cPt = boxes ? c / 6 : c;
    if (sa.kind === 'box') this.samplesA = boxSamples(sa.hw, sa.hh);
    if (sb.kind === 'box') this.samplesB = boxSamples(sb.hw, sb.hh);
    const rad = (s: BodyShape) => (s.kind === 'box' ? Math.hypot(s.hw, s.hh) : s.r);
    this.reach = rad(sa) + rad(sb);
  }

  private pose(b: SBody, q: ArrayLike<number>): PoseS {
    const th = b.dof === 3 ? q[b.idx + 2] : 0;
    return { x: q[b.idx], y: q[b.idx + 1], c: Math.cos(th), s: Math.sin(th) };
  }

  hits(q: ArrayLike<number>): PairHit[] {
    const pa = this.pose(this.a, q), pb = this.pose(this.b, q);
    if (Math.hypot(pa.x - pb.x, pa.y - pb.y) >= this.reach) return [];
    const { sa, sb } = this;
    const out: PairHit[] = [];
    if (sa.kind === 'circle' && sb.kind === 'circle') {
      const dx = pa.x - pb.x, dy = pa.y - pb.y;
      const d = Math.hypot(dx, dy);
      const depth = sa.r + sb.r - d;
      if (depth > 0) {
        const nx = d > 1e-12 ? dx / d : 0, ny = d > 1e-12 ? dy / d : 1;
        out.push({ px: pb.x + nx * sb.r, py: pb.y + ny * sb.r, nx, ny, depth });
      }
      return out;
    }
    if (sa.kind === 'circle' && sb.kind === 'box') {
      const h = circleBox(pb, sb.hw, sb.hh, pa.x, pa.y, sa.r);
      if (h) out.push(h);
      return out;
    }
    if (sa.kind === 'box' && sb.kind === 'circle') {
      const h = circleBox(pa, sa.hw, sa.hh, pb.x, pb.y, sb.r);
      if (h) out.push({ ...h, nx: -h.nx, ny: -h.ny });
      return out;
    }
    if (sa.kind === 'box' && sb.kind === 'box') {
      // A's outline inside B pushes A out through B's face, and vice versa
      for (const [lx, ly] of this.samplesA) {
        const px = pa.x + pa.c * lx - pa.s * ly, py = pa.y + pa.s * lx + pa.c * ly;
        const h = inBox(pb, sb.hw, sb.hh, px, py);
        if (h) out.push({ px, py, nx: h.nx, ny: h.ny, depth: h.depth });
      }
      for (const [lx, ly] of this.samplesB) {
        const px = pb.x + pb.c * lx - pb.s * ly, py = pb.y + pb.s * lx + pb.c * ly;
        const h = inBox(pa, sa.hw, sa.hh, px, py);
        if (h) out.push({ px, py, nx: -h.nx, ny: -h.ny, depth: h.depth });
      }
    }
    return out;
  }

  /** Penetration rate at a contact (positive = going deeper). */
  private rate(h: PairHit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const v = (b: SBody) => {
      const w = b.dof === 3 ? qd[b.idx + 2] : 0;
      const rx = h.px - q[b.idx], ry = h.py - q[b.idx + 1];
      return [qd[b.idx] - w * ry, qd[b.idx + 1] + w * rx];
    };
    const va = v(this.a), vb = v(this.b);
    return -((va[0] - vb[0]) * h.nx + (va[1] - vb[1]) * h.ny);
  }

  private forceOf(h: PairHit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const ramp = Math.min(1, h.depth / 1e-3);
    return Math.max(0, this.kPt * h.depth + ramp * this.cPt * this.rate(h, q, qd));
  }

  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      addPtForce(F, this.a, h.px - q[this.a.idx], h.py - q[this.a.idx + 1], f * h.nx, f * h.ny);
      addPtForce(F, this.b, h.px - q[this.b.idx], h.py - q[this.b.idx + 1], -f * h.nx, -f * h.ny);
    }
  }

  energy(q: ArrayLike<number>): number {
    let e = 0;
    for (const h of this.hits(q)) e += 0.5 * this.kPt * h.depth * h.depth;
    return e;
  }

  power(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let p = 0;
    for (const h of this.hits(q)) p -= (this.forceOf(h, q, qd) - this.kPt * h.depth) * this.rate(h, q, qd);
    return p;
  }

  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      out.push({ body: this.a, x: h.px, y: h.py, fx: f * h.nx, fy: f * h.ny, label: `${this.b.name} contact`, kind: 'contact' });
      out.push({ body: this.b, x: h.px, y: h.py, fx: -f * h.nx, fy: -f * h.ny, label: `${this.a.name} contact`, kind: 'contact' });
    }
  }

  maxDepth(q: ArrayLike<number>): number {
    return this.hits(q).reduce((m, h) => Math.max(m, h.depth), 0);
  }
}
