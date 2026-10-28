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
// per edge) against the other polygon. A point inside is pushed along the
// direction that frees it with the least travel, chosen among the other
// polygon's face normals and the outward normals of the point's own edges.
// Taking that minimum keeps the contact energy continuous, and it is what
// lets a block meeting the knife-edge toe of a ramp ride up the slope rather
// than hit the toe like a wall. A corner carries the full k and c, edge
// samples 1/6 each, so two aligned faces (six edge samples) get the
// calibrated values too. Friction is regularized Coulomb with the larger μ of
// the two surfaces.

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
  /**
   * Edges buried in another fixed piece (a ramp's base on the floor, a wall's
   * foot): nothing can be pushed out through them.
   */
  buried?: boolean[];
  /** Coulomb friction coefficient of a fixed piece's surface. */
  mu?: number;
}

interface Prepared extends ContactSide {
  /** Outline sample points (local) with their stiffness weight. */
  samples: [number, number, number][];
  /** Indices of the edges each sample lies on (two for a corner). */
  sampleEdges: number[][];
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
  /**
   * |∇δ| when the depth is measured along an escape direction r that is not
   * the exit face normal n: δ = distance/(n·r), so the force n·(kδ + cδ̇)/(n·r)
   * is exactly −∇(½kδ²) and the contact stays conservative.
   */
  s: number;
  /** Friction coefficient at this contact (ground surfaces only). */
  mu?: number;
}

function prepare(side: ContactSide): Prepared {
  if (side.shape.kind === 'circle') return { ...side, samples: [], sampleEdges: [], normals: [], radius: side.shape.r };
  const buried = side.buried ?? [];
  const v = side.shape.verts;
  const samples: [number, number, number][] = [];
  const sampleEdges: number[][] = [];
  const normals: V2[] = [];
  for (let i = 0; i < v.length; i++) {
    const [ax, ay] = v[i], [bx, by] = v[(i + 1) % v.length];
    // a sample's free edges; points on a buried edge only are never in contact
    const corner = [(i + v.length - 1) % v.length, i].filter((e) => !buried[e]);
    if (corner.length) { samples.push([ax, ay, 1]); sampleEdges.push(corner); }
    if (!buried[i]) {
      for (const t of [0.25, 0.5, 0.75]) {
        samples.push([ax + (bx - ax) * t, ay + (by - ay) * t, 1 / 6]);
        sampleEdges.push([i]);
      }
    }
    const L = Math.hypot(bx - ax, by - ay) || 1;
    normals.push([(by - ay) / L, -(bx - ax) / L]);
  }
  return { ...side, samples, sampleEdges, normals, radius: Math.max(...v.map(([x, y]) => Math.hypot(x, y))) };
}

const MAX_GAIN = 4;

/** Shared empty result (never mutated). */
const NO_HITS: Hit[] = [];

/**
 * Remembers the contacts found for the last body coordinates: within one
 * integrator stage the same q is asked for by apply(), power() and energy(),
 * and the contact search is most of the cost of a step.
 */
class HitCache {
  private key: number[] = [];
  private val: Hit[] | null = null;
  constructor(private bodies: (SBody | null)[]) {}
  /** The stored contacts if the bodies have not moved since `store`, else null (and remember this q). */
  lookup(q: ArrayLike<number>): Hit[] | null {
    let k = 0, same = this.val !== null;
    for (const b of this.bodies) {
      if (!b) continue;
      for (let i = 0; i < b.dof; i++, k++) {
        const v = q[b.idx + i];
        if (this.key[k] !== v) { same = false; this.key[k] = v; }
      }
    }
    return same ? this.val : null;
  }

  store(h: Hit[]): Hit[] {
    this.val = h;
    return h;
  }
}

/** World axis-aligned bounds of a polygon at a pose. */
function bounds(P: Prepared, p: PoseS): [number, number, number, number] {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const [lx, ly] of (P.shape as { verts: V2[] }).verts) {
    const x = p.x + p.c * lx - p.s * ly, y = p.y + p.s * lx + p.c * ly;
    if (x < x0) x0 = x;
    if (x > x1) x1 = x;
    if (y < y0) y0 = y;
    if (y > y1) y1 = y;
  }
  return [x0, y0, x1, y1];
}

/**
 * For a world point inside polygon Y (pose py), the smallest push of the
 * point's own body that frees it. Candidate escape directions are Y's
 * outward face normals and `own` reversed (the point's body's outward edge
 * normals). Returns the travel, the normal of the face it leaves through
 * (world) and the gain 1/(n·r).
 */
function freePoint(Y: Prepared, py: PoseS, px: number, pyy: number, own: V2[]): { depth: number; nx: number; ny: number; s: number } | null {
  const v = (Y.shape as { verts: V2[] }).verts;
  const dx = px - py.x, dy = pyy - py.y;
  const u = py.c * dx + py.s * dy, w = -py.s * dx + py.c * dy;
  const dist: number[] = [];
  for (let i = 0; i < v.length; i++) {
    const d = (u - v[i][0]) * Y.normals[i][0] + (w - v[i][1]) * Y.normals[i][1];
    if (d >= 0) return null;
    dist.push(-d);
  }
  const buried = Y.buried ?? [];
  let best = Infinity, face = -1, gain = 1;
  // travel along local direction r until the point leaves Y
  const tryDir = (rx: number, ry: number) => {
    let t = Infinity, f = -1, dnf = 1;
    for (let i = 0; i < v.length; i++) {
      if (buried[i]) continue;
      const dn = Y.normals[i][0] * rx + Y.normals[i][1] * ry;
      if (dn > 1e-12 && dist[i] / dn < t) { t = dist[i] / dn; f = i; dnf = dn; }
    }
    if (t < best) { best = t; face = f; gain = Math.min(MAX_GAIN, 1 / dnf); }
  };
  for (let i = 0; i < v.length; i++) if (!buried[i]) tryDir(Y.normals[i][0], Y.normals[i][1]);
  for (const [wx, wy] of own) tryDir(-(py.c * wx + py.s * wy), -(-py.s * wx + py.c * wy));
  if (face < 0) return null;
  const [lx, ly] = Y.normals[face];
  return { depth: best, nx: py.c * lx - py.s * ly, ny: py.s * lx + py.c * ly, s: gain };
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
    return { px, py, nx: wx, ny: wy, depth: r + best, wt: 1, s: 1 };
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
  return { px, py, nx, ny, depth: r - dmin, wt: 1, s: 1 };
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

  /**
   * `mu` is the Coulomb coefficient between the two surfaces, regularized
   * like ground friction with the pair's effective mass `meff` and the
   * in-contact step `hStep`.
   */
  constructor(a: ContactSide, b: ContactSide, private k: number, private c: number, private mu = 0, private meff = 1, public hStep = 2.5e-4) {
    this.A = prepare(a);
    this.B = prepare(b);
    this.el = `${a.b?.id}:${b.b?.id ?? b.name}:contact`;
    this.reach = this.A.radius + this.B.radius;
    this.cache = new HitCache([a.b, b.b]);
  }

  private cache: HitCache;

  private pose(P: Prepared, q: ArrayLike<number>): PoseS {
    if (!P.b) {
      const f = P.fixed!;
      return { x: f.x, y: f.y, c: Math.cos(f.theta), s: Math.sin(f.theta) };
    }
    const th = P.b.dof === 3 ? q[P.b.idx + 2] : 0;
    return { x: q[P.b.idx], y: q[P.b.idx + 1], c: Math.cos(th), s: Math.sin(th) };
  }

  hits(q: ArrayLike<number>): Hit[] {
    return this.cache.lookup(q) ?? this.cache.store(this.findHits(q));
  }

  private findHits(q: ArrayLike<number>): Hit[] {
    const { A, B } = this;
    // broad phase on the centres first (fixed pieces can be long: their radius covers the whole outline)
    const ax = A.b ? q[A.b.idx] : A.fixed!.x, ay = A.b ? q[A.b.idx + 1] : A.fixed!.y;
    const bx = B.b ? q[B.b.idx] : B.fixed!.x, by = B.b ? q[B.b.idx + 1] : B.fixed!.y;
    if ((ax - bx) * (ax - bx) + (ay - by) * (ay - by) >= this.reach * this.reach) return NO_HITS;
    const pa = this.pose(A, q), pb = this.pose(B, q);
    const out: Hit[] = [];
    const ca = A.shape.kind === 'circle', cb = B.shape.kind === 'circle';
    if (ca && cb) {
      const dx = pa.x - pb.x, dy = pa.y - pb.y;
      const d = Math.hypot(dx, dy);
      const ra = A.radius, rb = B.radius;
      if (d < ra + rb) {
        const nx = d > 1e-12 ? dx / d : 0, ny = d > 1e-12 ? dy / d : 1;
        out.push({ px: pb.x + nx * rb, py: pb.y + ny * rb, nx, ny, depth: ra + rb - d, wt: 1, s: 1 });
      }
      return out;
    }
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
    const ba = bounds(A, pa), bb = bounds(B, pb);
    if (ba[0] > bb[2] || bb[0] > ba[2] || ba[1] > bb[3] || bb[1] > ba[3]) return out;
    const ownNormals = (P: Prepared, pose: PoseS, k: number): V2[] =>
      P.sampleEdges[k].map((e): V2 => {
        const [x, y] = P.normals[e];
        return [pose.c * x - pose.s * y, pose.s * x + pose.c * y];
      });
    A.samples.forEach(([lx, ly, wt], k) => {
      const px = pa.x + pa.c * lx - pa.s * ly, py = pa.y + pa.s * lx + pa.c * ly;
      const f = freePoint(B, pb, px, py, ownNormals(A, pa, k));
      if (f) out.push({ px, py, nx: f.nx, ny: f.ny, depth: f.depth, wt, s: f.s });
    });
    B.samples.forEach(([lx, ly, wt], k) => {
      const px = pb.x + pb.c * lx - pb.s * ly, py = pb.y + pb.s * lx + pb.c * ly;
      const f = freePoint(A, pa, px, py, ownNormals(B, pb, k));
      if (f) out.push({ px, py, nx: -f.nx, ny: -f.ny, depth: f.depth, wt, s: f.s });
    });
    return out;
  }

  /** Velocity of A's material point at the contact relative to B's. */
  private relVel(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): V2 {
    const v = (b: SBody | null): V2 => {
      if (!b) return [0, 0];
      const w = b.dof === 3 ? qd[b.idx + 2] : 0;
      const rx = h.px - q[b.idx], ry = h.py - q[b.idx + 1];
      return [qd[b.idx] - w * ry, qd[b.idx + 1] + w * rx];
    };
    const va = v(this.A.b), vb = v(this.B.b);
    return [va[0] - vb[0], va[1] - vb[1]];
  }

  /** Penetration rate at a contact (positive = going deeper). */
  private rate(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const [vx, vy] = this.relVel(h, q, qd);
    return -h.s * (vx * h.nx + vy * h.ny);
  }

  /** Friction on A (B gets the opposite), regularized Coulomb as for ground contact. */
  private friction(h: Hit, fn: number, q: ArrayLike<number>, qd: ArrayLike<number>): V2 {
    if (!this.mu || fn <= 0) return [0, 0];
    const [vx, vy] = this.relVel(h, q, qd);
    const vn = vx * h.nx + vy * h.ny;
    const tx = vx - vn * h.nx, ty = vy - vn * h.ny;
    const vt = Math.hypot(tx, ty);
    if (vt < 1e-12) return [0, 0];
    const vs = (4 * this.mu * fn * this.hStep) / this.meff;
    const f = (this.mu * fn) / Math.sqrt(vt * vt + vs * vs);
    return [-f * tx, -f * ty];
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
      const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
      const fx = f * h.s * h.nx + ffx, fy = f * h.s * h.ny + ffy;
      if (a) addPtForce(F, a, h.px - q[a.idx], h.py - q[a.idx + 1], fx, fy);
      if (b) addPtForce(F, b, h.px - q[b.idx], h.py - q[b.idx + 1], -fx, -fy);
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
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      p -= (f - h.wt * this.k * h.depth) * this.rate(h, q, qd);
      if (f > 0 && this.mu) {
        const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
        const [vx, vy] = this.relVel(h, q, qd);
        p += ffx * vx + ffy * vy;
      }
    }
    return p;
  }

  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    const a = this.A.b, b = this.B.b;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      const fx = f * h.s * h.nx, fy = f * h.s * h.ny;
      if (a) out.push({ body: a, x: h.px, y: h.py, fx, fy, label: `${this.B.name} contact`, kind: 'contact' });
      if (b) out.push({ body: b, x: h.px, y: h.py, fx: -fx, fy: -fy, label: `${this.A.name} contact`, kind: 'contact' });
      const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
      if (!ffx && !ffy) continue;
      if (a) out.push({ body: a, x: h.px, y: h.py, fx: ffx, fy: ffy, label: `${this.B.name} friction`, kind: 'friction' });
      if (b) out.push({ body: b, x: h.px, y: h.py, fx: -ffx, fy: -ffy, label: `${this.A.name} friction`, kind: 'friction' });
    }
  }

  /** Fastest normal speed (or sliding speed, with friction) among the contacts at q. */
  maxRate(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let m = 0;
    for (const h of this.hits(q)) {
      m = Math.max(m, Math.abs(this.rate(h, q, qd)));
      if (this.mu) {
        const [vx, vy] = this.relVel(h, q, qd);
        const vn = vx * h.nx + vy * h.ny;
        m = Math.max(m, Math.hypot(vx - vn * h.nx, vy - vn * h.ny));
      }
    }
    return m;
  }

  /** Deepest penetration at q (for the initial-overlap check). */
  maxDepth(q: ArrayLike<number>): number {
    return this.hits(q).reduce((m, h) => Math.max(m, h.depth), 0);
  }
}

// ------------------------------------------------------- body vs all ground

/** A fixed piece of the world (floor, wall, ramp) prepared for contact. */
interface Part {
  P: Prepared;
  pose: PoseS;
  /** Outward normals in world coordinates. */
  wn: V2[];
  /** World bounds [x0, y0, x1, y1]. */
  box: [number, number, number, number];
}

/** An exposed outline sample of a ground piece, in world coordinates. */
interface GSample {
  x: number;
  y: number;
  wt: number;
  own: V2[];
  mu: number;
}

function partOf(side: ContactSide): Part {
  const P = prepare(side);
  const f = side.fixed!;
  const pose = { x: f.x, y: f.y, c: Math.cos(f.theta), s: Math.sin(f.theta) };
  return { P, pose, wn: P.normals.map(([x, y]): V2 => [pose.c * x - pose.s * y, pose.s * x + pose.c * y]), box: bounds(P, pose) };
}

/** Signed distances of a world point to a part's edge lines (all < 0 = inside), or null if outside. */
function insidePart(part: Part, px: number, py: number, tol = 0): number[] | null {
  const v = (part.P.shape as { verts: V2[] }).verts;
  const dx = px - part.pose.x, dy = py - part.pose.y;
  const u = part.pose.c * dx + part.pose.s * dy, w = -part.pose.s * dx + part.pose.c * dy;
  const out: number[] = [];
  for (let i = 0; i < v.length; i++) {
    const d = (u - v[i][0]) * part.P.normals[i][0] + (w - v[i][1]) * part.P.normals[i][1];
    if (d >= -tol) return null;
    out.push(d);
  }
  return out;
}

/** A stretch of the fixed world's outer boundary, with its outward normal. */
interface Seg {
  ax: number;
  ay: number;
  bx: number;
  by: number;
  nx: number;
  ny: number;
  mu: number;
  /** Bounds [x0, y0, x1, y1]. */
  box: [number, number, number, number];
}

/**
 * Contact of one moving body with the whole fixed world, treated as a single
 * solid (the union of all ground pieces). The union's outer boundary is
 * precomputed as the parts of every ground edge not covered by another piece.
 * A body point inside the union is pushed straight towards the nearest point
 * of that boundary, by its distance: the exact signed distance to the union,
 * which is continuous where pieces meet (a block slides from the floor onto
 * a ramp's toe, a wedge presses into the corner of floor and wall) and whose
 * gradient is the push direction, so the contact is conservative.
 */
export class GroundContactF implements ForceEl {
  el: string;
  private A: Prepared;
  private parts: Part[];
  private segs: Seg[] = [];
  /** The stretches of `segs` near the body in the current search. */
  private local: Seg[] = [];
  /** Per piece, its outline samples that are on the union's boundary (not covered by another piece). */
  private gsamples: GSample[][];

  private mus: number[];

  /**
   * `hStep` is the integration step while in contact; it sets how sharply
   * friction may switch direction and still integrate stably.
   */
  constructor(a: ContactSide, grounds: ContactSide[], private k: number, private c: number, public hStep = 2.5e-4) {
    this.A = prepare(a);
    this.parts = grounds.map(partOf);
    this.mus = grounds.map((g) => g.mu ?? 0);
    this.el = `${a.b?.id}:ground:contact`;
    const covered = (x: number, y: number, self: Part) => this.parts.some((o) => o !== self && insidePart(o, x, y, 1e-12));
    this.parts.forEach((part, pi) => {
      const v = toWorldV(part);
      v.forEach((a, i) => {
        const b = v[(i + 1) % v.length];
        const n = part.wn[i];
        // split the edge where other pieces' edges cross it
        const ts = [0, 1];
        for (const o of this.parts) {
          if (o === part) continue;
          const w = toWorldV(o);
          w.forEach((c, j) => {
            const t = segCross(a, b, c, w[(j + 1) % w.length]);
            if (t !== null) ts.push(t);
          });
        }
        ts.sort((x, y) => x - y);
        for (let k = 0; k + 1 < ts.length; k++) {
          const t0 = ts[k], t1 = ts[k + 1];
          if (t1 - t0 < 1e-9) continue;
          const tm = 0.5 * (t0 + t1);
          const mx = a[0] + (b[0] - a[0]) * tm + n[0] * 1e-7, my = a[1] + (b[1] - a[1]) * tm + n[1] * 1e-7;
          if (covered(mx, my, part)) continue;
          this.segs.push({
            ax: a[0] + (b[0] - a[0]) * t0, ay: a[1] + (b[1] - a[1]) * t0,
            bx: a[0] + (b[0] - a[0]) * t1, by: a[1] + (b[1] - a[1]) * t1, nx: n[0], ny: n[1], mu: this.mus[pi], box: [0, 0, 0, 0],
          });
          const g = this.segs[this.segs.length - 1];
          g.box = [Math.min(g.ax, g.bx), Math.min(g.ay, g.by), Math.max(g.ax, g.bx), Math.max(g.ay, g.by)];
        }
      });
    });
    this.cache = new HitCache([a.b]);
    this.gsamples = this.parts.map((part, pi) => {
      const out: GSample[] = [];
      part.P.samples.forEach(([lx, ly, wt], k) => {
        const x = part.pose.x + part.pose.c * lx - part.pose.s * ly, y = part.pose.y + part.pose.s * lx + part.pose.c * ly;
        if (!covered(x, y, part)) out.push({ x, y, wt, own: part.P.sampleEdges[k].map((e) => part.wn[e]), mu: this.mus[pi] });
      });
      return out;
    });
  }

  private bodyPose(q: ArrayLike<number>): PoseS {
    const b = this.A.b!;
    const th = b.dof === 3 ? q[b.idx + 2] : 0;
    return { x: q[b.idx], y: q[b.idx + 1], c: Math.cos(th), s: Math.sin(th) };
  }

  private cache: HitCache;

  hits(q: ArrayLike<number>): Hit[] {
    return this.cache.lookup(q) ?? this.cache.store(this.findHits(q));
  }

  private findHits(q: ArrayLike<number>): Hit[] {
    const A = this.A;
    const pa = this.bodyPose(q);
    const out: Hit[] = [];
    // body bounds; only pieces whose bounds overlap them can touch it
    const r0 = A.radius;
    const bb = A.shape.kind === 'circle' ? [pa.x - r0, pa.y - r0, pa.x + r0, pa.y + r0] : bounds(A, pa);
    const near: number[] = [];
    this.parts.forEach((p, i) => {
      if (!(p.box[0] > bb[2] || bb[0] > p.box[2] || p.box[1] > bb[3] || bb[1] > p.box[3])) near.push(i);
    });
    if (!near.length) return out;
    // boundary stretches close enough to be the nearest one to a body point
    // that is inside the ground (no deeper than the body is wide)
    const m = A.shape.kind === 'circle' ? 0 : 2 * r0;
    this.local.length = 0;
    for (const g of this.segs) {
      if (!(g.box[0] > bb[2] + m || bb[0] - m > g.box[2] || g.box[1] > bb[3] + m || bb[1] - m > g.box[3])) this.local.push(g);
    }
    if (A.shape.kind === 'circle') {
      // one contact against the union's boundary: smooth rolling over joints and curves
      const r = A.radius;
      const inside = near.some((i) => insidePart(this.parts[i], pa.x, pa.y));
      const c = this.closest(pa.x, pa.y);
      if (!c) return out;
      if (!inside && c.d >= r) return out;
      const sgn = inside ? -1 : 1; // inside: push towards the boundary point
      const nx = c.d > 1e-12 ? (sgn * (pa.x - c.x)) / c.d : c.nx, ny = c.d > 1e-12 ? (sgn * (pa.y - c.y)) / c.d : c.ny;
      out.push({ px: c.x, py: c.y, nx, ny, depth: inside ? r + c.d : r - c.d, wt: 1, s: 1, mu: c.mu });
      return out;
    }
    // body outline points inside the ground: push towards the nearest exposed boundary point
    for (const [lx, ly, wt] of A.samples) {
      const px = pa.x + pa.c * lx - pa.s * ly, py = pa.y + pa.s * lx + pa.c * ly;
      if (!near.some((i) => insidePart(this.parts[i], px, py))) continue;
      const c = this.closest(px, py);
      if (!c || c.d <= 0) continue;
      // direction to the closest boundary point (the face normal if it is mid-face)
      const nx = c.d > 1e-12 ? (c.x - px) / c.d : c.nx, ny = c.d > 1e-12 ? (c.y - py) / c.d : c.ny;
      out.push({ px, py, nx, ny, depth: c.d, wt, s: 1, mu: c.mu });
    }
    // ground corners and edges poking into the body
    for (const i of near) {
      for (const g of this.gsamples[i]) {
        if (g.x < bb[0] || g.x > bb[2] || g.y < bb[1] || g.y > bb[3]) continue;
        const f = freePoint(A, pa, g.x, g.y, g.own);
        if (f) out.push({ px: g.x, py: g.y, nx: -f.nx, ny: -f.ny, depth: f.depth, wt: g.wt, s: f.s, mu: g.mu });
      }
    }
    return out;
  }

  /** Nearest point on the union's exposed boundary. */
  private closest(px: number, py: number): { x: number; y: number; d: number; nx: number; ny: number; mu: number } | null {
    let best: { x: number; y: number; d: number; nx: number; ny: number; mu: number } | null = null;
    for (const g of this.local) {
      const ex = g.bx - g.ax, ey = g.by - g.ay;
      const t = Math.max(0, Math.min(1, ((px - g.ax) * ex + (py - g.ay) * ey) / (ex * ex + ey * ey || 1)));
      const x = g.ax + t * ex, y = g.ay + t * ey;
      const d = Math.hypot(x - px, y - py);
      if (!best || d < best.d) best = { x, y, d, nx: g.nx, ny: g.ny, mu: g.mu };
    }
    return best;
  }

  /** Velocity of the body's material point at a contact. */
  private pointVel(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): V2 {
    const b = this.A.b!;
    const w = b.dof === 3 ? qd[b.idx + 2] : 0;
    const rx = h.px - q[b.idx], ry = h.py - q[b.idx + 1];
    return [qd[b.idx] - w * ry, qd[b.idx + 1] + w * rx];
  }

  /**
   * Coulomb friction μN opposing the sliding velocity, regularized below a
   * slip speed v_s = 4μN·h/m so the force never switches faster than the
   * integrator can follow (a resting body creeps by at most a few mm/s).
   */
  private friction(h: Hit, fn: number, q: ArrayLike<number>, qd: ArrayLike<number>): V2 {
    if (!h.mu || fn <= 0) return [0, 0];
    const [vx, vy] = this.pointVel(h, q, qd);
    const vn = vx * h.nx + vy * h.ny;
    const tx = vx - vn * h.nx, ty = vy - vn * h.ny;
    const vt = Math.hypot(tx, ty);
    if (vt < 1e-12) return [0, 0];
    const vs = (4 * h.mu * fn * this.hStep) / this.A.b!.m;
    const f = (h.mu * fn) / Math.sqrt(vt * vt + vs * vs);
    return [-f * tx, -f * ty];
  }

  private rate(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const b = this.A.b!;
    const w = b.dof === 3 ? qd[b.idx + 2] : 0;
    const rx = h.px - q[b.idx], ry = h.py - q[b.idx + 1];
    return -h.s * ((qd[b.idx] - w * ry) * h.nx + (qd[b.idx + 1] + w * rx) * h.ny);
  }

  private forceOf(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const ramp = Math.min(1, h.depth / 1e-3);
    return h.wt * Math.max(0, this.k * h.depth + ramp * this.c * this.rate(h, q, qd));
  }

  apply(q: ArrayLike<number>, qd: ArrayLike<number>, F: Float64Array): void {
    const b = this.A.b!;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
      addPtForce(F, b, h.px - q[b.idx], h.py - q[b.idx + 1], f * h.s * h.nx + ffx, f * h.s * h.ny + ffy);
    }
  }

  energy(q: ArrayLike<number>): number {
    let e = 0;
    for (const h of this.hits(q)) e += 0.5 * h.wt * this.k * h.depth * h.depth;
    return e;
  }

  power(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let p = 0;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      p -= (f - h.wt * this.k * h.depth) * this.rate(h, q, qd);
      if (f > 0 && h.mu) {
        const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
        const [vx, vy] = this.pointVel(h, q, qd);
        p += ffx * vx + ffy * vy;
      }
    }
    return p;
  }

  report(q: ArrayLike<number>, qd: ArrayLike<number>, _l: ArrayLike<number>, out: PointForce[]): void {
    const b = this.A.b!;
    for (const h of this.hits(q)) {
      const f = this.forceOf(h, q, qd);
      if (f <= 0) continue;
      out.push({ body: b, x: h.px, y: h.py, fx: f * h.s * h.nx, fy: f * h.s * h.ny, label: 'ground contact', kind: 'contact' });
      const [ffx, ffy] = this.friction(h, f * h.s, q, qd);
      if (ffx || ffy) out.push({ body: b, x: h.px, y: h.py, fx: ffx, fy: ffy, label: 'friction', kind: 'friction' });
    }
  }

  maxRate(q: ArrayLike<number>, qd: ArrayLike<number>): number {
    let m = 0;
    for (const h of this.hits(q)) {
      m = Math.max(m, Math.abs(this.rate(h, q, qd)));
      if (h.mu) m = Math.max(m, this.slip(h, q, qd));
    }
    return m;
  }

  /** Tangential speed of the body's material point at a contact. */
  private slip(h: Hit, q: ArrayLike<number>, qd: ArrayLike<number>): number {
    const [vx, vy] = this.pointVel(h, q, qd);
    const vn = vx * h.nx + vy * h.ny;
    return Math.hypot(vx - vn * h.nx, vy - vn * h.ny);
  }

  maxDepth(q: ArrayLike<number>): number {
    return this.hits(q).reduce((m, h) => Math.max(m, h.depth), 0);
  }
}

function toWorldV(part: Part): V2[] {
  return (part.P.shape as { verts: V2[] }).verts.map(([lx, ly]): V2 => [
    part.pose.x + part.pose.c * lx - part.pose.s * ly,
    part.pose.y + part.pose.s * lx + part.pose.c * ly,
  ]);
}

/** Parameter along a→b where it crosses segment c→d (strictly inside a→b), or null. */
function segCross(a: V2, b: V2, c: V2, d: V2): number | null {
  const rx = b[0] - a[0], ry = b[1] - a[1], sx = d[0] - c[0], sy = d[1] - c[1];
  const den = rx * sy - ry * sx;
  if (Math.abs(den) < 1e-14) return null;
  const qx = c[0] - a[0], qy = c[1] - a[1];
  const t = (qx * sy - qy * sx) / den, u = (qx * ry - qy * rx) / den;
  return t > 1e-12 && t < 1 - 1e-12 && u >= -1e-12 && u <= 1 + 1e-12 ? t : null;
}
