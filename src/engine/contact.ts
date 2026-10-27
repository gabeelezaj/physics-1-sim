// Penalty contact between moving bodies and ground pieces (walls, floors).
//
// Ground pieces are solid oriented boxes. A contact is a stiff, one-sided
// spring-damper along the contact normal:
//   f = max(0, k δ + c δ̇),   V = ½ k δ²
// with k = m ω_c² (ω_c ≈ 300 rad/s, well inside RK4's stability at 1 ms) and c
// set from the coefficient of restitution e via ζ = −ln e / √(π² + ln² e).
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

/** Contact stiffness/damping for a body of mass m. */
export function contactGains(m: number, dt: number, restitution: number): { k: number; c: number } {
  const wc = Math.min(300, 0.3 / dt);
  const k = m * wc * wc;
  const e = Math.min(1, Math.max(1e-3, restitution));
  const ln = Math.log(e);
  const zeta = -ln / Math.sqrt(Math.PI * Math.PI + ln * ln);
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

  private forceOf(h: Hit, qd: ArrayLike<number>): number {
    const b = this.body;
    const w = b.dof === 3 ? qd[b.idx + 2] : 0;
    const vx = qd[b.idx] - w * h.ry, vy = qd[b.idx + 1] + w * h.rx;
    // penetration grows when the body point moves against the normal
    const rate = -(vx * h.nx + vy * h.ny);
    return Math.max(0, this.k * h.depth + this.c * rate);
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
