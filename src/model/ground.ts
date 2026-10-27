// Ground pieces are stored as boxes (centre, angle, width, thickness) whose
// local +y face is the solid surface. The editor works with the surface's two
// end points and the side the free space is on; these helpers convert.

import type { GroundEl, Vec2 } from './types';

export interface GroundEnds {
  a: Vec2;
  b: Vec2;
  /** Unit normal pointing from the surface into free space. */
  n: Vec2;
}

export function groundEnds(g: GroundEl): GroundEnds {
  const c = Math.cos(g.theta), s = Math.sin(g.theta);
  const at = (lx: number, ly: number): Vec2 => [g.x + c * lx - s * ly, g.y + s * lx + c * ly];
  return { a: at(-g.w / 2, g.h / 2), b: at(g.w / 2, g.h / 2), n: [-s, c] };
}

/**
 * Box for a surface from `a` to `b` with free space on the side of `side`
 * (only its sign relative to the segment matters).
 */
export function groundFromEnds(a: Vec2, b: Vec2, side: Vec2, h: number): Pick<GroundEl, 'x' | 'y' | 'theta' | 'w' | 'h'> {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const w = Math.max(Math.hypot(dx, dy), 1e-6);
  const ux = dx / w, uy = dy / w;
  const px = -uy, py = ux;
  const flip = px * side[0] + py * side[1] < 0;
  const theta = flip ? Math.atan2(-uy, -ux) : Math.atan2(uy, ux);
  const nx = flip ? -px : px, ny = flip ? -py : py;
  return { x: (a[0] + b[0]) / 2 - (nx * h) / 2, y: (a[1] + b[1]) / 2 - (ny * h) / 2, theta, w, h };
}

/**
 * Which side of a new surface should be free space: towards `ref` (usually
 * the moving bodies), falling back to "up" or "right".
 */
export function defaultSide(a: Vec2, b: Vec2, ref: Vec2 | null): Vec2 {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L = Math.hypot(dx, dy) || 1;
  const p: Vec2 = [-dy / L, dx / L];
  if (ref) {
    const m: Vec2 = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    const d = p[0] * (ref[0] - m[0]) + p[1] * (ref[1] - m[1]);
    if (Math.abs(d) > 1e-3) return d > 0 ? p : [-p[0], -p[1]];
  }
  if (Math.abs(p[1]) > 1e-6) return p[1] > 0 ? p : [-p[0], -p[1]];
  return p[0] > 0 ? p : [-p[0], -p[1]];
}

/** Snap `b` relative to `a`: angle to 15° steps and length to `step`. */
export function snapSegment(a: Vec2, b: Vec2, step = 0.05): Vec2 {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L = Math.round(Math.hypot(dx, dy) / step) * step;
  const inc = Math.PI / 12;
  const ang = Math.round(Math.atan2(dy, dx) / inc) * inc;
  const r = (v: number) => Math.round(v * 1e9) / 1e9;
  return [r(a[0] + L * Math.cos(ang)), r(a[1] + L * Math.sin(ang))];
}
