// Outline geometry shared by the engine (contact), renderer and editor.
//
// Boxes are centred on their origin. A wedge is the right triangle with base
// w and height h whose slope rises to the right (or left with `flip`); its
// origin is the centroid, so for a moving wedge the body origin is its
// centre of mass.

import type { BodyShapeKind } from './types';

export type V2 = [number, number];

export interface Outline {
  w: number;
  h: number;
  shape?: BodyShapeKind;
  flip?: boolean;
}

/** Local outline vertices, counter-clockwise. */
export function localPolygon(o: Outline): V2[] {
  const { w, h } = o;
  if (o.shape !== 'wedge') return [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]];
  // rising right: base-left, base-right, top-right
  const pts: V2[] = [[(-2 * w) / 3, -h / 3], [w / 3, -h / 3], [w / 3, (2 * h) / 3]];
  return o.flip ? pts.map(([x, y]): V2 => [-x, y]).reverse() : pts;
}

/** Centroidal moment of inertia of a uniform plate of this outline. */
export function plateInertia(o: Outline, m: number): number {
  return o.shape === 'wedge' ? (m * (o.w * o.w + o.h * o.h)) / 18 : (m * (o.w * o.w + o.h * o.h)) / 12;
}

export function toWorld(x: number, y: number, theta: number, pts: V2[]): V2[] {
  const c = Math.cos(theta), s = Math.sin(theta);
  return pts.map(([lx, ly]): V2 => [x + c * lx - s * ly, y + s * lx + c * ly]);
}

/** Is a local point inside a convex CCW polygon (with tolerance)? */
export function insidePolygon(pts: V2[], p: V2, tol = 0): boolean {
  for (let i = 0; i < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[(i + 1) % pts.length];
    const ex = bx - ax, ey = by - ay;
    const L = Math.hypot(ex, ey) || 1;
    // outward normal of a CCW edge is (ey, −ex)/L
    if (((p[0] - ax) * ey - (p[1] - ay) * ex) / L > tol) return false;
  }
  return true;
}

/** Corners, edge midpoints and quarter points, for snapping attachments. */
export function snapPoints(pts: V2[]): { p: V2; label: string }[] {
  const out: { p: V2; label: string }[] = [];
  pts.forEach((a, i) => {
    const b = pts[(i + 1) % pts.length];
    out.push({ p: a, label: 'corner' });
    for (const t of [0.25, 0.5, 0.75]) out.push({ p: [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], label: t === 0.5 ? 'edge middle' : 'edge' });
  });
  return out;
}

/**
 * Right-triangle ramp whose slope runs between two world points: the base is
 * horizontal at the lower point and the vertical side stands under the
 * higher one. Returns the centroid pose and size.
 */
export function rampFromSlope(a: V2, b: V2): { x: number; y: number; w: number; h: number; flip: boolean } | null {
  const [lo, hi] = a[1] <= b[1] ? [a, b] : [b, a];
  const w = Math.abs(hi[0] - lo[0]), h = hi[1] - lo[1];
  if (w < 0.02 || h < 0.02) return null;
  const flip = hi[0] < lo[0];
  const tri: V2[] = [lo, [hi[0], lo[1]], hi];
  return { x: (tri[0][0] + tri[1][0] + tri[2][0]) / 3, y: (tri[0][1] + tri[1][1] + tri[2][1]) / 3, w, h, flip };
}

/** World position of a wedge's low end (where the slope meets the base). */
export function wedgeLowEnd(o: Outline & { x: number; y: number; theta: number }): V2 {
  const lx = o.flip ? (2 * o.w) / 3 : (-2 * o.w) / 3, ly = -o.h / 3;
  const c = Math.cos(o.theta), s = Math.sin(o.theta);
  return [o.x + c * lx - s * ly, o.y + s * lx + c * ly];
}

/** Centroid of a wedge of size w × h whose low end sits at `low`. */
export function wedgeFromLowEnd(low: V2, w: number, h: number, flip: boolean, theta: number): { x: number; y: number } {
  const lx = flip ? (2 * w) / 3 : (-2 * w) / 3, ly = -h / 3;
  const c = Math.cos(theta), s = Math.sin(theta);
  return { x: low[0] - (c * lx - s * ly), y: low[1] - (s * lx + c * ly) };
}
