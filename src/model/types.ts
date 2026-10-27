// Scene schema: the single source of truth that the editor mutates, undo/redo
// snapshots, and save/load serializes. All quantities are SI (m, kg, s, rad).
// y points up; angles are counter-clockwise.

export type Vec2 = [number, number];

/**
 * A point on a body. `body` is the id of a body or ground element and `p` is
 * in that body's local frame. `body: null` means a fixed point in the world,
 * with `p` in world coordinates.
 */
export interface Attach {
  body: string | null;
  p: Vec2;
}

interface Base {
  id: string;
  name: string;
}

export interface GroundEl extends Base {
  kind: 'ground';
  x: number;
  y: number;
  theta: number;
  w: number;
  h: number;
}

export interface PointEl extends Base {
  /** Takes part in collisions (absent = yes). */
  collide?: boolean;
  kind: 'point';
  x: number;
  y: number;
  vx: number;
  vy: number;
  m: number;
  radius: number; // display only
}

export interface RectEl extends Base {
  /** Takes part in collisions (absent = yes). */
  collide?: boolean;
  kind: 'rect';
  x: number;
  y: number;
  theta: number;
  vx: number;
  vy: number;
  omega: number;
  m: number;
  w: number;
  h: number;
  /** Moment of inertia about the centroid; null = uniform plate m(w²+h²)/12. */
  I: number | null;
}

export interface PulleyEl extends Base {
  /** Takes part in collisions (absent = yes). */
  collide?: boolean;
  kind: 'pulley';
  x: number;
  y: number;
  theta: number;
  vx: number;
  vy: number;
  omega: number;
  m: number;
  r: number;
  /** Moment of inertia about the axle; null = uniform disc m r²/2. */
  I: number | null;
  /** Axle pinned to the world at (x, y). */
  fixedAxle: boolean;
}

export interface RodEl extends Base {
  kind: 'rod';
  a: Attach;
  b: Attach;
  length: number;
}

export interface PinEl extends Base {
  kind: 'pin';
  a: Attach;
  b: Attach;
}

export interface SliderEl extends Base {
  kind: 'slider';
  a: Attach;
  /** A point on the (world-fixed) track and the track direction. */
  origin: Vec2;
  angle: number;
  /** Coulomb friction coefficient against the track normal force. */
  mu: number;
  /** Also keep the body's angle fixed (prismatic joint, e.g. a cart). */
  lockRotation: boolean;
  /** Half-length of the drawn track (display only). */
  extent: number;
}

export interface SpringEl extends Base {
  kind: 'spring';
  a: Attach;
  b: Attach;
  k: number;
  L0: number;
  c: number;
}

export interface TSpringEl extends Base {
  kind: 'tspring';
  pin: string;
  k: number;
  /** Rest value of θ_a − θ_b for the pin's two bodies. */
  theta0: number;
  c: number;
}

export interface RopeRoute {
  pulley: string;
  /** Wrap direction: +1 counter-clockwise, −1 clockwise. */
  dir: 1 | -1;
}

export interface RopeEl extends Base {
  kind: 'rope';
  a: Attach;
  b: Attach;
  route: RopeRoute[];
  length: number;
}

export type BodyEl = GroundEl | PointEl | RectEl | PulleyEl;
export type DynBodyEl = PointEl | RectEl | PulleyEl;
export type ConnectorEl = RodEl | PinEl | SliderEl | SpringEl | TSpringEl | RopeEl;
export type El = BodyEl | ConnectorEl;
export type ElKind = El['kind'];

export interface SimSettings {
  dt: number;
  /** Baumgarte stabilization gains (s⁻¹). */
  alpha: number;
  beta: number;
  /** Bodies collide with ground pieces (absent in older saves = on). */
  contact?: boolean;
  /** Coefficient of restitution for ground contact, 0–1. */
  restitution?: number;
}

export interface Scene {
  version: 1;
  title: string;
  gravity: { on: boolean; g: number };
  settings: SimSettings;
  elements: El[];
}

export const isBody = (e: El): e is BodyEl =>
  e.kind === 'ground' || e.kind === 'point' || e.kind === 'rect' || e.kind === 'pulley';

export const isDynBody = (e: El): e is DynBodyEl =>
  e.kind === 'point' || e.kind === 'rect' || e.kind === 'pulley';

export function emptyScene(): Scene {
  return {
    version: 1,
    title: 'Untitled',
    gravity: { on: true, g: 9.81 },
    settings: { dt: 0.001, alpha: 10, beta: 10, contact: true, restitution: 0.5 },
    elements: [],
  };
}
