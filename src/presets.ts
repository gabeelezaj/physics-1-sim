// The three preset "nightmare" problems.

import { RopeC } from './engine/constraints';
import { System } from './engine/system';
import { at, ground, pin, point, pulley, rect, rod, rope, slider, spring, tspring } from './model/factory';
import { emptyScene, type Attach, type GroundEl, type RectEl, type Scene } from './model/types';

export interface Preset {
  id: string;
  title: string;
  build: () => Scene;
  /** Has a stable equilibrium with all natural frequencies positive. */
  modal?: boolean;
}

const deg = (d: number) => (d * Math.PI) / 180;

/** Attachment on a ground element given a world point. */
function onGround(g: GroundEl, wx: number, wy: number): Attach {
  const c = Math.cos(g.theta), s = Math.sin(g.theta);
  const dx = wx - g.x, dy = wy - g.y;
  return { body: g.id, p: [c * dx + s * dy, -s * dx + c * dy] };
}

/** World position of a local point on a rect at its scene pose. */
function worldOf(b: RectEl, lx: number, ly: number): [number, number] {
  const c = Math.cos(b.theta), s = Math.sin(b.theta);
  return [b.x + c * lx - s * ly, b.y + s * lx + c * ly];
}

/** Rect whose local point (lx, 0) sits at world `p` with angle `theta`. */
function barFrom(name: string, p: [number, number], theta: number, lx: number, m: number, L: number, h: number): RectEl {
  return rect(name, p[0] - lx * Math.cos(theta), p[1] - lx * Math.sin(theta), m, L, h, { theta });
}

const dist = (a: [number, number], b: [number, number]) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/** Set every rope's length to its current geometric path (taut). */
function tautenRopes(scene: Scene): void {
  const sys = new System(scene);
  const { q } = sys.initialState();
  for (const c of sys.cons) {
    if (!(c instanceof RopeC)) continue;
    const el = scene.elements.find((e) => e.id === c.el);
    if (el && el.kind === 'rope') el.length = c.geometry(q).total;
  }
}

/**
 * (1) Cart on a frictionless floor carrying two tilted bars on pivots. The bars
 * have torsional springs at their pivots and are coupled at the top by a
 * spring; the cart is tied to both walls.
 */
export function cartWithTwoBars(): Scene {
  const s = emptyScene();
  s.title = 'Cart with two tilted bars';
  const floor = ground('G1', 0, -0.06, { w: 4, h: 0.12 });
  const wallL = ground('G2', -1.86, 0.5, { theta: -Math.PI / 2, w: 1.2, h: 0.12 });
  const wallR = ground('G3', 1.86, 0.5, { theta: Math.PI / 2, w: 1.2, h: 0.12 });
  const cartH = 0.26, gap = 0.04;
  const cart = rect('C', 0, gap + cartH / 2, 4, 1.4, cartH);
  const top = cartH / 2;
  const bar1 = barFrom('B1', worldOf(cart, -0.4, top), deg(105), -0.5, 1, 1.0, 0.06);
  const bar2 = barFrom('B2', worldOf(cart, 0.4, top), deg(75), -0.5, 1, 1.0, 0.06);
  bar1.omega = 1.2;
  const o1 = pin('O1', at(bar1.id, -0.5, 0), at(cart.id, -0.4, top));
  const o2 = pin('O2', at(bar2.id, -0.5, 0), at(cart.id, 0.4, top));
  const kt1 = tspring('kt1', o1.id, 20, deg(105));
  const kt2 = tspring('kt2', o2.id, 20, deg(75));
  const t1 = worldOf(bar1, 0.5, 0), t2 = worldOf(bar2, 0.5, 0);
  const kc = spring('kc', at(bar1.id, 0.5, 0), at(bar2.id, 0.5, 0), 30, dist(t1, t2));
  const cy = cart.y;
  const kL = spring('k1', onGround(wallL, -1.8, cy), at(cart.id, -0.7, 0), 40, 1.1);
  const kR = spring('k2', at(cart.id, 0.7, 0), onGround(wallR, 1.8, cy), 40, 1.1);
  const track = slider('S1', at(cart.id, 0, -cartH / 2 - gap), [0, 0], 0, { lockRotation: true, extent: 1.75 });
  s.elements.push(floor, wallL, wallR, cart, bar1, bar2, o1, o2, kt1, kt2, kc, kL, kR, track);
  return s;
}

/**
 * (2) A pendulum on a pivot whose tip is pulled both ways by ropes running
 * over two massive pulleys; each rope carries a hanger from which a mass hangs
 * on a spring.
 */
export function pendulumAndPulleys(): Scene {
  const s = emptyScene();
  s.title = 'Pendulum with masses on springs over massive pulleys';
  const ceil = ground('G1', 0, 1.06, { theta: Math.PI, w: 3.2, h: 0.12 });
  const bar = barFrom('B1', [0, 1.0], deg(-90), -0.45, 1.5, 0.9, 0.07);
  bar.omega = 0.6;
  const o = pin('O1', at(bar.id, -0.45, 0), onGround(ceil, 0, 1.0));
  const tip = worldOf(bar, 0.45, 0); // (0, 0.1)
  const r = 0.1;
  const pL = pulley('P1', -0.9, tip[1] - r, 2, r);
  const pR = pulley('P2', 0.9, tip[1] - r, 2, r);
  const hL = point('h1', -0.9 - r, -0.5, 0.2, { radius: 0.035 });
  const hR = point('h2', 0.9 + r, -0.5, 0.2, { radius: 0.035 });
  const bh = 0.15;
  const mL = rect('m1', -0.9 - r, -1.1, 1.5, 0.24, bh);
  const mR = rect('m2', 0.9 + r, -1.1, 1.5, 0.24, bh);
  const g = 9.81;
  const drawn = 0.6 - bh / 2;
  const kL = spring('k1', at(hL.id, 0, 0), at(mL.id, 0, bh / 2), 60, drawn - (1.5 * g) / 60);
  const kR = spring('k2', at(hR.id, 0, 0), at(mR.id, 0, bh / 2), 90, drawn - (1.5 * g) / 90);
  const ropeL = rope('T1', at(bar.id, 0.45, 0), at(hL.id, 0, 0), [{ pulley: pL.id, dir: 1 }], 1);
  const ropeR = rope('T2', at(bar.id, 0.45, 0), at(hR.id, 0, 0), [{ pulley: pR.id, dir: -1 }], 1);
  s.elements.push(ceil, bar, o, pL, pR, hL, hR, mL, mR, kL, kR, ropeL, ropeR);
  tautenRopes(s);
  return s;
}

/**
 * (3) A heavy central body hung from a pivot, coupled by springs to two side
 * blocks on frictionless guides, each block carrying a small pendulum.
 */
export function centralMassWithSidePendulums(): Scene {
  const s = emptyScene();
  s.title = 'Central pendulum coupled to blocks with pendulums';
  const ceil = ground('G1', 0, 1.26, { theta: Math.PI, w: 1.0, h: 0.12 });
  const L = 0.9;
  const mid = barFrom('C', [0, 1.2], deg(-90), -L / 2, 3, L, 0.3);
  const o = pin('O1', at(mid.id, -L / 2, 0), onGround(ceil, 0, 1.2));
  const yb = 0.35;
  const lx = mid.y - yb; // local x (pointing down) at the block height
  const bw = 0.36, bhh = 0.22;
  const bL = rect('B1', -1.0, yb, 2, bw, bhh);
  const bR = rect('B2', 1.0, yb, 2, bw, bhh);
  const sL = slider('S1', at(bL.id, 0, 0), [-1.0, yb], 0, { lockRotation: true, extent: 0.7 });
  const sR = slider('S2', at(bR.id, 0, 0), [1.0, yb], 0, { lockRotation: true, extent: 0.7 });
  const kL = spring('k1', at(bL.id, bw / 2, 0), at(mid.id, lx, -0.15), 80, 1.0 - bw / 2 - 0.15);
  const kR = spring('k2', at(mid.id, lx, 0.15), at(bR.id, -bw / 2, 0), 80, 1.0 - bw / 2 - 0.15);
  const l = 0.4;
  const mL = point('m1', -1.0, yb - bhh / 2 - l, 0.5, { radius: 0.05, vx: 0.6 });
  const mR = point('m2', 1.0, yb - bhh / 2 - l, 0.5, { radius: 0.05 });
  const rL = rod('R1', at(bL.id, 0, -bhh / 2), at(mL.id, 0, 0), l);
  const rR = rod('R2', at(bR.id, 0, -bhh / 2), at(mR.id, 0, 0), l);
  s.elements.push(ceil, mid, o, bL, bR, sL, sR, kL, kR, mL, mR, rL, rR);
  return s;
}

/**
 * (4) Carts on a frictionless track: a heavy cart runs into two lighter ones;
 * the last is held off the right wall by a bumper spring.
 */
export function collidingCarts(): Scene {
  const s = emptyScene();
  s.title = 'Colliding carts';
  s.settings.restitution = 0.8;
  const floor = ground('G1', 0, -0.06, { w: 4.8, h: 0.12 });
  const wallL = ground('G2', -2.36, 0.4, { theta: -Math.PI / 2, w: 0.9, h: 0.12 });
  const wallR = ground('G3', 2.36, 0.4, { theta: Math.PI / 2, w: 0.9, h: 0.12 });
  const h = 0.24, gap = 0.04, y = gap + h / 2;
  const A = rect('A', -1.5, y, 2, 0.6, h, { vx: 2.5 });
  const B = rect('B', 0, y, 1, 0.45, h);
  const C = rect('C', 0.8, y, 1, 0.45, h);
  const track = (b: RectEl, n: string) => slider(n, at(b.id, 0, -h / 2 - gap), [b.x, 0], 0, { lockRotation: true, extent: 0.4 });
  const bumper = spring('k1', at(C.id, 0.225, 0), onGround(wallR, 2.3, y), 150, 2.3 - 1.025);
  s.elements.push(floor, wallL, wallR, A, B, C, track(A, 'S1'), track(B, 'S2'), track(C, 'S3'), bumper);
  return s;
}

export const PRESETS: Preset[] = [
  { id: 'cart-bars', title: '1 · Cart with two tilted bars', build: cartWithTwoBars, modal: true },
  { id: 'pendulum-pulleys', title: '2 · Pendulum + masses over pulleys', build: pendulumAndPulleys, modal: true },
  { id: 'central-side', title: '3 · Central mass + side pendulums', build: centralMassWithSidePendulums, modal: true },
  { id: 'colliding-carts', title: '4 · Colliding carts', build: collidingCarts },
];
