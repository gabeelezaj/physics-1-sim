// Preset scenes: the original "nightmare" problems plus showcase machines.

import { RopeC } from './engine/constraints';
import { System } from './engine/system';
import { at, force, ground, pin, point, pulley, rect, rod, rope, slider, spring, tspring } from './model/factory';
import { groundFromEnds } from './model/ground';
import { rampFromSlope } from './model/shapes';
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

/**
 * (5) The classic: a block released on a frictionless wedge that is itself
 * free to slide on a frictionless floor. Wedge acceleration
 * A = m g sinθ cosθ / (M + m sin²θ). The block reaches the floor heading
 * left and runs up a fixed ramp; the wedge recoils right.
 */
export function blockOnSlidingWedge(): Scene {
  const s = emptyScene();
  s.title = 'Block on a sliding wedge';
  s.settings.restitution = 0.4;
  const th = deg(30);
  const floor = ground('G1', 0, -0.06, { w: 5.6, h: 0.12 });
  const wallL = ground('G2', -2.76, 0.45, { theta: -Math.PI / 2, w: 1.0, h: 0.12 });
  const wallR = ground('G3', 2.76, 0.45, { theta: Math.PI / 2, w: 1.0, h: 0.12 });
  const L = 1.4;
  const r = rampFromSlope([-0.6, 0], [-0.6 + L * Math.cos(th), L * Math.sin(th)])!;
  const wedge = rect('W', r.x, r.y, 3, r.w, r.h, { shape: 'wedge', flip: r.flip });
  const d = 1.15, lift = 0.06;
  const block = rect('B', -0.6 + d * Math.cos(th) - lift * Math.sin(th), d * Math.sin(th) + lift * Math.cos(th), 1, 0.24, 0.12, { theta: th });
  const ramp = rampFromSlope([-0.95, 0], [-2.35, 0.55])!;
  const fixedRamp = ground('G4', ramp.x, ramp.y, { shape: 'wedge', w: ramp.w, h: ramp.h, flip: ramp.flip });
  s.elements.push(floor, wallL, wallR, fixedRamp, wedge, block);
  return s;
}

/**
 * (6) Skate park: a rough floor between a quarter-pipe and a hump. A wheel
 * rolls into the quarter-pipe and back; a block is pushed along by a constant
 * force until it meets the hump.
 */
export function skatePark(): Scene {
  const s = emptyScene();
  s.title = 'Skate park';
  s.settings.restitution = 0.3;
  const mu = 0.4;
  const floor = ground('G1', 0, -0.06, { w: 5, h: 0.12, mu });
  const pipe = ground('G2', 1.9, 0.6, { shape: 'curve', profile: 'concave', w: 1.2, h: 1.2, mu });
  // two convex quarters meeting at the top make a hump
  const hump = ground('G3', -1.1, 0.2, { shape: 'curve', profile: 'convex', w: 0.6, h: 0.4, mu });
  const hump2 = ground('G4', -0.5, 0.2, { shape: 'curve', profile: 'convex', w: 0.6, h: 0.4, mu, flip: true });
  const wallL = ground('G5', -2.56, 0.6, { theta: -Math.PI / 2, w: 1.3, h: 0.12 });
  const wheel = pulley('D1', 0.3, 0.15, 1, 0.15, { fixedAxle: false, vx: 3, omega: -20 });
  const block = rect('B1', -2.1, 0.1, 1, 0.3, 0.2);
  const push = force('F1', at(block.id, -0.15, 0), 6, 0);
  s.elements.push(floor, pipe, hump, hump2, wallL, wheel, block, push);
  return s;
}

/**
 * (7) Newton's cradle: five steel balls on rods, with perfectly elastic
 * contact. The first ball is pulled out 40° and let go.
 */
export function newtonsCradle(): Scene {
  const s = emptyScene();
  s.title = "Newton's cradle";
  s.settings.restitution = 1;
  const top = 1.6, L = 1.1, r = 0.1, n = 5;
  const beam = ground('G1', 0, top + 0.06, { theta: Math.PI, w: 1.6, h: 0.12 });
  s.elements.push(beam);
  for (let i = 0; i < n; i++) {
    // a hair of clearance so each impact passes cleanly down the line
    const px = (i - (n - 1) / 2) * (2 * r + 0.002);
    const th = i === 0 ? deg(40) : 0;
    const ball = point(`m${i + 1}`, px - L * Math.sin(th), top - L * Math.cos(th), 1, { radius: r });
    s.elements.push(ball, rod(`R${i + 1}`, onGround(beam, px, top), at(ball.id, 0, 0), L));
  }
  return s;
}

/**
 * (8) Domino amplification: each domino is 1.25× the size of the one before,
 * so a nudge to a 12 cm domino ends up toppling one well over a metre tall.
 */
export function dominoes(): Scene {
  const s = emptyScene();
  s.title = 'Domino amplification';
  s.elements.push(ground('G1', 1.6, -0.06, { w: 5.6, h: 0.12, mu: 0.6 }));
  let x = -0.9, h = 0.12;
  for (let i = 0; i < 11; i++) {
    const w = h / 6.5;
    const d = rect(`D${i + 1}`, x, h / 2, 2.4 * w * h * 1.1, w, h);
    if (i === 0) {
      // the first one starts tipped 10° onto its right-hand corner
      const th = deg(-10), cx = x + w / 2;
      d.theta = th;
      d.x = cx - (Math.cos(th) * w / 2 + Math.sin(th) * h / 2);
      d.y = -(Math.sin(th) * w / 2 - Math.cos(th) * h / 2);
    }
    s.elements.push(d);
    x += 0.55 * h;
    h *= 1.25;
  }
  return s;
}

/**
 * (9) Double pendulum released from horizontal: chaotic motion (turn on
 * motion trails in the View menu to see the path of the lower bob).
 */
export function doublePendulum(): Scene {
  const s = emptyScene();
  s.title = 'Chaotic double pendulum';
  const ceil = ground('G1', 0, 1.26, { theta: Math.PI, w: 0.6, h: 0.12 });
  const a = barFrom('B1', [0, 1.2], deg(0), -0.4, 1, 0.8, 0.05);
  const b = barFrom('B2', [0.8, 1.2], deg(30), -0.35, 0.7, 0.7, 0.05);
  s.elements.push(ceil, a, b, pin('O1', at(a.id, -0.4, 0), onGround(ceil, 0, 1.2)), pin('O2', at(a.id, 0.4, 0), at(b.id, -0.35, 0)));
  return s;
}

/** Fixed plank whose surface runs from a to b, with free space on the `side`. */
function plank(name: string, a: [number, number], b: [number, number], side: [number, number], mu = 0, t = 0.06): GroundEl {
  return ground(name, 0, 0, { ...groundFromEnds(a, b, side, t), mu });
}

/**
 * (10) Rube Goldberg machine, left to right and top to bottom:
 *  1. a ball rolls down a rough ramp and along a shelf,
 *  2. into a row of dominoes,
 *  3. the last domino knocks a marble off the end of the shelf,
 *  4. the marble zig-zags down three planks, bouncing off two walls,
 *  5. and rolls along a ledge into a heavy weight, pushing it off the edge;
 *  6. the falling weight pulls a rope over a pulley and drags a cart along a table,
 *  7. the cart slams into a stop and the ball it carries flies on (inertia),
 *  8. into a hanging basket, which now outweighs its counterweight,
 *  9. the basket sinks, the counterweight rises over a big pulley
 * 10. and rings the bell.
 */
export function rubeGoldberg(): Scene {
  const s = emptyScene();
  s.title = 'Rube Goldberg machine';
  s.settings.restitution = 0.3;
  const up: [number, number] = [0, 1], left: [number, number] = [-1, 0], right: [number, number] = [1, 0];
  const els = s.elements;

  els.push(ground('G0', -1.4, -0.06, { w: 9.6, h: 0.12, mu: 0.5 }));

  // 1–3: ramp, shelf, dominoes, marble
  const r1 = rampFromSlope([-4.6, 3.0], [-6.0, 3.45])!;
  els.push(ground('R1', r1.x, r1.y, { shape: 'wedge', w: r1.w, h: r1.h, flip: r1.flip, mu: 0.6 }));
  els.push(plank('S1', [-4.6, 3.0], [-2.0, 3.0], up, 0.6, 0.1));
  const dx = 1.4, dy = -0.45, L = Math.hypot(dx, dy), rA = 0.09;
  const along = 0.15;
  els.push(pulley('A', -6.0 + (along * dx) / L + (rA * -dy) / L, 3.45 + (along * dy) / L + (rA * dx) / L, 0.4, rA, { fixedAxle: false }));
  for (let i = 0; i < 6; i++) els.push(rect(`D${i + 1}`, -3.5 + 0.2 * i, 3.15, 0.12, 0.04, 0.3));
  els.push(pulley('M', -2.2, 3.07, 0.25, 0.07, { fixedAxle: false }));

  // 4: marble run
  els.push(plank('Z1', [-2.35, 2.55], [-0.6, 2.2], up, 0.4));
  els.push(plank('W1', [-0.42, 1.75], [-0.42, 2.75], left));
  els.push(plank('Z2', [-0.42, 1.95], [-2.1, 1.6], up, 0.4));
  els.push(plank('W2', [-2.3, 1.2], [-2.3, 2.15], right));
  els.push(plank('Z3', [-2.3, 1.4], [0.0, 1.0], up, 0.4));

  // 5–7: ledge with the weight, table with the cart and its cargo, rope over a pulley
  els.push(plank('S2', [0.0, 1.0], [1.6, 1.0], up, 0, 0.1));
  els.push(plank('T1', [-0.2, 1.9], [1.25, 1.9], up, 0, 0.1));
  els.push(ground('Stop', 1.28, 1.99, { w: 0.06, h: 0.24 }));
  const W = rect('W', 1.5, 1.075, 1.0, 0.15, 0.15);
  const K = rect('K', 0.3, 2.05, 0.5, 0.4, 0.3);
  const C = point('C', 0.35, 2.26, 0.3, { radius: 0.06, mu: 0.9 });
  const P1 = pulley('P1', 1.42, 1.9, 0.2, 0.08);
  els.push(W, K, C, P1);
  els.push(rope('T1r', at(W.id, 0, 0.075), at(K.id, 0.2, -0.07), [{ pulley: P1.id, dir: 1 }], 1));
  els.push(slider('Tk', at(K.id, 0, -0.15), [0.3, 1.9], 0, { lockRotation: true, extent: 0.7 }));

  // 8–10: hanging basket and counterweight over a big pulley, and the bell
  els.push(ground('Wall', 1.8, 0.45, { w: 0.06, h: 0.9 })); // keeps the swinging weight away from the basket
  const bx = 2.22;
  const P2 = pulley('P2', bx + 0.25, 3.0, 0.5, 0.25);
  const hook = point('h', bx, 2.1, 0.05, { radius: 0.03, collide: false });
  const E = rect('E', bx, 1.4, 0.3, 0.6, 0.05);
  els.push(P2, hook, E);
  // basket walls welded to the floor of the basket (two pins each)
  for (const sx of [-1, 1]) {
    const w = rect(sx < 0 ? 'E1' : 'E2', bx + sx * 0.28, 1.505, 0.05, 0.04, 0.16);
    els.push(w, pin(`${w.name}a`, at(w.id, 0, -0.08), at(E.id, sx * 0.28, 0.025)), pin(`${w.name}b`, at(w.id, 0, 0.08), at(E.id, sx * 0.28, 0.185)));
  }
  const Q = rect('Q', bx + 0.5, 0.1, 0.5, 0.2, 0.2);
  const bellX = bx + 0.58;
  const bell = point('Bell', bellX, 1.4, 0.3, { radius: 0.08 });
  const bellTop = ground('G9', bellX, 2.26, { theta: Math.PI, w: 0.3, h: 0.12 });
  els.push(Q, bellTop, bell);
  const sling = Math.hypot(0.3, 0.675);
  els.push(rod('R1h', at(hook.id, 0, 0), at(E.id, -0.3, 0.025), sling), rod('R2h', at(hook.id, 0, 0), at(E.id, 0.3, 0.025), sling));
  els.push(rope('T2r', at(hook.id, 0, 0), at(Q.id, 0, 0.1), [{ pulley: P2.id, dir: -1 }], 1));
  els.push(rod('Rb', onGround(bellTop, bellX, 2.2), at(bell.id, 0, 0), 0.8));
  tautenRopes(s);
  return s;
}

export const PRESETS: Preset[] = [
  { id: 'cart-bars', title: '1 · Cart with two tilted bars', build: cartWithTwoBars, modal: true },
  { id: 'pendulum-pulleys', title: '2 · Pendulum + masses over pulleys', build: pendulumAndPulleys, modal: true },
  { id: 'central-side', title: '3 · Central mass + side pendulums', build: centralMassWithSidePendulums, modal: true },
  { id: 'colliding-carts', title: '4 · Colliding carts', build: collidingCarts },
  { id: 'sliding-wedge', title: '5 · Block on a sliding wedge', build: blockOnSlidingWedge },
  { id: 'skate-park', title: '6 · Skate park (curves, friction, rolling)', build: skatePark },
  { id: 'cradle', title: "7 · Newton's cradle", build: newtonsCradle },
  { id: 'dominoes', title: '8 · Domino amplification', build: dominoes },
  { id: 'double-pendulum', title: '9 · Chaotic double pendulum', build: doublePendulum },
  { id: 'rube-goldberg', title: '10 · Rube Goldberg machine', build: rubeGoldberg },
];
