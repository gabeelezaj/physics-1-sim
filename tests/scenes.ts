// Scene builders shared by the verification tests.

import { at, ground, pin, point, pulley, rect, rod, rope, slider, spring } from '../src/model/factory';
import { emptyScene, type Scene } from '../src/model/types';

export function simplePendulum(L: number, theta0: number, m = 1, g = 9.81): Scene {
  const s = emptyScene();
  s.gravity.g = g;
  const bob = point('m1', L * Math.sin(theta0), -L * Math.cos(theta0), m);
  s.elements.push(bob, rod('R1', at(null, 0, 0), at(bob.id, 0, 0), L));
  return s;
}

/** Wall — k — m — k — m — k — wall on a frictionless horizontal track. */
export function twoMassThreeSprings(m: number, k: number, x1 = -0.5, x2 = 0.5): Scene {
  const s = emptyScene();
  const wl = ground('G1', -1.56, 0, { theta: -Math.PI / 2, w: 0.6 });
  const wr = ground('G2', 1.56, 0, { theta: Math.PI / 2, w: 0.6 });
  const a = point('m1', x1, 0, m);
  const b = point('m2', x2, 0, m);
  s.elements.push(
    wl, wr, a, b,
    slider('S1', at(a.id, 0, 0), [0, 0], 0),
    slider('S2', at(b.id, 0, 0), [0, 0], 0),
    spring('k1', at(null, -1.5, 0), at(a.id, 0, 0), k, 1),
    spring('k2', at(a.id, 0, 0), at(b.id, 0, 0), k, 1),
    spring('k3', at(b.id, 0, 0), at(null, 1.5, 0), k, 1),
  );
  return s;
}

/** Atwood machine over a massive, fixed-axle pulley (uniform disc). */
export function atwood(m1: number, m2: number, M: number, r: number): Scene {
  const s = emptyScene();
  const p = pulley('P1', 0, 0, M, r);
  const a = point('m1', -r, -1, m1);
  const b = point('m2', r, -1, m2);
  s.elements.push(p, a, b, rope('T1', at(a.id, 0, 0), at(b.id, 0, 0), [{ pulley: p.id, dir: -1 }], 2 + Math.PI * r));
  return s;
}

/** Two rigid bars in a chain, pinned to the world at the top. */
export function doubleBarPendulum(theta1: number, theta2: number): Scene {
  const s = emptyScene();
  const L = 0.8;
  const c1 = [(L / 2) * Math.sin(theta1), -(L / 2) * Math.cos(theta1)];
  const e1 = [L * Math.sin(theta1), -L * Math.cos(theta1)];
  const c2 = [e1[0] + (L / 2) * Math.sin(theta2), e1[1] - (L / 2) * Math.cos(theta2)];
  const b1 = rect('B1', c1[0], c1[1], 1, L, 0.06, { theta: theta1 - Math.PI / 2 });
  const b2 = rect('B2', c2[0], c2[1], 1, L, 0.06, { theta: theta2 - Math.PI / 2 });
  s.elements.push(
    b1, b2,
    pin('O1', at(b1.id, L / 2, 0), at(null, 0, 0)),
    pin('O2', at(b1.id, -L / 2, 0), at(b2.id, L / 2, 0)),
  );
  return s;
}
