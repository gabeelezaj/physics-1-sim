import { describe, expect, it } from 'vitest';
import { Simulation } from '../src/engine/sim';
import { at, force, ground, point, pulley, rect } from '../src/model/factory';
import { rampFromSlope } from '../src/model/shapes';
import { emptyScene, type Scene } from '../src/model/types';

const g = 9.81, deg = (d: number) => (d * Math.PI) / 180;

/** Fixed incline rising right from the origin, plus a floor, with friction μ. */
function incline(th: number, mu: number): Scene {
  const s = emptyScene();
  const r = rampFromSlope([0, 0], [2 * Math.cos(th), 2 * Math.sin(th)])!;
  s.elements.push(ground('R', r.x, r.y, { shape: 'wedge', w: r.w, h: r.h, flip: r.flip, mu }));
  return s;
}
const onSlope = (th: number, d: number, lift: number) => [d * Math.cos(th) - lift * Math.sin(th), d * Math.sin(th) + lift * Math.cos(th)];
const downSpeed = (sim: Simulation, th: number) => {
  const b = sim.sys.bodies[0];
  return -(sim.qd[b.idx] * Math.cos(th) + sim.qd[b.idx + 1] * Math.sin(th));
};

describe('applied loads', () => {
  it('a constant force accelerates a block on a frictionless floor at F/m', () => {
    const s = emptyScene();
    const b = rect('B', 0, 0.1, 2, 0.4, 0.2);
    s.elements.push(ground('G', 0, -0.06, { w: 6, h: 0.12 }), b, force('P', at(b.id, -0.2, 0), 10, 0));
    const sim = new Simulation(s);
    sim.run(1);
    expect(sim.qd[0]).toBeCloseTo(5, 1);
    expect(Math.abs(sim.driftPercent())).toBeLessThan(0.5);
  });
});

describe('friction on ground surfaces', () => {
  it('a block slides down a rough incline at g(sinθ − μ cosθ)', () => {
    const th = deg(20), mu = 0.2;
    const s = incline(th, mu);
    const [x, y] = onSlope(th, 1.6, 0.05);
    s.elements.push(rect('B', x, y, 1, 0.2, 0.1, { theta: th }));
    const sim = new Simulation(s);
    sim.run(0.1);
    const v1 = downSpeed(sim, th);
    sim.run(0.4);
    const a = (downSpeed(sim, th) - v1) / 0.4;
    const exact = g * (Math.sin(th) - mu * Math.cos(th));
    expect(Math.abs(a - exact) / exact).toBeLessThan(0.05);
    expect(Math.abs(sim.driftPercent())).toBeLessThan(1);
  });

  it('a block stays put when μ > tanθ (creeps < 1 cm/s)', () => {
    const th = deg(20);
    const s = incline(th, 0.6);
    const [x, y] = onSlope(th, 1.6, 0.05);
    s.elements.push(rect('B', x, y, 1, 0.2, 0.1, { theta: th }));
    const sim = new Simulation(s);
    sim.run(1);
    expect(Math.abs(downSpeed(sim, th))).toBeLessThan(0.01);
  });

  it('a disc rolls down at (2/3) g sinθ and spins at v/r', () => {
    const th = deg(20), r = 0.1;
    const s = incline(th, 0.5);
    const [x, y] = onSlope(th, 1.6, r);
    s.elements.push(pulley('D', x, y, 1, r, { fixedAxle: false }));
    const sim = new Simulation(s);
    sim.run(0.1);
    const v1 = downSpeed(sim, th);
    sim.run(0.4);
    const v2 = downSpeed(sim, th);
    const exact = (2 / 3) * g * Math.sin(th);
    expect(Math.abs((v2 - v1) / 0.4 - exact) / exact).toBeLessThan(0.05);
    expect(Math.abs(sim.qd[2]) * r).toBeCloseTo(v2, 1);
  });
});

describe('curved ramp', () => {
  it('a ball released inside a frictionless quarter-pipe reaches the bottom at √(2gΔh)', () => {
    const s = emptyScene();
    const R = 1, r = 0.05;
    // quarter-pipe box: x ∈ [−0.5, 0.5], y ∈ [0, 1]; the arc's centre is at (−0.5, 1)
    s.elements.push(
      ground('G', -2, -0.06, { w: 3, h: 0.12 }),
      ground('C', 0, 0.5, { shape: 'curve', profile: 'concave', w: R, h: R }),
    );
    const phi = deg(60);
    const y0 = 1 - (R - r) * Math.cos(phi);
    s.elements.push(point('m', -0.5 + (R - r) * Math.sin(phi), y0, 1, { radius: r }));
    const sim = new Simulation(s);
    let vmax = 0;
    sim.run(1.5, (sm) => (vmax = Math.max(vmax, Math.hypot(sm.qd[0], sm.qd[1]))));
    const exact = Math.sqrt(2 * g * (y0 - r));
    expect(Math.abs(vmax - exact) / exact).toBeLessThan(0.03);
    expect(Math.abs(sim.driftPercent())).toBeLessThan(1);
  });
});

describe('friction between bodies', () => {
  it('a block sliding on a cart drags it along until they move together (momentum kept)', () => {
    const s = emptyScene();
    // smooth floor; rough contact between the top block and the cart
    const cart = rect('C', 0, 0.1, 3, 1.2, 0.2);
    const top = rect('B', -0.3, 0.25, 1, 0.2, 0.1, { vx: 2, mu: 0.4 });
    s.elements.push(ground('G', 0, -0.06, { w: 8, h: 0.12 }), cart, top);
    const sim = new Simulation(s);
    const [C, B] = sim.sys.bodies;
    // relative speed 2 m/s dies out at μg(1 + m/M) ≈ 5.2 m/s² → about 0.4 s
    sim.run(0.15);
    const aRel = (2 - (sim.qd[B.idx] - sim.qd[C.idx])) / 0.15;
    expect(Math.abs(aRel - 0.4 * g * (1 + 1 / 3)) / (0.4 * g * (4 / 3))).toBeLessThan(0.08);
    sim.run(0.6);
    expect(sim.qd[B.idx]).toBeCloseTo(0.5, 1);
    expect(sim.qd[C.idx]).toBeCloseTo(0.5, 1);
    expect(sim.dissipated).toBeGreaterThan(1.2); // ½·1·2² − ½·4·0.5² = 1.5 J lost to sliding
    expect(Math.abs(sim.driftPercent())).toBeLessThan(1);
  });
});

describe('mouse grab', () => {
  it('pulls a body to the cursor and books the work it does', () => {
    const s = emptyScene();
    s.elements.push(ground('G', 0, -0.06, { w: 8, h: 0.12 }), rect('B', 0, 0.1, 2, 0.4, 0.2));
    const sim = new Simulation(s);
    const b = sim.sys.bodies[0];
    sim.sys.mouse.a = { b, lx: 0.1, ly: 0, angle: 0 };
    for (let i = 0; i < 100; i++) { sim.sys.mouse.tx = 0.1 + i * 0.01; sim.sys.mouse.ty = 0.6; sim.run(0.01); }
    expect(sim.q[b.idx + 1]).toBeGreaterThan(0.3); // lifted off the floor
    sim.sys.mouse.a = null;
    sim.run(1);
    expect(Math.abs(sim.driftPercent())).toBeLessThan(1);
  });
});
