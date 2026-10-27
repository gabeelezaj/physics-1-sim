import { describe, expect, it } from 'vitest';
import { Simulation } from '../src/engine/sim';
import { ground, point, rect } from '../src/model/factory';
import { rampFromSlope } from '../src/model/shapes';
import { emptyScene, type Scene } from '../src/model/types';

const g = 9.81;
const deg = (d: number) => (d * Math.PI) / 180;

/** Point on a slope from (0, 0) rising right at angle th, at distance s along it. */
const onSlope = (th: number, s: number, lift: number): [number, number] =>
  [s * Math.cos(th) - lift * Math.sin(th), s * Math.sin(th) + lift * Math.cos(th)];

function fixedRamp(th: number, L = 1.6): Scene {
  const s = emptyScene();
  const r = rampFromSlope([0, 0], [L * Math.cos(th), L * Math.sin(th)])!;
  s.elements.push(ground('R1', r.x, r.y, { shape: 'wedge', w: r.w, h: r.h, flip: r.flip, theta: 0 }));
  return s;
}

/** Acceleration along the slope (downhill positive) measured between t1 and t2. */
function slopeAccel(sim: Simulation, th: number, t1: number, t2: number): number {
  const b = sim.sys.bodies[sim.sys.bodies.length - 1];
  const down = (s: Simulation) => -(s.qd[b.idx] * Math.cos(th) + s.qd[b.idx + 1] * Math.sin(th));
  sim.run(t1);
  const v1 = down(sim);
  sim.run(t2 - t1);
  return (down(sim) - v1) / (t2 - t1);
}

describe('ramps', () => {
  it('a block slides down a fixed frictionless ramp at g sin θ', () => {
    const th = deg(30);
    const s = fixedRamp(th);
    const [x, y] = onSlope(th, 1.3, 0.05);
    s.elements.push(rect('B', x, y, 1, 0.2, 0.1, { theta: th }));
    const sim = new Simulation(s);
    expect(sim.sys.warnings).toEqual([]);
    const a = slopeAccel(sim, th, 0.1, 0.5);
    expect(Math.abs(a - g * Math.sin(th)) / (g * Math.sin(th))).toBeLessThan(0.02);
    const b = sim.sys.bodies[0];
    expect(Math.abs(sim.q[b.idx + 2] - th)).toBeLessThan(0.02); // stays flat on the slope
  });

  it('a ball slides (no friction, so no rolling) at g sin θ on a steeper ramp', () => {
    const th = deg(45);
    const s = fixedRamp(th);
    const [x, y] = onSlope(th, 1.3, 0.05);
    s.elements.push(point('m', x, y, 0.5, { radius: 0.05 }));
    const a = slopeAccel(new Simulation(s), th, 0.1, 0.5);
    expect(Math.abs(a - g * Math.sin(th)) / (g * Math.sin(th))).toBeLessThan(0.02);
  });

  it('block on a sliding wedge: A = m g sinθ cosθ / (M + m sin²θ), horizontal momentum conserved', () => {
    const th = deg(30), M = 3, m = 1;
    const s = emptyScene();
    s.elements.push(ground('G1', 0, -0.06, { w: 6, h: 0.12 }));
    const r = rampFromSlope([0, 0], [1.4 * Math.cos(th), 1.4 * Math.sin(th)])!;
    const wedge = rect('W', r.x, r.y, M, r.w, r.h, { shape: 'wedge', flip: r.flip });
    const [x, y] = onSlope(th, 1.15, 0.05);
    s.elements.push(wedge, rect('B', x, y, m, 0.2, 0.1, { theta: th }));
    const sim = new Simulation(s);
    expect(sim.sys.warnings).toEqual([]);
    const [W, B] = sim.sys.bodies;
    sim.run(0.1);
    const v1 = sim.qd[W.idx];
    sim.run(0.4);
    const A = (sim.qd[W.idx] - v1) / 0.4;
    const exact = (m * g * Math.sin(th) * Math.cos(th)) / (M + m * Math.sin(th) ** 2);
    expect(Math.abs(A - exact) / exact).toBeLessThan(0.03);
    expect(Math.abs(M * sim.qd[W.idx] + m * sim.qd[B.idx])).toBeLessThan(1e-6);
    expect(Math.abs(sim.q[W.idx + 2])).toBeLessThan(0.01); // the wedge doesn't rock
  });

  it('wedge geometry: centroid origin and inertia m(w² + h²)/18', () => {
    const s = emptyScene();
    s.elements.push(rect('W', 0, 0, 2, 0.9, 0.6, { shape: 'wedge' }));
    const sim = new Simulation(s);
    expect(sim.sys.bodies[0].I).toBeCloseTo((2 * (0.81 + 0.36)) / 18, 12);
  });
});
