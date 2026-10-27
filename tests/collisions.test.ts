import { describe, expect, it } from 'vitest';
import { Simulation } from '../src/engine/sim';
import { System } from '../src/engine/system';
import { at, ground, point, rect, slider } from '../src/model/factory';
import { emptyScene, type Scene } from '../src/model/types';

/** Two carts on one frictionless horizontal track, A moving right into B. */
function carts(mA: number, vA: number, mB: number, e: number): Scene {
  const s = emptyScene();
  s.gravity.on = false;
  s.settings.restitution = e;
  const A = rect('A', -0.5, 0, mA, 0.4, 0.2, { vx: vA });
  const B = rect('B', 0.5, 0, mB, 0.4, 0.2);
  s.elements.push(A, B, slider('S1', at(A.id, 0, 0), [0, 0], 0), slider('S2', at(B.id, 0, 0), [0, 0], 0));
  return s;
}

const vx = (sim: Simulation, name: string) => sim.qd[sim.sys.bodies.find((b) => b.name === name)!.idx];

describe('body–body collisions', () => {
  it('equal carts, e = 1: velocities swap and momentum is conserved', () => {
    const sim = new Simulation(carts(1, 2, 1, 1));
    let minGap = Infinity;
    const [a, b] = sim.sys.bodies;
    sim.run(1, (s) => (minGap = Math.min(minGap, s.q[b.idx] - s.q[a.idx] - 0.4)));
    expect(vx(sim, 'A')).toBeCloseTo(0, 1);
    expect(vx(sim, 'B')).toBeCloseTo(2, 1);
    expect(Math.abs(vx(sim, 'A') + vx(sim, 'B') - 2)).toBeLessThan(1e-6);
    expect(minGap).toBeGreaterThan(-0.01); // under 1 cm of overlap at 2 m/s
  });

  it('unequal carts, e = 0.5: matches v′ = ((m₁ − e m₂)v, (1 + e) m₁ v) / (m₁ + m₂)', () => {
    const m1 = 2, m2 = 1, v = 3, e = 0.5;
    const sim = new Simulation(carts(m1, v, m2, e));
    sim.run(1);
    expect(vx(sim, 'A')).toBeCloseTo(((m1 - e * m2) * v) / (m1 + m2), 1);
    expect(vx(sim, 'B')).toBeCloseTo(((1 + e) * m1 * v) / (m1 + m2), 1);
    expect(Math.abs(m1 * vx(sim, 'A') + m2 * vx(sim, 'B') - m1 * v)).toBeLessThan(1e-6);
    // the kinetic energy lost is booked as dissipated, so the energy balance still closes
    expect(Math.abs(sim.driftPercent())).toBeLessThan(0.5);
  });

  it('blocks sliding on a frictionless floor (no track) collide head-on', () => {
    const s = emptyScene();
    s.settings.restitution = 1;
    const A = rect('A', -0.6, 0.1, 1, 0.4, 0.2, { vx: 1.5 });
    const B = rect('B', 0.4, 0.1, 1, 0.4, 0.2);
    s.elements.push(ground('G1', 0, -0.06, { w: 4, h: 0.12 }), A, B);
    const sim = new Simulation(s);
    sim.run(1.5);
    expect(vx(sim, 'A')).toBeCloseTo(0, 1);
    expect(vx(sim, 'B')).toBeCloseTo(1.5, 1);
    const b = sim.sys.bodies[1];
    expect(Math.abs(sim.q[b.idx + 2])).toBeLessThan(0.02); // a square hit doesn't tip it over
  });

  it('a block dropped on another stacks on top of it', () => {
    const s = emptyScene();
    s.elements.push(
      ground('G1', 0, -0.06, { w: 4, h: 0.12 }),
      rect('A', 0, 0.1, 2, 0.5, 0.2),
      rect('B', 0.05, 0.7, 1, 0.3, 0.2),
    );
    const sim = new Simulation(s);
    sim.run(3);
    const top = sim.sys.bodies[1];
    expect(sim.q[top.idx + 1]).toBeCloseTo(0.3, 2);
    expect(sim.energy().kinetic).toBeLessThan(1e-4);
  });

  it('a ball bounces off a block on a track', () => {
    const s = emptyScene();
    s.gravity.on = false;
    s.settings.restitution = 1;
    const block = rect('B', 0.5, 0, 3, 0.4, 0.4);
    const ball = point('m', -0.5, 0, 1, { radius: 0.05, vx: 2 });
    s.elements.push(block, ball, slider('S', at(block.id, 0, 0), [0, 0], 0));
    const sim = new Simulation(s);
    sim.run(1);
    // elastic: ball −1 m/s, block +1 m/s
    expect(vx(sim, 'm')).toBeCloseTo(-1, 1);
    expect(vx(sim, 'B')).toBeCloseTo(1, 1);
  });

  it('pinned bodies and bodies with collisions off do not collide', () => {
    const s = carts(1, 0, 1, 0.5);
    (s.elements[1] as any).x = -0.3; // overlaps A
    expect(new System(s).warnings.join(' ')).toMatch(/start overlapping/);
    (s.elements[1] as any).collide = false;
    expect(new System(s).forces.length).toBe(0);
  });
});
