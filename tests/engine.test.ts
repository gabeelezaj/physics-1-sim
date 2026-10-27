import { describe, expect, it } from 'vitest';
import { findEquilibrium, linearize } from '../src/engine/analysis';
import { Simulation } from '../src/engine/sim';
import { generalizedEigen, jacobiEigen, solve } from '../src/math/linalg';
import { doubleBarPendulum, simplePendulum, twoMassThreeSprings } from './scenes';

/** Mean period from upward zero crossings of a sampled signal. */
function measurePeriod(sim: Simulation, signal: () => number, duration: number): number {
  const crossings: number[] = [];
  let prev = signal(), prevT = sim.t;
  sim.run(duration, (s) => {
    const v = signal();
    if (prev < 0 && v >= 0) crossings.push(prevT + ((s.t - prevT) * -prev) / (v - prev));
    prev = v;
    prevT = s.t;
  });
  expect(crossings.length).toBeGreaterThan(2);
  return (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
}

describe('linear algebra', () => {
  it('solves a rank-deficient system with a particular solution', () => {
    const A = [[1, 1], [2, 2]];
    const { x, rank } = solve(A, [2, 4]);
    expect(rank).toBe(1);
    expect(x[0] + x[1]).toBeCloseTo(2, 12);
  });

  it('Jacobi and generalized eigen agree with known answers', () => {
    const { values } = jacobiEigen([[2, 1], [1, 2]]);
    expect(values[0]).toBeCloseTo(1, 12);
    expect(values[1]).toBeCloseTo(3, 12);
    const ge = generalizedEigen([[2, -1], [-1, 2]], [[2, 0], [0, 2]]);
    expect(ge.values[0]).toBeCloseTo(0.5, 12);
    expect(ge.values[1]).toBeCloseTo(1.5, 12);
  });
});

describe('simple pendulum', () => {
  it('small-angle period matches 2π√(L/g) within 1%', () => {
    const L = 1.2, g = 9.81;
    const sim = new Simulation(simplePendulum(L, (2 * Math.PI) / 180, 1, g));
    const bob = sim.sys.bodies[0];
    const T = measurePeriod(sim, () => sim.q[bob.idx], 12);
    const Texact = 2 * Math.PI * Math.sqrt(L / g);
    expect(Math.abs(T - Texact) / Texact).toBeLessThan(0.01);
  });

  it('linearization gives ω = √(g/L)', () => {
    const L = 0.7;
    const sim = new Simulation(simplePendulum(L, 0.3));
    const eq = findEquilibrium(sim.sys, sim.q);
    expect(eq.converged).toBe(true);
    const lin = linearize(sim.sys, eq.q, eq.lambda);
    expect(lin.omega.length).toBe(1);
    expect(lin.omega[0]).toBeCloseTo(Math.sqrt(9.81 / L), 5);
  });
});

describe('two masses, three springs', () => {
  const m = 1.5, k = 120;

  it('normal-mode frequencies match √(k/m) and √(3k/m)', () => {
    const sim = new Simulation(twoMassThreeSprings(m, k, -0.45, 0.6));
    const eq = findEquilibrium(sim.sys, sim.q);
    expect(eq.converged).toBe(true);
    const lin = linearize(sim.sys, eq.q, eq.lambda);
    expect(lin.omega.length).toBe(2);
    expect(lin.omega[0]).toBeCloseTo(Math.sqrt(k / m), 6);
    expect(lin.omega[1]).toBeCloseTo(Math.sqrt((3 * k) / m), 6);
    // in-phase then anti-phase mode shapes
    expect(lin.modesZ[0][0]).toBeCloseTo(lin.modesZ[1][0], 6);
    expect(lin.modesZ[0][1]).toBeCloseTo(-lin.modesZ[1][1], 6);
    // equilibrium is the symmetric configuration
    expect(eq.q[0]).toBeCloseTo(-0.5, 8);
    expect(eq.q[2]).toBeCloseTo(0.5, 8);
  });

  it('simulated in-phase mode oscillates at √(k/m)', () => {
    const sim = new Simulation(twoMassThreeSprings(m, k, -0.45, 0.55));
    const T = measurePeriod(sim, () => sim.q[0] + 0.5, 5);
    expect(Math.abs(T - 2 * Math.PI * Math.sqrt(m / k)) / T).toBeLessThan(0.002);
  });
});

describe('energy conservation (undamped, 60 s)', () => {
  const cases: [string, () => Simulation][] = [
    ['large-amplitude pendulum', () => new Simulation(simplePendulum(1, Math.PI / 2))],
    ['chaotic double bar pendulum', () => new Simulation(doubleBarPendulum(2.0, 2.6))],
    ['two masses three springs', () => new Simulation(twoMassThreeSprings(1, 80, -0.6, 0.65))],
  ];
  for (const [name, make] of cases) {
    it(`${name} drifts < 0.1%`, () => {
      const sim = make();
      let worst = 0;
      sim.run(60, (s) => {
        const d = Math.abs(s.driftPercent());
        if (d > worst) worst = d;
      });
      expect(worst).toBeLessThan(0.1);
    });
  }
});
