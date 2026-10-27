import { describe, expect, it } from 'vitest';
import { findEquilibrium, linearize } from '../src/engine/analysis';
import { Simulation } from '../src/engine/sim';
import { atwood, compoundPulley, springAtwood } from './scenes';

describe('Atwood machine with a massive pulley', () => {
  const m1 = 3, m2 = 1, M = 2, r = 0.1;
  const I = 0.5 * M * r * r;
  const aExact = ((m1 - m2) * 9.81) / (m1 + m2 + I / (r * r));

  it('acceleration matches (m1−m2)g/(m1+m2+I/r²)', () => {
    const sim = new Simulation(atwood(m1, m2, M, r));
    const [p, a, b] = sim.sys.bodies;
    const y1 = sim.q[a.idx + 1], y2 = sim.q[b.idx + 1];
    const T = 0.6;
    sim.run(T);
    const a1 = (-2 * (sim.q[a.idx + 1] - y1)) / (T * T);
    const a2 = (2 * (sim.q[b.idx + 1] - y2)) / (T * T);
    expect(Math.abs(a1 - aExact) / aExact).toBeLessThan(1e-3);
    expect(Math.abs(a2 - aExact) / aExact).toBeLessThan(1e-3);
    // no slip: the rope speed equals ω r (heavy side descends on the left → CCW)
    const omega = sim.qd[p.idx + 2];
    expect(omega * r).toBeCloseTo(aExact * T, 4);
  });

  it('instantaneous acceleration and tensions from the multiplier solve', () => {
    const sim = new Simulation(atwood(m1, m2, M, r));
    const acc = sim.currentAccel();
    const [, a] = sim.sys.bodies;
    expect(-acc[a.idx + 1]).toBeCloseTo(aExact, 8);
    const T = sim.sys.ropeTensions(sim.sys.cons.findIndex((c) => c.kind === 'rope'));
    // T1 = m1(g − a), T2 = m2(g + a)
    expect(T[0]).toBeCloseTo(m1 * (9.81 - aExact), 8);
    expect(T[1]).toBeCloseTo(m2 * (9.81 + aExact), 8);
  });
});

describe('free pulley + fixed massive pulley', () => {
  it('matches (M − 2m)g / (M + I_p/r² + 4 I_f/r² + 4m)', () => {
    const M = 3, m = 1, Mf = 2, r = 0.1;
    const Ip = 0.5 * M * r * r, If = 0.5 * Mf * r * r;
    const aExact = ((M - 2 * m) * 9.81) / (M + Ip / (r * r) + (4 * If) / (r * r) + 4 * m);
    const sim = new Simulation(compoundPulley(M, m, Mf, r));
    const [free, fixed, hang] = sim.sys.bodies;
    const y0 = sim.q[free.idx + 1], ym = sim.q[hang.idx + 1];
    const T = 0.5;
    sim.run(T);
    const ap = (-2 * (sim.q[free.idx + 1] - y0)) / (T * T);
    const am = (2 * (sim.q[hang.idx + 1] - ym)) / (T * T);
    expect(Math.abs(ap - aExact) / aExact).toBeLessThan(1e-3);
    expect(Math.abs(am - 2 * aExact) / aExact).toBeLessThan(2e-3);
    expect(Math.abs(sim.qd[free.idx + 2]) * r).toBeCloseTo(aExact * T, 4);
    expect(Math.abs(sim.qd[fixed.idx + 2]) * r).toBeCloseTo(2 * aExact * T, 4);
  });
});

describe('rope systems with springs', () => {
  it('conserve energy within 0.1% over 60 s', () => {
    const sim = new Simulation(springAtwood());
    expect(sim.assemblyResidual).toBeLessThan(1e-9);
    let worst = 0;
    sim.run(60, (s) => { worst = Math.max(worst, Math.abs(s.driftPercent())); });
    expect(sim.keMax).toBeGreaterThan(0.01);
    expect(worst).toBeLessThan(0.1);
  });

  it('vertical mode includes pulley inertia', () => {
    const sim = new Simulation(springAtwood());
    const eq = findEquilibrium(sim.sys, sim.q);
    expect(eq.converged).toBe(true);
    const lin = linearize(sim.sys, eq.q, eq.lambda);
    // the coupled vertical mode: (k1 + k2) / (m1 + m2 + I/r²)
    const I = 0.5 * 1.5 * 0.12 * 0.12;
    const wExact = Math.sqrt((60 + 90) / (2 + 1.2 + I / 0.0144));
    expect(lin.omega.some((w) => Math.abs(w - wExact) / wExact < 1e-4)).toBe(true);
  });
});
