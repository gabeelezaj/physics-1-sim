import { describe, expect, it } from 'vitest';
import { findEquilibrium, linearize } from '../src/engine/analysis';
import { Simulation } from '../src/engine/sim';
import { System } from '../src/engine/system';
import { PRESETS } from '../src/presets';

describe.each(PRESETS.map((p) => [p.title, p] as const))('preset %s', (_title, preset) => {
  it('compiles without warnings and assembles consistently', () => {
    const sys = new System(preset.build());
    expect(sys.warnings).toEqual([]);
    const sim = new Simulation(preset.build());
    expect(sim.assemblyResidual).toBeLessThan(1e-9);
  });

  it.skipIf(!preset.modal)('has a stable static equilibrium with positive natural frequencies', () => {
    const scene = preset.build();
    const sys = new System(scene);
    const { q, qd } = sys.initialState();
    sys.assemble(q, qd);
    const eq = findEquilibrium(sys, q);
    expect(eq.converged).toBe(true);
    const lin = linearize(sys, eq.q, eq.lambda);
    expect(lin.indep.length).toBeGreaterThan(0);
    for (const w2 of lin.omega2) expect(w2).toBeGreaterThan(0);
  });

  // contact-heavy presets run 10 s unless FULL=1 (npm run test:full)
  const T = preset.modal || process.env.FULL ? 60 : 10;
  it(`keeps its energy balance over ${T} s (< 0.1%, or < 1% with impacts)`, () => {
    const sim = new Simulation(preset.build());
    let worst = 0;
    sim.run(T, (s) => { worst = Math.max(worst, Math.abs(s.driftPercent())); });
    expect(sim.q.every(Number.isFinite)).toBe(true);
    expect(sim.keMax).toBeGreaterThan(1e-3);
    expect(worst).toBeLessThan(sim.dissipated > 1e-9 ? 1 : 0.1);
  });
});

it('preset 1 has the three expected degrees of freedom', () => {
  const sys = new System(PRESETS[0].build());
  const { q, qd } = sys.initialState();
  sys.assemble(q, qd);
  const eq = findEquilibrium(sys, q);
  const lin = linearize(sys, eq.q, eq.lambda);
  expect(lin.indep.map((k) => sys.coords[k].name)).toEqual(['x_C', 'θ_B1', 'θ_B2']);
});

it('preset 3 has five degrees of freedom', () => {
  const sys = new System(PRESETS[2].build());
  const { q, qd } = sys.initialState();
  sys.assemble(q, qd);
  const eq = findEquilibrium(sys, q);
  const lin = linearize(sys, eq.q, eq.lambda);
  expect(lin.indep.length).toBe(5);
});

it('preset 4: the carts collide and stay in order on the track', () => {
  const sim = new Simulation(PRESETS[3].build());
  const [A, B, C] = sim.sys.bodies;
  let ordered = true;
  sim.run(20, (s) => {
    const xa = s.q[A.idx], xb = s.q[B.idx], xc = s.q[C.idx];
    if (xb - xa < 0.5 || xc - xb < 0.42 || xa < -2.32 || xc > 2.1) ordered = false;
  });
  expect(ordered).toBe(true);
  expect(sim.dissipated).toBeGreaterThan(0.5); // inelastic hits (e = 0.8) took energy out
});
