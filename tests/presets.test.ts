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

  it('has a stable static equilibrium with positive natural frequencies', () => {
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

  it('conserves energy within 0.1% over 60 s (undamped)', () => {
    const sim = new Simulation(preset.build());
    let worst = 0;
    sim.run(60, (s) => { worst = Math.max(worst, Math.abs(s.driftPercent())); });
    expect(sim.q.every(Number.isFinite)).toBe(true);
    expect(sim.keMax).toBeGreaterThan(1e-3);
    expect(worst).toBeLessThan(0.1);
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
