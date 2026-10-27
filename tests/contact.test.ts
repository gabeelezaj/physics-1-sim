import { describe, expect, it } from 'vitest';
import { Simulation } from '../src/engine/sim';
import { System } from '../src/engine/system';
import { at, ground, pin, point, rect, spring } from '../src/model/factory';
import { emptyScene } from '../src/model/types';

/** Floor surface at y = 0. */
const floor = () => ground('G1', 0, -0.06, { w: 4, h: 0.12 });

describe('ground contact', () => {
  it('a tumbling bar lands on the floor without passing through and comes to rest', () => {
    const s = emptyScene();
    const bar = rect('B1', 0, 0.6, 1, 0.8, 0.06, { theta: 0.5, omega: 3 });
    s.elements.push(floor(), bar);
    const sim = new Simulation(s);
    const b = sim.sys.bodies[0];
    let minCorner = Infinity;
    sim.run(6, (sm) => {
      const x = sm.q[b.idx], y = sm.q[b.idx + 1], th = sm.q[b.idx + 2];
      for (const [lx, ly] of [[-0.4, -0.03], [0.4, -0.03], [0.4, 0.03], [-0.4, 0.03]]) {
        minCorner = Math.min(minCorner, y + Math.sin(th) * lx + Math.cos(th) * ly);
      }
      void x;
    });
    expect(minCorner).toBeGreaterThan(-0.006);
    expect(sim.energy().kinetic).toBeLessThan(1e-3);
    expect(sim.q[b.idx + 1]).toBeCloseTo(0.03, 2);
  });

  it('a pendulum bar swinging into a wall bounces back instead of passing through', () => {
    const s = emptyScene();
    const wall = ground('G1', 0.56, 0, { theta: Math.PI / 2, w: 2, h: 0.12 }); // surface at x = 0.5
    const L = 0.8;
    const bar = rect('B1', -L / 2, 0, 1, L, 0.05, { theta: Math.PI, omega: -4 });
    s.elements.push(wall, bar, pin('O1', at(bar.id, -L / 2, 0), at(null, 0, 0)));
    const sim = new Simulation(s);
    const b = sim.sys.bodies[0];
    let maxTip = -Infinity;
    sim.run(3, (sm) => {
      const x = sm.q[b.idx], th = sm.q[b.idx + 2];
      maxTip = Math.max(maxTip, x + (L / 2) * Math.abs(Math.cos(th)) + 0.025 * Math.abs(Math.sin(th)));
    });
    expect(maxTip).toBeGreaterThan(0.45); // it did reach the wall
    expect(maxTip).toBeLessThan(0.51); // < 1 cm of penalty overlap at ~3 m/s impact
  });

  it('a perfectly elastic ball keeps its energy over many bounces', () => {
    const s = emptyScene();
    s.settings.restitution = 1;
    s.elements.push(floor(), point('m1', 0, 0.5, 1, { radius: 0.05 }));
    const sim = new Simulation(s);
    let bounces = 0, prevVy = 0;
    sim.run(5, (sm) => {
      const vy = sm.qd[1];
      if (prevVy < 0 && vy >= 0) bounces++;
      prevVy = vy;
    });
    expect(bounces).toBeGreaterThanOrEqual(4);
    expect(Math.abs(sim.driftPercent())).toBeLessThan(0.5);
  });

  it('restitution e = 0.5 halves the rebound speed', () => {
    const s = emptyScene();
    s.settings.restitution = 0.5;
    s.elements.push(floor(), point('m1', 0, 0.3, 2, { radius: 0.05, vy: -2 }));
    const sim = new Simulation(s);
    let vIn = 0, vOut = 0;
    sim.run(0.5, (sm) => {
      const vy = sm.qd[1];
      if (sm.q[1] > 0.06) { if (vy < 0 && vOut === 0) vIn = vy; else if (vy > 0 && vOut === 0) vOut = vy; }
    });
    expect(vOut / -vIn).toBeGreaterThan(0.45);
    expect(vOut / -vIn).toBeLessThan(0.55);
  });

  it('skips pairs that start overlapping, and can be switched off', () => {
    const s = emptyScene();
    s.elements.push(floor(), point('m1', 0, -0.02, 1, { radius: 0.05 }));
    expect(new System(s).warnings.join(' ')).toMatch(/starts inside/);
    s.settings.contact = false;
    expect(new System(s).forces.length).toBe(0);
  });
});

describe('energy balance with dissipation', () => {
  it('a damped spring–mass: T + V + W stays equal to E₀', () => {
    const s = emptyScene();
    s.gravity.on = false;
    const m = point('m1', 0.3, 0, 1, { radius: 0.05 });
    s.elements.push(m, spring('k1', at(null, -0.7, 0), at(m.id, 0, 0), 100, 0.8, 1.5));
    const sim = new Simulation(s);
    let worst = 0;
    sim.run(10, (sm) => { worst = Math.max(worst, Math.abs(sm.driftPercent())); });
    expect(sim.dissipated).toBeGreaterThan(0.9 * sim.E0); // most of the energy is gone
    expect(worst).toBeLessThan(0.01);
  });

  it('a bar bouncing to rest on the floor: impacts are accounted for', () => {
    const s = emptyScene();
    s.elements.push(floor(), rect('B1', 0, 0.6, 1, 0.8, 0.06, { theta: 0.5, omega: 3 }));
    const sim = new Simulation(s);
    let worst = 0;
    sim.run(4, (sm) => { worst = Math.max(worst, Math.abs(sm.driftPercent())); });
    expect(sim.dissipated).toBeGreaterThan(0);
    expect(worst).toBeLessThan(0.5);
  });
});
