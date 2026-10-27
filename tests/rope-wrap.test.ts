import { describe, expect, it } from 'vitest';
import { RopeC } from '../src/engine/constraints';
import { Simulation } from '../src/engine/sim';
import { at, point, pulley, rope, slider } from '../src/model/factory';
import { emptyScene } from '../src/model/types';

const ropeOf = (sim: Simulation) => sim.sys.cons.find((c) => c instanceof RopeC) as RopeC;

describe('ropes leaving and catching pulleys', () => {
  it('a mass swung up over a pulley lets the rope come off instead of knotting', () => {
    // anchor → over the top of a fixed pulley (clockwise) → hanging mass, kicked sideways
    const s = emptyScene();
    const p = pulley('P1', 0, -0.1, 1, 0.1);
    const m = point('m1', 0.1, -1, 1, { vx: 5.5 });
    s.elements.push(p, m, rope('T1', at(null, -1, 0), at(m.id, 0, 0), [{ pulley: p.id, dir: -1 }], 1 + 0.9 + (Math.PI / 2) * 0.1));
    const sim = new Simulation(s);
    const con = ropeOf(sim);
    expect(con.active).toEqual([true]);
    let released = -1, maxJump = 0, maxT = 0, worst = 0, prevWrap: number | null = null;
    const ci = sim.sys.cons.indexOf(con);
    sim.run(2, (sm) => {
      if (released < 0 && !con.active[0]) released = sm.t;
      const g = con.geometry(sm.q);
      const w = con.active[0] ? g.wraps[0] : null;
      if (w !== null && prevWrap !== null) maxJump = Math.max(maxJump, Math.abs(w - prevWrap));
      prevWrap = w;
      sm.currentAccel();
      for (const t of sm.sys.ropeTensions(ci)) maxT = Math.max(maxT, Math.abs(t));
      worst = Math.max(worst, Math.abs(sm.driftPercent()));
    });
    expect(released).toBeGreaterThan(0);
    expect(maxJump).toBeLessThan(0.05); // the wrap never jumps by 2π
    expect(maxT).toBeLessThan(200); // no runaway tensions
    expect(worst).toBeLessThan(0.1); // releasing is smooth: no energy jump
  });

  it('a free pulley catches a swinging rope and starts to spin', () => {
    // a pendulum hung from a cart on a track; its rope sweeps down onto a pulley
    const s = emptyScene();
    const p = pulley('P1', 0.5, -0.3, 0.5, 0.08);
    const cart = point('m0', 0, 0, 2);
    const m = point('m1', 1, 0, 1);
    s.elements.push(p, cart, m, slider('S1', at(cart.id, 0, 0), [0, 0], 0),
      rope('T1', at(cart.id, 0, 0), at(m.id, 0, 0), [{ pulley: p.id, dir: -1 }], 1));
    const sim = new Simulation(s);
    const con = ropeOf(sim);
    expect(con.active).toEqual([false]); // not touching yet
    let caughtAt = -1, worst = 0, minGap = Infinity;
    const pb = sim.sys.byId.get(p.id)!, mb = sim.sys.byId.get(m.id)!;
    sim.run(1.5, (sm) => {
      if (caughtAt < 0 && con.active[0]) caughtAt = sm.t;
      worst = Math.max(worst, Math.abs(sm.driftPercent()));
      // the rope's straight runs never cut through the pulley
      for (const pc of con.geometry(sm.q).pieces) {
        const wx = sm.q[pb.idx] - pc.t0[0], wy = sm.q[pb.idx + 1] - pc.t0[1];
        const along = wx * pc.ux + wy * pc.uy;
        if (along > 1e-6 && along < pc.len - 1e-6) minGap = Math.min(minGap, Math.abs(pc.ux * wy - pc.uy * wx));
      }
    });
    expect(caughtAt).toBeGreaterThan(0);
    expect(Math.abs(sim.qd[pb.idx + 2])).toBeGreaterThan(0.1); // dragged into rotation by the rope
    expect(sim.dissipated).toBeGreaterThan(0); // the catch is a plastic impulse
    expect(worst).toBeLessThan(0.5);
    expect(minGap).toBeGreaterThan(0.08 - 2e-3);
    void mb;
  });
});
