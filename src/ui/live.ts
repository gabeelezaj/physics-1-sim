// Live readout for the selected element: state, forces and rope status at the
// current simulation time.

import { RodC, RopeC, SliderC } from '../engine/constraints';
import { SpringF, TSpringF } from '../engine/forces';
import { angleOf, omegaOf, ptPos } from '../engine/kin';
import type { El } from '../model/types';
import type { App } from './app';
import { deg, num, si } from './format';

type Row = [string, string];

export function liveRows(app: App, e: El): Row[] {
  const { sim, sys } = app;
  const q = sim.q, qd = sim.qd;
  try { sim.currentAccel(); } catch { return []; }
  const lam = sys.lambda;
  const conIdx = (id: string) => sys.cons.findIndex((c) => c.el === id);
  const name = (id: string | null) => (id ? app.scene.elements.find((x) => x.id === id)?.name ?? '?' : 'world');

  switch (e.kind) {
    case 'point':
    case 'rect':
    case 'pulley': {
      const b = sys.byId.get(e.id);
      if (!b) return [];
      const vx = qd[b.idx], vy = qd[b.idx + 1];
      const rows: Row[] = [
        ['position', `(${num(q[b.idx], 4)}, ${num(q[b.idx + 1], 4)}) m`],
        ['speed', si(Math.hypot(vx, vy), 'm/s')],
      ];
      let ke = 0.5 * b.m * (vx * vx + vy * vy);
      if (b.dof === 3) {
        rows.push(['angle', deg(q[b.idx + 2])], ['spin', `${num(qd[b.idx + 2], 3)} rad/s`]);
        ke += 0.5 * b.I * qd[b.idx + 2] ** 2;
      }
      rows.push(['kinetic energy', si(ke, 'J')]);
      return rows;
    }
    case 'spring': {
      const f = sys.forces.find((x) => x.el === e.id) as SpringF | undefined;
      if (!f) return [];
      const pa = ptPos(f.a, q), pb = ptPos(f.b, q);
      const L = Math.hypot(pb.x - pa.x, pb.y - pa.y);
      const T = f.k * (L - f.L0);
      return [
        ['length ℓ', si(L, 'm')],
        ['stretch ℓ − L₀', si(L - f.L0, 'm')],
        [T >= 0 ? 'tension' : 'compression', si(Math.abs(T), 'N')],
        ['stored energy', si(f.energy(q), 'J')],
      ];
    }
    case 'tspring': {
      const f = sys.forces.find((x) => x.el === e.id) as TSpringF | undefined;
      if (!f) return [];
      const d = angleOf(f.a, q) - angleOf(f.b, q) - f.theta0;
      const tau = -f.k * d - f.c * (omegaOf(f.a, qd) - omegaOf(f.b, qd));
      return [['twist Δ', deg(d)], ['torque on A', si(tau, 'N·m')], ['stored energy', si(f.energy(q), 'J')]];
    }
    case 'pin': {
      const ci = conIdx(e.id);
      if (ci < 0) return [];
      const r0 = sys.rowStart[ci];
      const fx = lam[r0], fy = lam[r0 + 1];
      return [
        [`force on ${name(e.a.body)}`, `(${si(fx, 'N')}, ${si(fy, 'N')})`],
        ['|R|', si(Math.hypot(fx, fy), 'N')],
      ];
    }
    case 'rod': {
      const ci = conIdx(e.id);
      if (ci < 0) return [];
      const c = sys.cons[ci] as RodC;
      const l = lam[sys.rowStart[ci]];
      const pa = ptPos(c.a, q), pb = ptPos(c.b, q);
      return [
        [l <= 0 ? 'tension' : 'compression', si(Math.abs(l), 'N')],
        ['length error', si(Math.hypot(pb.x - pa.x, pb.y - pa.y) - c.L, 'm')],
      ];
    }
    case 'slider': {
      const ci = conIdx(e.id);
      if (ci < 0) return [];
      const c = sys.cons[ci] as SliderC;
      const r0 = sys.rowStart[ci];
      const rows: Row[] = [['normal force N', si(lam[r0], 'N')]];
      if (c.lockBody) rows.push(['holding moment', si(lam[r0 + 1], 'N·m')]);
      return rows;
    }
    case 'rope': {
      const ci = conIdx(e.id);
      if (ci < 0) return [];
      const c = sys.cons[ci] as RopeC;
      const T = sys.ropeTensions(ci);
      const pulleys = (idx: number[]) => idx.map((i) => name(c.route[i].b.id)).join(', ');
      const on = [...c.activeRoute];
      const off = c.route.map((_, i) => i).filter((i) => !c.active[i]);
      const rows: Row[] = T.map((t, j) => [`T${j + 1}`, t < -1e-9 ? `<span class="err">${si(t, 'N')} (would go slack)</span>` : si(t, 'N')]);
      rows.push(['path length', si(c.geometry(q).total, 'm')]);
      if (c.route.length) rows.push(['on', on.length ? pulleys(on) : 'none'], ['off (rope left them)', off.length ? pulleys(off) : 'none']);
      return rows;
    }
    case 'force': {
      const b = e.a.body ? sys.byId.get(e.a.body) : null;
      const rows: Row[] = [['|F|', si(Math.hypot(e.fx, e.fy), 'N')]];
      if (b) {
        const v = [qd[b.idx], qd[b.idx + 1]];
        rows.push(['power F·v (at centre)', si(e.fx * v[0] + e.fy * v[1], 'W')]);
      }
      if (e.torque) rows.push(['torque', si(e.torque, 'N·m')]);
      return rows;
    }
    default:
      return [];
  }
}
