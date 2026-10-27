// "All the calculations": live plots and energy drift, equilibrium + normal
// modes, the equations of motion in LaTeX, and a live table of forces.

import katex from 'katex';
import { Linearization, chooseIndependent, findEquilibrium, linearize, type Equilibrium } from '../engine/analysis';
import { PinC, RodC, RopeC, SliderC } from '../engine/constraints';
import { SpringF, TSpringF } from '../engine/forces';
import {
  bmatrix, constraintTex, daeTex, energyTex, forceTex, linearEomTex, massTex, num, zVecTex,
} from '../engine/latex';
import { ptPos } from '../engine/kin';
import { System } from '../engine/system';
import { App } from './app';
import { si } from './format';
import { PALETTE, Plot } from './plots';

const tex = (s: string, display = true) => katex.renderToString(s, { displayMode: display, throwOnError: false, strict: false });
const texBlock = (s: string) => `<div class="katex-block">${tex(s)}</div>`;
const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);
const f = (v: number, sig = 4) => (!Number.isFinite(v) ? '—' : Math.abs(v) < 1e-12 ? '0' : Number(v.toPrecision(sig)).toString());

interface ModesResult {
  sys: System;
  eq: Equilibrium;
  lin: Linearization | null;
  error?: string;
}

export class AnalysisPanel {
  private tab = 'plots';
  private selected: string[] = [];
  private coordPlot!: Plot;
  private energyPlot!: Plot;
  private driftPlot!: Plot;
  private statsEl!: HTMLElement;
  private chipsEl!: HTMLElement;
  private addSel!: HTMLSelectElement;
  private lastPlot = 0;
  private lastForces = 0;
  private modes: ModesResult | null = null;
  private modesStale = false;

  constructor(private app: App) {
    document.querySelectorAll<HTMLButtonElement>('#analysis .tabs button').forEach((b) =>
      b.addEventListener('click', () => this.showTab(b.dataset.tab!)),
    );
    this.buildPlots();
    app.onTick(() => this.tick());
    app.onState(() => this.tick(true));
    app.onSceneChange(() => {
      this.modesStale = this.modes !== null;
      this.syncCoordChoices();
      this.renderModes();
      if (this.tab === 'equations') this.renderEquations();
      this.tick(true);
    });
    this.syncCoordChoices();
    this.renderModes();
    this.tick(true);
  }

  private showTab(t: string): void {
    this.tab = t;
    document.querySelectorAll<HTMLButtonElement>('#analysis .tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === t));
    for (const id of ['plots', 'modes', 'equations', 'forces']) (document.getElementById(`tab-${id}`) as HTMLElement).hidden = id !== t;
    if (t === 'equations') this.renderEquations();
    this.tick(true);
  }

  private tick(force = false): void {
    const now = performance.now();
    if (this.tab === 'plots' && (force || now - this.lastPlot > 66)) {
      this.lastPlot = now;
      this.drawPlots();
    }
    if (this.tab === 'forces' && (force || now - this.lastForces > 120)) {
      this.lastForces = now;
      this.renderForces();
    }
  }

  // ------------------------------------------------------------------ plots

  private buildPlots(): void {
    const root = document.getElementById('tab-plots')!;
    root.innerHTML = `
      <div class="stat-row" id="stats"></div>
      <div class="plot-box">
        <div class="plot-head"><span class="section-title">Coordinates</span><select id="add-coord"></select></div>
        <div class="chips" id="coord-chips"></div>
        <canvas id="plot-coords"></canvas>
      </div>
      <div class="plot-box">
        <div class="plot-head"><span class="section-title">Energy</span><span class="muted">potential measured from t = 0</span></div>
        <div class="chips">
          <span class="chip"><i style="background:${PALETTE[0]}"></i>kinetic T</span>
          <span class="chip"><i style="background:${PALETTE[1]}"></i>potential ΔV</span>
          <span class="chip"><i style="background:#111"></i>total ΔE</span>
          <span class="chip"><i style="background:${PALETTE[3]}"></i>dissipated W</span>
        </div>
        <canvas id="plot-energy"></canvas>
      </div>
      <div class="plot-box">
        <div class="plot-head"><span class="section-title">Energy balance error</span><span class="muted">(T + V + W − E₀) / max T, %</span></div>
        <canvas id="plot-drift" style="height:110px"></canvas>
      </div>`;
    this.statsEl = root.querySelector('#stats')!;
    this.chipsEl = root.querySelector('#coord-chips')!;
    this.addSel = root.querySelector('#add-coord')!;
    this.coordPlot = new Plot(root.querySelector('#plot-coords')!, 'm  /  rad');
    this.energyPlot = new Plot(root.querySelector('#plot-energy')!, 'J');
    this.driftPlot = new Plot(root.querySelector('#plot-drift')!, '%');
    this.addSel.addEventListener('change', () => {
      if (this.addSel.value && !this.selected.includes(this.addSel.value)) this.selected.push(this.addSel.value);
      this.addSel.value = '';
      this.syncCoordChoices();
      this.drawPlots();
    });
  }

  private syncCoordChoices(): void {
    const names = this.app.sys.coords.map((c) => c.name);
    this.selected = this.selected.filter((s) => names.includes(s));
    if (!this.selected.length && names.length) {
      try {
        const { indep } = chooseIndependent(this.app.sys, this.app.sim.q);
        this.selected = indep.slice(0, 3).map((k) => names[k]);
      } catch {
        this.selected = names.slice(0, 1);
      }
    }
    this.addSel.innerHTML = '<option value="">+ add coordinate</option>' +
      names.filter((n) => !this.selected.includes(n)).map((n) => `<option value="${esc(n)}">${esc(n)}</option>`).join('');
    this.chipsEl.innerHTML = '';
    this.selected.forEach((n, i) => {
      const chip = document.createElement('span');
      chip.className = 'chip';
      chip.innerHTML = `<i style="background:${PALETTE[i % PALETTE.length]}"></i>${esc(n)}`;
      const x = document.createElement('button');
      x.textContent = '×';
      x.title = 'Remove';
      x.addEventListener('click', () => {
        this.selected = this.selected.filter((s) => s !== n);
        if (!this.selected.length) this.selected = ['__none__'];
        this.syncCoordChoices();
        this.drawPlots();
      });
      chip.appendChild(x);
      this.chipsEl.appendChild(chip);
    });
  }

  private drawPlots(): void {
    const { recorder: rec, sim } = this.app;
    const n = sim.sys.n;
    const e = sim.energy();
    const drift = sim.driftPercent();
    // penalty impacts are integrated less exactly than smooth motion
    const ok = Math.abs(drift) < (sim.dissipated > 1e-9 ? 1 : 0.1);
    this.statsEl.innerHTML = [
      ['t', `${sim.t.toFixed(3)} s`],
      ['T (kinetic)', `${f(e.kinetic)} J`],
      ['V (potential)', `${f(e.potential)} J`],
      ['E = T + V', `${f(e.total, 6)} J`],
      ['dissipated W', `${f(sim.dissipated)} J`],
      ['balance error', `<span class="${ok ? 'drift-ok' : 'drift-bad'}">${drift.toExponential(2)} %</span>`],
    ].map(([k, v]) => `<div class="stat"><span>${k}</span><span>${v}</span></div>`).join('');
    const series = this.selected
      .map((name, i) => ({ name, i, k: rec.names.indexOf(name) }))
      .filter((s) => s.k >= 0)
      .map((s) => ({ label: s.name, color: PALETTE[s.i % PALETTE.length], data: rec.ch[s.k] }));
    this.coordPlot.draw(rec.t, series);
    const ke = rec.ch[n] ?? [], pe = rec.ch[n + 1] ?? [];
    const pe0 = pe[0] ?? 0;
    const dpe = pe.map((v) => v - pe0);
    const de = dpe.map((v, i) => v + ke[i]);
    const W = rec.ch[n + 3] ?? [];
    this.energyPlot.draw(rec.t, [
      { label: 'T', color: PALETTE[0], data: ke },
      { label: 'ΔV', color: PALETTE[1], data: dpe },
      { label: 'ΔE', color: '#111', data: de },
      { label: 'W', color: PALETTE[3], data: W },
    ]);
    const ref = Math.max(sim.keMax, 1e-12);
    const E0 = sim.E0;
    const tot = rec.ch[n + 2] ?? [];
    this.driftPlot.draw(rec.t, [{ label: 'balance', color: '#7c3aed', data: tot.map((v, i) => (100 * (v + (W[i] ?? 0) - E0)) / ref) }]);
  }

  // ------------------------------------------------------------------ modes

  private runModes(): void {
    const sys = new System(this.app.scene);
    const { q, qd } = sys.initialState();
    sys.assemble(q, qd);
    const eq = findEquilibrium(sys, q);
    let lin: Linearization | null = null;
    let error: string | undefined;
    try {
      lin = linearize(sys, eq.q, eq.lambda);
    } catch (err) {
      error = (err as Error).message;
    }
    this.modes = { sys, eq, lin, error };
    this.modesStale = false;
    this.renderModes();
    if (this.tab === 'equations') this.renderEquations();
  }

  private renderModes(): void {
    const root = document.getElementById('tab-modes')!;
    const head = `
      <div class="props-actions" style="margin-top:0">
        <button id="run-modes" class="small">Find equilibrium &amp; normal modes</button>
        ${this.modes?.eq.converged ? '<button id="apply-eq" class="small">Use equilibrium as initial state</button>' : ''}
      </div>
      <div class="props-note">Newton on F(q) + Jᵀλ = 0, g(q) = 0 from the current initial configuration, then
      K = −∂(F + Jᵀλ*)/∂q restricted to the constraint tangent space and the generalized eigenproblem K φ = ω² M φ.</div>`;
    if (!this.modes) {
      root.innerHTML = head;
    } else {
      const { sys, eq, lin, error } = this.modes;
      let html = head;
      if (this.modesStale) html += '<div class="props-note err">The scene changed since this analysis — run it again.</div>';
      html += `<div class="section-title">Static equilibrium</div>
        <div class="${eq.converged ? '' : 'err'}">${eq.converged ? 'Converged' : 'Did not converge'} in ${eq.iterations} Newton iterations ·
        max|F + Jᵀλ| = ${eq.forceResidual.toExponential(2)} · max|g| = ${eq.constraintResidual.toExponential(2)}</div>`;
      if (error) html += `<div class="err">Linearization failed: ${esc(error)}</div>`;
      if (lin) {
        const z = lin.indep.map((k) => sys.coords[k]);
        html += `<table class="data"><tr><th>coordinate</th><th>equilibrium value</th></tr>${z.map((c, i) =>
          `<tr><td>${tex(c.tex, false)}</td><td>${f(eq.q[lin.indep[i]], 6)} ${c.kind === 'theta' ? 'rad' : 'm'}</td></tr>`).join('')}</table>`;
        if (!lin.indep.length) {
          html += '<div class="muted">No degrees of freedom: the mechanism is fully constrained.</div>';
        } else {
          html += `<div class="section-title">Linearized model about equilibrium</div>
            <div class="muted">${lin.indep.length} independent coordinate${lin.indep.length > 1 ? 's' : ''} (displacements from equilibrium); the other ${lin.dep.length} follow from the constraints.</div>`;
          html += texBlock(`\\mathbf z = ${zVecTex(sys, lin)},\\qquad \\mathbf M\\ddot{\\mathbf z} + \\mathbf C\\dot{\\mathbf z} + \\mathbf K\\mathbf z = \\mathbf 0`);
          html += texBlock(`\\mathbf M = ${bmatrix(lin.M)}`);
          html += texBlock(`\\mathbf K = ${bmatrix(lin.K)}`);
          if (lin.C.some((r) => r.some((v) => v !== 0))) html += texBlock(`\\mathbf C = ${bmatrix(lin.C)}`);
          html += texBlock(`\\det\\left(\\mathbf K - \\omega^2\\mathbf M\\right) = 0`);
          html += `<div class="section-title">Natural frequencies</div>
            <table class="data"><tr><th>mode</th><th>ω² [1/s²]</th><th>ω [rad/s]</th><th>f [Hz]</th><th>T [s]</th><th>ζ</th><th></th></tr>
            ${lin.omega2.map((w2, i) => {
              const stable = w2 > 1e-9 * Math.max(1, Math.abs(lin.omega2[lin.omega2.length - 1]));
              const zero = Math.abs(w2) <= 1e-9 * Math.max(1, Math.abs(lin.omega2[lin.omega2.length - 1]));
              return `<tr><td>${i + 1}</td><td>${f(w2)}</td>
                <td>${stable ? f(lin.omega[i]) : zero ? '0 (rigid)' : `<span class="err">${f(Math.sqrt(-w2))}i (unstable)</span>`}</td>
                <td>${stable ? f(lin.freqHz[i]) : '—'}</td><td>${stable ? f((2 * Math.PI) / lin.omega[i]) : '—'}</td>
                <td>${stable && lin.zeta[i] ? f(lin.zeta[i], 3) : '—'}</td>
                <td><button class="small" data-anim="${i}">Animate</button></td></tr>`;
            }).join('')}</table>`;
          html += `<div class="section-title">Mode shapes φ (largest entry = 1)</div>
            <table class="data"><tr><th></th>${lin.omega.map((_, i) => `<th>φ${i + 1}</th>`).join('')}</tr>
            ${z.map((c, r) => `<tr><td>${tex(c.tex, false)}</td>${lin.modesZ[r].map((v) => `<td>${f(v, 3)}</td>`).join('')}</tr>`).join('')}</table>`;
        }
      }
      root.innerHTML = html;
      root.querySelectorAll<HTMLButtonElement>('[data-anim]').forEach((b) =>
        b.addEventListener('click', () => {
          if (!this.modes?.lin || this.modesStale) return;
          this.app.startModeAnim(this.modes.lin, Number(b.dataset.anim));
        }),
      );
      root.querySelector('#apply-eq')?.addEventListener('click', () => {
        const m = this.modes;
        if (!m) return;
        this.app.commit(() => {
          const zero = new Float64Array(m.sys.n);
          m.sys.writeBack(m.eq.q, zero);
        });
      });
    }
    root.querySelector('#run-modes')?.addEventListener('click', () => this.runModes());
  }

  // -------------------------------------------------------------- equations

  private renderEquations(): void {
    const root = document.getElementById('tab-equations')!;
    const sys = this.app.sys;
    const q = this.app.sim.q;
    let rank = 0;
    try { rank = sys.m ? chooseIndependent(sys, q).dep.length : 0; } catch { /* ignore */ }
    let html = `<div class="section-title">Generalized (maximal) coordinates</div>
      <div class="muted">${sys.n} coordinates, ${sys.m} constraint rows (rank ${rank}) → ${sys.n - rank} degrees of freedom.</div>`;
    html += texBlock(`\\mathbf q = ${sys.n ? `\\left[${sys.coords.map((c) => c.tex).join(',\\ ')}\\right]^{\\mathsf T}` : '\\varnothing'}`);
    html += texBlock(massTex(sys));
    if (sys.bodies.length) html += texBlock(energyTex(sys));
    html += `<div class="section-title">Equations of motion (index-1 DAE with Baumgarte)</div>${texBlock(daeTex(sys))}`;
    const cons = constraintTex(sys, q);
    if (cons.length) {
      html += '<div class="section-title">Constraints g(q) = 0</div>';
      for (const c of cons) html += `<div class="muted">${esc(c.label)}</div>${texBlock(c.tex)}`;
    }
    const forces = forceTex(sys);
    if (forces.length) {
      html += '<div class="section-title">Applied forces and potentials</div>';
      for (const c of forces) html += `<div class="muted">${esc(c.label)}</div>${texBlock(c.tex)}`;
    }
    const m = this.modes;
    if (m?.lin && m.lin.indep.length && !this.modesStale) {
      html += `<div class="section-title">Linearized equations of motion about equilibrium</div>${texBlock(linearEomTex(m.sys, m.lin))}`;
    } else {
      html += '<div class="props-note">Run “Find equilibrium &amp; normal modes” to add the linearized equations of motion here.</div>';
    }
    root.innerHTML = html;
  }

  // ----------------------------------------------------------------- forces

  private renderForces(): void {
    const root = document.getElementById('tab-forces')!;
    const { sys, sim } = this.app;
    const q = sim.q, qd = sim.qd;
    try {
      sim.currentAccel(); // refresh λ without stabilization terms
    } catch { return; }
    const lam = sys.lambda;
    const name = (id: string) => this.app.scene.elements.find((e) => e.id === id.split(':')[0])?.name ?? id;
    const rows: [string, string, string][] = [];
    sys.cons.forEach((c, ci) => {
      const r0 = sys.rowStart[ci];
      if (c instanceof PinC) {
        const fx = lam[r0], fy = lam[r0 + 1];
        rows.push([name(c.el), c.kind === 'axle' ? 'axle' : 'pin', `|R| = ${si(Math.hypot(fx, fy), 'N')} &nbsp;<span class="muted">(${si(fx, 'N')}, ${si(fy, 'N')})</span>`]);
      } else if (c instanceof RodC) {
        rows.push([name(c.el), 'rod', `${lam[r0] <= 0 ? 'tension' : 'compression'} ${si(Math.abs(lam[r0]), 'N')}`]);
      } else if (c instanceof SliderC) {
        let s = `N = ${si(lam[r0], 'N')}`;
        if (c.lockBody) s += `, M = ${si(lam[r0 + 1], 'N·m')}`;
        rows.push([name(c.el), 'track', s]);
      } else if (c instanceof RopeC) {
        const T = sys.ropeTensions(ci);
        const slack = T.some((t) => t < -1e-9) ? ' <span class="err">(negative: a real rope would go slack)</span>' : '';
        rows.push([name(c.el), 'rope', T.map((t, j) => `T${j + 1} = ${si(t, 'N')}`).join(', ') + slack]);
      }
    });
    for (const fe of sys.forces) {
      if (fe instanceof SpringF) {
        const pa = ptPos(fe.a, q), pb = ptPos(fe.b, q);
        const L = Math.hypot(pb.x - pa.x, pb.y - pa.y);
        rows.push([fe.name, 'spring', `${si(fe.k * (L - fe.L0), 'N')} at ℓ = ${si(L, 'm')}${fe.c ? ' (+ damping)' : ''}`]);
      } else if (fe instanceof TSpringF) {
        const out: import('../engine/forces').PointForce[] = [];
        fe.report(q, qd, lam, out);
        rows.push([fe.name, 'torsional', `τ on A = ${si(out[0]?.torque ?? 0, 'N·m')}`]);
      }
    }
    const lamRows: string[] = [];
    sys.cons.forEach((c, ci) => {
      c.labels.forEach((lb, r) => {
        lamRows.push(`<tr><td>${tex(`\\lambda_{${sys.rowStart[ci] + r + 1}}`, false)}</td><td class="l">${esc(name(c.el))}${lb ? ` · ${esc(lb.replace(/\\/g, ''))}` : ''}</td><td>${num(lam[sys.rowStart[ci] + r])}</td></tr>`);
      });
    });
    root.innerHTML = `<div class="muted">Live at t = ${sim.t.toFixed(3)} s. Constraint forces are the Lagrange multipliers λ resolved per joint (Baumgarte terms excluded).</div>
      <table class="data"><tr><th>element</th><th>type</th><th style="text-align:left">value</th></tr>${rows.map(([a, b, c]) => `<tr><td>${esc(a)}</td><td>${b}</td><td class="l">${c}</td></tr>`).join('')}</table>
      <div class="section-title">Multipliers λ</div>
      <table class="data"><tr><th></th><th style="text-align:left">constraint row</th><th>value</th></tr>${lamRows.join('')}</table>`;
  }
}
