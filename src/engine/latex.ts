// LaTeX for the equations panel: coordinates, constraint equations, applied
// forces, and the linearized equations of motion, all with the scene's numbers.

import type { Mat } from '../math/linalg';
import type { Linearization } from './analysis';
import { PinC, RodC, RopeC, SliderC } from './constraints';
import { FrictionF, SpringF, TSpringF } from './forces';
import type { APt } from './kin';
import { System } from './system';

/** Number formatting: 4 significant figures, scientific when extreme. */
export function num(v: number, sig = 4): string {
  if (!Number.isFinite(v)) return '\\text{NaN}';
  if (v === 0 || Math.abs(v) < 1e-12) return '0';
  const a = Math.abs(v);
  if (a >= 1e5 || a < 1e-3) {
    const e = Math.floor(Math.log10(a));
    const m = v / 10 ** e;
    return `${Number(m.toPrecision(sig))}\\times10^{${e}}`;
  }
  return `${Number(v.toPrecision(sig))}`;
}

export function bmatrix(M: Mat, sig = 4): string {
  if (!M.length) return '\\varnothing';
  return `\\begin{bmatrix}${M.map((r) => r.map((v) => num(v, sig)).join(' & ')).join(' \\\\ ')}\\end{bmatrix}`;
}

export function colVec(items: string[]): string {
  return `\\begin{bmatrix}${items.join(' \\\\ ')}\\end{bmatrix}`;
}

function coordTex(sys: System, b: NonNullable<APt['b']>, k: 'x' | 'y' | 'theta'): string {
  const c = sys.coords.find((c) => c.body === b && c.kind === k);
  return c ? c.tex : '?';
}

/** Linear combination "a·s1 + b·s2 + …" with tidy signs. */
function combo(terms: [number, string][]): string {
  let out = '';
  for (const [c, s] of terms) {
    if (Math.abs(c) < 1e-12) continue;
    const mag = Math.abs(c);
    const body = s === '' ? num(mag) : `${Math.abs(mag - 1) < 1e-12 ? '' : num(mag)}${s}`;
    out += out === '' ? (c < 0 ? `-${body}` : body) : c < 0 ? ` - ${body}` : ` + ${body}`;
  }
  return out || '0';
}

/** World x or y of an attachment point as an expression in the coordinates. */
export function pointExpr(sys: System, a: APt, axis: 0 | 1): string {
  if (!a.b) return num(axis ? a.ly : a.lx);
  const base = coordTex(sys, a.b, axis ? 'y' : 'x');
  if (a.b.dof === 2 || (a.lx === 0 && a.ly === 0)) return base;
  const th = coordTex(sys, a.b, 'theta');
  const terms: [number, string][] = axis === 0
    ? [[1, base], [a.lx, `\\cos${th}`], [-a.ly, `\\sin${th}`]]
    : [[1, base], [a.lx, `\\sin${th}`], [a.ly, `\\cos${th}`]];
  return combo(terms);
}

const paren = (s: string) => (/[+-]/.test(s.slice(1)) ? `\\left(${s}\\right)` : s);

/** "a − b" with numeric b folded into the sign. */
function sub(a: string, b: string): string {
  if (b === '0') return a;
  if (/^-?[\d.]+(\\times10\^\{-?\d+\})?$/.test(b)) return b.startsWith('-') ? `${a} + ${b.slice(1)}` : `${a} - ${b}`;
  return `${a} - ${paren(b)}`;
}

export interface TexLine {
  label: string;
  tex: string;
}

export function constraintTex(sys: System, q: ArrayLike<number>): TexLine[] {
  const out: TexLine[] = [];
  const nameOf = (id: string) => sys.scene.elements.find((e) => e.id === id.split(':')[0])?.name ?? id;
  for (const c of sys.cons) {
    const nm = nameOf(c.el);
    if (c instanceof PinC) {
      const lines = [0, 1].map((ax) => `${sub(pointExpr(sys, c.a, ax as 0 | 1), pointExpr(sys, c.b, ax as 0 | 1))} = 0`);
      out.push({ label: `${nm} (${c.kind === 'axle' ? 'fixed axle' : 'pin'})`, tex: `\\begin{aligned}${lines.join(' \\\\ ')}\\end{aligned}` });
    } else if (c instanceof RodC) {
      const dx = sub(pointExpr(sys, c.b, 0), pointExpr(sys, c.a, 0));
      const dy = sub(pointExpr(sys, c.b, 1), pointExpr(sys, c.a, 1));
      out.push({ label: `${nm} (rod)`, tex: `\\left(${dx}\\right)^2 + \\left(${dy}\\right)^2 - ${num(c.L)}^2 = 0` });
    } else if (c instanceof SliderC) {
      const X = pointExpr(sys, c.a, 0), Y = pointExpr(sys, c.a, 1);
      const main = combo([[c.nx, paren(sub(X, num(c.ox)))], [c.ny, paren(sub(Y, num(c.oy)))]]);
      let tex = `${main} = 0`;
      if (c.lockBody) tex = `\\begin{aligned}${tex} \\\\ ${sub(coordTex(sys, c.lockBody, 'theta'), num(c.theta0))} = 0\\end{aligned}`;
      out.push({ label: `${nm} (track)`, tex });
    } else if (c instanceof RopeC) {
      const g = c.geometry(q);
      const segs = g.pieces.map((p) => num(p.len)).join(' + ');
      const arcs = g.wraps.map((w, i) => `${num((c.nodes[i + 1] as any).r)}\\cdot${num(w)}`).join(' + ');
      const np = c.nodes.length - 2;
      const tex = `\\begin{aligned}&\\textstyle\\sum_j \\ell_j(\\mathbf q) + \\sum_i r_i\\varphi_i(\\mathbf q) = L = ${num(c.L)} \\\\ &\\quad\\text{now: } ${segs}${arcs ? ` + ${arcs}` : ''} = ${num(g.total)}${np ? ` \\\\ &\\hat{\\mathbf u}_j\\cdot(\\mathbf v_{T_{j+1}} - \\mathbf v_{T_j}) = 0,\\; j = 0..${np - 1}\\;\\text{(no slip)}` : ''}\\end{aligned}`;
      out.push({ label: `${nm} (rope)`, tex });
    }
  }
  return out;
}

export function forceTex(sys: System): TexLine[] {
  const out: TexLine[] = [];
  if (sys.g !== 0) out.push({ label: 'gravity', tex: `\\mathbf F_i = -m_i\\,${num(sys.g)}\\,\\hat{\\mathbf y},\\qquad V_g = \\textstyle\\sum_i m_i g\\,y_i` });
  for (const f of sys.forces) {
    if (f instanceof SpringF) {
      const dx = sub(pointExpr(sys, f.b, 0), pointExpr(sys, f.a, 0));
      const dy = sub(pointExpr(sys, f.b, 1), pointExpr(sys, f.a, 1));
      const damp = f.c ? ` + ${num(f.c)}\\,\\dot\\ell` : '';
      out.push({
        label: `${f.name} (spring)`,
        tex: `\\begin{aligned}\\ell &= \\sqrt{\\left(${dx}\\right)^2 + \\left(${dy}\\right)^2} \\\\ T &= ${num(f.k)}\\,(\\ell - ${num(f.L0)})${damp},\\quad V = \\tfrac12\\,${num(f.k)}\\,(\\ell - ${num(f.L0)})^2\\end{aligned}`,
      });
    } else if (f instanceof TSpringF) {
      const ta = f.a.b ? coordTex(sys, f.a.b, 'theta') : num(f.a.angle);
      const tb = f.b.b && f.b.b.dof === 3 ? coordTex(sys, f.b.b, 'theta') : num(f.b.angle);
      const rel = sub(sub(ta, tb), num(f.theta0));
      const damp = f.c ? ` - ${num(f.c)}\\,\\dot\\Delta` : '';
      out.push({ label: `${f.name} (torsional)`, tex: `\\tau = -${num(f.k)}\\,\\Delta${damp},\\quad \\Delta = ${rel},\\quad V = \\tfrac12\\,${num(f.k)}\\,\\Delta^2` });
    } else if (f instanceof FrictionF) {
      out.push({ label: `${f.name} (friction)`, tex: `F_f = -${num(f.mu)}\\,|N|\\,\\tanh(v_t/10^{-3})` });
    }
  }
  return out;
}

/** The constrained DAE solved at every RK4 stage. */
export function daeTex(sys: System): string {
  return `\\begin{aligned}
\\mathbf M\\,\\ddot{\\mathbf q} &= \\mathbf Q(\\mathbf q, \\dot{\\mathbf q}) + \\mathbf J^{\\mathsf T}\\boldsymbol\\lambda \\\\
\\mathbf J\\,\\ddot{\\mathbf q} &= -\\dot{\\mathbf J}\\dot{\\mathbf q} - ${num(2 * sys.alpha)}\\,\\mathbf J\\dot{\\mathbf q} - ${num(sys.beta * sys.beta)}\\,\\mathbf g(\\mathbf q) \\\\
\\Rightarrow\\; (\\mathbf J\\mathbf M^{-1}\\mathbf J^{\\mathsf T})\\,\\boldsymbol\\lambda &= -\\dot{\\mathbf J}\\dot{\\mathbf q} - 2\\alpha\\mathbf J\\dot{\\mathbf q} - \\beta^2\\mathbf g - \\mathbf J\\mathbf M^{-1}\\mathbf Q
\\end{aligned}`;
}

export function massTex(sys: System): string {
  return `\\mathbf M = \\operatorname{diag}\\left(${Array.from(sys.mass, (m) => num(m)).join(',\\ ')}\\right)`;
}

export function energyTex(sys: System): string {
  const parts = sys.bodies.map((b) => {
    const x = coordTex(sys, b, 'x'), y = coordTex(sys, b, 'y');
    const tr = `${num(b.m)}\\left(\\dot ${x}^2 + \\dot ${y}^2\\right)`;
    return b.dof === 3 ? `${tr} + ${num(b.I)}\\,\\dot ${coordTex(sys, b, 'theta')}^2` : tr;
  });
  return `T = \\tfrac12\\left[${parts.join(' + ')}\\right]`;
}

/** Linearized EOM written out row by row. */
export function linearEomTex(sys: System, lin: Linearization): string {
  const z = lin.indep.map((k) => sys.coords[k].tex);
  const rows = lin.M.map((_, i) => {
    const terms: [number, string][] = [];
    lin.M[i].forEach((v, j) => terms.push([v, `\\ddot ${z[j]}`]));
    lin.C[i].forEach((v, j) => terms.push([v, `\\dot ${z[j]}`]));
    lin.K[i].forEach((v, j) => terms.push([v, `\\,${z[j]}`]));
    return `${combo(terms)} &= 0`;
  });
  return `\\begin{aligned}${rows.join(' \\\\ ')}\\end{aligned}`;
}

export function zVecTex(sys: System, lin: Linearization): string {
  return colVec(lin.indep.map((k) => sys.coords[k].tex));
}
