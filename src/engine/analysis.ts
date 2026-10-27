// Static equilibrium, linearization and normal modes.
//
// Equilibrium solves the KKT system  F(q) + J(q)ᵀλ = 0,  g(q) = 0  by Newton.
// Linearization about (q*, λ*) uses
//   K = −∂(F + Jᵀλ*)/∂q   (includes constraint-curvature stiffness)
//   C = −∂F/∂q̇
// restricted to the constraint tangent space δq = N δz, where z are actual
// maximal coordinates chosen as independent (bar angles, cart positions, …).

import {
  Mat, congruenceDiag, generalizedEigen, matMul, solve, symmetrize, transpose, zeros,
} from '../math/linalg';
import { SliderC } from './constraints';
import { System } from './system';

export interface Equilibrium {
  q: Float64Array;
  lambda: Float64Array;
  converged: boolean;
  forceResidual: number;
  constraintResidual: number;
  iterations: number;
}

function generalizedTotal(sys: System, q: ArrayLike<number>, lambda: ArrayLike<number>, out: Float64Array): void {
  const zero = new Float64Array(sys.n);
  sys.appliedForce(q, zero, out, false);
  const { J } = sys.evalConstraints(q, zero, false);
  const { n, m } = sys;
  for (let i = 0; i < m; i++) {
    const l = lambda[i];
    if (l === 0) continue;
    for (let k = 0; k < n; k++) out[k] += J[i * n + k] * l;
  }
}

/** −∂(F + Jᵀλ)/∂q by central differences. */
export function stiffnessMatrix(sys: System, q: ArrayLike<number>, lambda: ArrayLike<number>, h = 1e-6): Mat {
  const n = sys.n;
  const K = zeros(n, n);
  const qp = Float64Array.from(q);
  const gp = new Float64Array(n), gm = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    const q0 = qp[j];
    qp[j] = q0 + h;
    generalizedTotal(sys, qp, lambda, gp);
    qp[j] = q0 - h;
    generalizedTotal(sys, qp, lambda, gm);
    qp[j] = q0;
    for (let i = 0; i < n; i++) K[i][j] = -(gp[i] - gm[i]) / (2 * h);
  }
  return K;
}

/** −∂F/∂q̇ at rest. */
export function dampingMatrix(sys: System, q: ArrayLike<number>, h = 1e-6): Mat {
  const n = sys.n;
  const C = zeros(n, n);
  const v = new Float64Array(n);
  const fp = new Float64Array(n), fm = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    v[j] = h;
    sys.appliedForce(q, v, fp, false);
    v[j] = -h;
    sys.appliedForce(q, v, fm, false);
    v[j] = 0;
    for (let i = 0; i < n; i++) C[i][j] = -(fp[i] - fm[i]) / (2 * h);
  }
  return C;
}

export function findEquilibrium(sys: System, q0: ArrayLike<number>, maxIter = 100): Equilibrium {
  const { n, m } = sys;
  const holo = sys.holoMask();
  const q = Float64Array.from(q0);
  let lambda = new Float64Array(m);
  const zero = new Float64Array(n);
  const G = new Float64Array(n);

  const scaleF = (() => {
    sys.appliedForce(q, zero, G, false);
    let s = 1e-9;
    for (const b of sys.bodies) s = Math.max(s, b.m * Math.max(sys.g, 1));
    for (let k = 0; k < n; k++) s = Math.max(s, Math.abs(G[k]));
    return s;
  })();

  const residual = (qq: Float64Array, ll: Float64Array) => {
    generalizedTotal(sys, qq, ll, G);
    const { c } = sys.evalConstraints(qq, zero, false);
    let rf = 0, rc = 0, merit = 0;
    for (let k = 0; k < n; k++) { rf = Math.max(rf, Math.abs(G[k])); merit += (G[k] / scaleF) ** 2; }
    for (let i = 0; i < m; i++) if (holo[i]) { rc = Math.max(rc, Math.abs(c[i])); merit += c[i] ** 2; }
    return { rf, rc, merit, R: [...G, ...Array.from(c, (v, i) => (holo[i] ? v : 0))] };
  };

  // initial multipliers: least squares on F + Jᵀλ = 0
  {
    sys.appliedForce(q, zero, G, false);
    const { J } = sys.evalConstraints(q, zero, false);
    const A = zeros(m, m), b = new Array(m).fill(0);
    for (let i = 0; i < m; i++) {
      for (let j = 0; j < m; j++) {
        let s = 0;
        for (let k = 0; k < n; k++) s += J[i * n + k] * J[j * n + k];
        A[i][j] = s;
      }
      let s = 0;
      for (let k = 0; k < n; k++) s -= J[i * n + k] * G[k];
      b[i] = s;
    }
    if (m) lambda = Float64Array.from(solve(A, b, 1e-12).x);
  }

  let r = residual(q, lambda);
  let it = 0;
  for (; it < maxIter; it++) {
    if (r.rf < 1e-9 * scaleF && r.rc < 1e-11) break;
    const K = stiffnessMatrix(sys, q, lambda); // = −∂R₁/∂q
    const { J } = sys.evalConstraints(q, zero, false);
    const N = n + m;
    const A = zeros(N, N);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) A[i][j] = -K[i][j];
    for (let i = 0; i < m; i++) {
      for (let k = 0; k < n; k++) {
        const v = J[i * n + k];
        A[k][n + i] = v; // ∂R₁/∂λ = Jᵀ
        A[n + i][k] = v; // ∂g/∂q (no-slip rows: J δq = 0)
      }
    }
    const rhs = r.R.map((v) => -v);
    const { x } = solve(A, rhs, 1e-13);
    let step = 1;
    let accepted = false;
    for (let ls = 0; ls < 30; ls++) {
      const qn = Float64Array.from(q, (v, k) => v + step * x[k]);
      const ln = Float64Array.from(lambda, (v, i) => v + step * x[n + i]);
      const rn = residual(qn, ln);
      if (rn.merit < r.merit || rn.merit < 1e-24) {
        q.set(qn);
        lambda = ln;
        r = rn;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) break;
  }
  return {
    q, lambda,
    converged: r.rf < 1e-6 * scaleF && r.rc < 1e-8,
    forceResidual: r.rf,
    constraintResidual: r.rc,
    iterations: it,
  };
}

/** Preference for a coordinate to be kept independent (higher = keep). */
function coordScores(sys: System): number[] {
  const scores: number[] = [];
  const sliderOf = new Map<string, SliderC>();
  for (const c of sys.cons) if (c instanceof SliderC && c.a.b) sliderOf.set(c.a.b.id, c);
  for (const c of sys.coords) {
    const b = c.body;
    const s = sliderOf.get(b.id);
    let v: number;
    if (b.kind === 'point') v = c.kind === 'x' ? 3 : 2.5;
    else if (b.kind === 'rect') v = c.kind === 'theta' ? 4 : c.kind === 'x' ? 2 : 1.5;
    else v = c.kind === 'theta' ? 1.8 : c.kind === 'x' ? 1 : 0.9;
    if (s) {
      const alongX = Math.abs(Math.cos(s.angle)) >= Math.abs(Math.sin(s.angle));
      if (c.kind === 'x') v = alongX ? 5 : 0.5;
      if (c.kind === 'y') v = alongX ? 0.5 : 5;
      if (c.kind === 'theta' && s.lockBody) v = 0;
    }
    scores.push(v);
  }
  return scores;
}

export interface Linearization {
  q: Float64Array;
  lambda: Float64Array;
  /** Indices (into q) of the independent coordinates z. */
  indep: number[];
  dep: number[];
  /** δq = N δz (n × k). */
  N: Mat;
  M: Mat;
  K: Mat;
  C: Mat;
  omega2: number[];
  omega: number[];
  freqHz: number[];
  zeta: number[];
  /** Mode shapes in z (columns, max |entry| = 1). */
  modesZ: Mat;
  /** Mode shapes in full maximal coordinates (columns). */
  modesQ: Mat;
}

/**
 * Choose which maximal coordinates are independent: greedily mark the least
 * "natural" coordinates as dependent (Gram–Schmidt on the columns of J)
 * until they span the constraint Jacobian's column space.
 */
export function chooseIndependent(sys: System, q: ArrayLike<number>): { indep: number[]; dep: number[]; J: Float64Array } {
  const { n, m } = sys;
  const zero = new Float64Array(n);
  const Jc = Float64Array.from(sys.evalConstraints(q, zero, false).J);
  const col = (k: number) => Array.from({ length: m }, (_, i) => Jc[i * n + k]);
  const scores = coordScores(sys);
  const order = [...Array(n).keys()].sort((a, b) => scores[a] - scores[b] || a - b);
  const basis: number[][] = [];
  const dep: number[] = [];
  for (const k of order) {
    const v = col(k);
    const nv = Math.hypot(...v);
    if (nv < 1e-12) continue;
    for (const e of basis) {
      let d = 0;
      for (let i = 0; i < m; i++) d += e[i] * v[i];
      for (let i = 0; i < m; i++) v[i] -= d * e[i];
    }
    const nr = Math.hypot(...v);
    if (nr > 1e-8 * nv) {
      basis.push(v.map((x) => x / nr));
      dep.push(k);
    }
  }
  const depSet = new Set(dep);
  const indep = [...Array(n).keys()].filter((k) => !depSet.has(k));
  return { indep, dep, J: Jc };
}

export function linearize(sys: System, q: Float64Array, lambda: Float64Array): Linearization {
  const { n, m } = sys;
  const { indep, dep, J: Jc } = chooseIndependent(sys, q);
  const k = indep.length, r = dep.length;

  // δq_D = X δq_I with (J_Dᵀ J_D) X = −J_Dᵀ J_I
  const N = zeros(n, k);
  indep.forEach((qi, j) => (N[qi][j] = 1));
  if (r > 0 && k > 0) {
    const JD = Array.from({ length: m }, (_, i) => dep.map((d) => Jc[i * n + d]));
    const JI = Array.from({ length: m }, (_, i) => indep.map((d) => Jc[i * n + d]));
    const JDt = transpose(JD);
    const A = matMul(JDt, JD);
    const B = matMul(JDt, JI);
    for (let j = 0; j < k; j++) {
      const { x } = solve(A, B.map((row) => -row[j]), 1e-14);
      dep.forEach((d, i) => (N[d][j] = x[i]));
    }
  }

  const Kfull = symmetrize(stiffnessMatrix(sys, q, lambda));
  const Cfull = symmetrize(dampingMatrix(sys, q));
  const Nt = transpose(N);
  const M = congruenceDiag(N, sys.mass);
  const K = symmetrize(matMul(Nt, matMul(Kfull, N)));
  const C = symmetrize(matMul(Nt, matMul(Cfull, N)));
  clean(M); clean(K); clean(C);

  const { values, vectors } = generalizedEigen(K, M);
  const modesZ = vectors.map((row) => row.slice());
  for (let j = 0; j < k; j++) {
    let big = 0;
    for (let i = 0; i < k; i++) if (Math.abs(modesZ[i][j]) > Math.abs(big)) big = modesZ[i][j];
    if (big !== 0) for (let i = 0; i < k; i++) modesZ[i][j] /= big;
  }
  const modesQ = matMul(N, modesZ);
  const omega = values.map((v) => Math.sign(v) * Math.sqrt(Math.abs(v)));
  const zeta = values.map((w2, j) => {
    if (w2 <= 0) return 0;
    const phi = modesZ.map((row) => row[j]);
    const quad = (A: Mat) => phi.reduce((s, pi, a) => s + pi * A[a].reduce((t, v, b) => t + v * phi[b], 0), 0);
    return quad(C) / (2 * Math.sqrt(w2) * quad(M));
  });
  return {
    q, lambda, indep, dep, N, M, K, C,
    omega2: values, omega,
    freqHz: omega.map((w) => w / (2 * Math.PI)),
    zeta, modesZ, modesQ,
  };
}

/** Zero out finite-difference noise relative to the matrix scale. */
function clean(A: Mat): void {
  let s = 0;
  for (const row of A) for (const v of row) s = Math.max(s, Math.abs(v));
  const tol = 1e-9 * s;
  for (const row of A) for (let j = 0; j < row.length; j++) if (Math.abs(row[j]) < tol) row[j] = 0;
}
