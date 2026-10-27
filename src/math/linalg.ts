// Small dense linear algebra for the engine and the modal analysis.
// Matrices are number[][] (row-major) except in the hot path, which uses
// flat Float64Array buffers.

export type Mat = number[][];

export function zeros(r: number, c: number): Mat {
  const m: Mat = new Array(r);
  for (let i = 0; i < r; i++) m[i] = new Array(c).fill(0);
  return m;
}

export function identity(n: number): Mat {
  const m = zeros(n, n);
  for (let i = 0; i < n; i++) m[i][i] = 1;
  return m;
}

export function transpose(a: Mat): Mat {
  const r = a.length, c = r ? a[0].length : 0;
  const t = zeros(c, r);
  for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) t[j][i] = a[i][j];
  return t;
}

export function matMul(a: Mat, b: Mat): Mat {
  const r = a.length, k = b.length, c = k ? b[0].length : 0;
  const out = zeros(r, c);
  for (let i = 0; i < r; i++) {
    const ai = a[i], oi = out[i];
    for (let p = 0; p < k; p++) {
      const v = ai[p];
      if (v === 0) continue;
      const bp = b[p];
      for (let j = 0; j < c; j++) oi[j] += v * bp[j];
    }
  }
  return out;
}

export function matVec(a: Mat, x: ArrayLike<number>): number[] {
  return a.map((row) => {
    let s = 0;
    for (let j = 0; j < row.length; j++) s += row[j] * x[j];
    return s;
  });
}

/** Aᵀ diag(d) A — used for NᵀMN with a diagonal M. */
export function congruenceDiag(a: Mat, d: ArrayLike<number>): Mat {
  const n = a.length, k = n ? a[0].length : 0;
  const out = zeros(k, k);
  for (let p = 0; p < n; p++) {
    const ap = a[p], dp = d[p];
    if (dp === 0) continue;
    for (let i = 0; i < k; i++) {
      const v = ap[i] * dp;
      if (v === 0) continue;
      for (let j = 0; j < k; j++) out[i][j] += v * ap[j];
    }
  }
  return out;
}

export function symmetrize(a: Mat): Mat {
  const n = a.length;
  const s = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) s[i][j] = 0.5 * (a[i][j] + a[j][i]);
  return s;
}

export function norm(x: ArrayLike<number>): number {
  let s = 0;
  for (let i = 0; i < x.length; i++) s += x[i] * x[i];
  return Math.sqrt(s);
}

/**
 * Solve A x = b with full pivoting, in place on flat buffers (A is n×n,
 * row-major). Rank deficient systems are handled by zeroing the unknowns
 * whose pivots fall below `relTol · max|A|`: this gives a particular
 * solution, which is exactly what redundant constraints need (their extra
 * multipliers are arbitrary). Returns the numerical rank.
 */
export function solveFullPivotInPlace(
  A: Float64Array,
  b: Float64Array,
  n: number,
  x: Float64Array,
  relTol = 1e-11,
  colPerm?: Int32Array,
): number {
  const perm = colPerm ?? new Int32Array(n);
  for (let i = 0; i < n; i++) perm[i] = i;
  let maxAbs = 0;
  for (let i = 0; i < n * n; i++) maxAbs = Math.max(maxAbs, Math.abs(A[i]));
  const tol = relTol * (maxAbs || 1);
  let rank = 0;
  for (let k = 0; k < n; k++) {
    // pivot search over the trailing submatrix
    let pr = k, pc = k, best = 0;
    for (let i = k; i < n; i++) {
      const row = i * n;
      for (let j = k; j < n; j++) {
        const v = Math.abs(A[row + j]);
        if (v > best) { best = v; pr = i; pc = j; }
      }
    }
    if (best <= tol) break;
    rank++;
    if (pr !== k) {
      for (let j = 0; j < n; j++) {
        const t = A[k * n + j]; A[k * n + j] = A[pr * n + j]; A[pr * n + j] = t;
      }
      const t = b[k]; b[k] = b[pr]; b[pr] = t;
    }
    if (pc !== k) {
      for (let i = 0; i < n; i++) {
        const t = A[i * n + k]; A[i * n + k] = A[i * n + pc]; A[i * n + pc] = t;
      }
      const t = perm[k]; perm[k] = perm[pc]; perm[pc] = t;
    }
    const piv = A[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = A[i * n + k] / piv;
      if (f === 0) continue;
      A[i * n + k] = 0;
      for (let j = k + 1; j < n; j++) A[i * n + j] -= f * A[k * n + j];
      b[i] -= f * b[k];
    }
  }
  const y = new Float64Array(n);
  for (let k = rank - 1; k >= 0; k--) {
    let s = b[k];
    for (let j = k + 1; j < rank; j++) s -= A[k * n + j] * y[j];
    y[k] = s / A[k * n + k];
  }
  for (let i = 0; i < n; i++) x[perm[i]] = y[i];
  return rank;
}

/** Convenience wrapper on number[][]; does not modify its inputs. */
export function solve(a: Mat, b: ArrayLike<number>, relTol = 1e-11): { x: number[]; rank: number } {
  const n = a.length;
  const A = new Float64Array(n * n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) A[i * n + j] = a[i][j];
  const B = Float64Array.from(b);
  const X = new Float64Array(n);
  const rank = solveFullPivotInPlace(A, B, n, X, relTol);
  return { x: Array.from(X), rank };
}

/** Lower-triangular Cholesky factor of an SPD matrix; throws if not SPD. */
export function cholesky(a: Mat): Mat {
  const n = a.length;
  const L = zeros(n, n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = a[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k];
      if (i === j) {
        if (s <= 0) throw new Error('matrix is not positive definite');
        L[i][i] = Math.sqrt(s);
      } else {
        L[i][j] = s / L[j][j];
      }
    }
  }
  return L;
}

/** Solve L X = B (L lower triangular) column by column. */
export function forwardSubst(L: Mat, B: Mat): Mat {
  const n = L.length, c = B[0]?.length ?? 0;
  const X = zeros(n, c);
  for (let col = 0; col < c; col++) {
    for (let i = 0; i < n; i++) {
      let s = B[i][col];
      for (let k = 0; k < i; k++) s -= L[i][k] * X[k][col];
      X[i][col] = s / L[i][i];
    }
  }
  return X;
}

/** Solve Lᵀ X = B (L lower triangular). */
export function backSubstT(L: Mat, B: Mat): Mat {
  const n = L.length, c = B[0]?.length ?? 0;
  const X = zeros(n, c);
  for (let col = 0; col < c; col++) {
    for (let i = n - 1; i >= 0; i--) {
      let s = B[i][col];
      for (let k = i + 1; k < n; k++) s -= L[k][i] * X[k][col];
      X[i][col] = s / L[i][i];
    }
  }
  return X;
}

/**
 * Cyclic Jacobi eigen-decomposition of a symmetric matrix.
 * Returns eigenvalues ascending and eigenvectors as columns.
 */
export function jacobiEigen(s: Mat, maxSweeps = 100): { values: number[]; vectors: Mat } {
  const n = s.length;
  const a = s.map((r) => r.slice());
  const v = identity(n);
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let off = 0, diag = 0;
    for (let i = 0; i < n; i++) {
      diag += a[i][i] * a[i][i];
      for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    }
    if (off <= 1e-30 * (diag || 1)) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = a[p][q];
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (a[q][q] - a[p][p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), sn = t * c;
        for (let k = 0; k < n; k++) {
          const akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - sn * akq;
          a[k][q] = sn * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - sn * aqk;
          a[q][k] = sn * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - sn * vkq;
          v[k][q] = sn * vkp + c * vkq;
        }
      }
    }
  }
  const order = [...Array(n).keys()].sort((i, j) => a[i][i] - a[j][j]);
  return {
    values: order.map((i) => a[i][i]),
    vectors: v.map((row) => order.map((i) => row[i])),
  };
}

/**
 * Generalized symmetric eigenproblem K φ = λ M φ with M SPD.
 * Eigenvectors are M-orthonormal columns, eigenvalues ascending.
 */
export function generalizedEigen(K: Mat, M: Mat): { values: number[]; vectors: Mat } {
  const n = K.length;
  if (n === 0) return { values: [], vectors: [] };
  const L = cholesky(M);
  // A = L⁻¹ K L⁻ᵀ
  const Y = forwardSubst(L, K); // L⁻¹ K
  const A = transpose(forwardSubst(L, transpose(Y))); // (L⁻¹ (L⁻¹K)ᵀ)ᵀ
  const { values, vectors } = jacobiEigen(symmetrize(A));
  return { values, vectors: backSubstT(L, vectors) };
}
