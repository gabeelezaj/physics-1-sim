# Mechanism Sandbox — plan

A browser sandbox for building planar multibody "nightmare" dynamics problems
(carts, pivoted bars, pendulums, linear/torsional springs, ropes over massive
pulleys) and simulating them with every intermediate quantity shown.

## Stack

- Vite + TypeScript + `<canvas>`; no backend (`npm run dev`).
- KaTeX for equations. Own dense linear algebra (`src/math/linalg.ts`).
- Vitest for the verification suite (`npm test`).

## Architecture

```
src/
  math/linalg.ts        dense solve (full-pivot GE, rank aware), Cholesky,
                        Jacobi symmetric eigen, generalized eigenproblem
  model/types.ts        Scene JSON schema (the only persisted format)
  model/factory.ts      element constructors, ids, default names
  engine/system.ts      compile Scene -> System: DOF layout, bodies,
                        constraint + force objects, energy
  engine/kin.ts         body-point position / velocity / Jacobian helpers
  engine/constraints.ts pin, rod, slider (+rotation lock), axle, rope rows
  engine/rope.ts        rope routing geometry (tangents, wraps, lengths)
  engine/forces.ts      gravity, spring-dampers, torsional springs, friction
  engine/sim.ts         Lagrange-multiplier accel solve, Baumgarte, RK4,
                        assembly (projection onto the constraint manifold)
  engine/analysis.ts    equilibrium (Newton on KKT), linearization,
                        independent-coordinate selection, modes
  engine/latex.ts       LaTeX for coordinates, constraints, M, K, EOM
  ui/*                  canvas renderer, editor tools, properties panel,
                        plots, analysis panel, history (undo/redo)
  presets.ts            the three preset scenes
tests/                  engine verification
```

### Dynamics

Maximal coordinates: each point mass has `(x, y)`, each rigid body / pulley
has `(x, y, θ)`. Ground pieces have no DOFs.

```
M q̈ = F(q, q̇) + Jᵀλ
J q̈ = −J̇q̇ − 2α J q̇ − β² g(q)        (Baumgarte, α = β = 10 s⁻¹ default)
```

`M` is diagonal, so each evaluation solves the small system
`(J M⁻¹ Jᵀ) λ = rhs − J M⁻¹ F` with a rank-revealing full-pivot elimination
(redundant constraints get λ = 0 instead of blowing up). RK4 at a fixed
`dt = 1 ms`, decoupled from rendering via an accumulator.

### Ropes over pulleys

A rope is `end A → pulley₁ → … → pulleyₙ → end B` with a wrap direction per
pulley. Straight segments are common tangents (signed-radius formula);
wrap arcs are measured between tangent points.

- One holonomic row: total length `Σℓⱼ + Σ rᵢφᵢ − L = 0`.
- `n` no-slip rows: for each piece `j < n`, the rope material between
  material points on consecutive pulleys is constant, whose rate is
  `uⱼ · (v(Tⱼ₊₁) − v(Tⱼ)) = 0` with `T` the tangent points treated as
  body-fixed points (so pulley rotation enters). This couples pulley spin to
  rope motion, so pulley inertia appears in the dynamics.
- `J̇q̇` for rope rows is a central difference of `J(q)q̇` along `q̇`.

### Analysis

- Equilibrium: Newton on `F(q) + Jᵀλ = 0, g(q) = 0` with a numeric Jacobian
  and line search.
- Linearization: `K = −∂(F + Jᵀλ*)/∂q` (includes constraint-curvature
  stiffness), `C = −∂F/∂q̇`. Pick independent coordinates greedily (slider
  directions and bar angles preferred), build the null-space map
  `δq = N δz`, then `M_r = NᵀMN`, `K_r = NᵀKN`. Solve `K_r φ = ω² M_r φ`
  via Cholesky + Jacobi.

## Milestones (commit after each, tests green before moving on)

1. Engine core + tests (pendulum period, 2-mass 3-spring modes, energy).
2. Basic elements + canvas rendering (bodies, ground, springs, pins, rods).
3. Constraints + pulleys/ropes (Atwood test with massive pulley).
4. Analysis panel (plots, energy drift, equilibrium, modes, LaTeX, FBD).
5. Editor UI polish (palette DnD, snap, select/move/delete, undo/redo, JSON).
6. Presets (three scenes) + 60 s energy tests on them.
