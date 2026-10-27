# Mechanism Sandbox

A 2D sandbox for building classic planar dynamics problems (carts on frictionless floors, pivoted bars, pendulums, linear and torsional springs, ropes over massive pulleys) and simulating them with all the math shown.

```bash
npm install
npm run dev      # http://localhost:5173
npm test         # verification suite (Vitest)
npm run build    # typecheck + production bundle
```

## Using it

- **Palette (left):** drag an item onto the canvas, or click it and then click in the scene.
  - Ground, point mass, rigid body, pulley: click to place.
  - Pin: click a body. It pins to the body underneath, to a ground piece, or to the world.
  - Rod / spring: click two points. An end on empty space is fixed to the world.
  - Slider: click a body to put it on a straight track through that point. Rotation is locked for rigid bodies (a cart).
  - Torsional spring: click a pin.
  - Rope: click the first end, then each pulley in order, then the other end. The wrap direction is chosen automatically and can be changed in the properties panel.
- **Select / move:** click to select, drag bodies to move them. The rest of the mechanism re-assembles around the dragged body. `[` `]` rotates, the arrow keys nudge, Del deletes, Ctrl/⌘ Z and Shift Z undo and redo. Drag empty space to pan, scroll to zoom.
- **Properties (right):** mass, inertia (leave it blank for the uniform-body value), size, initial position, angle, velocity, k, L₀, c, μ, rope length, and so on. All units are SI; angles are entered in degrees. A coordinate you type is held fixed while the constraints are re-satisfied.
- **Toolbar:** play/pause (Space), step 10 ms, reset (R), time scale, gravity, snap, free body diagram, labels, presets, and save/load as JSON. The scene is also autosaved to localStorage.

### The calculations panel

- **Plots:** any coordinate against time, plus kinetic energy, potential energy (relative to t = 0), and total energy. Energy drift is shown as (E − E₀) / max T in percent.
- **Equilibrium & modes:** Newton's method on the KKT system finds static equilibrium. The panel picks independent coordinates (cart positions, bar angles and so on), linearizes to get **M**, **K** (and **C** if there is damping), then solves det(**K** − ω²**M**) = 0 for the natural frequencies, damping ratios and mode shapes. *Animate* plays each mode.
- **Equations:** the generalized coordinates, the mass matrix, kinetic energy, the constrained DAE with its Baumgarte gains, every constraint equation g(q) = 0 and applied-force law with the scene's numbers, and the linearized equations of motion.
- **Forces:** live pin reactions, rod tensions, track normal forces and moments, rope tension in each segment, spring forces, and the raw multipliers λ.

## How it works

Every body uses maximal coordinates: (x, y) for point masses, (x, y, θ) for rigid bodies and pulleys. Constraint forces come from Lagrange multipliers:

```
M q̈ = Q(q, q̇) + Jᵀλ
J q̈ = −J̇q̇ − 2α J q̇ − β² g(q)          (Baumgarte, α = β = 10 s⁻¹)
```

`M` is diagonal, so each evaluation solves the small system `(J M⁻¹ Jᵀ) λ = …` with rank-revealing full-pivot elimination, which keeps redundant constraints harmless. Integration is fixed-step RK4 (1 ms by default), run independently of the render frame rate.

**Ropes over pulleys:** the straight runs are common tangents (signed-radius formula). One holonomic row fixes the total length, Σℓ + Σrφ = L. One no-slip row per pulley says the rope material between pulleys is conserved, `u·(v(T₂) − v(T₁)) = 0`, with the tangent points treated as points on the pulleys. That couples pulley spin to rope motion, which is how pulley inertia enters the dynamics. `J̇q̇` for the rope rows is a central difference along q̇.

See [PLAN.md](PLAN.md) for the architecture and milestones.

## Verification (`npm test`)

| Check | Result |
|---|---|
| Simple pendulum, 2° amplitude: period vs 2π√(L/g) | within 1% (also exact ω from linearization) |
| Wall–k–m–k–m–k–wall: ω vs √(k/m), √(3k/m) | matches to 1e-6, correct mode shapes |
| Atwood with massive pulley: a vs (m₁−m₂)g/(m₁+m₂+I/r²) | within 0.1%; tensions and ω = a t / r match |
| Free pulley + fixed massive pulley | matches the analytic acceleration |
| Energy over 60 s (pendulum at 90°, chaotic double pendulum, springs, spring-loaded Atwood, all three presets) | drift < 0.1% of peak kinetic energy |
| Presets | assemble cleanly, have a stable equilibrium and the expected number of DOFs |

## Limitations

- Ropes are bilateral: they never go slack. The Forces tab flags any negative tension.
- Friction on sliders is a regularized Coulomb law, μ|N| tanh(v / 1 mm/s), using N from the previous solve, so there is no true stiction.
- No collisions or contact between bodies.
- A point mass has no rotational coordinate, so a torsional spring on its pin does nothing.
