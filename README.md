# Mechanism Sandbox

A 2D sandbox for building classic planar dynamics problems (carts on frictionless floors, pivoted bars, pendulums, linear and torsional springs, ropes over massive pulleys) and simulating them with all the math shown.

**Live demo:** https://gabeelezaj.github.io/physics-1-sim/

```bash
npm install
npm run dev      # http://localhost:5173
npm test         # verification suite (Vitest)
npm run build    # typecheck + production bundle
```

On a Mac you can also double-click **Start Sandbox.command** in Finder: it installs dependencies on first run, starts the server and opens the app in your browser. Close its Terminal window to stop it.

## Using it

- **Palette (left):** drag an item onto the canvas, or click it and then click in the scene.
  - Ground: click two points to draw a floor, wall or ceiling between them. The angle snaps to 15° (hold Shift for a free angle), and the solid side faces away from your bodies. Select a ground piece to drag its square end handles, or use *Flip solid side*.
  - Ramp / wedge (**W**): click the bottom of the slope, then the top. The angle snaps to 5°. The properties panel sets the slope angle, base, height and which way it rises; tick **movable** to turn the fixed ramp into a wedge that slides on the floor.
  - Point mass, rigid body, pulley: click to place. Shift-click places several.
  - Pin: click a body. It pins to the body underneath, to a ground piece, or to the world.
  - Rod / spring: click two points. An end on empty space is fixed to the world.
  - Slider: click a body to put it on a straight track through that point. Rotation is locked for rigid bodies (a cart).
  - Torsional spring: click a pin.
  - Rope: click the first end, then each pulley in order, then the other end. The wrap direction is chosen automatically and can be changed in the properties panel.
- **Keyboard:** every tool has a letter (shown on hover in the palette): V select, G ground, M mass, B body, P pulley, J pin, L rod, T track, S spring, Q torsional spring, C rope.
- **Select / move:** click to select, drag bodies to move them. The rest of the mechanism re-assembles around the dragged body. `[` `]` rotates, the arrow keys nudge, Del deletes, Ctrl/⌘ Z and Shift Z undo and redo. Drag empty space to pan, scroll to zoom.
- **Properties (right):** mass, inertia (leave it blank for the uniform-body value), size, initial position, angle, velocity, k, L₀, c, μ, rope length, and so on. All units are SI; angles are entered in degrees. A coordinate you type is held fixed while the constraints are re-satisfied.
- **Scene menu** (top left): new, open, save as JSON, and ten presets: the three original textbook problems, colliding carts, a block on a sliding wedge, a skate park, Newton's cradle, domino amplification (each domino 1.25× the last), a chaotic double pendulum, and a ten-stage Rube Goldberg machine (ramp → dominoes → marble run → weight off a ledge → rope-pulled cart → flying ball → basket and counterweight → bell).
- **Live values:** select anything to see its current state under its properties: body position, speed and energy; spring stretch and force; pin reaction; rope tension per segment and which pulleys it is on.
- **Grab and throw:** while the simulation runs, drag any body to pull it with a soft spring; let go to throw it. The work your mouse does is booked in the energy balance.
- **Shortcuts:** press **?** (or the ? button) for every keyboard and mouse shortcut.
- **Duplicate:** ⌘/Ctrl D copies the selected bodies plus the joints, springs and ropes between them. **F** fits the view. Drag the side panel's left edge to resize it.
- **Toolbar:** play/pause (Space), step 10 ms, reset (R), speed presets (0.1× to 10×, with a "CPU-limited" note if a heavy scene can't keep up), gravity, collisions, snap, force arrows, labels, presets, and save/open as JSON. The scene is also autosaved to localStorage.

### The calculations panel

- **Plots:** any coordinate against time, plus kinetic energy, potential energy (relative to t = 0), total energy, and the energy W removed by damping, friction and impacts (minus any work done by grabbing bodies with the mouse). The energy balance error (T + V + W − E₀) / max T checks the integrator even when the system is damped.
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

**Collisions:** bodies (boxes, wedges, circles) collide with the ground and with each other through a stiff penalty contact, f = max(0, kδ + c δ̇), with k = m_eff·(300 rad/s)² (m_eff is the reduced mass for a pair). The damping c is calibrated numerically so that the restitution you set in the scene settings is what you get. All ground pieces together are treated as one solid. A body point inside it is pushed toward the nearest point of the union's exposed boundary, which is precomputed by splitting every ground edge where other pieces cover it. That distance is continuous where pieces meet, so a block slides from the floor onto a ramp's toe and a body can sit in the corner of floor and wall. Between two bodies, points sampled along each outline are pushed out along whichever direction frees them soonest: the other body's face normals, or their own edge normals reversed. The force is always the exact gradient of the contact energy, so the energy balance closes. Circles (point masses, pulleys) are tested exactly. The force is equal and opposite on the two bodies, so momentum is conserved. Pairs joined directly by a pin or rod, bodies with *collides* unticked, and pairs that start overlapping are skipped. During an impact (a contact point closing, opening or sliding faster than 2 cm/s, or a new contact about to start), each 1 ms step is split into four RK4 substeps; bodies just resting on each other take full steps. Contact searches are cached per pose and pruned with bounding boxes, and the constraint matrix is assembled from J's nonzero columns only, so a twenty-body machine still runs several times faster than real time.

**Ropes over pulleys:** the straight runs are common tangents (signed-radius formula). One holonomic row fixes the total length, Σℓ + Σrφ = L. One no-slip row per pulley says the rope material between pulleys is conserved, `u·(v(T₂) − v(T₁)) = 0`, with the tangent points treated as points on the pulleys. That couples pulley spin to rope motion, which is how pulley inertia enters the dynamics. `J̇q̇` for the rope rows is a central difference along q̇.

Wrap angles are tracked continuously. When a pulley's wrap passes through zero, the rope lets go of that pulley; it no longer flips to a 2π wrap and knots itself around it. A free pulley on the route catches the rope again when a straight run cuts into it, with the wrap direction set by the side it hits. The catch is a plastic no-slip impulse, and its energy loss is booked as dissipated energy.

See [PLAN.md](PLAN.md) for the architecture and milestones.

## Verification (`npm test`)

| Check | Result |
|---|---|
| Simple pendulum, 2° amplitude: period vs 2π√(L/g) | within 1% (also exact ω from linearization) |
| Wall–k–m–k–m–k–wall: ω vs √(k/m), √(3k/m) | matches to 1e-6, correct mode shapes |
| Atwood with massive pulley: a vs (m₁−m₂)g/(m₁+m₂+I/r²) | within 0.1%; tensions and ω = a t / r match |
| Free pulley + fixed massive pulley | matches the analytic acceleration |
| Energy over 60 s (pendulum at 90°, chaotic double pendulum, springs, spring-loaded Atwood, all three presets) | drift < 0.1% of peak kinetic energy |
| Presets | assemble cleanly; the textbook ones have a stable equilibrium and the expected number of DOFs; contact presets keep their energy balance within 1% of peak kinetic energy; the Rube Goldberg machine runs all the way to ringing the bell |
| Collisions | a tumbling bar lands and rests on the floor; a pendulum bar bounces off a wall (< 1 cm overlap); e = 1 conserves energy; e = 0.5 halves the rebound speed |
| Ramps | a block and a ball slide down a fixed frictionless incline at g sin θ; block on a sliding wedge matches A = m g sinθ cosθ/(M + m sin²θ) with horizontal momentum conserved; a block runs up onto a ramp from the floor; a body pressed into a floor–wall corner has no energy kicks |
| Cart collisions | equal carts with e = 1 swap velocities; unequal carts with e = 0.5 match v′ = ((m₁ − e m₂)v, (1 + e)m₁v)/(m₁ + m₂); momentum conserved to 1e-6; blocks stack; a ball bounces off a block |
| Rope leaving / catching pulleys | a mass swung over a pulley releases the rope with no energy jump or wrap flip; a sweeping rope is caught by a pulley and spins it up |
| Friction | a block slides down a rough incline at g(sinθ − μ cosθ) and holds still when μ > tanθ; a disc rolls at ⅔ g sinθ; a block sliding on a cart decelerates relative to it at μg(1 + m/M) and both end at the momentum-conserving speed |
| Energy balance with dissipation | damped oscillator and bouncing bar: T + V + W stays equal to E₀ |

## Limitations

- Ropes are bilateral: they never go slack. The Forces tab flags any negative tension.
- Friction on sliders is a regularized Coulomb law, μ|N| tanh(v / 1 mm/s), using N from the previous solve, so there is no true stiction.
- Contact friction (ground pieces and bodies each have a μ; a contact uses the larger of the two) is regularized Coulomb, so a body resting on a slope can creep by a few mm/s instead of sticking exactly.
- Curved ramps are fixed only.
- A point mass has no rotational coordinate, so a torsional spring on its pin does nothing.
