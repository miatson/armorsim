# ArmorSim — 2D impact & material response simulator

A learning project: a small **hydrocode** (the kind of program used to simulate
armor penetration, explosions and hypervelocity impacts) written from scratch in
C++20. It uses **SPH (Smoothed Particle Hydrodynamics)**: every material is a
cloud of particles that carry density, velocity, stress and energy.
The app uses raylib for drawing and Dear ImGui for the user interface.
The solver itself has no dependencies.

| Stage | Content | Status |
|---|---|---|
| A | SPH core, elastic solids, plane strain + axisymmetric, adaptive time step, threads, record & replay UI | ✅ |
| B | Mie-Grüneisen shock EOS, Johnson-Cook plasticity, Taylor bar validation, energy-compatible forces | ✅ |
| C | Temperature, melting, damage, spall, fragmentation | 🔨 in progress |
| D | Scenario builder: projectile/armor library (JSON, real data), probes, graphs, CSV | ⏳ |
| E | Validation: Lanz-Odermatt long-rod penetration, V50 ballistic-limit tool | ⏳ |
| F | Ceramics (JH-2), layered/spaced armor, obliquity & ricochet | ⏳ |
| G | Explosives (JWL + programmed burn): HEAT jets, HESH | ⏳ |
| H | Optional: performance, 3D | ⏳ |

> **Metals yield, flow and heat up** (Johnson-Cook), shocks follow measured Hugoniots,
> and **since Stage C (in progress) they fail**: Johnson-Cook ductile fracture, spall
> under sustained tension, melting. Failed material can be pushed but not pulled.

---

## Building

Requires **CMake ≥ 3.25** and a C++20 compiler. raylib, Dear ImGui, rlImGui and
doctest are downloaded automatically on the first configure.

```
cmake --preset msvc                     # Windows / Visual Studio 2022
cmake --build --preset msvc-release     # use Release: physics in Debug is ~10x slower
ctest --preset msvc-release             # run the tests (~10 s)
build\msvc\bin\Release\armorsim.exe
```

MinGW: `mingw-release` presets. Linux: `linux-release` presets (install the X11/GL
dev packages first, see raylib's docs). `-DARMOR_BUILD_APP=OFF` builds only the
solver and tests.

Command line (optional): `armorsim --scenario 4 --run --field 2` starts scenario 4,
runs it immediately and shows field 2 (von Mises stress).
`armorsim --headless --scenario 5` runs scenario 5 without a window and prints a
summary (energy balance, peak plastic strain / temperature / pressure, body sizes).

## Using the app

1. **Scenario** — pick a preset and set impact speed, particle spacing (resolution)
   and end time. Press **Simulate**. The run happens in the background, and frames
   appear live.
2. **Playback** — play, pause and scrub through the recorded frames
   (`Space`, `←`/`→`).
3. **View** — choose what the colours show: material, pressure, von Mises stress,
   plastic strain, temperature, speed, density and stress components. Signed quantities use blue
   (compression) / white (zero) / red (tension). The legend shows highest to lowest.
   The colour range can be fixed over the whole run or per frame.
4. **Probe** — hover any particle to see all its values.
5. Mouse wheel zooms, right-drag pans, and `F` fits the view.

In **axisymmetric** scenes the lower half is a mirror image. The simulation only
computes the upper half (radius ≥ 0); the mirror shows the full cross-section.

## Architecture

```
core/   Vec2, Mat2, SymTensor             header-only maths (double precision)
sim/    the solver library "armor_sim"    no rendering, no UI
  Kernel.h        cubic spline kernel W and its gradient
  Material.h/cpp  material data: elastic constants, Mie-Grüneisen EOS and
                  Johnson-Cook constants, all with sources
  Particles.h     particle data, one vector per field
  Scenario.h/cpp  scene description + particle generation
  NeighborGrid    spatial hash -> neighbour lists (deterministic)
  ThreadPool.h    parallel loops; each particle written by one thread only
  Solver.h/cpp    the SPH equations and time stepping
  Recorder.h      frames for replay (lock-free hand-off to the UI thread)
app/    raylib + Dear ImGui user interface, presets, colour maps
tests/  doctest unit + validation tests
```

Units inside the solver are SI (m, kg, s, Pa, J). The UI shows mm, µs and MPa/GPa.
Results are **deterministic**: identical inputs give bit-identical results on any
number of threads (tested).

---

## How it works — Stage A

### The equations

A solid obeys three conservation laws, written per particle moving with the material:

```
mass      dρ/dt = −ρ ∇·v
momentum  dv/dt = (1/ρ) ∇·σ
energy    de/dt = (1/ρ) σ : ε̇
```

The **stress** σ splits into pressure (volume change) and deviatoric stress
(shape change): `σ = −p I + S`.

- Pressure from the equation of state. Stage A uses a linear one: `p = K (ρ/ρ0 − 1)`.
- Deviatoric stress from Hooke's law in rate form: `dS/dt = 2G (ε̇ − ⅓ tr ε̇ I)`,
  plus rotation terms (Jaumann rate), so that a spinning body doesn't build up
  fake stress.
- `ε̇ = ½(∇v + ∇vᵀ)` is the strain rate.

### SPH

Every field is a kernel-weighted average over neighbouring particles, and every
gradient uses the kernel's gradient:

```
∇v_i    ≈ Σ_j (m_j/ρ_j) (v_j − v_i) ⊗ ∇W_ij
dv_i/dt = Σ_j m_j (σ_i/ρ_i² + σ_j/ρ_j² − Π_ij I + R_ij f_ij) · ∇W_ij
```

The symmetric form makes the forces between two particles equal and opposite, so
momentum is conserved. On top of that come three extra pieces:

- **Artificial viscosity Π.** Shocks are thinner than a particle, and without extra
  dissipation the solution rings. The kinetic energy Π removes becomes heat.
- **Artificial stress R.** Plain SPH is unstable in tension: particles clump.
  R pushes them apart in proportion to the tensile stress.
- **Kernel gradient correction.** Near free surfaces, particles have neighbours on one
  side only. The strain-rate gradient is corrected with `M⁻¹`, where
  `M = Σ V_j (x_j − x_i) ⊗ ∇W_ij`.

### Two kinds of 2D

- **Plane strain.** A slice through infinitely long objects. It supports any impact
  angle, but a rod is really an infinitely wide plate.
- **Axisymmetric.** x is the axis and y is the radius. Every particle is a **ring**
  with mass `ρ dx² 2πr`, so a rectangle on the axis is a real cylinder and a
  half-disc is a sphere. This is exact for straight-on impacts. The equations gain
  hoop terms: `∇·v += v_r/r`, `ε̇_θθ = v_r/r`, `ρ a_r += (σ_rr − σ_θθ)/r`,
  `ρ a_z += σ_rz/r`. Near the axis, mirror "ghost" particles complete each
  particle's neighbourhood.

### Contact

Particles of different bodies push (pressure only, p > 0), slide without
friction, and can't pull.

Two details turned out to matter:
1. Another body's particles count toward a particle's compression only while they
   approach, or while that particle is still compressed.
2. Once the contact has unloaded, bodies moving apart must not count as "stretching";
   otherwise fake tension builds up at the interface.

### Time stepping

- **CFL condition:** `dt = 0.25 · h / (c + |v|)`, where c is the longitudinal wave
  speed. A signal must not cross more than a fraction of a particle per step.
  Steel at 0.5 mm spacing gives a step of about 28 ns.
- **Semi-implicit (symplectic) Euler:**
  1. update velocities from the current stress
  2. compute strain rates from the **new** velocities
  3. update stress, density and energy, then move the particles

  Updating stress with the *old* velocities (forward Euler) pumped about 0.3% energy
  per µs into every vibration. That was the root cause of an explosion found
  during development.

## How it works — Stage B

### Shock equation of state (Mie-Grüneisen)

A linear spring law is fine for small strains, but metals get much stiffer under
strong compression. Shock experiments measure a simple relation between the shock
speed Us and the particle velocity up behind the shock: `Us = c0 + s·up`. That
**Hugoniot** fixes the pressure on the shocked state; Mie-Grüneisen adds how
pressure changes with heat at a given density (Grüneisen parameter Γ0):

```
η = 1 − ρ0/ρ
p_H = ρ0 c0² η / (1 − s η)²        (pressure on the Hugoniot)
p   = p_H + Γ0 ρ0 (e − e_H)        e_H = p_H η / (2ρ0)
```

Constants (c0, s, Γ0) for all five materials are from Steinberg (1996).

### Strength: Johnson-Cook plasticity

Above the **yield stress** a metal stops springing back and flows. Johnson & Cook
fitted how the yield stress grows with plastic strain εp (work hardening), strain
rate (rate hardening) and falls with temperature (thermal softening):

```
σ_y = (A + B εp^n) · (1 + C ln ε̇p*) · (1 − T*^m)      T* = (T − T_room)/(T_melt − T_room)
```

The rate in the formula is the **plastic** strain rate from the previous step,
which is what the constants were fitted with. The total strain rate in SPH is noisy
(particle jitter) and gave spurious rate hardening.

### Radial return

Each step first assumes the deformation was elastic (trial stress). If the trial
von Mises stress exceeds σ_y, the deviatoric stress is scaled straight back onto
the yield surface (Wilkins 1964). The excess becomes plastic strain
`Δεp = (σ_vm − σ_y)/(3G)`. 90% of the plastic work becomes heat (Taylor-Quinney):
`ΔT = 0.9 σ_y Δεp / (ρ c_v)`.

### Numerical pieces that turned out to matter

- **Energy-compatible forces.** The internal-energy rate is the primary equation,
  and every force is derived from it as minus the coefficient of that particle's
  velocity in the total internal power. Energy is then conserved by construction:
  ```
  dv_i/dt = Σ_j a_j/(ρ_i ρ_j) [σ_i + (r_j/r_i) σ_j] ∇W_ij − σθθ_i ŷ/(ρ_i r_i)
  ```
  where a_j is the planar mass. In plane strain (r = 1) this is the standard
  symmetric form. In axisymmetric mode r_j/r_i is the ring-mass ratio, which also
  produces the σxy/r and σyy/r hoop forces, and axial momentum is conserved exactly.
  The earlier form only matched approximately: an axisymmetric steel ball resting
  on an aluminium plate *gained* 23% energy, and a tungsten rod into steel gained
  9% plus a 156 GPa pressure spike.
- **Balsara switch** (Balsara 1995). The artificial viscosity is scaled by
  `|∇·v| / (|∇·v| + |∇×v|)`: full in shocks (compression), reduced in shear.
  Plastic flow is mostly shear, and without the switch the viscosity made copper
  behave too stiff (Taylor bar 9% too long). A floor of 0.4 keeps some damping
  of particle noise.
- **Artificial stress work** enters the energy equation like any pair force
  (it created 4% energy in a plane-strain impact before).
- **Shepard density filter** every 20 steps, only for particles with εp > 0.1. It
  removes the density drift that long, violent plastic flow accumulates (5% too
  light in copper means ~7 GPa of fake tension).
- Artificial stress is **off in axisymmetric mode**: near the axis it pushes rings
  apart, which raises the hoop tension further, a runaway.

## How it works — Stage C (in progress)

### Failure

- **Ductile fracture (Johnson & Cook 1985).** Damage grows with plastic strain,
  `D += Δεp / εf`, with `εf = [D1 + D2 exp(D3 σ*)][1 + D4 ln ε̇*][1 + D5 T*]` and
  triaxiality `σ* = −p/σ_vm`. Pulled apart (σ* > 0), metals fail after little strain;
  squeezed, they flow almost forever. At D = 1 the particle **fails fully**: no shear
  strength, and every pair with it acts like contact (push only). Cracks open and
  fragments separate on their own. Melting fails a particle the same way.
- **Spall.** Tension beyond the spall strength, **sustained** for at least the time
  a sound wave takes to cross one smoothing length (the shortest pulse the particles
  resolve), opens **voids**. A spalled particle stays bonded and keeps its shear
  strength, but can no longer carry hydrostatic tension: its pressure counts as
  max(p, 0) in forces and pressure work alike. This is how ductile metals behave
  (incipient spall barely weakens them in compression). Treating spalled copper as
  strengthless instead made the Taylor rod's core squirt out and inflated its foot to
  16.8 mm.
- Both plug into the energy-compatible scheme exactly: a fully failed pair is simply
  on the "other body" side of every sum.

### A prediction confirmed

In the Taylor bar, release waves from the side surface converge on the axis and pull
the core to about −4.4 GPa for ~0.7 µs (the same at two resolutions). The model opens
spall voids there, centred 2.9 mm behind the impact face. Sectioned OFHC copper rods
show "ductile void damage within the central region … approximately 3 mm from the
impact interface" at 175 m/s (Gray, Maudlin & Chen, Los Alamos LA-UR-01-5614).

## Validation (automated tests)

| Test | Theory | Result |
|---|---|---|
| Plate impact (uniaxial strain), plane strain: wave speed | cL = √((K+4G/3)/ρ) = 5878 m/s | within 0.5% |
| Plate impact, axisymmetric: wave speed | 5878 m/s | within 0.5% |
| Plate impact: stress | σ = ρ·cL·v = 231 MPa | within 0.5% |
| Rod impact (axisymmetric): stress | σ = ρ·√(E/ρ)·v = 202 MPa | within 1% |
| Rod/strip wave speed | √(E/ρ) and √(E/(ρ(1−ν²))) | −5% (*) |
| Bar collision incl. separation, 60 µs: total energy | conserved | 97–99% (viscosity heating) |
| Disc collision: total energy, momentum | conserved | 99%, exact |
| Body at rest | stays at rest | ✓ |
| Determinism | bit-identical runs | ✓ |
| **Stage B** | | |
| Mie-Grüneisen pressure on the measured Hugoniot | ρ0·Us·up | exact |
| Radial return caps von Mises at yield, plastic heating | σ_vm ≤ σ_y | ✓ |
| Copper plate impact at 500 m/s: shock pressure | Hugoniot 20.99 GPa | 20.79 GPa (−0.9%) |
| **Taylor bar** (OFHC copper, 190 m/s, dx 0.127 mm): final length | experiment 16.2 mm | 16.75 mm (+3.4%) |
| Taylor bar: foot diameter | experiment ~13.5 mm | 15.0 mm (*) |
| Ball on plate in contact (axisymmetric 10 & 500 m/s, plane strain 500 m/s): total energy | never grows | max +0.04%, final −0.2…−0.6% |
| **Stage C** | | |
| Spall: 2 mm Cu flyer on 6 mm Cu target, ±20 m/s | no spall (0.7 GPa < 1.3 GPa) | none ✓ |
| Spall: same at ±100 m/s, spall plane | 4.0 mm (one flyer thickness from the back face) | 3.85 mm |
| Taylor bar: internal spall voids | ~3 mm behind the impact face, central (sectioned rod, Gray et al.) | mean 2.94 mm, r 0.77 mm |

(*) Rod/strip wave speed: this is real physics, not error. A wave in a bar of finite width **disperses**
(Pochhammer–Chree): its front spreads as it travels, so the half-amplitude point
arrives late while the leading edge arrives early. The same −5% appears at half the
particle spacing; the non-dispersive plate impact is exact.

(*) Taylor foot: the simulated wall is frictionless (a mirror-image rod). A real anvil
grips the rod face and holds the foot back.

Not yet a validation, but a good sign: the tungsten long rod preset (L/D 10,
1500 m/s into steel) reaches 0.86 L, inside the 0.8–0.9 L that real tests give
(Lanz & Odermatt). Proper validation comes in Stage E, after erosion (Stage C).

## Known limitations

- Spall strengths in the material data are approximate (see `Material.cpp`), and spall
  is a fixed threshold plus a minimum duration, not yet rate-dependent.
- Fragments are not tracked or counted yet.
- Plastic impacts need fine spacing for converged numbers: the Taylor bar is
  ~17.2 mm at 0.254 mm spacing and 16.75 mm at 0.127 mm.
- Total energy drops by 1–5% over violent runs. The time integrator always dissipates
  a little, and contact can only push. It never grows.
- Frictionless contact.
- Constant smoothing length (fine for metals; very large expansions need variable h).
- Replay frames are stored as float32 in memory. Large scenes (≥ 200k particles)
  will need compressed frames (planned in Stage D).
- 2D only. Plane-strain penetration numbers are qualitative. Axisymmetric is
  exact, but only for 0° impacts.

## References

- J. J. Monaghan, *Smoothed particle hydrodynamics*, Rep. Prog. Phys. 68 (2005)
- L. D. Libersky, A. G. Petschek et al., *High strain Lagrangian hydrodynamics*, J. Comput. Phys. 109 (1993)
- P. W. Randles, L. D. Libersky, *Smoothed particle hydrodynamics: some recent improvements*, CMAME 139 (1996)
- J. P. Gray, J. J. Monaghan, R. P. Swift, *SPH elastic dynamics*, CMAME 190 (2001)
- G. R. Johnson et al., *SPH for high velocity impact computations*, CMAME 139 (1996)
- K. L. Johnson, *Contact Mechanics*, Cambridge (1985) — Hertz impact theory
- J. A. Zukas (ed.), *High Velocity Impact Dynamics*, Wiley (1990)
- G. R. Johnson, W. H. Cook, *A constitutive model and data for metals subjected to large strains, high strain rates and high temperatures*, 7th Int. Symp. Ballistics (1983)
- D. J. Steinberg, *Equation of state and strength properties of selected materials*, LLNL UCRL-MA-106439 (1996)
- G. I. Taylor, *The use of flat-ended projectiles for determining dynamic yield stress*, Proc. R. Soc. A 194 (1948)
- M. L. Wilkins, *Calculation of elastic-plastic flow*, Methods Comput. Phys. 3 (1964)
- D. S. Balsara, *von Neumann stability analysis of SPH*, J. Comput. Phys. 121 (1995)
- A. Colagrossi, M. Landrini, *Numerical simulation of interfacial flows by SPH*, J. Comput. Phys. 191 (2003)
- G. R. Johnson, W. H. Cook, *Fracture characteristics of three metals subjected to various strains, strain rates, temperatures and pressures*, Eng. Fract. Mech. 21 (1985)
- G. T. Gray III, P. J. Maudlin, S. R. Chen, *The assessment of damage evolution in metals as a function of microstructure using Taylor impact testing*, LANL LA-UR-01-5614 (2001)
- F. L. Addessio, J. N. Johnson, P. J. Maudlin, *The effect of void growth on Taylor cylinder impact experiments*, J. Appl. Phys. 73 (1993)
- W. Lanz, W. Odermatt, *Penetration limits of conventional large caliber anti tank guns/kinetic energy projectiles*, 13th Int. Symp. Ballistics (1992)
