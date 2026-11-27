# MiataPhySim — 2D impact & armour physics simulator

> **Work in progress.** Stages A–C are largely done; Stage D (scenario builder) is
> under way. Expect changes and fixes.

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
| D | Scenario builder: projectile/armor library (JSON, real data), probes, graphs, CSV | 🔨 in progress (files, probes, graphs, CSV done) |
| E | Validation: Lanz-Odermatt long-rod penetration, V50 ballistic-limit tool | 🔨 in progress (long rods ✅, V50 tool ✅, plate perforation ✅ at fine spacing) |
| F | Ceramics (JH-2), layered/spaced armor, obliquity & ricochet | 🔨 in progress (alumina & boron carbide ✅) |
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
build\msvc\bin\Release\miataphysim.exe
```

**A build to give to others** (no machine paths inside, no Visual C++ runtime needed):

```
cmake -S . -B build/dist -G "Visual Studio 17 2022" -A x64 -DMIATAPHYSIM_SOURCE_PATHS=OFF -DARMOR_BUILD_TESTS=OFF -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded
cmake --build build/dist --config Release --target miataphysim
```

Ship `miataphysim.exe` with the `scenarios/` and `library/` folders next to it.
Ready-made Windows builds are on the [Releases](https://github.com/miatson/miataphysim/releases) page.

MinGW: `mingw-release` presets. Linux: `linux-release` presets (install the X11/GL
dev packages first, see raylib's docs). `-DARMOR_BUILD_APP=OFF` builds only the
solver and tests.

Command line (optional): `miataphysim --scenario 4 --run --field 2` starts scenario 4,
runs it immediately and shows field 2 (von Mises stress).
`miataphysim --headless --scenario 5` runs scenario 5 without a window and prints a
summary (energy balance, peak plastic strain / temperature / pressure, pieces and
debris, body sizes). `miataphysim --headless --file scenarios/spall_copper.json --csv out.csv`
runs a scenario file and writes its probe time histories to CSV.

### Scenario files and probes (Stage D)

Any setup can be described in a JSON file (mm, µs, m/s, materials by key) without
touching the code. See [scenarios/README.md](scenarios/README.md) for the format.
Files in `scenarios/` appear in the app's dropdown as **[file] ...**. A file can
define **probes**: gauges that follow a material point and record pressure,
velocity, porosity and more over time. In the app they are plotted live in the
**Probes** window (bottom right), with **Save CSV**. The example
`spall_copper.json` shows a VISAR-like back-face velocity trace with the spall
pullback.

## Using the app

A loading screen, then two tabs on the left.

**Simulate** (the beginner path, top to bottom):
1. **What to simulate.** Choose *Ready-made* (classic experiments and scenario files)
   or *Build your own*. The builder lets you pick a projectile and armour from the
   [library](library/README.md), set the material of each piece, stack up to 6 armour
   layers with gaps (spaced armour), set the speed and the angle, and save the setup
   as a scenario file.
2. **Run.** Press **Simulate**. The run happens in the background, frames appear live,
   and the bar shows the time left.
3. **Watch.** Play, pause, and drag the timeline (`Space`, `←`/`→`).
4. **Colours show.** Pressure, stress, plastic strain, temperature, damage, speed and
   more. Hover any particle to read all its values. Mouse wheel zooms, right-drag pans,
   `F` fits the view.

Gauges (probes) are plotted live in the **Probes** window, bottom right, with
**Save CSV**.

**Details** (sorted, folded away): Results (energy, sizes), Theory & checks,
Resolution & duration (automatic in the builder), Display, Solver settings
(advanced), Materials (with sources), Controls, About.

Start-up options: `--builder` opens the builder, `--details` opens the Details tab.

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
io/     scenario files (JSON, nlohmann/json) and probes (CSV) - "armor_io"
app/    raylib + Dear ImGui user interface, presets, colour maps, probe graphs
scenarios/  example scenario files (JSON)
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
  resolve), nucleates **voids**. A spalled particle stays bonded and keeps its shear
  strength; its **porosity φ is a state variable** (P-α family, Herrmann 1969). The
  metal between the voids obeys the EOS (`p = p_EOS(ρ/(1−φ), e)·(1−φ)`), and a
  return mapping lets voids grow only while tension exceeds the Carroll-Holt
  threshold `min(spall strength, (2/3) σ_y ln(1/φ))`, close under compression, and
  **coalesce** into a full crack at φ = 0.25 (Tvergaard-Needleman). Two simpler
  versions failed: strengthless spalled copper made the Taylor core squirt out (foot
  16.8 mm), and "no tension at all" let porosity ratchet up from SPH noise.
- **Fully failed material carries no tension anywhere**, including in the axisymmetric
  hoop force −σθθ/(ρr), which is not a pair term. Missing that flung freshly failed
  particles off the axis at up to 1000 m/s.
- Both plug into the energy-compatible scheme exactly: a fully failed pair is simply
  on the "other body" side of every sum.

### A prediction confirmed

In the Taylor bar, release waves from the side surface converge on the axis and pull
the core to about −4.4 GPa for ~0.7 µs (the same at two resolutions). The model opens
spall voids there, centred 3.4 mm behind the impact face, and the rod stays one piece. Sectioned OFHC copper rods
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
| Taylor bar: internal spall voids | ~3 mm behind the impact face, central (sectioned rod, Gray et al.) | mean 3.4 mm, r 0.95 mm, rod in one piece |
| Spall signal: back-face velocity peak, ±100 / ±200 m/s | 2v = 200 / 400 m/s | 198 / 401 m/s |
| Spall strength from pullback, σ ≈ ½ρ0c0Δu | input 1.3 GPa | 1.6 / 1.7 GPa |

(*) Rod/strip wave speed: this is real physics, not error. A wave in a bar of finite width **disperses**
(Pochhammer–Chree): its front spreads as it travels, so the half-amplitude point
arrives late while the leading edge arrives early. The same −5% appears at half the
particle spacing; the non-dispersive plate impact is exact.

(*) Taylor foot: the simulated wall is frictionless (a mirror-image rod). A real anvil
grips the rod face and holds the foot back.

## How it works — Stage F (in progress): ceramics

Armour ceramics use the **Johnson-Holmquist JH-2** model (Johnson & Holmquist 1994).
Strength is normalised by the stress at the Hugoniot elastic limit:

```
intact   σ_i* = A (P* + T*)^N (1 + C ln ε̇*)
rubble   σ_f* = B (P*)^M (1 + C ln ε̇*)  ≤ σ_f,max
current  σ*   = σ_i* − D (σ_i* − σ_f*)        D += Δεp / (D1 (P* + T*)^D2)
pressure P = K1 μ + K2 μ² + K3 μ³ + ΔP       (tension: K1 μ, limited to −T (1 − D))
```

A ceramic is very strong when squeezed and weak when pulled. As it cracks it
becomes rubble that still resists in proportion to pressure, and it **bulks**: the
elastic energy released as strength drops raises the pressure,
`ΔP_new = −K1 μ + sqrt((K1 μ + ΔP_old)² + 2 β K1 ΔU)`. Ceramic particles never
"fully fail" like metals do; the rubble keeps its strength. Pressure and work use
the same stored value, so energy stays exact.

**Materials:**
- **Alumina AD-995:** A 0.93, B 0.31, M 0.6, N 0.6, D1 0.005, D2 1, HEL 2.79 GPa,
  K1 130.95 GPa, G 90.16 GPa.
- **Boron carbide** (Johnson & Holmquist 1999): A 0.927, B 0.7, C 0.005, M 0.85,
  N 0.67, T 0.26 GPa, HEL 19 GPa, K1/K2/K3 233/−593/2800 GPa, G 197 GPa.
- **Cross-check:** the pressure at the HEL is not a free number. It follows from K1–K3,
  G and the HEL (uniaxial strain), and the code reproduces the published PHEL to
  0.2% for both (1.46 and 8.71 GPa); this is a test.
- **Approximate (single source):** alumina's tensile strength (0.2 GPa) and its
  fractured-strength cap (0.2).
- **Silicon carbide left out:** the sources conflict, and the check does not close.

**Tile test:** a 10 mm steel ball at 500 m/s into a 10 mm alumina tile gives a
structured, physical damage pattern rather than noise: a fully crushed cone under
the ball, cracking spreading outward through the middle and back of the tile, and an
intact front surface away from the contact. Bulking reaches 2.7 GPa, and total
energy never grows (final 0.998).

**Open: the composite-armour advantage does not show yet.** A tungsten rod (30 × 6 mm,
15 g, 900 m/s) against three targets of equal weight (64 kg/m²):

| Target | Rod leaves at |
|---|---|
| 10 mm alumina + 10 mm aluminium | 686 m/s |
| 23.7 mm aluminium | 682 m/s |
| 8.2 mm steel armour | 641 m/s |

Real ceramic composites beat equal-weight metal clearly. Follow-up checks (rod mass
that got through and its speed, at 250 µs, with leftover energy relative to target A):

| Variant of target A | Rod mass through | Speed | Relative energy |
|---|---|---|---|
| as built | 85% | 769 m/s | 1.00 |
| no cap on alumina rubble strength | 84% | 760 m/s | 0.96 |
| alumina that never cracks (reference) | 51% | 460 m/s | 0.22 |

Plain aluminium came out at 0.96 and steel at 0.78. So the rubble strength is not
the problem: **the tile cracks too early**, before it can erode the rod. The likely
cause is that tile and backing were separate bodies that can only push on each other.
That joint reflects the impact wave as tension, and alumina withstands only 0.2 GPa,
so the tile cracked from the back within microseconds. Real composite armour is
**glued**, so the builder now glues ceramic layers to the layer behind them by default
(one bonded body, `contact_group` in files), with a checkbox to turn it off. How much
this closes the gap has not been measured: that needs long runs, left for when the
machine is free. Until then, treat ceramic results as qualitative.

## Validation against experiments (Stage E)

### Long-rod penetration (Lanz-Odermatt)

The standard benchmark for kinetic-energy penetrators: how deep a tungsten long
rod gets into thick armour steel, as a function of speed. Lanz & Odermatt fitted

```
P/L = a · 1/tanh(b0 + b1·L/D) · sqrt(ρp/ρt) · exp(−s²/v²),   s² = (c0 + c1·BHN)·BHN/ρp
a = 0.921, b0 = 0.283, b1 = 0.0656, c0 = 138, c1 = −0.10   (tungsten into RHA)
```

to 148 tests (standard deviation 3.9%). Coefficients are as used in Willi
Odermatt's calculator. Units: kg/m³ and km/s; BHN = 286 for the simulator's
4340-class steel. Simulated with `scenarios/validation_long_rod_ld10.json` (rod
100 × 10 mm, 200 mm steel block, 0.5 mm spacing, 300 µs), depth = deepest rod
material:

| Speed | Simulated P/L | Lanz-Odermatt | Difference |
|---|---|---|---|
| 1000 m/s | 0.335 | 0.317 | +5.8% |
| 1250 m/s | 0.606 | 0.602 | +0.7% |
| 1500 m/s | 0.872 | 0.852 | +2.4% |
| 1750 m/s | 1.106 | 1.050 | +5.3% |
| 2000 m/s | 1.242 | 1.203 | +3.3% |

All within 6%, on average 3.5% deep, against a fit scatter of ±3.9%. Total
energy stayed within 0.2% in every run. Not yet checked: convergence at finer
spacing (the rod is only 10 particles across). Rerun any speed with
`miataphysim --headless --file scenarios/validation_long_rod_ld10.json --speed 1750`,
or the slow test `armor_tests -tc="VALIDATION*" --no-skip`. The builder shows the
Lanz-Odermatt prediction (Details → Theory) whenever you fire a tungsten rod at
steel.

### Ballistic limit (V50) of a steel plate (Børvik et al.)

The **V50 tool** finds the speed at which a projectile just perforates a plate, as
experiments do. It fires shots at two speeds that bracket the limit, then halves
the gap. A shot counts as **perforated** when the projectile has passed the
plate's current back surface (measured just outside the projectile's path, so a
punched-out plug doesn't count) and moves faster than the plate. It also fits the
Recht-Ipson curve `v_r = a·sqrt(v² − v_bl²)` to the residual velocities.

```
miataphysim --headless --file scenarios/validation_borvik_blunt.json --v50 170 215 --iterations 3
```

Benchmark: a flat-nosed hardened steel projectile (20 × 80 mm, 197 g) against a
12 mm Weldox 460 E plate. Measured ballistic limit **184.5 m/s**, with failure by
shear plugging. Material data: Børvik et al. 2001 (Johnson-Cook strength and
fracture for Weldox 460 E); projectile yield 1.9 GPa.

| Spacing (particles across the plate) | V50 (bisection) | Recht-Ipson fit | Experiment |
|---|---|---|---|
| 0.4 mm (30) | 207.8 m/s (+12.6%) | 203.1 m/s (+10.1%) | 184.5 m/s |
| **0.25 mm (48)** | **184.1 m/s (−0.2%)** | **182.6 m/s (−1.0%)** | 184.5 m/s |

At 0.25 mm the shots bracket the limit between 181.2 m/s (stopped) and 186.9 m/s
(perforated). The coarse run's 13% error was **resolution**: the plug is cut out
along a narrow shear band, and with too few particles across the plate the band
is too wide and the plate too hard to get through. Two resolutions show the
trend but don't yet prove convergence; a 0.2 mm check would. The builder warns
when a plate has fewer than about 40 particles across it.

## Known limitations

- Ceramics: alumina and boron carbide only (silicon carbide constants conflict between
  sources). The ceramic model is checked for consistency, not yet against a ceramic
  penetration experiment.
- Spall strengths in the material data are approximate (see `Material.cpp`), and spall
  is a fixed threshold plus a minimum duration, not yet rate-dependent.
- Fragments are counted (`miataphysim --headless`), not yet shown in the app. A spall
  scab can stay attached at a plate's rim, where edge release keeps the tension
  low and ductile copper stretches instead of tearing.
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
- T. Børvik, O. S. Hopperstad, T. Berstad, M. Langseth, *A computational model of viscoplasticity and ductile damage for impact and penetration*, Eur. J. Mech. A/Solids 20 (2001)
- T. Børvik, M. Langseth, O. S. Hopperstad, K. A. Malo, *Perforation of 12 mm thick steel plates by 20 mm diameter projectiles with flat, hemispherical and conical noses*, Int. J. Impact Eng. 27 (2002)
- A. J. Recht, T. W. Ipson, *Ballistic perforation dynamics*, J. Appl. Mech. 30 (1963)
- M. M. Carroll, A. C. Holt, *Static and dynamic pore-collapse relations for ductile porous materials*, J. Appl. Phys. 43 (1972)
- W. Herrmann, *Constitutive equation for the dynamic compaction of ductile porous materials*, J. Appl. Phys. 40 (1969)
- V. Tvergaard, A. Needleman, *Analysis of the cup-cone fracture in a round tensile bar*, Acta Metall. 32 (1984)
- F. L. Addessio, J. N. Johnson, P. J. Maudlin, *The effect of void growth on Taylor cylinder impact experiments*, J. Appl. Phys. 73 (1993)
- G. R. Johnson, T. J. Holmquist, *An improved computational constitutive model for brittle materials*, High-Pressure Science and Technology, AIP (1994)
- G. R. Johnson, T. J. Holmquist, *Response of boron carbide subjected to large strains, high strain rates, and high pressures*, J. Appl. Phys. 85 (1999)
- D. S. Cronin, K. Bui, C. Kaufmann et al., *Implementation and validation of the Johnson-Holmquist ceramic material model in LS-DYNA*, 4th European LS-DYNA Users Conference (2003)
- W. Odermatt, long-rod penetration calculator, longrods.ch (source of the Lanz-Odermatt coefficients used here)
- W. Lanz, W. Odermatt, *Penetration limits of conventional large caliber anti tank guns/kinetic energy projectiles*, 13th Int. Symp. Ballistics (1992)

## License

MIT, see [LICENSE](LICENSE). The bundled third-party libraries (raylib, Dear ImGui,
rlImGui, nlohmann/json, doctest) are downloaded at build time under their own licenses.
