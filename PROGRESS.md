# Progress notes

## 2026-11-26 — end of session

### Stage A — done ✅
SPH solver for elastic solids, both 2D modes (plane strain + axisymmetric),
threads, deterministic, record & replay app. Validated: plate impact wave
speed and stress within 0.5%, energy conserved over full collisions.

### Stage B — in progress (about 80%)

**Done (solver):**
- Mie-Grüneisen EOS (Hugoniot form) for all 5 materials, with Steinberg data.
- Johnson-Cook strength + radial return plasticity, plastic heating
  (Taylor-Quinney 0.9). New particle fields: plastic strain, temperature.
- Material library rewritten with JC + EOS constants and sources.
- Part options: `temperature`, `contactGroup` (bonded parts).
- Recorder: new fields (plastic strain, temperature), body id per particle, body names.
- Stability and energy fixes found along the way:
  - shear work S:ε̇ uses the uncorrected gradient (a plastic disc collision
    gained 26% energy without this);
  - artificial stress is off in axisymmetric mode (it caused a runaway
    near the axis);
  - Shepard density filter every 20 steps, only for particles with εp > 0.1
    (removes density drift in long plastic flows; the Taylor bar blew up
    without it).
- Test fix: doctest `Approx` needs `.scale(0)` for values below ~1, otherwise
  the tolerance is meaningless (the Taylor length check was passing at +12%).

**Validated:**
- Copper shock pressure at 500 m/s: 20.75 GPa vs Hugoniot 20.99 GPa (1.1%) ✅
- Radial return keeps von Mises ≤ yield, with plastic strain and heating ✅
- Taylor bar foot diameter: 13.5 mm vs experiment ~13.5 mm ✅

**Open (updated 2026-11-27, end of session 2):**
1. ~~Taylor bar length +12%~~ Resolved (see below).
2. ~~App does not compile~~ Fixed.
3. ~~Stage B app work~~ Done: 4 new presets (Taylor bar, copper shock, steel
   ball 500 m/s, tungsten long rod), body sizes readout (length x diameter
   + x range), Stage B panel text and window title, colormap fix.
4. ~~Axisymmetric energy not conserved in contact~~ **Fixed 2026-11-27
   (session 3)** with energy-compatible forces; see below.
5. Plastic disc collision (preset 3, 20 m/s) now loses ~4.5% (was 11-13%).
   Elastic bars lose 3.6-4.4% over a full collision. Remaining losses: the
   symplectic Euler step (always dissipates ~dt^2 * sum m a^2 / 2) and the
   contact rules (a tensile particle compressed by another body gains density
   but no energy, since contact work uses max(p, 0)). Small; revisit if a
   validation case needs it.
6. ~~README Stage B section~~ Done (session 4).
7. Foot diameter 14.8 mm vs ~13.5 mm measured. Likely the frictionless
   wall (symmetry plane); a real anvil grips the rod face.

## 2026-11-27 — Taylor bar resolved

- JC rate from previous-step **plastic** strain rate: 18.12 -> 17.61 mm.
- Root cause of the rest: the Monaghan artificial viscosity acted on every
  approaching particle pair, including pure shear, so it damped real plastic
  flow (copper behaved too stiff). Added the **Balsara switch**
  (`balsaraSwitch`, factor |div v|/(|div v|+|curl v|+1e-4 c/h); compression
  measured with the all-body gradient so contact still counts). Shocks keep
  full viscosity; the copper Hugoniot test is unchanged (20.75 GPa).
- Pure Balsara blew up at dx = 0.254 mm, so a floor `balsaraMin` was added:
  0.1/0.2 blew up, 0.3+ stable at 170/190/210 m/s; default **0.4** for margin.
- Convergence at dx = 0.127 mm: 16.51-16.73 mm regardless of floor (0-0.4).
  The Taylor test now runs at dx = 0.127 mm: **16.73 mm vs 16.2 mm (+3.3%)**,
  energy ratio 0.999, foot 15.1 mm. Full suite: 20/20 pass, ~36 s.
- Elastic disc collision energy ratio stays within 0.1%.

### Build & test commands
```
cmake --build --preset msvc-release
build\msvc\bin\Release\armor_tests.exe -s
```
CMake is at `C:\Program Files\CMake\bin\cmake.exe` if it's not on PATH.

## 2026-11-27 — session 2: Stage B presets, two solver fixes

- **App:** 4 Stage B presets; `--headless --scenario N` runs a preset with no
  window and prints a summary (energy ratio, max plastic strain/temperature/
  pressure, body extents); body sizes in the readout; colour scale starts at
  turbo(0.08) because turbo(0) is near-black and vanished on the background
  (that was the "invisible bodies" bug); end-time slider down to 1 us.
- **Fix: artificial stress did work that never entered the energy equation**
  (plane strain only; it is off in axisymmetric). Its pair work is now added to
  eDot with the same split as viscous heating. Plane-strain ball at 500 m/s:
  +4.2% -> -0.9%.
- **Fix: Balsara floor 0.4 -> 0.6.** At 0.4 an axisymmetric ball on a plate at
  50 m/s had the rows next to the axis slide back and forth and heat without
  bound (energy x3.9 in 60 us); 0.5+ stable. Forcing full viscosity only
  near the axis made it WORSE, so it was reverted. Taylor bar now 16.85 mm
  (+4.0%) at dx 0.127, foot 14.8 mm; ~17.2 mm at dx 0.254.
- Headless check of all presets (default settings): energy ratio
  bar axi 0.97, bar plane 0.96, disc 0.87, ball-on-plate (10 m/s) 1.24,
  Taylor 1.00, shock 0.98 (21.2 GPa vs 21.0 Hugoniot), ball 500 m/s 1.03,
  long rod 1.09 (tip ~94 mm deep vs ~85 mm real, qualitative).
- Tests: 20/20 pass. Two skipped diagnostics kept on purpose:
  `DIAG ball energy`, `DIAG axis blowup` (see open item 4).

## 2026-11-27 — session 3: energy-compatible forces

Root cause of the axisymmetric energy gain: the internal energy rate and the
forces were discretised separately. In axisymmetric mode each particle is a
ring with mass m = 2 pi r a, and the old pair force a_j (s_i/rho_i^2 +
s_j/rho_j^2) plus explicit hoop forces only matched the energy equation
approximately; worst for the rows next to the axis (ring masses 1:3).

Fix (Solver.cpp, header "ENERGY-COMPATIBLE FORCES"): keep the energy equation,
derive each force as minus the coefficient of v_i in the total internal power:
  dv_i/dt = sum_j a_j/(rho_i rho_j) [s_i + (r_j/r_i) s_j] gradW_ij - s_tt/(rho_i r_i) y
- plane strain (r = 1): the standard 1/(rho_i rho_j) form, which also makes
  material interfaces (steel/aluminium) consistent;
- the r_j/r_i factor supplies the s_xy/r and s_yy/r hoop forces, so only the
  hoop stress s_tt is added explicitly; axial momentum is exactly conserved;
- ghosts use |r|; viscosity / artificial stress use symmetric pair weight
  a_j * mean(r)/r_i with heating split equally;
- pressure work always uses the RAW divergence, and contact work max(p, 0).

Results (energy ratio at the end of each preset, before -> after):
ball on plate 10 m/s 1.241 -> 0.994; long rod 1.091 -> 0.998 (156 GPa spike
gone, max 10.9 GPa); ball 500 m/s 1.027 -> 0.998; disc 0.869 -> 0.955;
Taylor 0.990; shock 0.983. Every preset now loses slightly, never gains.
- Balsara floor back to 0.4: the "axis runaway" at 0.4 was this bug; with
  compatible forces 0.3 and 0.4 are stable.
- Taylor bar 16.75 mm (+3.4%), foot 15.0 mm; copper shock 20.79 GPa (-0.9%).
- Long rod at 1500 m/s now reaches 86.5 mm = 0.86 L (real tests ~0.8-0.9 L;
  was 94 mm with the phantom energy). Qualitative until erosion (Stage C).
- New regression test "ball on plate conserves energy in contact": highest
  ratio must stay < 1.005, final > 0.97. Diagnostics removed. 21/21 pass, ~37 s.

## 2026-11-27 — session 4: README Stage B, Stage C started (failure)

**Stage C step 1 is in** (Material.h, Particles.h, Solver.cpp "FAILURE"):
- Johnson-Cook fracture model, D += dEp / eps_f(triaxiality, rate, T*),
  constants: 4340 steel 0.05/3.44/-2.12/0.002/0.61 (JC 1985), OFHC Cu
  0.54/4.89/-3.03/0.014/1.12 (JC 1985), W alloy 0/0.33/-1.5/0/0 (JC 1985),
  Al 6061-T6 -0.77/1.45/-0.47/0/1.6 and Ti-6Al-4V -0.09/0.25/-0.5/0.014/3.87
  (Lesuer 2000). eps_f floor 0.01, triaxiality clamped to +-1.5.
- Spall: tension beyond spallStrength SUSTAINED for max(h/c,
  spallIncubationTime). Spall strengths are APPROXIMATE (steel 3.0, Al 1.1,
  Cu 1.3, W alloy 3.5, Ti 4.0 GPa) - replace with sourced values.
- Melting: T >= T_melt fails the particle.
- Failed particle: S = 0, and any pair involving it is treated as contact
  (push only, p+). Plugs into the energy-compatible scheme exactly.
- New view field "Damage" (index 5; Speed etc. moved up by one). Setting
  `failure` (default on), `spallIncubationTime`.

**Validated:** spall plate test (2 mm Cu flyer on 6 mm Cu target): no spall
at +-20 m/s; at +-50/+-100 m/s spall plane at mean x 3.89/3.85 mm vs 4.0 mm
theory. New regression test. Energy unchanged (ball tests still < +0.04%).
22/22 tests pass.

**OPEN - top of Stage C list: spall in the Taylor bar.** With failure on, the
Taylor preset spalls ~300 particles at the rim (from 0.15 us) and the axis
region; foot grows to 16.8 mm. Measured with failure off: axis tension
reaches -4.4 GPa for 0.7 us about 4 mm behind the impact face, converged
(dx 0.254: -4.28 GPa / 0.66 us; dx 0.127: -4.39 / 0.70 us). Real OFHC rods
at 190 m/s don't crack. A constant 1.3 GPa strength cannot pass both the
plate test and the Taylor bar; any monotone time-integral criterion
(Tuler-Butcher) fails too, since 4.4 GPa x 0.7 us is harsher than the plate
pulse. So either (a) copper's real resistance to such short, fast pulses is
much higher (spall strength rises with strain rate - need data), or (b) the
axis tension is overpredicted (axisymmetric axis treatment, or missing
physics such as the void-growth time). Data source to get: AFATL-TR-90-81
(DTIC ADA233600, Rajendran et al., "A Dynamic Failure Model for Ductile
Materials") - DTIC refused automated download (HTTP 403); fetch manually.
For now the Taylor validation runs with failure off (it tests plasticity).

**Presets with failure on** (energy ratio): disc 0.955, Taylor 0.992,
ball 500 m/s 0.998 (plate tears, back-face failure), long rod 0.998 (tip
87 mm, failed steel sprayed back out of the crater up to ~1440 m/s).

**Stage C next steps:** resolve the Taylor spall question; fragment
tracking/readout (count and sizes); erosion behaviour check on the long rod;
a validation for ductile fracture (e.g. plugging of a plate at known V50 -
overlaps Stage E).

## 2026-11-27 — session 5: Taylor axis tension investigated -> spall voids

**Finding: the axis tension is physical, not an artifact.** Radial pressure
profile at the worst moment (failure off): tension beyond 1.3 GPa over ~1.5-2 mm
radius, same shape at dx 0.254 and 0.127 (peak -4.2/-4.4 GPa on row 2, axis row
only -2.2/-2.3). Mechanism: the ~7 GPa impact state releases from the side
surface and the converging cylindrical release overshoots into tension,
1.4-2 mm behind the face at 1.2-1.4 us.
**Experiment agrees:** Gray, Maudlin & Chen (LANL LA-UR-01-5614): annealed OFHC
Cu Taylor rod at 175 m/s, sectioned: "ductile void damage within the central
region ... approximately 3 mm from the impact interface". My earlier note that
real rods don't crack was wrong - J&C only measured the outside shape.
Failure map with failure on: ~370 failed particles, all central (r < 2 mm),
0-6 mm behind the face; only 1 at the rim.

**What was wrong was the failed-state behaviour:** spalled particles became
strengthless, the core squirted out under re-compression, foot 16.8 mm.
**Fix:** spall = VOIDS (FailureKind kSpall): stays bonded, keeps shear strength
(still radial-returned), pressure counts as max(p, 0) in forces, pressure work
and triaxiality. Ductile damage keeps accumulating; D >= 1 or melting = full
failure (kDuctile/kMelted, as before). Damage field shows spall as >= 0.5.

**Results:** Taylor test now runs WITH failure: voids at mean start x 2.94 mm,
r 0.77 mm (experiment ~3 mm, central), 0 fully failed; length 16.66 mm
(+2.9%), foot 15.38 mm. Spall plate still 3.85 mm. Long rod: tip 87 mm,
backsplash reduced (to -105 mm, 800 m/s; was -210 mm, 1440 m/s). Energy
ratios 0.993-0.998. 22/22 tests, 186 assertions. Diagnostics removed.
The OPEN item from session 4 (Taylor spall) is resolved.

**Stage C next:** sourced spall strengths (and rate dependence, e.g. data in
AFATL-TR-90-81 / Addessio-Johnson-Maudlin 1993); fragment tracking and
readout (count, sizes, speeds); erosion check on the long rod; a ductile-
fracture validation (plate plugging / V50 overlaps Stage E).
