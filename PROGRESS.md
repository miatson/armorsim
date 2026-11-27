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

## 2026-11-27 — session 6: first commit, fragments, scab investigation

- **First commit** `ddf49b4` (WIP, Stage C in progress; author miatson,
  set for this repo only, GitHub no-reply email).
- **Fragments** (sim/include/armor/Fragments.h, `Solver::fragments()`):
  union-find over bonded particles (same body, not fully failed) closer
  than 1.5 dx; fully failed particles = loose debris. Headless prints pieces
  per body (mass, speed) and debris. Results: Taylor rod 1 piece 10.4 g
  (= cylinder mass); ball 500 m/s: ball 4.11 g intact, plate 1 piece +
  0.19 g debris (dented, not perforated); long rod: only 17 g of 138 g
  rod still one piece, 121 g debris (erosion). Tests: synthetic split test,
  Taylor rod stays 1 piece. 23/23 pass.
- **Scab separation - NOT solved, coalescence OFF by default.**
  Spalled particles stay bonded, so a copper target at +-200 m/s never
  separates (the opening spreads over the 1.5 mm spall band). Tried:
  1. void coalescence at porosity 0.25 (1 - rho/rho0, Tvergaard-Needleman
     f_F): scab flies off correctly (5.9 g at +136 m/s, free-surface
     physics), but the Taylor rod blows up (> 380 m/s by ~2 us): 720+ axis
     particles reach 25% porosity within 1 us.
  2. Carroll-Holt residual tension (2/3) Y ln(1/phi) for voids: scab no
     longer separates, Taylor axis still coalesces. Reverted.
  3. coalescence excluded within one kernel radius of the axis (ring volume
     there follows tiny radial motions): scab OK, Taylor still blows up.
  Setting `coalescencePorosity` kept (default 1.0 = never). Open questions:
  where outside the axis zone the Taylor rod coalesces, and whether the
  free (zero-resistance) void growth after spall is the real problem -
  real void growth is rate-limited (inertia/viscosity: dynamic void growth
  models, e.g. Johnson 1981; Addessio-Johnson-Maudlin 1993 for Taylor).
- **Stage C remaining:** the scab/coalescence issue above; sourced,
  rate-dependent spall strengths; a fragment readout in the app; a full-
  fracture validation (plugging / V50 overlaps Stage E).
- **Stage D not started yet.**

## 2026-11-27 — session 7: scab / void coalescence fixed

- **Root cause of the Taylor blow-ups with coalescence:** a fully failed
  particle's pressure came from the EOS at its expanded density (-20 GPa).
  Pair forces ignored it (contact = push only), but the axisymmetric HOOP
  force -s_tt/(rho r) is not a pair term: near the axis it flung the
  particle out at up to 1000 m/s. Fix: pEff = max(p, 0) for fully failed
  particles (computeDerived), used in forces and work alike. The earlier
  "axis porosity" explanation was wrong.
- **Spall voids are now a P-alpha porosity model** (Solver::updatePorosity):
  phi per particle, p = p_EOS(rho/(1-phi), e)(1-phi), return mapping: grow
  while tension > min(spall, (2/3) Y ln(1/phi)) (Carroll-Holt), close in
  compression, coalesce at phi = 0.25 (Tvergaard-Needleman, now ON by
  default), initial phi 1e-3. Why: "no tension at all" let porosity ratchet
  up from SPH noise (expansion free, compression resisted).
- **The scab DOES separate** physically: at +-200 m/s the layer behind the
  spall plane flies at +113 m/s while the front moves at -160 m/s, band fully
  torn. The fragment finder reports one piece because an un-spalled rim ring
  (edge release) still connects them, even at 15 mm radius - plausible for
  ductile copper (spall cap). New validation instead, as experiments measure:
  **free-surface pullback**: peak 198/401 m/s (2v = 200/400), pullback ->
  spall strength 1.6/1.7 GPa vs 1.3 GPa input.
- Taylor: voids at 3.40 mm (exp ~3), 19 fully failed, 1 piece, length
  16.68 mm (+3.0%), foot 14.91 mm (better: 15.38 before). 24/24 tests.
- Presets: energy 0.955-0.999; long rod 17.8 g of rod left intact, 120 g
  debris; ball 500 m/s throws 0.2 g debris up to 569 m/s (jetting).
- Stage C remaining: sourced / rate-dependent spall strengths; fragment
  readout in the app; full-fracture validation (plugging, with Stage E).

## 2026-11-27 — session 8: repo published, Stage D started

- GitHub: https://github.com/miatson/armorsim (public since this session;
  scanned for personal data first - none). Commits: ddf49b4, 060d814,
  8962f1b (Stage D start).
- **Scenario files** (io/, library armor_io, nlohmann/json 3.11.3 via
  FetchContent; the solver stays dependency-free): JSON in mm/us/m/s,
  materials by key (Material::key: steel_rha, aluminium_6061, copper_ofhc,
  tungsten_wha, titanium_6al4v), shapes rect/disc/cylinder/sphere, optional
  settings. Strict: unknown keys and bad values fail with "$.path: why".
  Format in scenarios/README.md. Examples: spall_copper, taylor_bar_copper,
  long_rod_wha_rha.
- **Probes** (io/Probes.h): Lagrangian gauges (nearest particle at t = 0),
  15 quantities, fixed interval, CSV with units in headers.
- **Headless:** --file x.json [--csv out.csv]. **App:** files appear as
  "[file]" presets (from $ARMORSIM_SCENARIOS, ./scenarios, or the repo dir);
  the speed slider scales all moving parts; probes are recorded on the worker
  thread and plotted live in a floating "Probes (gauges)" window (bottom
  right) with the value at the playback time and Save CSV.
- Checked in the app (window-only capture with PrintWindow): spall file shows
  back face -100 -> +96.5 m/s then pullback; spall-plane pressure +3.65 GPa
  then -1.76 GPa then snaps to ~0 when voids open; porosity -> 0.17.
- Tests: test_stage_d.cpp (all shipped files load; units/shapes exact; error
  messages; spall probe peak +96.5 m/s vs +100). 247 assertions pass.
- **Stage D next:** projectile/armour LIBRARY with sourced data (e.g. real
  rod/plate geometries, impact conditions from published tests); a builder
  panel in the app (pick projectile + armour + speed + spacing, save as a
  scenario file); probe placement in the app (click to add a gauge).
- Stage C still open: sourced / rate-dependent spall strengths; full-
  fracture validation (plugging, with Stage E).

## 2026-11-27 — session 9: library, builder, new UI, loading screen, rename

- **Renamed to MiataPhySim** (miataphysim): CMake project, executable
  (build/msvc/bin/Release/miataphysim.exe), window title, docs, env vars
  MIATAPHYSIM_SCENARIOS / MIATAPHYSIM_LIBRARY. The local folder is still
  C:/Users/pawel/armorsim; internal C++ namespace is still `armor`.
- **Library** (library/projectiles, library/armor; io/Library.h): items =
  shape (sphere / rod / plate) + default material; 7 projectiles, 9 armour
  items. Format in library/README.md. buildScenario() turns projectile +
  material + speed + layers (thickness, gap) + angle into a scenario with two
  probes; angle > 0 = plane strain with rotated plates, projectile placed clear
  of the tilted face. suggestSpacing (1/20 projectile dia, 1/8 thinnest layer,
  ~40k particle cap) and suggestEndTime. scenarioToJson() saves any scenario.
- **UI**: Simulate tab (1 what to simulate: ready-made / build your own, 2 run
  with time-left bar, 3 watch with time slider, 4 colours) and Details tab
  (Results, Theory & checks, Resolution & duration, Display, Solver settings
  (advanced, only applied once edited), Materials table, Controls, About).
  New style (rounded, blue accent). Loading screen with real steps.
  --builder / --details start options. Builder default: steel ball 10 mm ->
  RHA 10 mm at 500 m/s (ball flattens and rebounds at ~-42 m/s).
- **Ceramics**: deliberately NOT added - they need JH-2 (Stage F); stated in
  the material dropdown, Details > Materials, library/README.md, README.
- Tests: library loads, all 63 projectile x armour pairs build, oblique
  0/30/60 deg projectile clear of plates, JSON round trip. 534 assertions.
