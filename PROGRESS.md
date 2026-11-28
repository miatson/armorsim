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

## 2026-11-27 — session 10: MIT license, release, Stage E started

- **Release** v0.4.0-alpha (GitHub pre-release, zip with exe + scenarios +
  library). Build option MIATAPHYSIM_SOURCE_PATHS=OFF for shareable builds
  (no machine paths in the exe; checked: no username/paths/email in the
  binary), static MSVC runtime. Tag v0.4.0-alpha. MIT license added (58c0abd).
- **Stage D**: core done (files, library, builder, probes, graphs, CSV).
  Leftover: place gauges by clicking in the view.
- **Stage E, long rods DONE**: io/include/armor/Validation.h has the
  Lanz-Odermatt fit (coefficients from Odermatt's calculator via
  xplicitmaterials.com, 148 tests, 3.9%; units kg/m3 + km/s inferred by
  requiring P/L ~0.85 at 1.5 km/s for L/D 10). Simulated (0.5 mm spacing,
  validation_long_rod_ld10.json, 200 mm block, 300 us): P/L 0.335 / 0.606 /
  0.872 / 1.106 / 1.242 at 1000..2000 m/s vs 0.317 / 0.602 / 0.852 / 1.050 /
  1.203: +0.7..+5.8%, mean +3.5%. Energy 0.998. Each run 3.5-5 min.
- Headless overrides --speed, --spacing (mm), --end-time (us) for files.
- Builder: Theory & checks shows the LO prediction for tungsten rods into steel.
- Tests: test_stage_e.cpp (LO values/trends; slow skipped VALIDATION case).
- **Stage E next**: convergence of the long-rod result at 0.25 mm (one speed;
  ~8x cost, run in the background); V50 ballistic-limit tool (bisection on
  speed with a perforation criterion, residual velocity, Lambert-Jonas fit)
  and a published ballistic-limit case to compare against.

## 2026-11-27 — session 11: V50 tool, Borvik plate benchmark

- **Materials** (appended, ids kWeldox460E, kToolSteelHRC53): Weldox 460 E
  (JC A 490, B 807, n 0.73, C 0.0114, m 0.94, Tm 1800, ref rate 5e-4;
  fracture D1..D5 0.0705, 1.732, -0.54, -0.015, 0; Borvik et al. 2001) and
  hardened tool steel (Arne HRC 53: yield 1.9 GPa, tangent 15 GPa; no
  fracture). Library: borvik_blunt_20mm projectile, weldox460e_12mm plate.
- **V50 tool** (io/BallisticLimit.h): fireShot (perforated = projectile past
  the plate's current back surface, measured in a ring 1.5-3 projectile radii
  out, AND faster than that ring; ends early once clear), findBallisticLimit
  (bracket low/high, widen x0.7 / x1.4 up to 4 times, then bisect),
  rechtIpsonFit (p = 2, least squares on v_r^2 vs v^2). CLI:
  --headless --file F --v50 LO HI [--iterations N].
- **Borvik blunt vs 12 mm Weldox 460 E** (validation_borvik_blunt.json,
  0.4 mm, free 200 mm plate): V50 207.8 m/s (206.25 stopped, 209.375
  perforated), Recht-Ipson 203.1 m/s (a = 0.69), experiment 184.5 m/s:
  +12.6% / +10.1%. Partial match. Suspects: shear-band resolution, free vs
  clamped 500 mm plate, fracture data rate reference. Each shot 1-2.5 min.
- Quick check (5 mm steel ball vs 1 mm Al): V50 146.9, fit 149.4 m/s.
- **Long-rod convergence run at 0.25 mm was stopped on the user's request**
  (it was ~200k particles, competing for CPU); rerun when the machine is idle:
  --file scenarios/validation_long_rod_ld10.json --speed 1500 --spacing 0.25
- Tests: Recht-Ipson synthetic fit, fireShot stop/perforate, slow skipped
  VALIDATION Borvik case (±15%). 629 assertions pass.
- **Stage E next**: resolution study of the Borvik case (0.25-0.2 mm, run in
  the background, one or two speeds near 185-210 m/s); long-rod convergence
  rerun; then the hemispherical / conical nose cases need nose shapes in the
  library (Stage F overlap). Stage D leftover: click-to-place gauges.

## 2026-11-27 — session 12: Borvik resolution study

- 0.25 mm (48 particles across the 12 mm plate), bracket 170-215, 3 bisections:
  170 stopped, 215 perf (v_r 79.8), 192.5 perf (38.3), 181.2 stopped,
  186.9 perf (32.7). **V50 184.1 m/s (-0.2% vs 184.5)**, Recht-Ipson 182.6
  (a = 0.70). At 0.4 mm it was 207.8 (+12.6%): the gap was resolution of
  the plugging shear band. Shots took 1.5-6 min each (~20 min total).
- Not proven converged (2 resolutions); a 0.2 mm check would confirm.
- validation_borvik_blunt.json now defaults to 0.25 mm; slow test uses the
  same bracket and +-5%.
- Builder (Theory & checks) warns when a plate has < 40 particles across it
  and suggests spacing = thinnest / 48. The automatic spacing stays coarse
  for speed - consider an "accurate" option later.
- Long-rod convergence rerun (0.25 mm) still pending.

## 2026-11-27 — session 13: Stage F started - ceramics (JH-2)

- Material.h: PolynomialEos (K1..K3) and JohnsonHolmquist2 strength;
  Solver::integrateCeramic (EOS + bulking pressure, intact/rubble strength by
  damage, radial return, damage from plastic strain, bulking from released
  elastic energy, tension cut -T(1-D)). Particles::bulking. Ceramics skip the
  density filter and never become "fully failed".
- Materials (ids 7, 8): alumina AD-995, boron carbide (constants + sources in
  Material.cpp). PHEL cross-check from EOS + G + HEL reproduces 1.46 / 8.71 GPa
  (test). Alumina T 0.2 GPa and sigma_f,max 0.2 single-source. SiC left out
  (sources conflict: rho 3163 vs 3215, HEL 11.7 vs 14.6; JDEU 2025 table
  mixes in AlN damage constants and gives alumina HEL 19 GPa - unreliable).
- Library: alumina_tile_10mm, boron_carbide_tile_8mm. App notes updated
  (materials table shows HEL for ceramics).
- Tile test (steel ball 500 m/s, 10 mm alumina, 0.25 mm): crushed core
  (mean D 1.0), front away from contact intact (0.001), bulking 2.7 GPa,
  energy max 1.000 / final 0.998. Damage map structured (cone + radial
  cracking from the back), not noise. A backed tile cracked slightly MORE
  than unbacked (1698 vs 1620) in 25 us - expectation dropped, not explained.
- **OPEN, top of Stage F**: equal-weight comparison (64 kg/m2, W rod 30x6 mm
  at 900 m/s, 0.2 mm): alumina10+Al10 686 m/s, Al 23.7 682 m/s, RHA 8.2
  641 m/s - no ceramic advantage (real composites win clearly). Suspects:
  fixed 120 us window (rods still slowing?), residual mass/erosion ignored,
  sigma_f,max 0.2 maybe low, rod tip break-up under-resolved. Next: report
  residual MASS and run longer; check rubble strength sensitivity; find a
  published ceramic DOP experiment to validate against.
- 706 assertions pass.

## 2026-11-27 — session 14: ceramic gap investigated; NO more long runs

- **User request: no more long tests/simulation runs on their PC** (they eat
  its resources). Reason things out; ask before anything heavy. (Memory saved.)
- Equal-weight gap is real, not a measurement artifact (250 us, rod mass
  through + speed): A (alumina10+Al10) 85% @ 769 m/s, B (Al 23.7) 88.5% @ 742,
  C (RHA 8.2) 75% @ 725 -> relative residual energy A 1.00, B 0.96, C 0.78.
- Variants of A: no cap on alumina rubble strength 0.956 (not the cause);
  alumina that never cracks 0.215 (huge). => the tile cracks too early.
- Fix applied (by reasoning, NOT measured): tile and backing were separate
  push-only bodies; that joint reflects the wave as tension and alumina
  (T = 0.2 GPa) cracks from the back at once. Real armour is glued:
  ArmorLayer::bondedToNext (shared contact group), builder checkbox "Glued to
  the next layer", on by default for ceramic layers; builder theory text says
  ceramic results are qualitative. A bonded-tile run was stopped after ~30 min
  by the user, so its effect is unknown.
- Other suspect left: JH-2 cracks instantly in tension (eps_pf floor 1e-6 when
  P* + T* <= 0); LS-DYNA has a minimum fracture strain (EFMIN) but no sourced
  value - not changed.
- Built only; tests not run this session at the user's request.

## 2026-11-28 — session 15: click-to-place gauges (Stage D leftover done)

- Simulate > Watch: "Add gauge", then click a particle. A "Gauges" window plots
  the selected colour field at that particle over all recorded frames (no new
  simulation), with the value at the playback time and "remove". Numbered green
  rings mark gauges in the view (they move with the material). Esc cancels;
  gauges clear on a new run (particle numbering changes).
- Compiled cleanly; NOT tried in the app yet (no runs, per the user's request).
- Stage D is now complete.

## 2026-11-28 — session 16: filling gaps in Stages C-F (no long runs)

- **Spall strengths** checked against published plate-impact ranges: Al 6061-T6
  1.1 (0.8-2.3), Ti-6Al-4V 4.0 (3.3-4.9), Cu 1.3 (~1.28), steels 3.0 (~1.6-3.5,
  no 4340 value found) - kept. **Tungsten alloy 3.5 -> 2.0 GPa** (data
  1.7-2.6; was ~75% high). The long-rod validation was run with 3.5; not re-run.
- **Fragment readout in the app**: at the end of a run, Details > Results lists
  pieces per body (largest piece mass and speed) and loose debris.
- **Replay memory budget**: frames are reduced so playback fits ~1.5 GB (min
  30), shown in Results. 200 frames x 200k particles would be ~2.2 GB.
- **Silicon carbide (SiC-B)** added (material id 9, key silicon_carbide): K1/K2/K3
  220/361/0 GPa, G 193, HEL 11.7, A 0.96 N 0.65 B 0.35 M 1 C 0.009 (two
  sources agree); PHEL 5.58 derived (test); T 0.37, D1 = D2 = 0.48, cap 0.8
  approximate. Library tile silicon_carbide_tile_8mm.
- Only instant / non-simulating tests run (510 assertions pass).
- **Still open (need long runs - ask first):** Stage E convergence (long rod at
  0.25 mm, Borvik at 0.2 mm); Stage F: measure the glued-ceramic fix, validate
  ricochet / oblique impact against an experiment.

## 2026-11-28 — session 17: glued ceramic measured (short capped runs)

- User allowed short runs only (each capped at 5 min; these took ~2 min each).
- W rod 30x6 mm, 900 m/s, vs alumina 10 + Al 10 mm, 0.25 mm, 150 us:
  loose 12.7 g at 767 m/s, glued 12.5 g at 760 m/s. **Gluing does not close
  the ceramic gap** (kept as the default anyway: real tiles are bonded).
- Tried a crack-speed limit on JH-2 damage (dD/dt <= 0.4 c_s / dx): 761 m/s,
  no effect -> reverted, not committed.
- Diagnostic shots (temporary env switch, not committed): no tension damage
  -> 12.5 g @ 760 (no change); no compression damage -> 5.66 g @ 359. So the
  tile crushed too easily in COMPRESSION; tension was not the cause.
- Cause: alumina constants were a weaker low-purity set (rho 3700, G 90, HEL
  2.79). Replaced with AD-995 from Zhang et al. 2019 (Compos. Struct. 111258,
  Table 6): rho 3890, G 152, K1/K2/K3 231/-160/2774, HEL 6.57, A 0.88 N 0.64
  B 0.28 M 0.6 T 0.312 D1 0.01 D2 0.7; C 0.007 (paper prints 0.07); no cap.
  PHEL 3.51 derived (test).
- Same shot: 11.3 g @ 706 m/s (247 s wall), rod energy -22% vs before ->
  about the old steel result, better than equal-weight Al. Tile test passes
  (9 s). Equal-weight table in README is from the OLD constants (not re-run).

## 2026-11-28 — session 18: tank rounds and tank-front estimates

- New shape: `PolygonShape` (scenario files: "polygon", points_mm). Rods can
  have `nose_length_mm` (cone). projectileVolume / rodWorkingLength (cone = 1/3).
- Library: apfsds_dm53 (685x23, 1720), apfsds_m829a3 (670x25, 1555),
  apfsds_3bm60 (735x21, 1660), apfsds_3bm59 (735x21, 1650). DU modelled as
  tungsten: no sourced DU JC set found (JC 1983 table is a scan only).
- Tank fronts (steel_rha blocks, estimate_range_mm): M1A2 turret 770 [700,900],
  hull 550 [470,600]; T-90A turret 790 [700,850]; T-80U turret 750 [630,860],
  hull 715 [630,800]; Challenger 2 hull 530 [400,660]; Leopard 2A6 turret 825
  [800,850]. Sources: forum / Russian-attributed estimates (see README).
- App: Details > Tank check (Lanz-Odermatt, range slider, 60 m/s per km
  assumed); builder Theory notes a tank front's verdict; thickness slider to
  1000 mm; suggested end time up to 1500 us.
- LO muzzle depths: DM53 502, M829A3 438, 3BM60 510, 3BM59 506 mm.
- One full run (user allowed short runs): DM53 1720 m/s vs 770 mm, 1.5 mm,
  1000 us, 262 s -> stopped at ~660 mm (LO 502, +31%). Open: convergence
  (resolution ~8 particles per rod radius, L/D 30); needs longer runs.
- Fast tests pass (1040 assertions: library, builder, tank data, JH-2).

## 2026-11-28 — session 19: perforation equation; sim depth gap checked

- Tank check switched from Lanz-Odermatt semi-infinite DoP to Odermatt's
  perforation equation (longrods.ch 2022, read from the site's image):
  b0 0.283 b1 0.0656 m -0.224; W a 0.994 c0 134.5 c1 -0.148 (99 tests,
  3.3%); DU a 0.825 c0 90 c1 -0.0849 (42 tests, 3.5%). Plate BHN 260. DU
  rounds flagged (`depleted_uranium`), density 18600 in the formula only.
- Muzzle / 2 km: DM53 612/568, M829A3 594/555 (nose removed: 670 mm is
  the DU main rod behind a 100 mm steel tip), 3BM60 627/577, 3BM59
  632/597. Published claims mostly 630-700 (DM53 652 in War Thunder).
- Sim gap: DM53 to 300 us, 1.5 mm -> tip 247 mm (36 s), 1.0 mm -> 257 mm
  (231 s). NOT resolution. Full run 660 mm vs LO DoP 502 (+31%); the L/D 10
  at 1500 m/s check was within 6%. Next: L/D 10 at 1720 m/s (speed vs L/D).
- Speed vs slenderness: L/D 10 at 1750 m/s was already in the validation
  (+5.3%), so not speed. Same 1.5 mm spacing, 1720 m/s: L/D 10 full size
  (230x23) 266 mm vs LO 236 (+13%, 55 s); DM53 L/D 28 660 vs 499 (+32%).
  Sim misses about half of the experimental L/D penalty (tanh term).
  Recorded as a known limitation; app warns for Lw/D > 15. (A 0.25 mm L/D-30
  run hit the 5 min cap - 86k particles, too big; not repeated.)

## 2026-11-28 — session 20: setup view (free placement) + tank cross-sections

- Setup view (Build your own, before Simulate): parts drawn as shapes
  (ear-clipped polygons), drag to move, Q/E / Ctrl+wheel rotate (Shift 1 deg),
  R reset, flight arrow, overlap warning; panel "Position" with Move/Rotate/
  Reset; "Back to setup" / "Show last result". BuildSpec.placements (index 0
  projectile, k+1 layer k), applied about each item's centre; projectile
  velocity rotates with it; any move -> plane strain (needsSlice()).
- transformShape/shapeBounds in Scenario; ItemShape::Assembly (polygons with
  own materials, optional bond groups), front normalised to x = 0.
- tools/gen_tank_sections.py -> 5 turret sections (T-90A, T-80U, M1A2,
  Leopard 2A6, Challenger 2; vertical slices, 300 mm high). Rubber/explosive
  = gaps (materials step next: rubber for NERA, explosive for ERA).
- Tank check lists only the steel stand-in blocks (sections are the same tanks).
- Fast tests pass (1334 assertions). Setup view checked by a window capture.

## 2026-11-28 — session 21: "held by the tank" mounts

- Part.held; Solver::setupMounts: shot line from moving bodies (centre of
  mass, momentum direction); per held body MountZone(farthest distance,
  projectile diameter, dx): free within 3D + 10 mm, absorbing band (max(6dx,
  15% of reach)) at the far edges, rate 4c/band * w^2; v *= exp(-rate dt);
  release when failed / damage > 0.3 / plastic strain > 0.3; absorbed KE
  counted in Diagnostics/Frame (energy ratio includes it).
- Scenario files: "held": true. Builder: per-layer "Held by the tank" (default
  on for tank estimates and assemblies). Setup view hatches held bands with
  the same rule, warns when a layer has no room to be held.
- Test (30 us, 2 s): outer plate speed free 23.6 vs held 5.9 m/s, energy
  ratio 0.996 incl. absorbed. Hatching not yet checked visually.

## 2026-11-28 — session 22: flying debris (erosion) in 2D slices

- FailureKind kDebris; SolverSettings.erodeDebris (file key erode_debris,
  default on) applied only when NOT axisymmetric (erode_). Fully failed ->
  debris: skipped in force/strain-rate/time-step/ghost loops, pressure 0,
  keeps mass/velocity/energy. stickDebris: touching (< 0.75 dx, > 0.71 lattice
  gap) -> inelastic collision with all solid within 1.5 dx (same dv for the
  group), loss -> debris heat. Isolated solid (< 2 solid neighbours in
  1.5 dx) -> debris (fixed 40-85 GPa spikes). Frame.debris -> drawn small.
- Slice check (W slab 1500 m/s, 3 x 8 mm plates, 90 us): max speed 3171 ->
  1690 m/s, end max pressure 8.4 -> 2.0 GPa, energy 0.996 -> 1.000, rod
  1318 -> 1332 m/s.
- Round mode: erosion made L/D-10 P/L 0.942 (was 0.872, LO 0.852) and energy
  1.005 -> kept off there; validated behaviour unchanged.
