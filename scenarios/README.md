# Scenario files

Each `.json` file here describes one simulation. The app lists them in the
scenario dropdown as **[file] ...**, and you can run one without a window:

```
miataphysim --headless --file scenarios/spall_copper.json --csv spall.csv
```

The app looks for scenario files in `$MIATAPHYSIM_SCENARIOS`, then in `./scenarios`,
then in this folder of the source tree. Comments (`//`) are allowed. **Unknown keys
are an error**, so a typo can't be silently ignored.

## Format

| Key | Meaning |
|---|---|
| `name` | Title (required) |
| `description` | Text shown in the app |
| `mode` | `"axisymmetric"` (x = axis, y = radius; real round bodies, straight-on impacts only) or `"plane_strain"` (a slice through infinitely long bodies; any angle) |
| `spacing_mm` | Particle spacing. Plastic impacts want ≈ 0.13 mm or finer for converged numbers |
| `end_time_us` | Simulated time |
| `frames` | Frames recorded for replay in the app (default 200) |
| `settings` | Optional solver settings: `failure`, `cfl`, `visc_alpha`, `visc_beta`, `balsara_switch`, `balsara_min`, `density_filter_interval`, `coalescence_porosity`, `spall_incubation_time_us`, `threads` |
| `parts` | List of bodies (below) |
| `probes` | Optional list of probes (below) |
| `probe_interval_us` | Probe sampling interval (default end time / 1000) |

### Parts

| Key | Meaning |
|---|---|
| `name` | Name (shown in the readout) |
| `material` | `steel_rha`, `aluminium_6061`, `copper_ofhc`, `tungsten_wha`, `titanium_6al4v`, `weldox_460e`, `tool_steel_hrc53`, `alumina_ad995`, `boron_carbide` (or the full name) |
| `shape` | See below |
| `velocity_mps` | `[vx, vy]`, m/s. Axisymmetric: vy must be 0 |
| `temperature_K` | Initial temperature (default 293.15) |
| `contact_group` | Parts with the same group (≥ 0) are bonded into one body (e.g. glued layers); default −1 = separate |

Where parts overlap, the earlier one in the list wins.

### Shapes

| `type` | Keys | Notes |
|---|---|---|
| `cylinder` | `x_mm: [from, to]`, `radius_mm` | Axisymmetric: a solid round rod or plate on the axis |
| `sphere` | `center_x_mm`, `radius_mm` | Axisymmetric: a ball on the axis |
| `rect` | `center_mm: [x, y]`, `size_mm: [w, h]`, `angle_deg` | Plane strain; in axisymmetric mode a rect off the axis is a ring |
| `disc` | `center_mm`, `radius_mm` | Plane strain: a long round bar seen end-on |

### Probes

A probe follows the material point nearest `at_mm` at t = 0, like a gauge embedded
in the material (or a VISAR spot on a surface). Quantities: `velocity_x`,
`velocity_y`, `speed`, `pressure`, `von_mises`, `stress_xx`, `stress_yy`,
`shear_xy`, `density`, `plastic_strain`, `temperature`, `damage`, `porosity`,
`x`, `y`. The CSV has one row per sample, in SI units, with the unit in each header.

## Files here

- `spall_copper.json`: plate-impact spall with a back-face "VISAR" probe.
- `taylor_bar_copper.json`: the Taylor test (Johnson & Cook 1983).
- `long_rod_wha_rha.json`: a tungsten long rod into armour steel at 1500 m/s.
