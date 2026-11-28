# Projectile & armour library

The building blocks for **Simulate → Build your own** in the app. Each item is one
small JSON file: a **shape** plus a **default material**. In the app you choose
the material for every item, so the same 10 mm plate can be tested in steel,
aluminium, titanium, copper or tungsten. Add your own items by copying a file.

```
library/
  projectiles/   spheres and flat-nosed rods
  armor/         plates
```

The app reads `$MIATAPHYSIM_LIBRARY` if set, then `./library`, then this folder of
the source tree. Unknown keys are an error, so typos can't slip through.

## Format

```json
{ "name": "Steel ball, 10 mm", "category": "projectile",
  "description": "Shown as a tooltip in the app.",
  "material": "steel_rha",
  "shape": { "type": "sphere", "diameter_mm": 10 },
  "speed_mps": 500 }
```

```json
{ "name": "Steel armour plate, 10 mm", "category": "armor",
  "material": "steel_rha",
  "shape": { "type": "plate", "thickness_mm": 10, "width_mm": 60 } }
```

| Key | Meaning |
|---|---|
| `category` | `"projectile"` (in `projectiles/`) or `"armor"` (in `armor/`) |
| `material` | Default material: `steel_rha`, `aluminium_6061`, `copper_ofhc`, `tungsten_wha`, `titanium_6al4v`, `weldox_460e`, `tool_steel_hrc53`, `alumina_ad995`, `boron_carbide`, `silicon_carbide` |
| `shape` | Projectiles: `sphere` (`diameter_mm`) or `rod` (`length_mm`, `diameter_mm`, flat nose). Armour: `plate` (`thickness_mm`, `width_mm`) |
| `speed_mps` | Suggested speed (projectiles) |

`width_mm` is the diameter of a round plate in straight-on shots, or the height of
the plate in angled (2D slice) shots.

**Ceramics** (`alumina_ad995`, `boron_carbide`) use the Johnson-Holmquist JH-2
model: very strong when squeezed, weak when pulled, and once cracked they turn
into rubble that still resists in proportion to pressure. Unbacked tiles shatter
widely, so put a metal layer directly behind them (gap 0), as in real composite
armour. Silicon carbide (`silicon_carbide`) is included too; its damage and tension constants are approximate.
