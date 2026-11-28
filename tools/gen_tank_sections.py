# Generates tank turret-cheek cross-sections (vertical slices: x = along the
# shot, from the front; y = up) as library assembly items. All layouts are
# approximate reconstructions from public sources; see each description.
import json, math, sys

out_dir = sys.argv[1]
H = 300.0  # height of the slice, mm (y from -150 to +150)

def rect(x0, x1, y0=-H / 2, y1=H / 2):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]

def clip(poly, x0, x1, y0, y1):
    """Sutherland-Hodgman clip of a polygon to a box."""
    def clip_edge(pts, inside, inter):
        out = []
        for i in range(len(pts)):
            a, b = pts[i - 1], pts[i]
            ia, ib = inside(a), inside(b)
            if ib:
                if not ia: out.append(inter(a, b))
                out.append(b)
            elif ia:
                out.append(inter(a, b))
        return out
    def ix(a, b, x):
        t = (x - a[0]) / (b[0] - a[0]); return [x, a[1] + t * (b[1] - a[1])]
    def iy(a, b, y):
        t = (y - a[1]) / (b[1] - a[1]); return [a[0] + t * (b[0] - a[0]), y]
    p = poly
    for inside, inter in [(lambda q: q[0] >= x0, lambda a, b: ix(a, b, x0)),
                          (lambda q: q[0] <= x1, lambda a, b: ix(a, b, x1)),
                          (lambda q: q[1] >= y0, lambda a, b: iy(a, b, y0)),
                          (lambda q: q[1] <= y1, lambda a, b: iy(a, b, y1))]:
        if not p: return []
        p = clip_edge(p, inside, inter)
    return [[round(q[0], 2), round(q[1], 2)] for q in p]

def slanted_plates(x0, x1, angle_deg, thickness, pitch, y0=-H / 2, y1=H / 2):
    """Parallel plates crossing the box [x0,x1]x[y0,y1]. angle_deg = tilt of the
    plates from vertical (0 = facing the shot squarely). thickness and pitch
    are measured perpendicular to the plates."""
    a = math.radians(angle_deg)
    n = (math.cos(a), math.sin(a))      # plate normal
    d = (-math.sin(a), math.cos(a))     # along the plate
    L = 4 * (x1 - x0 + y1 - y0)
    pieces = []
    # offsets along the normal covering the box
    corners = [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]
    s_min = min(c[0] * n[0] + c[1] * n[1] for c in corners)
    s_max = max(c[0] * n[0] + c[1] * n[1] for c in corners)
    s = s_min + 0.5 * (pitch - thickness)
    while s < s_max:
        p0 = (n[0] * s, n[1] * s)
        p1 = (n[0] * (s + thickness), n[1] * (s + thickness))
        band = [[p0[0] - d[0] * L, p0[1] - d[1] * L], [p0[0] + d[0] * L, p0[1] + d[1] * L],
                [p1[0] + d[0] * L, p1[1] + d[1] * L], [p1[0] - d[0] * L, p1[1] - d[1] * L]]
        c = clip(band, x0, x1, y0, y1)
        if len(c) >= 3:
            xs = [q[0] for q in c]; ys = [q[1] for q in c]
            if max(xs) - min(xs) > 1 and max(ys) - min(ys) > 1:
                pieces.append(c)
        s += pitch
    return pieces

def slab(x0, thick, angle_deg, y0=-H / 2, y1=H / 2):
    """A plate whose front face passes through (x0, 0), tilted angle_deg from
    vertical (top leaning back), cut to the slice height."""
    a = math.radians(angle_deg)
    t = math.tan(a)
    w = thick / math.cos(a)  # horizontal thickness
    return [[x0 + y0 * t, y0], [x0 + y0 * t + w, y0], [x0 + y1 * t + w, y1], [x0 + y1 * t, y1]]

def write(fname, name, desc, parts, rng=None):
    item = {"name": name, "category": "armor", "description": desc, "material": "steel_rha",
            "shape": {"type": "assembly", "parts": parts}}
    if rng: item["estimate_range_mm"] = rng
    json.dump(item, open(out_dir + "/" + fname, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(fname, len(parts), "pieces")

COMMON = (" Approximate reconstruction from public sources (forums, modders' and game reconstructions); real "
          "tank armour is secret. Rubber (NERA) and explosive (ERA) layers are empty gaps until those materials "
          "are added, so this shape protects LESS than the real thing. Simulated as a 2D vertical slice.")

# ---- T-90A turret cheek ----------------------------------------------------
# Kontakt-5 block (inert for now: two 15 mm plates, gap for the explosive),
# 65 mm outer wall, cavity with 'reflecting sheet' packets (21 mm steel +
# 6 mm rubber + 3 mm steel; steel merged to 24 mm), inner wall 150 mm.
parts = []
for k, (x, th) in enumerate([(0, 15), (60, 15)]):
    parts.append({"name": "Kontakt-5 plate %d (ERA, inert)" % (k + 1), "material": "steel_rha",
                  "points_mm": clip(slab(x + 60, th, 60), 0, 190, -H / 2, H / 2)})
parts.append({"name": "Outer wall 65 mm", "material": "steel_rha", "points_mm": rect(200, 265)})
for k, p in enumerate(slanted_plates(265, 765, 55, 24, 52)):
    parts.append({"name": "Reflecting sheet %d (steel; rubber = gap)" % (k + 1), "material": "steel_rha", "points_mm": p})
parts.append({"name": "Inner wall 150 mm", "material": "steel_rha", "points_mm": rect(765, 915)})
write("tank_t90a_turret_section.json", "Tank cross-section: T-90A turret cheek",
      "Welded turret: Kontakt-5 explosive reactive armour in front (two 15 mm plates at 60 deg; explosive not "
      "modelled), a 65 mm outer wall, a ~500 mm cavity of 'reflecting sheet' packets (steel-rubber-steel, tilted "
      "55 deg) and a 150 mm inner wall. Estimates vs long rods ~780-830 mm with Kontakt-5." + COMMON, parts, [700, 850])

# ---- T-80U turret cheek ----------------------------------------------------
parts = []
for k, (x, th) in enumerate([(0, 15), (60, 15)]):
    parts.append({"name": "Kontakt-5 plate %d (ERA, inert)" % (k + 1), "material": "steel_rha",
                  "points_mm": clip(slab(x + 60, th, 60), 0, 190, -H / 2, H / 2)})
parts.append({"name": "Cast outer wall 80 mm", "material": "steel_rha", "points_mm": rect(200, 280)})
parts.append({"name": "Cell insert 1 (alloy shell; polymer cells as aluminium)", "material": "aluminium_6061", "points_mm": rect(320, 400)})
parts.append({"name": "Cell insert 2 (alloy shell; polymer cells as aluminium)", "material": "aluminium_6061", "points_mm": rect(440, 520)})
parts.append({"name": "Cast inner wall 300 mm", "material": "steel_rha", "points_mm": rect(560, 860)})
write("tank_t80u_turret_section.json", "Tank cross-section: T-80U turret cheek",
      "Cast turret: Kontakt-5 in front (inert), cast steel walls and 'cellular' inserts - two 80 mm polymer-cell "
      "layers in alloy shells with 40 mm hollows (polymer stood in by aluminium). Inserts ~815 mm deep. Estimates "
      "vs long rods 630-860 mm with Kontakt-5." + COMMON, parts, [630, 860])

# ---- M1A2 Abrams turret cheek ---------------------------------------------
parts = [{"name": "Front plate 60 mm", "material": "steel_rha", "points_mm": rect(0, 60)}]
for k, p in enumerate(slanted_plates(60, 700, 60, 25, 70)):
    parts.append({"name": "NERA plate %d (steel; rubber = gap)" % (k + 1), "material": "steel_rha", "points_mm": p})
parts.append({"name": "Back plate 120 mm", "material": "steel_rha", "points_mm": rect(700, 820)})
write("tank_m1a2_turret_section.json", "Tank cross-section: M1A2 Abrams turret cheek",
      "Welded turret with a special-armour cavity: 60 mm front plate, a ~640 mm array of steel-rubber NERA plates "
      "tilted 60 deg (rubber not modelled; depleted-uranium layers of later versions not modelled) and a 120 mm back "
      "plate. Public estimates: about 430 mm of steel excluding the NERA panels; ~770 mm vs long rods." + COMMON,
      parts, [700, 900])

# ---- Leopard 2A6 turret cheek ---------------------------------------------
parts = []
for k, x in enumerate([0, 130]):
    parts.append({"name": "Wedge plate %d (45 mm hard steel)" % (k + 1), "material": "tool_steel_hrc53",
                  "points_mm": clip(slab(x + 40, 45, 60), 0, 320, -H / 2, H / 2)})
parts.append({"name": "Turret front plate 50 mm", "material": "steel_rha", "points_mm": rect(330, 380)})
for k, p in enumerate(slanted_plates(380, 900, 45, 20, 55)):
    parts.append({"name": "NERA plate %d (steel; rubber = gap)" % (k + 1), "material": "steel_rha", "points_mm": p})
parts.append({"name": "Back plate 100 mm", "material": "steel_rha", "points_mm": rect(900, 1000)})
write("tank_leopard2a6_turret_section.json", "Tank cross-section: Leopard 2A6 turret cheek",
      "Spaced add-on wedge (two 45 mm hard-steel plates at ~60 deg, with interlayers) a few hundred mm in front of "
      "the turret front, then a NERA array tilted 45 deg and a 100 mm back plate; cheek ~760 mm deep without the "
      "wedge. Estimates vs long rods ~800-850 mm." + COMMON, parts, [800, 850])

# ---- Challenger 2 turret front --------------------------------------------
parts = [{"name": "Front plate 80 mm", "material": "steel_rha", "points_mm": rect(0, 80)}]
x = 80
for k in range(6):
    parts.append({"name": "Array steel plate %d (20 mm)" % (k + 1), "material": "steel_rha", "points_mm": rect(x, x + 20)})
    x += 20 + 15  # 15 mm rubber interlayer = gap
    parts.append({"name": "Array ceramic tile %d (25 mm, assumed)" % (k + 1), "material": "alumina_ad995", "points_mm": rect(x, x + 25)})
    x += 25 + 40
parts.append({"name": "Back plate 120 mm", "material": "steel_rha", "points_mm": rect(x, x + 120)})
write("tank_challenger2_turret_section.json", "Tank cross-section: Challenger 2 turret front",
      "Dorchester (Chobham-family) armour is the least public of all: here a generic array of steel plates, rubber "
      "interlayers (gaps) and ceramic tiles between an 80 mm front plate and a 120 mm back plate, ~870 mm deep near "
      "the gun. SPECULATIVE layout. Estimates vs long rods vary widely." + COMMON, parts, [400, 750])
