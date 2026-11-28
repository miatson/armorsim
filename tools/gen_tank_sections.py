# Generates tank armour cross-sections as library assembly items:
#  - TURRET CHEEKS: horizontal slices (x = along the shot from the front,
#    y = sideways), so the cheek's real slant shows;
#  - HULL FRONTS: vertical slices (y = up), so the plate slopes show.
# All layouts are reconstructions from public sources (Tankograd, Below The
# Turret Ring, Wikipedia/Zaloga, forum analyses) - real armour is secret.
#
# ARMOUR AMOUNT: each section carries the steel-equivalent mass along the shot
# line that public estimates imply: protection estimate / mass efficiency,
# with mass efficiency ~1.3-1.5 for Western NERA ("Burlington") and ~1.17 for
# Soviet reflecting-plate / spaced arrays (Tankograd). ERA explosive is not
# counted (empty gap until explosives exist). Check with tools/los.py.
import json, math, sys

out_dir = sys.argv[1]
H = 300.0  # slice height / width, mm (y from -150 to +150)
XMAX = 1400.0


def clip_convex(poly, clip):
    """Sutherland-Hodgman: polygon clipped to a convex, counter-clockwise polygon."""
    def inside(p, a, b):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0

    def inter(p, q, a, b):
        x1, y1, x2, y2 = p[0], p[1], q[0], q[1]
        x3, y3, x4, y4 = a[0], a[1], b[0], b[1]
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return [x1 + t * (x2 - x1), y1 + t * (y2 - y1)]

    out = poly
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break
        for j in range(len(inp)):
            p, q = inp[j - 1], inp[j]
            if inside(q, a, b):
                if not inside(p, a, b):
                    out.append(inter(p, q, a, b))
                out.append(q)
            elif inside(p, a, b):
                out.append(inter(p, q, a, b))
    return [[round(x, 2), round(y, 2)] for x, y in out]


def box(x0=0.0, x1=XMAX, y0=-H / 2, y1=H / 2):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def band(a_deg, s0, s1, L=6000.0):
    """Region s0 <= n.p <= s1 with n = (cos a, sin a): a plate tilted by a
    (0 = facing the shot squarely; positive = the upper end (+y) leans back,
    like a glacis). Returned counter-clockwise."""
    a = math.radians(a_deg)
    n = (math.cos(a), -math.sin(a))
    d = (-n[1], n[0])
    p0, p1 = (n[0] * s0, n[1] * s0), (n[0] * s1, n[1] * s1)
    return [[p0[0] - d[0] * L, p0[1] - d[1] * L], [p1[0] - d[0] * L, p1[1] - d[1] * L],
            [p1[0] + d[0] * L, p1[1] + d[1] * L], [p0[0] + d[0] * L, p0[1] + d[1] * L]]


def ok(poly):
    if len(poly) < 3:
        return False
    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    return max(xs) - min(xs) > 0.5 and max(ys) - min(ys) > 0.5


def plate(x0, t, a_deg, region=None):
    """Plate of normal thickness t whose front face crosses the shot line at x0."""
    s0 = x0 * math.cos(math.radians(a_deg))
    return clip_convex(band(a_deg, s0, s0 + t), region or box())


def stack(x, a_deg, layers, parts, region=None):
    """Layers one behind the other, all tilted by a: (t, material|None, name[, group]).
    material None = air gap. Returns x of the back face on the shot line."""
    c = math.cos(math.radians(a_deg))
    for layer in layers:
        t, mat, name = layer[0], layer[1], layer[2]
        if mat:
            p = plate(x, t, a_deg, region)
            if ok(p):
                part = {"name": name, "material": mat, "points_mm": p}
                if len(layer) > 3:
                    part["group"] = layer[3]
                parts.append(part)
        x += t / c
    return x


def cavity(x0, x1, a_deg):
    """The cavity between two faces tilted by a, crossing the shot line at x0 and x1."""
    c = math.cos(math.radians(a_deg))
    return clip_convex(band(a_deg, x0 * c, x1 * c), box())


def nera(region, b_deg, layers, pitch, label, parts, group0, offset=0.0):
    """Sandwich packets inside a region, tilted by b: layers (t, material, name)
    repeated every `pitch` (normal to the packets); each packet bonded."""
    xs = [q[0] for q in region]
    ys = [q[1] for q in region]
    a = math.radians(b_deg)
    n = (math.cos(a), -math.sin(a))
    corners = [(x, y) for x in (min(xs), max(xs)) for y in (min(ys), max(ys))]
    smin = min(cx * n[0] + cy * n[1] for cx, cy in corners)
    smax = max(cx * n[0] + cy * n[1] for cx, cy in corners)
    k, s = 0, smin + offset
    while s < smax:
        o = s
        for t, mat, nm in layers:
            p = clip_convex(band(b_deg, o, o + t), region)
            if ok(p):
                parts.append({"name": "%s %d: %s" % (label, k + 1, nm), "material": mat, "group": group0 + k,
                              "points_mm": p})
            o += t
        s += pitch
        k += 1


def disc(cx, cy, r, n=16):
    return [[round(cx + r * math.cos(2 * math.pi * k / n), 2), round(cy + r * math.sin(2 * math.pi * k / n), 2)]
            for k in range(n)]


def write(fname, name, desc, parts, rng):
    item = {"name": name, "category": "armor", "description": desc, "material": "steel_rha",
            "estimate_range_mm": rng, "shape": {"type": "assembly", "parts": parts}}
    json.dump(item, open(out_dir + "/" + fname, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(fname, len(parts), "pieces")


COMMON = (" Reconstruction from public sources (Tankograd, Below The Turret Ring, Zaloga/Wikipedia, forum analyses); "
          "real tank armour is secret. Armour amount = public protection estimate / mass efficiency (NERA ~1.3-1.5, "
          "Soviet arrays ~1.17). The explosive inside ERA is an empty gap until explosives are added, and the model "
          "under-represents NERA breaking long rods - so a full simulation is penetrated more easily than the real "
          "armour; the Tank check uses the estimates. 2D slice.")

HHS, RHA, RUB = "tool_steel_hrc53", "steel_rha", "rubber_neoprene"


def kontakt5(parts, x, a):
    """Kontakt-5 block lying on the armour (same angle): 15 mm high-hardness
    front plate, explosive cells (gap), 9 mm rear wall (Tankograd: 15 mm plate,
    9 mm box walls, 4S22 cells), then a ~30 mm stand-off. Returns where the
    armour behind it starts on the shot line."""
    return stack(x, a, [(15, HHS, "Kontakt-5 front plate 15 mm"), (40, None, "explosive (gap)"),
                        (9, HHS, "Kontakt-5 rear wall 9 mm"), (30, None, "stand-off")], parts)


# =============================================================================
# TURRETS (horizontal slices)
# =============================================================================

# ---- T-90A turret cheek ----------------------------------------------------------
# Tankograd (T-72B, which the T-90 insert follows; T-90A welded ~10% heavier):
# cheek at ~58.7 deg to the frontal shot; outer wall 90, inner 80-90 + 45 mm
# BTK-1Sh; 433 mm steel excl. NERA along the line; NERA modules 21 steel +
# 6 rubber + 3 steel with 22 mm gaps; front weight-equivalent ~648 mm steel.
parts = []
a = 58.7
x = kontakt5(parts, 0.0, a)
x = stack(x, a, [(100, RHA, "Outer wall 100 mm (welded)")], parts)
xc0 = x
x += 250 / math.cos(math.radians(a))
nera(cavity(xc0, x, a), a, [(21, RHA, "steel 21 mm"), (6, RUB, "rubber 6 mm"), (3, RHA, "bulging plate 3 mm")], 52,
     "Reflecting module", parts, 100)
stack(x, a, [(95, RHA, "Inner wall 95 mm"), (45, HHS, "BTK-1Sh plate 45 mm")], parts)
write("tank_t90a_turret_section.json", "Tank cross-section: T-90A turret cheek",
      "Welded turret seen from above: Kontakt-5 block in front, then the cheek at ~59 deg to a frontal shot - 100 mm "
      "outer wall, ~250 mm cavity of 'reflecting' NERA modules (21 mm steel + 6 mm rubber + 3 mm steel, 22 mm gaps), "
      "95 mm inner wall and a 45 mm high-hardness plate. Estimates vs long rods ~780-830 mm with Kontakt-5." + COMMON,
      parts, [700, 850])

# ---- T-80U turret cheek ----------------------------------------------------------
# Tankograd: cast walls 98 (front) and 190 (rear), 260 mm insert, ~548 mm
# physical but >800 mm along a frontal shot (-> ~47 deg); Kontakt-5 panels at
# 50-55 deg. Insert: ceramic cylinders (~60 mm) in polyurethane (one public
# estimate) / cellular polymer NERA (another).
parts = []
a = 46.8
x = kontakt5(parts, 0.0, a)
x = stack(x, a, [(98, RHA, "Cast outer wall 98 mm")], parts)
xc0, xc1 = x, x + 260 / math.cos(math.radians(a))
reg = cavity(xc0, xc1, a)
c, sn = math.cos(math.radians(a)), math.sin(math.radians(a))
for row, depth in enumerate([50, 130, 210]):
    for j in range(-8, 9):
        u = j * 72 + (36 if row == 1 else 0)  # along the cavity face
        cx, cy = xc0 + depth * c + u * sn, -depth * sn + u * c  # depth behind the face crossing (xc0, 0)
        d = disc(cx, cy, 30)
        if len(clip_convex(d, reg)) == len(d) and len(clip_convex(d, box())) == len(d):
            parts.append({"name": "Ceramic cylinder r%d-%d" % (row + 1, j + 9), "material": "alumina_ad995",
                          "group": 200, "points_mm": d})
parts.append({"name": "Insert polyurethane matrix 260 mm", "material": "polyurethane", "group": 200, "points_mm": reg})
stack(xc1, a, [(190, RHA, "Cast rear wall 190 mm")], parts)
write("tank_t80u_turret_section.json", "Tank cross-section: T-80U turret cheek",
      "Cast turret seen from above: Kontakt-5 in front, then the cheek at ~47 deg to a frontal shot - 98 mm cast "
      "outer wall, a 260 mm insert of ~60 mm alumina cylinders in polyurethane (estimates disagree: ceramic "
      "cylinders vs cellular polymer), 190 mm cast rear wall. Estimates vs long rods 630-860 mm with Kontakt-5." + COMMON,
      parts, [630, 860])

# ---- M1A2 (SEP) Abrams turret cheek ----------------------------------------------
# Slanted cheek; special-armour cavity with Burlington-type NERA packets and
# (HA) depleted-uranium layers; SEP turret ~940-960 mm vs APFSDS (Wikipedia
# cites), M1A1HA ~800 (Zaloga 2009). Mass target ~650 mm steel-eq (ME ~1.4).
parts = []
a = 30.0
x = stack(0.0, a, [(50, RHA, "Front plate 50 mm")], parts)
xc0 = x
x += 500 / math.cos(math.radians(a))
nera(cavity(xc0, x, a), a + 30, [(25, RHA, "steel 25 mm"), (10, RUB, "rubber 10 mm"), (6, RHA, "steel 6 mm")], 62,
     "NERA packet", parts, 100)
stack(x, a, [(10, None, "gap"), (25, "tungsten_wha", "DU armour layer 1 (tungsten stand-in)"), (10, None, "gap"),
             (25, "tungsten_wha", "DU armour layer 2 (tungsten stand-in)"), (10, None, "gap"),
             (120, RHA, "Back plate 120 mm")], parts)
write("tank_m1a2_turret_section.json", "Tank cross-section: M1A2 SEP Abrams turret cheek",
      "Slanted cheek (~30 deg) seen from above: 50 mm front plate, ~500 mm cavity of Burlington-type NERA packets "
      "(25 mm steel + 10 mm rubber + 6 mm steel, tilted a further 30 deg), two depleted-uranium armour layers (HA "
      "package; tungsten stand-in, no sourced DU data) and a 120 mm back plate. Estimates vs long rods: M1A1HA "
      "~800 mm, M1A2 SEP ~940-960 mm." + COMMON, parts, [770, 960])

# ---- Leopard 2A6 turret cheek -----------------------------------------------------
# Below The Turret Ring: Leopard 2A4 front ~860 mm LOS (800 without back
# plate), high-hardness steels + NERA, ME ~1.5 vs KE (forum); 2A5/2A6 wedge
# of two 45 mm high-hardness plates with interlayers, spaced in front.
parts = []
stack(30.0, 60.0, [(45, HHS, "Wedge plate 1 (45 mm high-hardness)"), (20, RUB, "Wedge interlayer (rubber)"),
                   (45, HHS, "Wedge plate 2 (45 mm high-hardness)")], parts, box(0.0, 330.0))
a = 10.0
x = stack(350.0, a, [(50, HHS, "Turret front plate 50 mm (high-hardness)")], parts)
xc0 = x
x += 600 / math.cos(math.radians(a))
nera(cavity(xc0, x, a), 45.0, [(20, HHS, "steel 20 mm"), (8, RUB, "rubber 8 mm")], 44, "NERA packet", parts, 100)
stack(x, a, [(60, RHA, "Back plate 60 mm")], parts)
write("tank_leopard2a6_turret_section.json", "Tank cross-section: Leopard 2A6 turret cheek",
      "Seen from above: the spaced add-on wedge (two 45 mm high-hardness plates with a rubber interlayer, ~60 deg) "
      "then the near-square turret front - 50 mm high-hardness front plate, ~600 mm NERA cavity (20 mm steel + "
      "8 mm rubber packets at 45 deg) and a 60 mm back plate (~860 mm deep, like the measured 2A4). Estimates vs "
      "long rods ~800-850 mm." + COMMON, parts, [800, 850])

# ---- Challenger 2 turret front ----------------------------------------------------
# Dorchester (second-generation Chobham) is the least public: ceramic tiles in
# a metal framework with elastic layers; ~870 mm deep near the gun. Sloped
# faces (~20 deg plan, ~30 deg vertical -> ~36 deg combined). Speculative.
parts = []
layers = [(60, RHA, "Front plate 60 mm")]
for k in range(5):
    layers += [(30, RHA, "Array steel %d (30 mm)" % (k + 1), 300 + k), (10, RUB, "Array rubber %d (10 mm)" % (k + 1), 300 + k),
               (30, "alumina_ad995", "Array ceramic %d (30 mm, assumed)" % (k + 1), 300 + k), (20, None, "gap")]
layers += [(120, RHA, "Back plate 120 mm")]
stack(0.0, 36.0, layers, parts)
write("tank_challenger2_turret_section.json", "Tank cross-section: Challenger 2 turret front",
      "Sloped turret front (~36 deg combined) seen from above: 60 mm front plate, five steel / rubber / ceramic "
      "groups with gaps, 120 mm back plate. SPECULATIVE - Dorchester's make-up is classified; layout follows the "
      "public Chobham description (ceramic tiles, metal framework, elastic layers)." + COMMON, parts, [600, 750])

# =============================================================================
# HULLS (vertical slices)
# =============================================================================

# ---- T-90A hull front -------------------------------------------------------------
# Tankograd: upper front plate at 68 deg; later T-72B/T-90 array 60-10-10-20-
# 20-50 (170 mm steel in 220 mm) ~550 mm vs APFSDS; Kontakt-5 on the glacis.
parts = []
a = 68.0
x = kontakt5(parts, 0.0, a)
stack(x, a, [(60, RHA, "Front plate 60 mm"), (10, None, "gap"), (10, RHA, "Plate 10 mm"), (10, None, "gap"),
                 (10, RHA, "Plate 10 mm"), (10, None, "gap"), (20, RHA, "Plate 20 mm"), (10, None, "gap"),
                 (20, RHA, "Plate 20 mm"), (10, None, "gap"), (50, RHA, "Rear plate 50 mm")], parts)
write("tank_t90a_hull_section.json", "Tank cross-section: T-90A hull front",
      "Upper front plate at 68 deg (side view): Kontakt-5 on top, then the spaced steel array 60-10-10-20-20-50 mm "
      "(170 mm of steel in ~220 mm). Estimates vs long rods ~550 mm without ERA, more with Kontakt-5." + COMMON,
      parts, [550, 750])

# ---- T-80U hull front -------------------------------------------------------------
# Tankograd: T-80BV/T-80U glacis 50 / 35 glass textolite / 50 / 35 / 50 at
# 68 deg (~430 mm RHA, Tarasenko) + Kontakt-5 panels at 68 deg.
parts = []
x = kontakt5(parts, 0.0, a)
stack(x, a, [(50, RHA, "Steel 50 mm"), (35, "glass_textolite", "Glass textolite 35 mm"), (50, RHA, "Steel 50 mm"),
                 (35, "glass_textolite", "Glass textolite 35 mm"), (50, RHA, "Steel 50 mm")], parts)
write("tank_t80u_hull_section.json", "Tank cross-section: T-80U hull front",
      "Upper front plate at 68 deg (side view): Kontakt-5 on top, then 50 mm steel / 35 mm glass textolite / 50 / 35 "
      "/ 50 mm (the T-80BV array). Estimates vs long rods ~430 mm without ERA, ~530-700 mm with Kontakt-5." + COMMON,
      parts, [530, 700])

# ---- M1A2 Abrams hull front -------------------------------------------------------
# Below The Turret Ring: the upper glacis is sloped plain steel; the lower
# front (nose) has the special armour - exterior steel, four NERA sandwich
# plates (about half the volume), thick inner steel; up to 2 ft thick
# (Green & Stewart). M1A1HA hull ~600 mm vs APFSDS (Zaloga 2009).
parts = []
layers = [(50, RHA, "Exterior steel 50 mm")]
for k in range(4):
    layers += [(25, RHA, "NERA sandwich %d: steel 25 mm" % (k + 1), 400 + k),
               (12, RUB, "NERA sandwich %d: rubber 12 mm" % (k + 1), 400 + k),
               (10, RHA, "NERA sandwich %d: steel 10 mm" % (k + 1), 400 + k), (50, None, "gap")]
layers += [(90, RHA, "Inner steel 90 mm")]
stack(0.0, 50.0, layers, parts)
write("tank_m1a2_hull_section.json", "Tank cross-section: M1A2 Abrams hull front (nose)",
      "Lower front / nose special armour at ~50 deg (side view): 50 mm exterior steel, four NERA sandwiches (25 mm "
      "steel + 12 mm rubber + 10 mm steel) taking about half the volume, 90 mm inner steel (~530 mm thick). The "
      "upper glacis above is thin, very sloped steel. Estimates vs long rods ~560-650 mm." + COMMON,
      parts, [560, 650])

# ---- Leopard 2A6 hull front -------------------------------------------------------
# Glacis relies on its extreme slope; the special armour is in the hull nose
# (estimate ~500-620 mm vs long rods for the late 2A4 hulls reused on 2A5/2A6).
parts = []
layers = [(40, HHS, "Exterior plate 40 mm (high-hardness)")]
for k in range(6):
    layers += [(20, HHS, "NERA %d: steel 20 mm" % (k + 1), 500 + k), (8, RUB, "NERA %d: rubber 8 mm" % (k + 1), 500 + k),
               (30, None, "gap")]
layers += [(110, RHA, "Inner plate 110 mm")]
stack(0.0, 50.0, layers, parts)
write("tank_leopard2a6_hull_section.json", "Tank cross-section: Leopard 2A6 hull front (nose)",
      "Hull nose special armour at ~50 deg (side view): 40 mm high-hardness exterior plate, six steel-rubber NERA "
      "packets with gaps, 110 mm inner plate. The glacis above relies on its extreme slope. Estimates vs long rods "
      "~500-620 mm." + COMMON, parts, [500, 620])

# ---- Challenger 2 hull front ------------------------------------------------------
parts = []
layers = [(50, RHA, "Front plate 50 mm")]
for k in range(3):
    layers += [(20, RHA, "Array steel %d (20 mm)" % (k + 1), 600 + k), (10, RUB, "Array rubber %d" % (k + 1), 600 + k),
               (25, "alumina_ad995", "Array ceramic %d (25 mm, assumed)" % (k + 1), 600 + k), (15, None, "gap")]
layers += [(60, RHA, "Back plate 60 mm")]
stack(0.0, 60.0, layers, parts)
write("tank_challenger2_hull_section.json", "Tank cross-section: Challenger 2 hull front",
      "Glacis at ~60 deg (side view): 50 mm front plate, three steel / rubber / ceramic groups, 60 mm back plate. "
      "SPECULATIVE (Dorchester is classified). Estimates vs long rods vary widely (~400-660 mm)." + COMMON,
      parts, [400, 660])
