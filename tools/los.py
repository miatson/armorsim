# Steel-equivalent mass of each tank cross-section along the shot line (mean of 5 lines
# around it): sum of thickness x density / 7850. Usage: python tools/los.py library/armor
import json, glob, sys
rho = {"steel_rha": 7850, "tool_steel_hrc53": 7850, "weldox_460e": 7850, "rubber_neoprene": 1439, "polyurethane": 1264,
       "alumina_ad995": 3890, "tungsten_wha": 17600, "aluminium_6061": 2703, "glass_textolite": 1942, "tungsten_y925": 17700}
def cross(poly, y):
    xs = []
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    xs.sort()
    return sum(xs[k + 1] - xs[k] for k in range(0, len(xs) - 1, 2))
for f in sorted(glob.glob(sys.argv[1] + "/tank_*_section.json")):
    it = json.load(open(f, encoding="utf-8"))
    per = {}
    for y in (-40, -20, 0, 20, 40):  # average over a few lines around the shot line
        for p in it["shape"]["parts"]:
            per[p["material"]] = per.get(p["material"], 0) + cross(p["points_mm"], y) / 5
    total = sum(per.values()); steq = sum(v * rho[m] / 7850 for m, v in per.items())
    print("%-45s depth %4.0f mm | material on the line %4.0f mm | steel-equivalent mass %4.0f mm | est %s" %
          (it["name"], max(q[0] for p in it["shape"]["parts"] for q in p["points_mm"]), total, steq, it.get("estimate_range_mm")))
    print("     " + ", ".join("%s %.0f" % (m, v) for m, v in per.items()))
