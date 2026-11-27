#include "armor/Library.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>

#include <nlohmann/json.hpp>

namespace armor {

namespace {

using json = nlohmann::json;
constexpr double kMm = 1e-3;

struct ItemReader {
    std::string source;
    [[noreturn]] void fail(const std::string& path, const std::string& what) const {
        throw ScenarioFileError(source + ": " + path + ": " + what);
    }
    void onlyKeys(const json& obj, const std::string& path, std::initializer_list<const char*> allowed) const {
        if (!obj.is_object()) fail(path, "expected an object { ... }");
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            bool ok = false;
            for (const char* a : allowed) ok = ok || it.key() == a;
            if (!ok) fail(path + "." + it.key(), "unknown key");
        }
    }
    const json& need(const json& obj, const std::string& path, const char* key) const {
        if (!obj.contains(key)) fail(path, std::string("missing \"") + key + "\"");
        return obj.at(key);
    }
    double positive(const json& v, const std::string& path) const {
        if (!v.is_number() || !(v.get<double>() > 0.0)) fail(path, "expected a positive number");
        return v.get<double>();
    }
    std::string text(const json& v, const std::string& path) const {
        if (!v.is_string()) fail(path, "expected a string");
        return v.get<std::string>();
    }
};

std::vector<std::filesystem::path> jsonFiles(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return files;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec))
        if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    return files;
}

std::string readFile(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) throw ScenarioFileError(p.string() + ": cannot open the file");
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Half-height of the projectile across the shot line (its "radius").
double projectileHalfHeight(const LibraryItem& p) { return 0.5 * p.diameter; }
double projectileLength(const LibraryItem& p) { return p.shape == ItemShape::Rod ? p.length : p.diameter; }

} // namespace

LibraryItem parseLibraryItem(const std::string& text, std::span<const Material> materials, const std::string& sourceName) {
    const ItemReader rd{sourceName};
    json root;
    try {
        root = json::parse(text, nullptr, true, /*ignore_comments=*/true);
    } catch (const json::parse_error& e) {
        throw ScenarioFileError(sourceName + ": not valid JSON: " + e.what());
    }
    rd.onlyKeys(root, "$", {"name", "category", "description", "material", "shape", "speed_mps"});
    LibraryItem item;
    item.file = sourceName;
    item.name = rd.text(rd.need(root, "$", "name"), "$.name");
    if (root.contains("description")) item.description = rd.text(root["description"], "$.description");
    const std::string cat = rd.text(rd.need(root, "$", "category"), "$.category");
    if (cat == "projectile") item.category = ItemCategory::Projectile;
    else if (cat == "armor") item.category = ItemCategory::Armor;
    else rd.fail("$.category", "expected \"projectile\" or \"armor\"");
    const std::string mat = rd.text(rd.need(root, "$", "material"), "$.material");
    item.material = findMaterial(materials, mat);
    if (item.material < 0) rd.fail("$.material", "unknown material \"" + mat + "\"");
    const json& s = rd.need(root, "$", "shape");
    const std::string type = rd.text(rd.need(s, "$.shape", "type"), "$.shape.type");
    if (type == "sphere") {
        rd.onlyKeys(s, "$.shape", {"type", "diameter_mm"});
        item.shape = ItemShape::Sphere;
        item.diameter = rd.positive(rd.need(s, "$.shape", "diameter_mm"), "$.shape.diameter_mm") * kMm;
    } else if (type == "rod") {
        rd.onlyKeys(s, "$.shape", {"type", "length_mm", "diameter_mm"});
        item.shape = ItemShape::Rod;
        item.length = rd.positive(rd.need(s, "$.shape", "length_mm"), "$.shape.length_mm") * kMm;
        item.diameter = rd.positive(rd.need(s, "$.shape", "diameter_mm"), "$.shape.diameter_mm") * kMm;
    } else if (type == "plate") {
        rd.onlyKeys(s, "$.shape", {"type", "thickness_mm", "width_mm"});
        item.shape = ItemShape::Plate;
        item.thickness = rd.positive(rd.need(s, "$.shape", "thickness_mm"), "$.shape.thickness_mm") * kMm;
        item.width = rd.positive(rd.need(s, "$.shape", "width_mm"), "$.shape.width_mm") * kMm;
    } else {
        rd.fail("$.shape.type", "unknown shape \"" + type + "\" (sphere, rod, plate)");
    }
    if (item.category == ItemCategory::Projectile && item.shape == ItemShape::Plate)
        rd.fail("$.shape.type", "a projectile must be a sphere or a rod");
    if (item.category == ItemCategory::Armor && item.shape != ItemShape::Plate)
        rd.fail("$.shape.type", "armour must be a plate");
    if (root.contains("speed_mps")) item.speed = rd.positive(root["speed_mps"], "$.speed_mps");
    return item;
}

Library loadLibrary(const std::filesystem::path& root, std::span<const Material> materials) {
    Library lib;
    auto load = [&](const std::filesystem::path& dir, std::vector<LibraryItem>& out, ItemCategory want) {
        for (const auto& path : jsonFiles(dir)) {
            try {
                LibraryItem item = parseLibraryItem(readFile(path), materials, path.filename().string());
                if (item.category != want)
                    throw ScenarioFileError(path.filename().string() + ": category does not match its folder");
                out.push_back(std::move(item));
            } catch (const ScenarioFileError& e) {
                lib.errors.push_back(e.what());
            }
        }
    };
    load(root / "projectiles", lib.projectiles, ItemCategory::Projectile);
    load(root / "armor", lib.armor, ItemCategory::Armor);
    return lib;
}

// Spacing: fine enough to resolve the smallest feature (1/20 of the
// projectile diameter, 1/8 of the thinnest layer), coarse enough to keep the
// run near 40,000 particles at most; then rounded to 0.01 mm.
double suggestSpacing(const BuildSpec& spec) {
    const bool axi = spec.obliquity == 0.0;
    double feature = spec.projectile.diameter / 20.0;
    double area = 0.0; // cross-section area in the simulated half/slice
    const double r = projectileHalfHeight(spec.projectile);
    area += spec.projectile.shape == ItemShape::Rod ? spec.projectile.length * (axi ? r : 2 * r)
                                                     : 3.14159265 * r * r * (axi ? 0.5 : 1.0);
    for (const ArmorLayer& l : spec.layers) {
        feature = std::min(feature, l.thickness / 8.0);
        area += l.thickness * (axi ? 0.5 : 1.0) * l.item.width;
    }
    const double budget = std::sqrt(area / 40000.0);
    double s = std::clamp(std::max(feature, budget), 0.05e-3, 2.0e-3);
    return std::round(s / 1e-5) * 1e-5;
}

// Duration: the projectile travels through the whole stack and its own
// length 1.5 times over at its initial speed (it slows down, so most of the
// action is covered), plus 5 us; 5 .. 400 us.
double suggestEndTime(const BuildSpec& spec) {
    double distance = projectileLength(spec.projectile);
    const double c = std::cos(spec.obliquity);
    for (std::size_t k = 0; k < spec.layers.size(); ++k)
        distance += (spec.layers[k].thickness + (k ? spec.layers[k].gapBefore : 0.0)) / c;
    const double t = 1.5 * distance / std::max(spec.speed, 1.0) + 5e-6;
    return std::round(std::clamp(t, 5e-6, 400e-6) / 1e-6) * 1e-6;
}

ScenarioFile buildScenario(const BuildSpec& spec, std::span<const Material> materials) {
    if (spec.layers.empty()) throw ScenarioFileError("builder: add at least one armour layer");
    ScenarioFile out;
    Scenario& s = out.scenario;
    const bool axi = spec.obliquity == 0.0;
    const double theta = spec.obliquity;
    const double c = std::cos(theta), tn = std::tan(theta);
    s.mode = axi ? GeometryMode::Axisymmetric : GeometryMode::PlaneStrain;
    s.dx = spec.spacing;
    s.endTime = spec.endTime;
    const double gap0 = spec.spacing; // keeps the projectile one spacing clear at t = 0

    // Armour stack along the shot line (y = 0), plates parallel. Along the
    // line a plate tilted by theta is T / cos(theta) thick.
    double x = 0.0;
    double firstCentre = 0.0, lastBack = 0.0;
    for (std::size_t k = 0; k < spec.layers.size(); ++k) {
        const ArmorLayer& l = spec.layers[k];
        if (k > 0) x += l.gapBefore / c;
        const double alongLine = l.thickness / c;
        const double cx = x + 0.5 * alongLine;
        if (k == 0) firstCentre = cx;
        Part part;
        part.name = "Layer " + std::to_string(k + 1) + ": " + l.item.name;
        part.material = l.material;
        if (axi) part.shape = RectShape{{cx, 0.25 * l.item.width}, {0.5 * l.thickness, 0.25 * l.item.width}, 0.0};
        else part.shape = RectShape{{cx, 0.0}, {0.5 * l.thickness, 0.5 * l.item.width}, theta};
        s.parts.push_back(part);
        x += alongLine;
        lastBack = x;
    }

    // Projectile: its nose just clear of the first plate's front face where it
    // will hit. A plate tilted by theta has its front face at
    // x = cx − T/(2cos θ) − y tan θ, nearest at the top of the projectile.
    const LibraryItem& p = spec.projectile;
    const double r = projectileHalfHeight(p);
    const double faceAtTop = firstCentre - 0.5 * spec.layers[0].thickness / c - r * tn;
    const double nose = std::min(0.0, faceAtTop) - gap0;
    Part proj;
    proj.name = p.name;
    proj.material = spec.projectileMaterial;
    proj.velocity = {spec.speed, 0.0};
    double tail = nose;
    if (p.shape == ItemShape::Rod) {
        tail = nose - p.length;
        if (axi) proj.shape = RectShape{{nose - 0.5 * p.length, 0.5 * r}, {0.5 * p.length, 0.5 * r}, 0.0};
        else proj.shape = RectShape{{nose - 0.5 * p.length, 0.0}, {0.5 * p.length, r}, 0.0};
    } else {
        tail = nose - 2.0 * r;
        proj.shape = DiscShape{{nose - r, 0.0}, r};
    }
    // The projectile first: where shapes overlap, the earlier part wins.
    s.parts.insert(s.parts.begin(), proj);

    s.name = p.name + " -> " + spec.layers[0].item.name + (spec.layers.size() > 1 ? " (+ more layers)" : "");
    char buf[160];
    std::snprintf(buf, sizeof buf, "%.0f m/s%s, %zu layer(s).", spec.speed,
                  axi ? ", straight on (axisymmetric)" : ", oblique (plane-strain slice)", spec.layers.size());
    out.description = std::string("Built in the app: ") + p.name + " at " + buf;

    const double py = axi ? 0.5 * spec.spacing : 0.0;
    out.probes.push_back({"projectile tail", {tail + 0.5 * spec.spacing, py}, {ProbeQuantity::PositionX, ProbeQuantity::VelocityX}});
    out.probes.push_back({"back of last layer", {lastBack - 0.5 * spec.spacing, py}, {ProbeQuantity::VelocityX, ProbeQuantity::Damage}});
    out.probeInterval = spec.endTime / 600.0;
    (void)materials;
    return out;
}

std::string scenarioToJson(const ScenarioFile& file, std::span<const Material> materials) {
    const Scenario& s = file.scenario;
    auto mm = [](double v) { return std::round(v * 1e3 * 1e6) / 1e6; }; // m -> mm, 6 decimals
    json root = json::object();
    root["name"] = s.name;
    if (!file.description.empty()) root["description"] = file.description;
    root["mode"] = s.mode == GeometryMode::Axisymmetric ? "axisymmetric" : "plane_strain";
    root["spacing_mm"] = mm(s.dx);
    root["end_time_us"] = std::round(s.endTime * 1e6 * 1e6) / 1e6;
    root["frames"] = s.frameCount;
    root["settings"] = {{"failure", s.settings.failure}};
    json parts = json::array();
    for (const Part& part : s.parts) {
        json pj;
        pj["name"] = part.name;
        const Material& m = materials[static_cast<std::size_t>(part.material)];
        pj["material"] = m.key.empty() ? m.name : m.key;
        if (const auto* r = std::get_if<RectShape>(&part.shape)) {
            pj["shape"] = {{"type", "rect"},
                           {"center_mm", {mm(r->center.x), mm(r->center.y)}},
                           {"size_mm", {mm(2 * r->halfSize.x), mm(2 * r->halfSize.y)}}};
            if (r->angle != 0.0) pj["shape"]["angle_deg"] = r->angle * 180.0 / 3.14159265358979323846;
        } else if (const auto* d = std::get_if<DiscShape>(&part.shape)) {
            pj["shape"] = {{"type", "disc"}, {"center_mm", {mm(d->center.x), mm(d->center.y)}}, {"radius_mm", mm(d->radius)}};
        }
        pj["velocity_mps"] = {part.velocity.x, part.velocity.y};
        if (part.temperature != 293.15) pj["temperature_K"] = part.temperature;
        if (part.contactGroup >= 0) pj["contact_group"] = part.contactGroup;
        parts.push_back(pj);
    }
    root["parts"] = parts;
    if (!file.probes.empty()) {
        json probes = json::array();
        for (const ProbeSpec& pr : file.probes) {
            json q = json::array();
            for (ProbeQuantity pq : pr.quantities) q.push_back(probeQuantityName(pq));
            probes.push_back({{"name", pr.name}, {"at_mm", {mm(pr.position.x), mm(pr.position.y)}}, {"quantities", q}});
        }
        root["probes"] = probes;
        if (file.probeInterval > 0) root["probe_interval_us"] = std::round(file.probeInterval * 1e6 * 1e6) / 1e6;
    }
    return root.dump(2);
}

} // namespace armor
