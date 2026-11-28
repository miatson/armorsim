#include "armor/ScenarioFile.h"

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>

#include <nlohmann/json.hpp>

namespace armor {

namespace {

using json = nlohmann::json;
constexpr double kMm = 1e-3, kUs = 1e-6, kDeg = 3.14159265358979323846 / 180.0;

// Everything below reports errors as "<file>: <path>: <problem>".
struct Reader {
    std::string source;

    [[noreturn]] void fail(const std::string& path, const std::string& what) const {
        throw ScenarioFileError(source + ": " + path + ": " + what);
    }

    void onlyKeys(const json& obj, const std::string& path, std::initializer_list<const char*> allowed) const {
        if (!obj.is_object()) fail(path, "expected an object { ... }");
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            bool ok = false;
            for (const char* a : allowed) ok = ok || it.key() == a;
            if (!ok) {
                std::string list;
                for (const char* a : allowed) list += std::string(list.empty() ? "" : ", ") + a;
                fail(path + "." + it.key(), "unknown key (allowed here: " + list + ")");
            }
        }
    }

    const json& need(const json& obj, const std::string& path, const char* key) const {
        if (!obj.contains(key)) fail(path, std::string("missing \"") + key + "\"");
        return obj.at(key);
    }

    double number(const json& v, const std::string& path) const {
        if (!v.is_number()) fail(path, "expected a number");
        const double d = v.get<double>();
        if (!std::isfinite(d)) fail(path, "not a finite number");
        return d;
    }

    double positive(const json& v, const std::string& path) const {
        const double d = number(v, path);
        if (d <= 0.0) fail(path, "must be positive");
        return d;
    }

    Vec2 pair(const json& v, const std::string& path) const {
        if (!v.is_array() || v.size() != 2) fail(path, "expected [x, y]");
        return {number(v[0], path + "[0]"), number(v[1], path + "[1]")};
    }

    std::string text(const json& v, const std::string& path) const {
        if (!v.is_string()) fail(path, "expected a string");
        return v.get<std::string>();
    }

    bool boolean(const json& v, const std::string& path) const {
        if (!v.is_boolean()) fail(path, "expected true or false");
        return v.get<bool>();
    }

    Shape shape(const json& s, const std::string& path, GeometryMode mode) const {
        const std::string type = text(need(s, path, "type"), path + ".type");
        const bool axi = mode == GeometryMode::Axisymmetric;
        if (type == "rect") {
            onlyKeys(s, path, {"type", "center_mm", "size_mm", "angle_deg"});
            const Vec2 c = pair(need(s, path, "center_mm"), path + ".center_mm");
            const Vec2 size = pair(need(s, path, "size_mm"), path + ".size_mm");
            if (size.x <= 0 || size.y <= 0) fail(path + ".size_mm", "sizes must be positive");
            const double angle = s.contains("angle_deg") ? number(s["angle_deg"], path + ".angle_deg") * kDeg : 0.0;
            if (axi && angle != 0.0) fail(path + ".angle_deg", "rotated shapes need plane_strain mode");
            return RectShape{c * kMm, size * (0.5 * kMm), angle};
        }
        if (type == "disc") {
            onlyKeys(s, path, {"type", "center_mm", "radius_mm"});
            return DiscShape{pair(need(s, path, "center_mm"), path + ".center_mm") * kMm,
                             positive(need(s, path, "radius_mm"), path + ".radius_mm") * kMm};
        }
        if (type == "cylinder") {
            if (!axi) fail(path + ".type", "\"cylinder\" needs axisymmetric mode (use \"rect\" in plane_strain)");
            onlyKeys(s, path, {"type", "x_mm", "radius_mm"});
            const Vec2 xr = pair(need(s, path, "x_mm"), path + ".x_mm");
            if (xr.y <= xr.x) fail(path + ".x_mm", "expected [from, to] with to > from");
            const double r = positive(need(s, path, "radius_mm"), path + ".radius_mm");
            return RectShape{Vec2{0.5 * (xr.x + xr.y), 0.5 * r} * kMm, Vec2{0.5 * (xr.y - xr.x), 0.5 * r} * kMm, 0.0};
        }
        if (type == "sphere") {
            if (!axi) fail(path + ".type", "\"sphere\" needs axisymmetric mode (use \"disc\" in plane_strain)");
            onlyKeys(s, path, {"type", "center_x_mm", "radius_mm"});
            return DiscShape{Vec2{number(need(s, path, "center_x_mm"), path + ".center_x_mm"), 0.0} * kMm,
                             positive(need(s, path, "radius_mm"), path + ".radius_mm") * kMm};
        }
        if (type == "polygon") {
            onlyKeys(s, path, {"type", "points_mm"});
            const json& pts = need(s, path, "points_mm");
            if (!pts.is_array() || pts.size() < 3) fail(path + ".points_mm", "expected at least 3 [x, y] points");
            PolygonShape poly;
            for (std::size_t k = 0; k < pts.size(); ++k)
                poly.points.push_back(pair(pts[k], path + ".points_mm[" + std::to_string(k) + "]") * kMm);
            return poly;
        }
        fail(path + ".type", "unknown shape \"" + type + "\" (rect, disc, cylinder, sphere, polygon)");
    }

    void settings(const json& s, const std::string& path, SolverSettings& out) const {
        onlyKeys(s, path, {"failure", "cfl", "visc_alpha", "visc_beta", "balsara_switch", "balsara_min",
                           "density_filter_interval", "coalescence_porosity", "spall_incubation_time_us", "erode_debris", "failed_friction",
                           "threads"});
        if (s.contains("failure")) out.failure = boolean(s["failure"], path + ".failure");
        if (s.contains("cfl")) out.cfl = positive(s["cfl"], path + ".cfl");
        if (s.contains("visc_alpha")) out.viscAlpha = number(s["visc_alpha"], path + ".visc_alpha");
        if (s.contains("visc_beta")) out.viscBeta = number(s["visc_beta"], path + ".visc_beta");
        if (s.contains("balsara_switch")) out.balsaraSwitch = boolean(s["balsara_switch"], path + ".balsara_switch");
        if (s.contains("balsara_min")) out.balsaraMin = number(s["balsara_min"], path + ".balsara_min");
        if (s.contains("density_filter_interval"))
            out.densityFilterInterval = static_cast<int>(number(s["density_filter_interval"], path + ".density_filter_interval"));
        if (s.contains("failed_friction")) out.failedFriction = number(s["failed_friction"], path + ".failed_friction");
        if (s.contains("erode_debris")) out.erodeDebris = boolean(s["erode_debris"], path + ".erode_debris");
        if (s.contains("coalescence_porosity"))
            out.coalescencePorosity = positive(s["coalescence_porosity"], path + ".coalescence_porosity");
        if (s.contains("spall_incubation_time_us"))
            out.spallIncubationTime = number(s["spall_incubation_time_us"], path + ".spall_incubation_time_us") * kUs;
        if (s.contains("threads")) out.threads = static_cast<unsigned>(number(s["threads"], path + ".threads"));
    }
};

} // namespace

ScenarioFile parseScenarioJson(const std::string& textIn, std::span<const Material> materials,
                               const std::string& sourceName) {
    const Reader rd{sourceName};
    json root;
    try {
        root = json::parse(textIn, nullptr, true, /*ignore_comments=*/true);
    } catch (const json::parse_error& e) {
        throw ScenarioFileError(sourceName + ": not valid JSON: " + e.what());
    }
    rd.onlyKeys(root, "$", {"name", "description", "mode", "spacing_mm", "end_time_us", "frames", "settings",
                            "parts", "probes", "probe_interval_us"});

    ScenarioFile out;
    Scenario& s = out.scenario;
    s.name = rd.text(rd.need(root, "$", "name"), "$.name");
    if (root.contains("description")) out.description = rd.text(root["description"], "$.description");
    const std::string mode = rd.text(rd.need(root, "$", "mode"), "$.mode");
    if (mode == "axisymmetric") s.mode = GeometryMode::Axisymmetric;
    else if (mode == "plane_strain") s.mode = GeometryMode::PlaneStrain;
    else rd.fail("$.mode", "expected \"axisymmetric\" or \"plane_strain\"");
    s.dx = rd.positive(rd.need(root, "$", "spacing_mm"), "$.spacing_mm") * kMm;
    s.endTime = rd.positive(rd.need(root, "$", "end_time_us"), "$.end_time_us") * kUs;
    if (root.contains("frames")) s.frameCount = static_cast<int>(rd.positive(root["frames"], "$.frames"));
    if (root.contains("settings")) rd.settings(root["settings"], "$.settings", s.settings);

    const json& parts = rd.need(root, "$", "parts");
    if (!parts.is_array() || parts.empty()) rd.fail("$.parts", "expected a non-empty list of parts");
    for (std::size_t k = 0; k < parts.size(); ++k) {
        const std::string path = "$.parts[" + std::to_string(k) + "]";
        const json& p = parts[k];
        rd.onlyKeys(p, path, {"name", "material", "shape", "velocity_mps", "temperature_K", "contact_group", "held"});
        Part part;
        part.name = rd.text(rd.need(p, path, "name"), path + ".name");
        const std::string mat = rd.text(rd.need(p, path, "material"), path + ".material");
        part.material = findMaterial(materials, mat);
        if (part.material < 0) {
            std::string keys;
            for (const Material& m : materials) keys += std::string(keys.empty() ? "" : ", ") + m.key;
            rd.fail(path + ".material", "unknown material \"" + mat + "\" (known: " + keys + ")");
        }
        part.shape = rd.shape(rd.need(p, path, "shape"), path + ".shape", s.mode);
        if (p.contains("velocity_mps")) part.velocity = rd.pair(p["velocity_mps"], path + ".velocity_mps");
        if (s.mode == GeometryMode::Axisymmetric && part.velocity.y != 0.0)
            rd.fail(path + ".velocity_mps", "axisymmetric bodies can only move along the axis (y velocity must be 0)");
        if (p.contains("temperature_K")) part.temperature = rd.positive(p["temperature_K"], path + ".temperature_K");
        if (p.contains("held")) part.held = rd.boolean(p["held"], path + ".held");
        if (p.contains("contact_group"))
            part.contactGroup = static_cast<int>(rd.number(p["contact_group"], path + ".contact_group"));
        s.parts.push_back(part);
    }

    if (root.contains("probes")) {
        const json& probes = root["probes"];
        if (!probes.is_array()) rd.fail("$.probes", "expected a list");
        for (std::size_t k = 0; k < probes.size(); ++k) {
            const std::string path = "$.probes[" + std::to_string(k) + "]";
            const json& p = probes[k];
            rd.onlyKeys(p, path, {"name", "at_mm", "quantities"});
            ProbeSpec spec;
            spec.name = rd.text(rd.need(p, path, "name"), path + ".name");
            spec.position = rd.pair(rd.need(p, path, "at_mm"), path + ".at_mm") * kMm;
            const json& q = rd.need(p, path, "quantities");
            if (!q.is_array() || q.empty()) rd.fail(path + ".quantities", "expected a non-empty list");
            for (std::size_t j = 0; j < q.size(); ++j) {
                const std::string name = rd.text(q[j], path + ".quantities[" + std::to_string(j) + "]");
                ProbeQuantity pq;
                if (!parseProbeQuantity(name, pq))
                    rd.fail(path + ".quantities[" + std::to_string(j) + "]",
                            "unknown quantity \"" + name + "\" (known: " + probeQuantityList() + ")");
                spec.quantities.push_back(pq);
            }
            out.probes.push_back(spec);
        }
    }
    if (root.contains("probe_interval_us"))
        out.probeInterval = rd.positive(root["probe_interval_us"], "$.probe_interval_us") * kUs;
    return out;
}

ScenarioFile loadScenarioFile(const std::filesystem::path& path, std::span<const Material> materials) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw ScenarioFileError(path.string() + ": cannot open the file");
    std::ostringstream ss;
    ss << in.rdbuf();
    return parseScenarioJson(ss.str(), materials, path.filename().string());
}

} // namespace armor
