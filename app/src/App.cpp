#include "App.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include <imgui.h>
#include <rlImGui.h>

#include <armor/Probes.h>
#include <armor/Validation.h>
#include <armor/Solver.h>

#include "Colormap.h"

using armor::Field;

namespace {

constexpr float kPanelWidth = 420.0f;

std::string formatPascal(double v) {
    char buf[64];
    const double a = std::abs(v);
    if (a >= 1e9) std::snprintf(buf, sizeof buf, "%.2f GPa", v / 1e9);
    else if (a >= 1e6) std::snprintf(buf, sizeof buf, "%.1f MPa", v / 1e6);
    else if (a >= 1e3) std::snprintf(buf, sizeof buf, "%.1f kPa", v / 1e3);
    else std::snprintf(buf, sizeof buf, "%.0f Pa", v);
    return buf;
}

std::string formatEnergy(double joules, bool perMetre) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.3g %s", joules, perMetre ? "J/m" : "J");
    return buf;
}

} // namespace

App::App(const StartOptions& options) : materials_(armor::defaultMaterials()) {
    presetIndex_ = std::clamp(options.scenario, 0, static_cast<int>(presets().size()) - 1);
    params_ = presets()[static_cast<std::size_t>(presetIndex_)].defaults;
    field_ = static_cast<Field>(std::clamp(options.field, 0, armor::kFieldCount - 1));
    runAtStart_ = options.runAtStart;
    sourceMode_ = options.builder ? 1 : 0;
    selectDetails_ = options.detailsTab;
    // Builder defaults: the first projectile at its suggested speed, onto the
    // first armour item.
    // A friendly first example if the files are there: a 10 mm steel ball
    // into a 10 mm steel plate; else whatever comes first.
    const armor::Library& lib = appLibrary();
    auto findItem = [](const std::vector<armor::LibraryItem>& items, const char* file) {
        for (std::size_t k = 0; k < items.size(); ++k)
            if (items[k].file == file) return static_cast<int>(k);
        return 0;
    };
    if (!lib.projectiles.empty()) {
        projItem_ = findItem(lib.projectiles, "steel_ball_10mm.json");
        const armor::LibraryItem& p = lib.projectiles[static_cast<std::size_t>(projItem_)];
        projMaterial_ = p.material;
        projSpeed_ = static_cast<float>(p.speed > 0 ? p.speed : 500.0);
    }
    if (!lib.armor.empty()) {
        layers_.push_back({});
        resetBuilderLayer(0, findItem(lib.armor, "rha_10mm.json"));
    }
}

// -----------------------------------------------------------------------------
// Builder (Simulate > Build your own)
// -----------------------------------------------------------------------------
void App::resetBuilderLayer(std::size_t k, int item) {
    const armor::Library& lib = appLibrary();
    if (lib.armor.empty() || k >= layers_.size()) return;
    item = std::clamp(item, 0, static_cast<int>(lib.armor.size()) - 1);
    layers_[k].item = item;
    layers_[k].material = lib.armor[static_cast<std::size_t>(item)].material;
    layers_[k].thicknessMm = static_cast<float>(lib.armor[static_cast<std::size_t>(item)].thickness * 1e3);
}

armor::BuildSpec App::builderSpec() const {
    const armor::Library& lib = appLibrary();
    armor::BuildSpec spec;
    if (lib.projectiles.empty() || lib.armor.empty()) return spec;
    spec.projectile = lib.projectiles[static_cast<std::size_t>(std::clamp(projItem_, 0, static_cast<int>(lib.projectiles.size()) - 1))];
    spec.projectileMaterial = projMaterial_;
    spec.speed = projSpeed_;
    for (const BuilderLayer& l : layers_) {
        armor::ArmorLayer layer;
        layer.item = lib.armor[static_cast<std::size_t>(std::clamp(l.item, 0, static_cast<int>(lib.armor.size()) - 1))];
        layer.material = l.material;
        layer.thickness = l.thicknessMm * 1e-3;
        layer.gapBefore = l.gapMm * 1e-3;
        spec.layers.push_back(layer);
    }
    spec.obliquity = obliquityDeg_ * 3.14159265358979323846 / 180.0;
    if (autoResolution_) {
        spec.spacing = armor::suggestSpacing(spec);
        spec.endTime = armor::suggestEndTime(spec);
    } else {
        spec.spacing = spacingMm_ * 1e-3;
        spec.endTime = endTimeUs_ * 1e-6;
    }
    return spec;
}

// Back-of-the-envelope numbers for Details > Theory.
std::string App::builderSummary() const {
    const armor::BuildSpec spec = builderSpec();
    if (spec.layers.empty()) return "Add an armour layer.";
    const armor::Material& pm = materials_[static_cast<std::size_t>(spec.projectileMaterial)];
    const double r = 0.5 * spec.projectile.diameter;
    const bool rod = spec.projectile.shape == armor::ItemShape::Rod;
    const double volume = rod ? 3.14159265 * r * r * spec.projectile.length : 4.0 / 3.0 * 3.14159265 * r * r * r;
    const double mass = pm.density0 * volume;
    char buf[512];
    std::string t;
    std::snprintf(buf, sizeof buf, "Projectile: %.3g g of %s, %.0f m/s -> kinetic energy %.3g J.\n", mass * 1e3,
                  pm.name.c_str(), spec.speed, 0.5 * mass * spec.speed * spec.speed);
    t += buf;
    double areal = 0.0, total = 0.0;
    for (const armor::ArmorLayer& l : spec.layers) {
        areal += materials_[static_cast<std::size_t>(l.material)].density0 * l.thickness;
        total += l.thickness;
    }
    std::snprintf(buf, sizeof buf, "Armour: %zu layer(s), %.1f mm of material, %.0f kg per square metre.\n",
                  spec.layers.size(), total * 1e3, areal);
    t += buf;
    if (rod && pm.key == "tungsten_wha" && materials_[static_cast<std::size_t>(spec.layers[0].material)].key == "steel_rha") {
        // Measured: Lanz-Odermatt fit to 148 tungsten-into-RHA tests (see Validation.h).
        const double pl = armor::lanzOdermattTungstenRHA(spec.projectile.length / spec.projectile.diameter, pm.density0,
                                                         materials_[static_cast<std::size_t>(spec.layers[0].material)].density0,
                                                         armor::kSteelRHABrinell, spec.speed);
        std::snprintf(buf, sizeof buf,
                      "Experiments (Lanz-Odermatt fit, 148 tests, +-4%%): this rod should get about %.0f mm deep (P/L = %.2f) "
                      "into thick steel armour. Compare with the rod's furthest point in Results.\n",
                      pl * spec.projectile.length * 1e3, pl);
        t += buf;
    } else if (rod) {
        const double rt = materials_[static_cast<std::size_t>(spec.layers[0].material)].density0;
        std::snprintf(buf, sizeof buf,
                      "Long-rod rule of thumb (hydrodynamic limit): a rod can go about L*sqrt(rho_p/rho_t) = %.0f mm deep "
                      "into the first layer's material; real rods get less at ordinary speeds.\n",
                      spec.projectile.length * std::sqrt(pm.density0 / rt) * 1e3);
        t += buf;
    }
    std::snprintf(buf, sizeof buf, "Resolution: %.2f mm particle spacing, %.0f us simulated%s.",
                  spec.spacing * 1e3, spec.endTime * 1e6, autoResolution_ ? " (automatic)" : "");
    t += buf;
    if (spec.obliquity != 0.0)
        t += "\nAngled shot: simulated as a 2D slice (plane strain) - the projectile is really an infinitely "
             "wide slab, so treat depths as qualitative.";
    return t;
}

// -----------------------------------------------------------------------------
// Simulation thread
// -----------------------------------------------------------------------------
void App::startSimulation() {
    cancelSimulation();
    armor::Scenario scenario;
    std::vector<armor::ProbeSpec> probes;
    double probeInterval = 0.0;
    if (sourceMode_ == 1) {
        try {
            const armor::ScenarioFile f = armor::buildScenario(builderSpec(), materials_);
            scenario = f.scenario;
            probes = f.probes;
            probeInterval = f.probeInterval;
        } catch (const armor::ScenarioFileError& e) {
            builderStatus_ = e.what();
            return;
        }
        runName_ = "custom";
        runTheory_ = builderSummary();
    } else {
        const Preset& preset = presets()[static_cast<std::size_t>(presetIndex_)];
        scenario = preset.build(params_);
        probes = preset.probes;
        probeInterval = preset.probeInterval;
        runName_ = preset.name;
        runTheory_ = preset.theory(params_, materials_);
    }
    if (advancedEdited_) {
        armor::SolverSettings& s = scenario.settings;
        s.failure = advanced_.failure;
        s.cfl = advanced_.cfl;
        s.viscAlpha = advanced_.viscAlpha;
        s.viscBeta = advanced_.viscBeta;
        s.balsaraSwitch = advanced_.balsaraSwitch;
        s.balsaraMin = advanced_.balsaraMin;
        s.threads = advanced_.threads;
    }

    std::vector<std::string> names;
    for (const auto& m : materials_) names.push_back(m.name);

    auto run = std::make_unique<SimulationRun>();
    run->recording = std::make_shared<armor::Recording>(scenario.mode, scenario.dx, names,
                                                        armor::bodyNames(scenario), static_cast<std::size_t>(scenario.frameCount) + 2);
    run->endTime = scenario.endTime;
    ranges_.clear();
    cursor_ = 0.0f;
    playing_ = false;
    followLive_ = true;
    viewFitted_ = false;
    hovered_ = -1;

    probeStatus_.clear();
    SimulationRun* r = run.get();
    run->thread = std::thread([r, scenario, mats = materials_, probes, probeInterval]() mutable {
        const auto start = std::chrono::steady_clock::now();
        auto elapsed = [&] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };
        try {
            armor::Solver solver(std::move(scenario), std::move(mats));
            r->particles = solver.particleCount();
            const double endTime = solver.scenario().endTime;
            // Probes: pick their particles now, publish each new sample to the UI.
            armor::ProbeRecorder probeRec(probes, solver, probeInterval > 0 ? probeInterval : endTime / 1000.0);
            std::size_t published = 0;
            auto publishProbes = [&] {
                if (probes.empty() || probeRec.times().size() == published) return;
                std::lock_guard<std::mutex> lock(r->probeMutex);
                if (r->probeColumns.empty()) {
                    r->probeColumns = probeRec.columnNames();
                    r->probeSeries.assign(r->probeColumns.size(), {});
                }
                for (; published < probeRec.times().size(); ++published) {
                    r->probeTimes.push_back(probeRec.times()[published]);
                    for (std::size_t c = 0; c < r->probeSeries.size(); ++c)
                        r->probeSeries[c].push_back(probeRec.values()[published][c]);
                }
            };
            publishProbes();
            const double interval = endTime / solver.scenario().frameCount;
            r->recording->push(solver.captureFrame());
            double nextFrame = interval;
            while (!solver.finished() && !r->cancel) {
                solver.step();
                probeRec.update(solver);
                publishProbes();
                r->steps = solver.stepCount();
                r->simTime = solver.time();
                r->dt = solver.dt();
                r->progress = solver.time() / endTime;
                r->wallSeconds = elapsed();
                if (solver.time() >= nextFrame * (1.0 - 1e-9) || solver.finished()) {
                    r->recording->push(solver.captureFrame());
                    nextFrame += interval;
                }
            }
            if (!probes.empty()) {
                std::ostringstream csv;
                probeRec.writeCsv(csv);
                std::lock_guard<std::mutex> lock(r->probeMutex);
                r->probeCsv = csv.str();
            }
        } catch (const std::exception& e) {
            r->error = e.what();
            r->failed = true;
        }
        r->wallSeconds = elapsed();
        r->done = true;
    });
    run_ = std::move(run);
}

void App::cancelSimulation() {
    if (run_ && !run_->done) {
        run_->cancel = true;
    }
    // Keep the finished/cancelled recording for replay; it is replaced on the next Simulate.
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
Rectangle App::viewArea() const {
    return {kPanelWidth, 0.0f, static_cast<float>(GetScreenWidth()) - kPanelWidth, static_cast<float>(GetScreenHeight())};
}

const armor::Recording* App::recording() const { return run_ ? run_->recording.get() : nullptr; }

int App::currentFrameIndex() const {
    const armor::Recording* rec = recording();
    if (!rec || rec->size() == 0) return -1;
    return std::clamp(static_cast<int>(cursor_), 0, static_cast<int>(rec->size()) - 1);
}

const armor::Frame* App::currentFrame() const {
    const int i = currentFrameIndex();
    return i < 0 ? nullptr : &recording()->frame(static_cast<std::size_t>(i));
}

void App::updateRanges() {
    const armor::Recording* rec = recording();
    if (!rec) return;
    while (ranges_.size() < rec->size()) {
        const armor::Frame& f = rec->frame(ranges_.size());
        std::array<std::pair<float, float>, armor::kFieldCount> r{};
        for (int k = 0; k < armor::kFieldCount; ++k) {
            const auto& v = f.values[static_cast<std::size_t>(k)];
            if (v.empty()) continue;
            const auto [mn, mx] = std::minmax_element(v.begin(), v.end());
            r[static_cast<std::size_t>(k)] = {*mn, *mx};
        }
        ranges_.push_back(r);
    }
}

// The colour scale's [min, max]. Signed fields use ±max|value| so that zero
// is always the white middle of the diverging scale.
std::pair<float, float> App::displayRange() const {
    const int fi = currentFrameIndex();
    if (fi < 0 || ranges_.empty()) return {0.0f, 1.0f};
    const std::size_t k = static_cast<std::size_t>(field_);
    float mn, mx;
    if (rangeMode_ == 0) {
        const std::size_t idx = std::min(static_cast<std::size_t>(fi), ranges_.size() - 1);
        mn = ranges_[idx][k].first;
        mx = ranges_[idx][k].second;
    } else {
        mn = ranges_[0][k].first;
        mx = ranges_[0][k].second;
        for (const auto& r : ranges_) {
            mn = std::min(mn, r[k].first);
            mx = std::max(mx, r[k].second);
        }
    }
    if (armor::fieldInfo(field_).isSigned) {
        const float m = std::max(std::max(std::abs(mn), std::abs(mx)), 1e-20f);
        return {-m, m};
    }
    if (mx - mn < 1e-20f) mx = mn + 1e-20f;
    return {mn, mx};
}

Color App::colorFor(float value, std::pair<float, float> range) const {
    if (field_ == Field::Material) return colormap::category(static_cast<int>(value));
    if (armor::fieldInfo(field_).isSigned) return colormap::diverging(value / range.second);
    // Turbo starts at near-black, which vanishes on the dark background: a
    // field sitting at its minimum (e.g. zero plastic strain in an elastic
    // body) made whole bodies invisible. Start the scale at a visible violet.
    const double t = (value - range.first) / (range.second - range.first);
    return colormap::turbo(0.08 + 0.92 * std::clamp(t, 0.0, 1.0));
}

std::string App::formatValue(Field field, double value) const {
    char buf[96];
    switch (armor::fieldInfo(field).kind) {
    case armor::FieldKind::Stress: return formatPascal(value);
    case armor::FieldKind::Speed: std::snprintf(buf, sizeof buf, "%.2f m/s", value); return buf;
    case armor::FieldKind::Density: std::snprintf(buf, sizeof buf, "%.1f kg/m^3", value); return buf;
    case armor::FieldKind::Strain: std::snprintf(buf, sizeof buf, "%.3f", value); return buf; // dimensionless
    case armor::FieldKind::Temperature: std::snprintf(buf, sizeof buf, "%.0f K", value); return buf;
    case armor::FieldKind::Category: {
        const auto* rec = recording();
        const std::size_t idx = static_cast<std::size_t>(value);
        return rec && idx < rec->materialNames().size() ? rec->materialNames()[idx] : "?";
    }
    }
    return "";
}

void App::fitView() {
    const armor::Frame* f = currentFrame();
    if (!f || f->x.empty()) return;
    const auto [x0, x1] = std::minmax_element(f->x.begin(), f->x.end());
    const auto [y0, y1] = std::minmax_element(f->y.begin(), f->y.end());
    double minY = *y0;
    if (recording()->mode() == armor::GeometryMode::Axisymmetric && mirror_) minY = -*y1;
    view_.fit(*x0, minY, *x1, *y1, viewArea());
    viewFitted_ = true;
}

// -----------------------------------------------------------------------------
// Per-frame update
// -----------------------------------------------------------------------------
void App::frame() {
    if (runAtStart_) {
        runAtStart_ = false;
        startSimulation();
    }
    handleInput();
    updateRanges();
    updatePlayback();
    if (!viewFitted_ && currentFrame()) fitView();

    drawParticles();
    drawOverlay();
    drawLegend();

    rlImGuiBegin();
    drawPanel();
    drawProbesPanel(); // floating window over the view, bottom right
    drawProbe();
    rlImGuiEnd();
}

void App::handleInput() {
    const ImGuiIO& io = ImGui::GetIO();
    const Vector2 mouse = GetMousePosition();
    hovered_ = -1;

    if (!io.WantCaptureKeyboard) {
        if (IsKeyPressed(KEY_SPACE)) {
            playing_ = !playing_;
            followLive_ = false;
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) { cursor_ += 1.0f; playing_ = false; followLive_ = false; }
        if (IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT)) { cursor_ = std::max(0.0f, cursor_ - 1.0f); playing_ = false; followLive_ = false; }
        if (IsKeyPressed(KEY_F)) fitView();
    }
    if (io.WantCaptureMouse || !CheckCollisionPointRec(mouse, viewArea())) return;

    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) view_.zoomAt(mouse, std::pow(1.15, wheel));
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        const Vector2 d = GetMouseDelta();
        view_.origin.x += d.x;
        view_.origin.y += d.y;
    }

    // Probe: nearest particle within a few pixels of the cursor.
    const armor::Frame* f = currentFrame();
    if (!f) return;
    const bool axi = recording()->mode() == armor::GeometryMode::Axisymmetric;
    const double mx = view_.toWorldX(mouse.x);
    double my = view_.toWorldY(mouse.y);
    if (axi && my < 0.0) my = -my; // mirrored half
    const double pickRadius = std::max(6.0 / view_.pixelsPerMetre, recording()->dx());
    double best = pickRadius * pickRadius;
    for (std::size_t i = 0; i < f->x.size(); ++i) {
        const double ddx = f->x[i] - mx, ddy = f->y[i] - my;
        const double d2 = ddx * ddx + ddy * ddy;
        if (d2 < best) {
            best = d2;
            hovered_ = static_cast<int>(i);
        }
    }
}

void App::updatePlayback() {
    const armor::Recording* rec = recording();
    if (!rec || rec->size() == 0) return;
    const float last = static_cast<float>(rec->size() - 1);
    if (followLive_ && !run_->done) {
        cursor_ = last;
        return;
    }
    if (playing_) {
        cursor_ += playFps_ * GetFrameTime();
        if (cursor_ > last) {
            if (loop_ && run_->done) cursor_ = 0.0f;
            else { cursor_ = last; playing_ = false; }
        }
    }
    cursor_ = std::clamp(cursor_, 0.0f, last);
}

// -----------------------------------------------------------------------------
// Drawing (raylib)
// -----------------------------------------------------------------------------
void App::drawParticles() {
    const armor::Frame* f = currentFrame();
    if (!f) return;
    const armor::Recording& rec = *recording();
    const bool mirror = mirror_ && rec.mode() == armor::GeometryMode::Axisymmetric;
    const auto range = displayRange();
    const auto& values = f->values[static_cast<std::size_t>(field_)];
    const float size = std::max(1.5f, static_cast<float>(rec.dx() * view_.pixelsPerMetre) * 1.02f);
    const Rectangle area = viewArea();

    BeginScissorMode(static_cast<int>(area.x), 0, static_cast<int>(area.width), static_cast<int>(area.height));
    auto drawAt = [&](Vector2 p, Color c) {
        if (p.x < area.x - size || p.x > area.x + area.width + size || p.y < -size || p.y > area.height + size) return;
        DrawRectangleV({p.x - size * 0.5f, p.y - size * 0.5f}, {size, size}, c);
    };
    for (std::size_t i = 0; i < f->x.size(); ++i) {
        const Color c = colorFor(values[i], range);
        drawAt(view_.toScreen(f->x[i], f->y[i]), c);
        if (mirror) {
            // The lower half is the mirror image: the same rings seen from the other side of the axis.
            const Color dim{static_cast<unsigned char>(c.r * 0.85f), static_cast<unsigned char>(c.g * 0.85f),
                            static_cast<unsigned char>(c.b * 0.85f), 255};
            drawAt(view_.toScreen(f->x[i], -f->y[i]), dim);
        }
    }
    if (hovered_ >= 0 && static_cast<std::size_t>(hovered_) < f->x.size()) {
        const Vector2 p = view_.toScreen(f->x[static_cast<std::size_t>(hovered_)], f->y[static_cast<std::size_t>(hovered_)]);
        DrawRectangleLinesEx({p.x - size, p.y - size, 2 * size, 2 * size}, 2.0f, YELLOW);
    }
    EndScissorMode();
}

void App::drawOverlay() {
    const Rectangle area = viewArea();
    const armor::Recording* rec = recording();
    if (!rec || !currentFrame()) {
        DrawText("Choose a scenario on the left and press Simulate.", static_cast<int>(area.x) + 40,
                 static_cast<int>(area.height / 2), 24, LIGHTGRAY);
        return;
    }

    // Symmetry axis (axisymmetric mode).
    if (rec->mode() == armor::GeometryMode::Axisymmetric) {
        const float y = view_.toScreen(0, 0).y;
        for (float x = area.x; x < area.x + area.width; x += 16.0f) {
            DrawLineEx({x, y}, {x + 8.0f, y}, 1.0f, Fade(WHITE, 0.5f));
        }
        DrawText("symmetry axis", static_cast<int>(area.x) + 10, static_cast<int>(y) - 18, 14, Fade(WHITE, 0.6f));
    }

    // Scale bar: pick 1, 2 or 5 x 10^k metres closest to 150 px.
    const double target = 150.0 / view_.pixelsPerMetre;
    const double p10 = std::pow(10.0, std::floor(std::log10(target)));
    double len = p10;
    for (double m : {1.0, 2.0, 5.0, 10.0}) {
        if (m * p10 <= target) len = m * p10;
    }
    const float px = static_cast<float>(len * view_.pixelsPerMetre);
    const float bx = area.x + 30.0f, by = area.height - 40.0f;
    DrawLineEx({bx, by}, {bx + px, by}, 3.0f, WHITE);
    DrawLineEx({bx, by - 6}, {bx, by + 6}, 2.0f, WHITE);
    DrawLineEx({bx + px, by - 6}, {bx + px, by + 6}, 2.0f, WHITE);
    char buf[128];
    std::snprintf(buf, sizeof buf, "%g mm", len * 1e3);
    DrawText(buf, static_cast<int>(bx), static_cast<int>(by) + 10, 18, WHITE);

    // Time.
    const armor::Frame& f = *currentFrame();
    std::snprintf(buf, sizeof buf, "t = %.2f us     frame %d / %zu", f.time * 1e6, currentFrameIndex() + 1, rec->size());
    DrawText(buf, static_cast<int>(area.x) + 20, 16, 22, WHITE);
    DrawText(armor::fieldInfo(field_).name, static_cast<int>(area.x) + 20, 44, 18, LIGHTGRAY);
}

void App::drawLegend() {
    const armor::Recording* rec = recording();
    if (!rec || !currentFrame()) return;
    const Rectangle area = viewArea();
    const int x = static_cast<int>(area.x + area.width) - 190;
    int y = 80;
    DrawRectangle(x - 10, y - 30, 190, 380, Fade(BLACK, 0.55f));
    DrawText(armor::fieldInfo(field_).name, x, y - 22, 14, WHITE);

    if (field_ == Field::Material) {
        for (std::size_t i = 0; i < rec->materialNames().size(); ++i) {
            DrawRectangle(x, y, 16, 16, colormap::category(static_cast<int>(i)));
            DrawText(rec->materialNames()[i].c_str(), x + 22, y + 2, 10, WHITE);
            y += 24;
        }
        return;
    }
    // Vertical bar: highest at the top, lowest at the bottom.
    const auto range = displayRange();
    const int barH = 300, steps = 100;
    for (int k = 0; k < steps; ++k) {
        const float t = 1.0f - static_cast<float>(k) / (steps - 1); // top = 1
        const float value = range.first + t * (range.second - range.first);
        DrawRectangle(x, y + k * barH / steps, 24, barH / steps + 1, colorFor(value, range));
    }
    auto label = [&](float t, const char* prefix) {
        const float value = range.first + t * (range.second - range.first);
        const std::string s = std::string(prefix) + formatValue(field_, value);
        DrawText(s.c_str(), x + 32, y + static_cast<int>((1.0f - t) * barH) - 6, 14, WHITE);
    };
    label(1.0f, "max ");
    label(0.5f, "");
    label(0.0f, "min ");
    if (armor::fieldInfo(field_).isSigned) {
        DrawText("red = tension (+)", x, y + barH + 8, 12, Color{230, 120, 120, 255});
        DrawText("blue = compression (-)", x, y + barH + 24, 12, Color{140, 160, 240, 255});
        if (field_ == Field::Pressure) {
            DrawText("(pressure: + is compression)", x, y + barH + 40, 10, LIGHTGRAY);
        }
    }
}

// -----------------------------------------------------------------------------
// Panels (Dear ImGui)
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Side panel: two tabs.
//  - "Simulate": the beginner path, top to bottom - what to simulate, run it,
//    watch it, choose what the colours show.
//  - "Details": everything advanced, sorted and folded away.
// -----------------------------------------------------------------------------
namespace {
// A small "(?)" that explains something on hover.
void helpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool materialCombo(const char* label, int& index, const std::vector<armor::Material>& mats) {
    bool changed = false;
    const int n = static_cast<int>(mats.size());
    index = std::clamp(index, 0, n - 1);
    if (ImGui::BeginCombo(label, mats[static_cast<std::size_t>(index)].name.c_str())) {
        for (int k = 0; k < n; ++k) {
            if (ImGui::Selectable(mats[static_cast<std::size_t>(k)].name.c_str(), k == index)) {
                index = k;
                changed = true;
            }
        }
        ImGui::Separator();
        ImGui::TextDisabled("Ceramics (alumina, boron carbide...) arrive with");
        ImGui::TextDisabled("Stage F: they need a brittle-fracture model (JH-2).");
        ImGui::EndCombo();
    }
    return changed;
}
} // namespace

void App::drawPanel() {
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({kPanelWidth, static_cast<float>(GetScreenHeight())});
    ImGui::Begin("MiataPhySim", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextColored({0.55f, 0.75f, 1.0f, 1.0f}, "Impact & armour physics");
    ImGui::SameLine();
    ImGui::TextDisabled("- work in progress");
    if (ImGui::BeginTabBar("tabs")) {
        if (ImGui::BeginTabItem("  Simulate  ")) {
            drawSimulateTab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("  Details  ", nullptr, selectDetails_ ? ImGuiTabItemFlags_SetSelected : 0)) {
            drawDetailsTab();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    selectDetails_ = false;
    ImGui::End();
}

void App::drawSimulateTab() {
    const bool running = run_ && !run_->done;

    // ---------------- 1. What to simulate ----------------
    ImGui::SeparatorText("1   What to simulate");
    ImGui::RadioButton("Ready-made", &sourceMode_, 0);
    helpMarker("Classic experiments and scenario files, ready to run.");
    ImGui::SameLine();
    ImGui::RadioButton("Build your own", &sourceMode_, 1);
    helpMarker("Pick a projectile and armour from the library, choose their materials, stack layers, set the speed.");
    ImGui::Spacing();
    if (sourceMode_ == 0) {
        const auto& list = presets();
        const Preset& current = list[static_cast<std::size_t>(presetIndex_)];
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##preset", current.name.c_str())) {
            for (std::size_t i = 0; i < list.size(); ++i) {
                if (ImGui::Selectable(list[i].name.c_str(), static_cast<int>(i) == presetIndex_)) {
                    presetIndex_ = static_cast<int>(i);
                    params_ = list[i].defaults;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("%s", current.description.c_str());
        ImGui::PopStyleColor();
        ImGui::SliderFloat("Speed (m/s)", &params_.velocity, 0.5f, current.maxVelocity, "%.0f");
    } else {
        drawBuilder();
    }

    // ---------------- 2. Run ----------------
    ImGui::SeparatorText("2   Run");
    if (!running) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.16f, 0.45f, 0.30f, 1.0f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.20f, 0.56f, 0.38f, 1.0f});
        if (ImGui::Button("Simulate", {-1.0f, 40.0f})) startSimulation();
        ImGui::PopStyleColor(2);
    } else {
        if (ImGui::Button("Cancel", {-1.0f, 40.0f})) cancelSimulation();
    }
    if (run_) {
        const double prog = run_->progress.load();
        char overlay[96];
        if (run_->failed) std::snprintf(overlay, sizeof overlay, "failed");
        else if (run_->done) std::snprintf(overlay, sizeof overlay, "%s in %.1f s", run_->cancel ? "Cancelled" : "Finished", run_->wallSeconds.load());
        else if (prog > 0.02) std::snprintf(overlay, sizeof overlay, "%.0f%%  -  about %.0f s left", prog * 100.0, run_->wallSeconds.load() * (1.0 - prog) / prog);
        else std::snprintf(overlay, sizeof overlay, "starting...");
        ImGui::ProgressBar(static_cast<float>(prog), {-1.0f, 0.0f}, overlay);
        if (run_->failed) ImGui::TextColored({1, 0.45f, 0.45f, 1}, "Error: %s", run_->error.c_str());
    } else {
        ImGui::TextDisabled("Press Simulate. The result plays back as it is computed.");
    }
    if (!builderStatus_.empty() && sourceMode_ == 1) ImGui::TextColored({1, 0.75f, 0.4f, 1}, "%s", builderStatus_.c_str());

    // ---------------- 3. Watch ----------------
    const armor::Recording* rec = recording();
    ImGui::SeparatorText("3   Watch");
    if (rec && rec->size() > 0) {
        if (ImGui::Button(playing_ ? "Pause" : "Play", {70, 0})) {
            playing_ = !playing_;
            followLive_ = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("|<", {32, 0})) { cursor_ = 0; followLive_ = false; }
        ImGui::SameLine();
        if (ImGui::Button(">|", {32, 0})) { cursor_ = static_cast<float>(rec->size() - 1); followLive_ = false; }
        ImGui::SameLine();
        ImGui::Checkbox("Loop", &loop_);
        int frameIndex = currentFrameIndex();
        char label[48];
        std::snprintf(label, sizeof label, "t = %.2f us", currentFrame() ? currentFrame()->time * 1e6 : 0.0);
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderInt("##timeline", &frameIndex, 0, static_cast<int>(rec->size()) - 1, label)) {
            cursor_ = static_cast<float>(frameIndex);
            playing_ = false;
            followLive_ = false;
        }
    } else {
        ImGui::TextDisabled("Nothing to watch yet.");
    }

    // ---------------- 4. Show ----------------
    ImGui::SeparatorText("4   Colours show");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo("##field", armor::fieldInfo(field_).name)) {
        for (int k = 0; k < armor::kFieldCount; ++k) {
            const Field f = static_cast<Field>(k);
            if (ImGui::Selectable(armor::fieldInfo(f).name, f == field_)) field_ = f;
        }
        ImGui::EndCombo();
    }
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", armor::fieldInfo(field_).help);
    ImGui::PopStyleColor();
    ImGui::Spacing();
    ImGui::TextDisabled("Hover a particle to read its values.");
    ImGui::TextDisabled("Wheel: zoom   Right-drag: pan   F: fit   Space: play");
}

void App::drawBuilder() {
    const armor::Library& lib = appLibrary();
    if (lib.projectiles.empty() || lib.armor.empty()) {
        ImGui::TextWrapped("No library found. Put projectile and armour files in library/projectiles and library/armor.");
        return;
    }
    // Projectile
    ImGui::TextUnformatted("Projectile");
    projItem_ = std::clamp(projItem_, 0, static_cast<int>(lib.projectiles.size()) - 1);
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo("##proj", lib.projectiles[static_cast<std::size_t>(projItem_)].name.c_str())) {
        for (int k = 0; k < static_cast<int>(lib.projectiles.size()); ++k) {
            const armor::LibraryItem& it = lib.projectiles[static_cast<std::size_t>(k)];
            if (ImGui::Selectable(it.name.c_str(), k == projItem_)) {
                projItem_ = k;
                projMaterial_ = it.material;
                if (it.speed > 0) projSpeed_ = static_cast<float>(it.speed);
            }
            if (ImGui::IsItemHovered() && !it.description.empty()) ImGui::SetItemTooltip("%s", it.description.c_str());
        }
        ImGui::EndCombo();
    }
    materialCombo("Material##proj", projMaterial_, materials_);
    ImGui::SliderFloat("Speed (m/s)##proj", &projSpeed_, 10.0f, 3000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    helpMarker("Rifle bullets: 800-950 m/s. Tank long rods: 1500-1700 m/s. Space debris: 2000-7000 m/s (needs fine spacing).");

    // Armour layers
    ImGui::Spacing();
    ImGui::TextUnformatted("Armour, front to back");
    int removeAt = -1;
    for (std::size_t k = 0; k < layers_.size(); ++k) {
        BuilderLayer& l = layers_[k];
        ImGui::PushID(static_cast<int>(k));
        ImGui::BeginChild("layer", {0.0f, 0.0f}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::Text("Layer %zu", k + 1);
        if (layers_.size() > 1) {
            ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50.0f);
            if (ImGui::SmallButton("remove")) removeAt = static_cast<int>(k);
        }
        l.item = std::clamp(l.item, 0, static_cast<int>(lib.armor.size()) - 1);
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##item", lib.armor[static_cast<std::size_t>(l.item)].name.c_str())) {
            for (int a = 0; a < static_cast<int>(lib.armor.size()); ++a) {
                const armor::LibraryItem& it = lib.armor[static_cast<std::size_t>(a)];
                if (ImGui::Selectable(it.name.c_str(), a == l.item)) resetBuilderLayer(k, a);
                if (ImGui::IsItemHovered() && !it.description.empty()) ImGui::SetItemTooltip("%s", it.description.c_str());
            }
            ImGui::EndCombo();
        }
        materialCombo("Material", l.material, materials_);
        ImGui::SliderFloat("Thickness (mm)", &l.thicknessMm, 0.5f, 150.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
        if (k > 0) {
            ImGui::SliderFloat("Gap before (mm)", &l.gapMm, 0.0f, 100.0f, "%.1f");
            helpMarker("Empty space in front of this layer: spaced armour.");
        }
        ImGui::EndChild();
        ImGui::PopID();
    }
    if (removeAt >= 0) layers_.erase(layers_.begin() + removeAt);
    if (layers_.size() < 6 && ImGui::Button("+ Add layer", {-1.0f, 0.0f})) {
        layers_.push_back({});
        resetBuilderLayer(layers_.size() - 1, layers_.size() > 1 ? layers_[layers_.size() - 2].item : 0);
        layers_.back().gapMm = 10.0f;
    }

    // Angle
    ImGui::Spacing();
    ImGui::SliderFloat("Angle (deg)", &obliquityDeg_, 0.0f, 75.0f, obliquityDeg_ == 0.0f ? "straight on" : "%.0f");
    helpMarker("0 = straight on: simulated exactly as real round bodies (axisymmetric). Any angle uses a 2D slice "
               "instead (plane strain): good for seeing ricochet and deflection, but depths are only qualitative.");

    if (ImGui::Button("Save as scenario file")) {
        try {
            const armor::ScenarioFile f = armor::buildScenario(builderSpec(), materials_);
            std::filesystem::path dir = scenarioDirectory();
            if (dir.empty()) dir = "scenarios";
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            int n = 1;
            std::filesystem::path path;
            do { path = dir / ("custom_" + std::to_string(n++) + ".json"); } while (std::filesystem::exists(path));
            std::ofstream out(path);
            out << armor::scenarioToJson(f, materials_) << "\n";
            builderStatus_ = out ? "Saved " + path.filename().string() + " - it appears under Ready-made next start."
                                 : "Could not write " + path.string();
        } catch (const armor::ScenarioFileError& e) {
            builderStatus_ = e.what();
        }
    }
    helpMarker("Writes this setup to the scenarios folder as a JSON file you can edit, share or run headless.");
}

void App::drawDetailsTab() {
    const armor::Recording* rec = recording();
    const armor::Frame* f = currentFrame();

    if (ImGui::CollapsingHeader("Results", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (run_) {
            ImGui::Text("Particles  %zu     Steps  %llu", run_->particles.load(), run_->steps.load());
            ImGui::Text("Time step  %.1f ns   Simulated  %.2f / %.2f us", run_->dt.load() * 1e9,
                        run_->simTime.load() * 1e6, run_->endTime.load() * 1e6);
            ImGui::Text("Computer time  %.1f s %s", run_->wallSeconds.load(),
                        run_->done ? (run_->cancel ? "(cancelled)" : "(finished)") : "(running)");
        }
        if (f && rec) {
            const bool perMetre = rec->mode() == armor::GeometryMode::PlaneStrain;
            ImGui::Separator();
            ImGui::Text("Kinetic energy   %s", formatEnergy(f->kineticEnergy, perMetre).c_str());
            ImGui::Text("Internal energy  %s", formatEnergy(f->internalEnergy, perMetre).c_str());
            ImGui::Text("Total            %s", formatEnergy(f->kineticEnergy + f->internalEnergy, perMetre).c_str());
            helpMarker("Kinetic: motion. Internal: compression, plastic work and heat. The total should stay nearly "
                       "constant - a few percent is lost to numerical damping, it should never grow.");
            if (perMetre) ImGui::TextDisabled("(2D slice: per metre of depth)");
            ImGui::Separator();
            drawBodySizes(*f, *rec);
        }
        if (!run_) ImGui::TextDisabled("Run a simulation to see results.");
    }

    if (ImGui::CollapsingHeader("Theory & checks")) {
        const std::string text = run_ ? runTheory_
                                      : (sourceMode_ == 1 ? builderSummary()
                                                          : presets()[static_cast<std::size_t>(presetIndex_)].theory(params_, materials_));
        ImGui::TextWrapped("%s", text.c_str());
    }

    if (ImGui::CollapsingHeader("Resolution & duration")) {
        if (sourceMode_ == 0) {
            ImGui::SliderFloat("Particle spacing (mm)", &params_.dxMm, 0.05f, 2.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
            helpMarker("Smaller = finer detail but slower: halving it gives 4x the particles and 2x the steps (~8x the time).");
            ImGui::SliderFloat("Simulated time (us)", &params_.endTimeUs, 1.0f, 500.0f, params_.endTimeUs < 10.0f ? "%.1f" : "%.0f");
        } else {
            // Unticking keeps the last automatic values as the starting point.
            ImGui::Checkbox("Automatic (recommended)", &autoResolution_);
            if (autoResolution_) {
                armor::BuildSpec s = builderSpec();
                spacingMm_ = static_cast<float>(s.spacing * 1e3);
                endTimeUs_ = static_cast<float>(s.endTime * 1e6);
                ImGui::TextDisabled("Spacing %.2f mm, %.0f us simulated", spacingMm_, endTimeUs_);
            } else {
                ImGui::SliderFloat("Particle spacing (mm)", &spacingMm_, 0.05f, 2.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Simulated time (us)", &endTimeUs_, 1.0f, 500.0f, "%.0f");
            }
            helpMarker("Automatic: fine enough for the smallest part (1/20 of the projectile, 1/8 of the thinnest "
                       "plate), capped near 40,000 particles; long enough for the projectile to cross the armour.");
        }
    }

    if (ImGui::CollapsingHeader("Display")) {
        ImGui::TextUnformatted("Colour range");
        ImGui::SameLine();
        ImGui::RadioButton("whole run", &rangeMode_, 1);
        ImGui::SameLine();
        ImGui::RadioButton("this frame", &rangeMode_, 0);
        ImGui::Checkbox("Mirror round bodies (show the full cross-section)", &mirror_);
        ImGui::SliderFloat("Replay speed (frames/s)", &playFps_, 1.0f, 120.0f, "%.0f");
        ImGui::Checkbox("Follow the running simulation", &followLive_);
        if (ImGui::Button("Fit view (F)")) fitView();
    }

    if (ImGui::CollapsingHeader("Solver settings (advanced)")) {
        ImGui::TextDisabled("Changes apply to the next run, also to ready-made scenarios.");
        bool ch = false;
        ch |= ImGui::Checkbox("Materials can fail (fracture, spall, melting)", &advanced_.failure);
        float cfl = static_cast<float>(advanced_.cfl), va = static_cast<float>(advanced_.viscAlpha),
              vb = static_cast<float>(advanced_.viscBeta), bm = static_cast<float>(advanced_.balsaraMin);
        if (ImGui::SliderFloat("Time-step safety (CFL)", &cfl, 0.05f, 0.5f, "%.2f")) { advanced_.cfl = cfl; ch = true; }
        helpMarker("Fraction of a particle a sound wave may cross per step. Lower = slower but safer.");
        if (ImGui::SliderFloat("Viscosity (linear)", &va, 0.0f, 2.0f, "%.2f")) { advanced_.viscAlpha = va; ch = true; }
        if (ImGui::SliderFloat("Viscosity (quadratic)", &vb, 0.0f, 4.0f, "%.2f")) { advanced_.viscBeta = vb; ch = true; }
        helpMarker("Artificial viscosity smears shocks over a few particles so they don't ring. Too low: noise; "
                   "too high: metals look too stiff.");
        ch |= ImGui::Checkbox("Balsara switch (less viscosity in shear)", &advanced_.balsaraSwitch);
        if (ImGui::SliderFloat("Balsara floor", &bm, 0.0f, 1.0f, "%.2f")) { advanced_.balsaraMin = bm; ch = true; }
        int threads = static_cast<int>(advanced_.threads);
        if (ImGui::SliderInt("CPU threads (0 = all)", &threads, 0, 64)) { advanced_.threads = static_cast<unsigned>(threads); ch = true; }
        if (ch) advancedEdited_ = true;
        if (advancedEdited_ && ImGui::Button("Reset to defaults")) {
            advanced_ = armor::SolverSettings{};
            advancedEdited_ = false;
        }
    }

    if (ImGui::CollapsingHeader("Materials")) {
        if (ImGui::BeginTable("mats", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Material");
            ImGui::TableSetupColumn("Density");
            ImGui::TableSetupColumn("Yield");
            ImGui::TableSetupColumn("Spall");
            ImGui::TableHeadersRow();
            for (const armor::Material& m : materials_) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(m.name.c_str());
                if (ImGui::IsItemHovered()) ImGui::SetItemTooltip("%s", m.source.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%.0f", m.density0);
                ImGui::TableNextColumn();
                if (const auto* jc = std::get_if<armor::JohnsonCookStrength>(&m.strength)) ImGui::Text("%.0f MPa", jc->A / 1e6);
                else ImGui::TextUnformatted("-");
                ImGui::TableNextColumn();
                if (std::isfinite(m.spallStrength)) ImGui::Text("%.1f GPa", m.spallStrength / 1e9);
                else ImGui::TextUnformatted("-");
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Density in kg/m3. Hover a name for the data sources.");
        ImGui::TextDisabled("Ceramics need a brittle model (JH-2): Stage F.");
    }

    if (ImGui::CollapsingHeader("Controls")) {
        ImGui::BulletText("Mouse wheel: zoom   Right-drag: pan   F: fit view");
        ImGui::BulletText("Space: play / pause   Left / Right: step frames");
        ImGui::BulletText("Hover a particle: all its values");
        ImGui::BulletText("Command line: miataphysim --headless --file x.json --csv out.csv");
    }

    if (ImGui::CollapsingHeader("About")) {
        ImGui::TextWrapped("MiataPhySim - a 2D impact and armour physics simulator (SPH hydrocode). "
                           "Work in progress: elastic solids, plasticity and shocks, and failure (spall, fracture, "
                           "melting) are in; the scenario builder is being built; ceramics, explosives and 3D come later.");
#ifdef MIATAPHYSIM_VERSION
        ImGui::TextDisabled("Version %s   github.com/miatson/miataphysim", MIATAPHYSIM_VERSION);
#else
        ImGui::TextDisabled("github.com/miatson/miataphysim");
#endif
    }
}

// Length (along x) and width (across) of every body in the frame, measured to
// the outer surface (particle centres sit half a spacing inside it). In
// axisymmetric mode the width is the diameter of the full round body.
void App::drawBodySizes(const armor::Frame& f, const armor::Recording& rec) const {
    const auto& bodies = rec.bodies();
    if (bodies.empty() || f.body.size() != f.x.size()) return;
    struct Box { float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f; };
    std::vector<Box> boxes(bodies.size());
    for (std::size_t i = 0; i < f.x.size(); ++i) {
        for (std::size_t b = 0; b < bodies.size(); ++b) {
            if (bodies[b].id != f.body[i]) continue;
            Box& bx = boxes[b];
            bx.x0 = std::min(bx.x0, f.x[i]);
            bx.x1 = std::max(bx.x1, f.x[i]);
            bx.y0 = std::min(bx.y0, f.y[i]);
            bx.y1 = std::max(bx.y1, f.y[i]);
            break;
        }
    }
    const bool axi = rec.mode() == armor::GeometryMode::Axisymmetric;
    const double pad = rec.dx() * 1e3; // half a spacing on each side, in mm
    ImGui::Text("Body sizes (length x %s):", axi ? "diameter" : "width");
    for (std::size_t b = 0; b < bodies.size(); ++b) {
        const Box& bx = boxes[b];
        if (bx.x1 < bx.x0) continue;
        const double len = (bx.x1 - bx.x0) * 1e3 + pad;
        const double wid = axi ? 2.0 * (bx.y1 * 1e3 + pad / 2) : (bx.y1 - bx.y0) * 1e3 + pad;
        ImGui::Text("  %-24s %6.2f x %6.2f mm", bodies[b].name.c_str(), len, wid);
        ImGui::TextDisabled("      x from %.2f to %.2f mm", bx.x0 * 1e3 - pad / 2, bx.x1 * 1e3 + pad / 2);
    }
}

// Probe time histories: one small graph per recorded quantity, with the value
// at the playback cursor, and a button to save everything as CSV.
void App::drawProbesPanel() {
    if (!run_) return;
    SimulationRun& r = *run_;
    std::lock_guard<std::mutex> lock(r.probeMutex);
    if (r.probeColumns.empty()) return;
    // Its own window in the bottom-right corner of the view, like an
    // oscilloscope next to the experiment; movable and collapsible.
    const float w = 420.0f;
    ImGui::SetNextWindowPos({static_cast<float>(GetScreenWidth()) - w - 10.0f, static_cast<float>(GetScreenHeight()) - 10.0f},
                            ImGuiCond_FirstUseEver, {0.0f, 1.0f});
    ImGui::SetNextWindowSize({w, 0.0f}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Probes (gauges)", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    // Sample nearest to the frame being shown.
    std::size_t at = r.probeTimes.empty() ? 0 : r.probeTimes.size() - 1;
    if (const armor::Frame* f = currentFrame(); f && !r.probeTimes.empty()) {
        const auto it = std::lower_bound(r.probeTimes.begin(), r.probeTimes.end(), f->time);
        at = std::min(static_cast<std::size_t>(it - r.probeTimes.begin()), r.probeTimes.size() - 1);
    }
    ImGui::TextDisabled("%zu samples to %.2f us; values at playback time %.2f us", r.probeTimes.size(),
                        r.probeTimes.empty() ? 0.0 : r.probeTimes.back() * 1e6,
                        r.probeTimes.empty() ? 0.0 : r.probeTimes[at] * 1e6);

    std::vector<float> plot;
    for (std::size_t c = 0; c < r.probeColumns.size(); ++c) {
        const std::vector<double>& s = r.probeSeries[c];
        if (s.empty()) continue;
        // Show stresses in GPa, everything else as recorded.
        const std::string& name = r.probeColumns[c];
        const bool pascal = name.find("[Pa]") != std::string::npos;
        const double scale = pascal ? 1e-9 : 1.0;
        plot.resize(s.size());
        float lo = 1e30f, hi = -1e30f;
        for (std::size_t k = 0; k < s.size(); ++k) {
            plot[k] = static_cast<float>(s[k] * scale);
            lo = std::min(lo, plot[k]);
            hi = std::max(hi, plot[k]);
        }
        if (hi - lo < 1e-12f) hi = lo + 1.0f;
        const std::string label = pascal ? name.substr(0, name.find("[Pa]")) + "[GPa]" : name;
        ImGui::Text("%s: %.4g", label.c_str(), plot[std::min(at, plot.size() - 1)]);
        char overlay[64];
        std::snprintf(overlay, sizeof overlay, "%.3g .. %.3g", lo, hi);
        ImGui::PushID(static_cast<int>(c));
        ImGui::PlotLines("##probe", plot.data(), static_cast<int>(plot.size()), 0, overlay, lo, hi, {400.0f, 60.0f});
        ImGui::PopID();
    }

    if (!r.probeCsv.empty()) {
        if (ImGui::Button("Save CSV")) {
            std::string stem;
            for (char ch : runName_) stem += (std::isalnum(static_cast<unsigned char>(ch)) ? ch : '_');
            while (!stem.empty() && stem.front() == '_') stem.erase(stem.begin());
            const std::string file = (stem.empty() ? std::string("probes") : stem.substr(0, 60)) + "_probes.csv";
            std::ofstream out(file);
            if (out) {
                out << r.probeCsv;
                probeStatus_ = "Saved " + file + " (in the folder the app was started from).";
            } else {
                probeStatus_ = "Could not write " + file;
            }
        }
        if (!probeStatus_.empty()) ImGui::TextWrapped("%s", probeStatus_.c_str());
    } else {
        ImGui::TextDisabled("Save CSV becomes available when the run ends.");
    }
    ImGui::End();
}

void App::drawProbe() {
    const armor::Frame* f = currentFrame();
    if (!f || hovered_ < 0 || static_cast<std::size_t>(hovered_) >= f->x.size()) return;
    const std::size_t i = static_cast<std::size_t>(hovered_);
    ImGui::BeginTooltip();
    ImGui::Text("%s", formatValue(Field::Material, f->values[0][i]).c_str());
    ImGui::Text("x = %.2f mm   %s = %.2f mm", f->x[i] * 1e3,
                recording()->mode() == armor::GeometryMode::Axisymmetric ? "r" : "y", f->y[i] * 1e3);
    ImGui::Separator();
    for (int k = 1; k < armor::kFieldCount; ++k) {
        const Field field = static_cast<Field>(k);
        ImGui::Text("%-26s %s", armor::fieldInfo(field).name,
                    formatValue(field, f->values[static_cast<std::size_t>(k)][i]).c_str());
    }
    ImGui::EndTooltip();
}
