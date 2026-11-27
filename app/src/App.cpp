#include "App.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <imgui.h>
#include <rlImGui.h>

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
}

// -----------------------------------------------------------------------------
// Simulation thread
// -----------------------------------------------------------------------------
void App::startSimulation() {
    cancelSimulation();
    const Preset& preset = presets()[static_cast<std::size_t>(presetIndex_)];
    armor::Scenario scenario = preset.build(params_);

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

    SimulationRun* r = run.get();
    run->thread = std::thread([r, scenario, mats = materials_]() mutable {
        const auto start = std::chrono::steady_clock::now();
        auto elapsed = [&] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };
        try {
            armor::Solver solver(std::move(scenario), std::move(mats));
            r->particles = solver.particleCount();
            const double endTime = solver.scenario().endTime;
            const double interval = endTime / solver.scenario().frameCount;
            r->recording->push(solver.captureFrame());
            double nextFrame = interval;
            while (!solver.finished() && !r->cancel) {
                solver.step();
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
void App::drawPanel() {
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({kPanelWidth, static_cast<float>(GetScreenHeight())});
    ImGui::Begin("ArmorSim", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::TextWrapped("Stage C (in progress): metals yield, flow and heat up (Johnson-Cook), shocks follow measured "
                       "Hugoniots, and material now FAILS: ductile fracture (Johnson-Cook damage), spall under "
                       "sustained tension, and melting. Failed material can be pushed but not pulled, so cracks "
                       "open and fragments fly. View 'Damage' to see it.");
    ImGui::Separator();

    const bool running = run_ && !run_->done;

    // ---------------- Scenario ----------------
    if (ImGui::CollapsingHeader("1. Scenario", ImGuiTreeNodeFlags_DefaultOpen)) {
        const auto& list = presets();
        const Preset& current = list[static_cast<std::size_t>(presetIndex_)];
        if (ImGui::BeginCombo("Scenario", current.name.c_str())) {
            for (std::size_t i = 0; i < list.size(); ++i) {
                const bool selected = static_cast<int>(i) == presetIndex_;
                if (ImGui::Selectable(list[i].name.c_str(), selected)) {
                    presetIndex_ = static_cast<int>(i);
                    params_ = list[i].defaults;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::TextWrapped("%s", list[static_cast<std::size_t>(presetIndex_)].description.c_str());
        const Preset& p = list[static_cast<std::size_t>(presetIndex_)];
        ImGui::SliderFloat("Impact speed (m/s)", &params_.velocity, 0.5f, p.maxVelocity, "%.1f");
        ImGui::SliderFloat("Particle spacing (mm)", &params_.dxMm, 0.1f, 2.0f, "%.2f");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Smaller spacing = finer detail but slower.\nHalving it gives 4x the particles and 2x the steps: ~8x the time.");
        }
        ImGui::SliderFloat("End time (us)", &params_.endTimeUs, 1.0f, 500.0f, params_.endTimeUs < 10.0f ? "%.1f" : "%.0f");

        if (!running) {
            if (ImGui::Button("Simulate", {-1.0f, 36.0f})) startSimulation();
        } else {
            if (ImGui::Button("Cancel", {-1.0f, 36.0f})) cancelSimulation();
        }
        if (run_) {
            char overlay[64];
            std::snprintf(overlay, sizeof overlay, "%.0f%%", run_->progress.load() * 100.0);
            ImGui::ProgressBar(static_cast<float>(run_->progress.load()), {-1.0f, 0.0f}, overlay);
            ImGui::Text("Particles: %zu    Steps: %llu", run_->particles.load(), run_->steps.load());
            ImGui::Text("Time step: %.1f ns    Simulated: %.2f / %.2f us", run_->dt.load() * 1e9,
                        run_->simTime.load() * 1e6, run_->endTime.load() * 1e6);
            ImGui::Text("Computer time: %.1f s %s", run_->wallSeconds.load(),
                        run_->done ? (run_->cancel ? "(cancelled)" : "(finished)") : "(running)");
            if (run_->failed) ImGui::TextColored({1, 0.4f, 0.4f, 1}, "Error: %s", run_->error.c_str());
        }
    }

    // ---------------- Playback ----------------
    const armor::Recording* rec = recording();
    if (ImGui::CollapsingHeader("2. Playback", ImGuiTreeNodeFlags_DefaultOpen) && rec && rec->size() > 0) {
        if (ImGui::Button(playing_ ? "Pause" : "Play", {80, 0})) {
            playing_ = !playing_;
            followLive_ = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("|<", {40, 0})) { cursor_ = 0; followLive_ = false; }
        ImGui::SameLine();
        if (ImGui::Button(">|", {40, 0})) { cursor_ = static_cast<float>(rec->size() - 1); followLive_ = false; }
        ImGui::SameLine();
        ImGui::Checkbox("Loop", &loop_);
        ImGui::SameLine();
        ImGui::Checkbox("Live", &followLive_);
        int frameIndex = currentFrameIndex();
        if (ImGui::SliderInt("Frame", &frameIndex, 0, static_cast<int>(rec->size()) - 1)) {
            cursor_ = static_cast<float>(frameIndex);
            playing_ = false;
            followLive_ = false;
        }
        ImGui::SliderFloat("Replay speed (frames/s)", &playFps_, 1.0f, 120.0f, "%.0f");
        ImGui::TextDisabled("Space: play/pause   Left/Right: step frames");
    }

    // ---------------- View ----------------
    if (ImGui::CollapsingHeader("3. View", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::BeginCombo("Show", armor::fieldInfo(field_).name)) {
            for (int k = 0; k < armor::kFieldCount; ++k) {
                const Field f = static_cast<Field>(k);
                if (ImGui::Selectable(armor::fieldInfo(f).name, f == field_)) field_ = f;
            }
            ImGui::EndCombo();
        }
        ImGui::TextWrapped("%s", armor::fieldInfo(field_).help);
        ImGui::Text("Colour range:");
        ImGui::SameLine();
        ImGui::RadioButton("whole run", &rangeMode_, 1);
        ImGui::SameLine();
        ImGui::RadioButton("this frame", &rangeMode_, 0);
        if (rec && rec->mode() == armor::GeometryMode::Axisymmetric) {
            ImGui::Checkbox("Mirror axisymmetric view (show full cross-section)", &mirror_);
        }
        if (ImGui::Button("Fit view (F)")) fitView();
        ImGui::TextDisabled("Mouse wheel: zoom   Right-drag: pan   Hover: probe values");
    }

    // ---------------- Readout ----------------
    if (ImGui::CollapsingHeader("4. Readout & theory", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (const armor::Frame* f = currentFrame()) {
            const bool perMetre = rec->mode() == armor::GeometryMode::PlaneStrain;
            ImGui::Text("Kinetic energy:  %s", formatEnergy(f->kineticEnergy, perMetre).c_str());
            ImGui::Text("Internal energy: %s", formatEnergy(f->internalEnergy, perMetre).c_str());
            ImGui::Text("Total:           %s", formatEnergy(f->kineticEnergy + f->internalEnergy, perMetre).c_str());
            if (perMetre) ImGui::TextDisabled("(plane strain: per metre of depth)");
            drawBodySizes(*f, *rec);
        }
        ImGui::Separator();
        const Preset& p = presets()[static_cast<std::size_t>(presetIndex_)];
        ImGui::TextWrapped("%s", p.theory(params_, materials_).c_str());
    }

    // ---------------- Materials ----------------
    if (ImGui::CollapsingHeader("Material data")) {
        for (const auto& m : materials_) {
            ImGui::Text("%s", m.name.c_str());
            ImGui::Text("  density %.0f kg/m^3   E %.0f GPa   nu %.2f", m.density0, m.youngsModulus() / 1e9, m.poissonRatio());
            ImGui::Text("  wave speed %.0f m/s", m.soundSpeed(m.density0));
            ImGui::TextDisabled("  %s", m.source.c_str());
        }
    }
    ImGui::End();
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
