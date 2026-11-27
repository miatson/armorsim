// -----------------------------------------------------------------------------
// MiataPhySim - 2D impact and armour physics simulator.
// Window setup, loading screen and the main loop; everything else lives in App.
// -----------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <string>
#include <string_view>

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

#include "App.h"
#include "Headless.h"
#include "Presets.h"

namespace {

// Dark, rounded, roomy widgets with a blue accent.
void applyStyle() {
    ImGuiStyle& st = ImGui::GetStyle();
    st.WindowRounding = 0.0f;
    st.ChildRounding = 6.0f;
    st.FrameRounding = 5.0f;
    st.GrabRounding = 5.0f;
    st.TabRounding = 5.0f;
    st.PopupRounding = 6.0f;
    st.ScrollbarRounding = 6.0f;
    st.WindowPadding = {12.0f, 10.0f};
    st.FramePadding = {8.0f, 5.0f};
    st.ItemSpacing = {8.0f, 7.0f};
    st.SeparatorTextBorderSize = 2.0f;
    st.SeparatorTextPadding = {4.0f, 6.0f};
    ImVec4* c = st.Colors;
    const ImVec4 accent{0.26f, 0.52f, 0.86f, 1.0f};
    c[ImGuiCol_WindowBg] = {0.09f, 0.10f, 0.12f, 1.0f};
    c[ImGuiCol_ChildBg] = {0.12f, 0.13f, 0.16f, 1.0f};
    c[ImGuiCol_FrameBg] = {0.16f, 0.18f, 0.22f, 1.0f};
    c[ImGuiCol_FrameBgHovered] = {0.21f, 0.24f, 0.30f, 1.0f};
    c[ImGuiCol_Header] = {0.18f, 0.27f, 0.40f, 1.0f};
    c[ImGuiCol_HeaderHovered] = {0.22f, 0.34f, 0.52f, 1.0f};
    c[ImGuiCol_Tab] = {0.14f, 0.17f, 0.22f, 1.0f};
    c[ImGuiCol_TabSelected] = accent;
    c[ImGuiCol_TabHovered] = {0.30f, 0.56f, 0.90f, 1.0f};
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_CheckMark] = {0.45f, 0.70f, 1.0f, 1.0f};
    c[ImGuiCol_Button] = {0.20f, 0.30f, 0.45f, 1.0f};
    c[ImGuiCol_ButtonHovered] = {0.26f, 0.40f, 0.60f, 1.0f};
    c[ImGuiCol_PlotHistogram] = accent;
    c[ImGuiCol_SeparatorHovered] = accent;
}

// Loading screen: a title card with drifting "particles" and a progress bar
// that follows the real loading steps. Each step stays on screen for at least
// a moment so the card reads smoothly instead of flickering.
void runLoadingScreen(const std::vector<std::pair<const char*, std::function<void()>>>& steps) {
    struct Dot { float x, y, vx, vy, r; };
    std::vector<Dot> dots;
    unsigned seed = 12345u;
    auto rnd = [&seed] { seed = seed * 1664525u + 1013904223u; return static_cast<float>(seed >> 8) / 16777216.0f; };
    for (int k = 0; k < 90; ++k) dots.push_back({rnd(), rnd(), (rnd() - 0.5f) * 0.03f, (rnd() - 0.5f) * 0.03f, 1.5f + 3.0f * rnd()});
    auto frame = [&](float progress, const char* label) {
        const float w = static_cast<float>(GetScreenWidth()), h = static_cast<float>(GetScreenHeight());
        const float dt = GetFrameTime();
        BeginDrawing();
        ClearBackground(Color{14, 16, 22, 255});
        for (Dot& d : dots) {
            d.x += d.vx * dt;
            d.y += d.vy * dt;
            if (d.x < 0 || d.x > 1) d.vx = -d.vx;
            if (d.y < 0 || d.y > 1) d.vy = -d.vy;
            const float t = 0.5f + 0.5f * d.x;
            DrawCircleV({d.x * w, d.y * h}, d.r, Color{static_cast<unsigned char>(60 + 120 * t), 110, static_cast<unsigned char>(230 - 120 * t), 90});
        }
        const char* title = "MiataPhySim";
        const int ts = 72;
        DrawText(title, static_cast<int>(w / 2 - MeasureText(title, ts) / 2), static_cast<int>(h * 0.36f), ts, Color{235, 240, 250, 255});
        const char* sub = "impact & armour physics";
        DrawText(sub, static_cast<int>(w / 2 - MeasureText(sub, 24) / 2), static_cast<int>(h * 0.36f) + 84, 24, Color{120, 165, 235, 255});
        const float bw = 420.0f, bh = 8.0f, bx = w / 2 - bw / 2, by = h * 0.62f;
        DrawRectangleRounded({bx, by, bw, bh}, 1.0f, 8, Color{40, 46, 60, 255});
        DrawRectangleRounded({bx, by, bw * progress, bh}, 1.0f, 8, Color{66, 133, 220, 255});
        DrawText(label, static_cast<int>(w / 2 - MeasureText(label, 18) / 2), static_cast<int>(by + 20), 18, Color{150, 158, 175, 255});
        const char* foot = "work in progress  -  github.com/miatson/miataphysim";
        DrawText(foot, static_cast<int>(w / 2 - MeasureText(foot, 14) / 2), static_cast<int>(h - 34), 14, Color{90, 98, 115, 255});
        EndDrawing();
    };
    const std::size_t n = steps.size();
    for (std::size_t k = 0; k < n && !WindowShouldClose(); ++k) {
        const auto t0 = std::chrono::steady_clock::now();
        frame(static_cast<float>(k) / static_cast<float>(n), steps[k].first); // show the label first
        steps[k].second();
        while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(300) && !WindowShouldClose())
            frame(static_cast<float>(k + 1) / static_cast<float>(n), steps[k].first);
    }
    for (int k = 0; k < 10 && !WindowShouldClose(); ++k) frame(1.0f, "Ready");
}

} // namespace

// Command line (all optional):
//   --scenario N   start with scenario N selected (1 = first)
//   --field N      start showing view field N (0 = material, 1 = pressure, ...)
//   --run          start simulating immediately
//   --builder      start in 'Build your own'
//   --details      start on the Details tab
//   --headless     no window: run the scenario, print a summary, exit
//   --file F.json  use a scenario file (with --headless)
//   --csv out.csv  write probe time histories (with --headless)
//   --speed V      (with --file) fastest part at V m/s, others scaled alike
//   --spacing MM   (with --file) particle spacing in mm
//   --end-time US  (with --file) simulated time in microseconds
int main(int argc, char** argv) {
    StartOptions options;
    bool headless = false;
    std::string file, csv;
    HeadlessOverrides overrides;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--run") options.runAtStart = true;
        else if (arg == "--headless") headless = true;
        else if (arg == "--builder") options.builder = true;
        else if (arg == "--details") options.detailsTab = true;
        else if (arg == "--file" && i + 1 < argc) file = argv[++i];
        else if (arg == "--csv" && i + 1 < argc) csv = argv[++i];
        else if (arg == "--speed" && i + 1 < argc) overrides.speed = std::atof(argv[++i]);
        else if (arg == "--spacing" && i + 1 < argc) overrides.spacing = std::atof(argv[++i]) * 1e-3;
        else if (arg == "--end-time" && i + 1 < argc) overrides.endTime = std::atof(argv[++i]) * 1e-6;
        else if (arg == "--scenario" && i + 1 < argc) options.scenario = std::atoi(argv[++i]) - 1;
        else if (arg == "--field" && i + 1 < argc) options.field = std::atoi(argv[++i]);
    }

    if (headless) return file.empty() ? runHeadless(options.scenario, csv) : runHeadlessFile(file, csv, overrides);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "MiataPhySim - impact & armour physics (work in progress)");
    SetWindowMinSize(1100, 650);
    SetExitKey(KEY_NULL); // Esc shouldn't close the app
    SetTargetFPS(60);

    rlImGuiSetup(true); // dark theme
    ImGui::GetIO().IniFilename = nullptr; // don't write imgui.ini next to the exe
    applyStyle();

    runLoadingScreen({
        {"Reading material data", [] { (void)armor::defaultMaterials(); }},
        {"Loading projectiles and armour", [] { (void)appLibrary(); }},
        {"Loading scenarios", [] { (void)presets(); }},
        {"Preparing the interface", [] {}},
    });

    {
        App app(options);
        while (!WindowShouldClose()) {
            BeginDrawing();
            ClearBackground(Color{22, 24, 30, 255});
            app.frame();
            EndDrawing();
        }
    } // App destroyed (and the simulation thread stopped) before the window closes

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
