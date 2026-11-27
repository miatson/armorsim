// -----------------------------------------------------------------------------
// ArmorSim - 2D impact and material response simulator.
// Window setup and the main loop; everything else lives in App.
// -----------------------------------------------------------------------------
#include <cstdlib>
#include <string_view>

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

#include "App.h"
#include "Headless.h"

// Command line (all optional):
//   --scenario N   start with scenario N selected (1 = first)
//   --field N      start showing view field N (0 = material, 1 = pressure, ...)
//   --run          start simulating immediately
//   --headless     no window: run the scenario, print a summary, exit
int main(int argc, char** argv) {
    StartOptions options;
    bool headless = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--run") options.runAtStart = true;
        else if (arg == "--headless") headless = true;
        else if (arg == "--scenario" && i + 1 < argc) options.scenario = std::atoi(argv[++i]) - 1;
        else if (arg == "--field" && i + 1 < argc) options.field = std::atoi(argv[++i]);
    }

    if (headless) return runHeadless(options.scenario);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "ArmorSim - impact & material response (Stage C: failure, in progress)");
    SetWindowMinSize(1100, 650);
    SetExitKey(KEY_NULL); // Esc shouldn't close the app
    SetTargetFPS(60);

    rlImGuiSetup(true); // dark theme
    ImGui::GetIO().IniFilename = nullptr; // don't write imgui.ini next to the exe

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
