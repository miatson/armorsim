#pragma once
// -----------------------------------------------------------------------------
// App.h - the MiataPhySim user interface.
//
// Workflow: pick a scenario -> Simulate (runs on a background thread while
// frames appear live) -> replay, scrub, switch view modes, probe values.
// -----------------------------------------------------------------------------
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <raylib.h>

#include <armor/Library.h>
#include <armor/Material.h>
#include <armor/Recorder.h>
#include <armor/Scenario.h>

#include "Presets.h"
#include "View.h"

// One simulation running (or finished) on a worker thread.
struct SimulationRun {
    std::shared_ptr<armor::Recording> recording;
    std::thread thread;
    std::atomic<bool> cancel{false};
    std::atomic<bool> done{false};
    std::atomic<bool> failed{false};
    std::atomic<double> progress{0.0};
    std::atomic<double> simTime{0.0};
    std::atomic<double> endTime{0.0};
    std::atomic<double> dt{0.0};
    std::atomic<double> wallSeconds{0.0};
    std::atomic<unsigned long long> steps{0};
    std::atomic<std::size_t> particles{0};
    std::string error; // written before `failed` is set

    // Probe time histories (written by the worker thread, read by the UI;
    // both under probeMutex). probeSeries[column][sample], SI units.
    std::mutex probeMutex;
    std::vector<std::string> probeColumns;
    std::vector<double> probeTimes;
    std::vector<std::vector<double>> probeSeries;
    std::string probeCsv; // full CSV text, filled when the run ends

    ~SimulationRun() {
        cancel = true;
        if (thread.joinable()) {
            thread.join();
        }
    }
};

struct StartOptions {
    int scenario = 0;      // preset index
    int field = 1;         // view field index (1 = pressure)
    bool runAtStart = false;
    bool builder = false;     // start in 'Build your own'
    bool detailsTab = false;  // start on the Details tab
};

class App {
public:
    explicit App(const StartOptions& options = {});
    void frame(); // input + drawing for one window frame

private:
    void startSimulation();
    void cancelSimulation();
    void handleInput();
    void updatePlayback();
    void updateRanges();
    void fitView();

    void drawParticles();
    void drawOverlay();
    void drawLegend();
    void drawPanel();
    void drawProbe();
    void drawBodySizes(const armor::Frame& f, const armor::Recording& rec) const;
    void drawProbesPanel();
    void drawSimulateTab();
    void drawDetailsTab();
    void drawBuilder();
    armor::BuildSpec builderSpec() const;
    void resetBuilderLayer(std::size_t k, int item);
    std::string builderSummary() const;

    Rectangle viewArea() const;
    const armor::Recording* recording() const;
    const armor::Frame* currentFrame() const;
    int currentFrameIndex() const;
    std::pair<float, float> displayRange() const;
    Color colorFor(float value, std::pair<float, float> range) const;
    std::string formatValue(armor::Field field, double value) const;

    std::vector<armor::Material> materials_;
    bool runAtStart_ = false;
    int presetIndex_ = 0;
    std::string probeStatus_; // last 'Save CSV' result

    // What to simulate: 0 = ready-made (presets + scenario files), 1 = build your own.
    int sourceMode_ = 0;
    bool selectDetails_ = false; // switch to the Details tab on the first frame
    struct BuilderLayer {
        int item = 0;
        int material = 0;
        float thicknessMm = 10.0f;
        float gapMm = 0.0f; // in front of this layer (not used for the first)
    };
    int projItem_ = 0;
    int projMaterial_ = 0;
    float projSpeed_ = 500.0f;
    std::vector<BuilderLayer> layers_;
    float obliquityDeg_ = 0.0f;
    float spacingMm_ = 0.5f;
    float endTimeUs_ = 50.0f;
    bool autoResolution_ = true;
    std::string builderStatus_;
    std::string runName_ = "run";      // name of what is running (for file names)
    std::string runTheory_;              // Details > Theory text of the current setup
    // Advanced solver settings (Details tab); applied only once edited, so
    // scenario files keep their own settings otherwise.
    armor::SolverSettings advanced_;
    bool advancedEdited_ = false;
    PresetParams params_;
    std::unique_ptr<SimulationRun> run_;

    // playback
    float cursor_ = 0.0f;
    bool playing_ = false;
    float playFps_ = 30.0f;
    bool loop_ = true;
    bool followLive_ = true;

    // view
    armor::Field field_ = armor::Field::Pressure;
    int rangeMode_ = 1; // 0 = per frame, 1 = whole run
    bool mirror_ = true;
    View view_;
    bool viewFitted_ = false;
    int hovered_ = -1;

    // min/max of every field in every frame, filled as frames arrive
    std::vector<std::array<std::pair<float, float>, armor::kFieldCount>> ranges_;
};
