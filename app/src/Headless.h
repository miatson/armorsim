#pragma once
#include <string>
// Run preset `scenarioIndex` (0-based) with its default settings, or a scenario
// file, with no window; print a summary to stdout, and write the probe time
// histories to `csvPath` if given. Returns 0 = finished, 1 = blew up,
// 2 = bad input.
int runHeadless(int scenarioIndex, const std::string& csvPath = {});

// Optional overrides for a scenario file (0 = keep the file's value).
struct HeadlessOverrides {
    double speed = 0.0;   // m/s: scales every moving part so the fastest has this speed
    double spacing = 0.0; // m
    double endTime = 0.0; // s
};
int runHeadlessFile(const std::string& path, const std::string& csvPath = {}, const HeadlessOverrides& o = {});
