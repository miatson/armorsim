#pragma once
// Run preset `scenarioIndex` (0-based) with its default settings, no window,
// and print a summary to stdout. Returns 0 = finished, 1 = blew up, 2 = bad index.
int runHeadless(int scenarioIndex);
