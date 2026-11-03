#pragma once
// Phase 5 - 2D visual simulation (SFML 2.x).
#include <string>

#include "Graph.h"
#include "Plan.h"

#ifndef EV_NO_GUI
// Opens the window and runs until it is closed.
//   shotFrames > 0 : advance that many fixed frames, save a screenshot to shotPath and return (used for testing).
int runSimulation(Graph& g, const Params& params, int src, int dst, const std::string& assetsDir,
                  int shotFrames = 0, const std::string& shotPath = "", int viewMode = 3, int scenario = -1);
#endif
