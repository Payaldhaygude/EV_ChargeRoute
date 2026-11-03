#pragma once
// Orchestrates the workflow from the roadmap:
//   A* (route) -> CSP (battery feasibility) -> ACO (optimised charging sequence) -> final route
// and defines the five test scenarios.
#include <string>
#include <vector>

#include "Plan.h"

struct PipelineResult {
    int src = -1, dst = -1;
    Plan astar, csp, aco;      // the three stages shown side by side
    Plan final;                // route that will be simulated
    int finalWhich = 0;        // 0 = none feasible, 1 = A*, 2 = A*+CSP, 3 = A*+CSP+ACO
    bool anyFeasible = false;
    bool connected = true;     // false if A* found no road path at all
    std::string status;        // "FEASIBLE - direct", "FEASIBLE - 2 charging stop(s)", "NO FEASIBLE ROUTE"
    std::vector<double> acoHistory;
};

PipelineResult runPipeline(const Graph& g, const Params& p, int src, int dst);

// ----- testing scenarios (section 7 of the roadmap) -----
struct Scenario {
    std::string title;
    std::string src, dst;
    double batteryPct;
    bool blockFirstStop;      // scenario 4: knock out the station the planner would use first
    std::string expectation;
};
// Test scenarios come from dataset/scenarios.csv (written by tools/build_dataset.py from the real data).
bool loadScenarios(const std::string& csvPath, std::string& err);
const std::vector<Scenario>& defaultScenarios();

struct ScenarioRun {
    PipelineResult result;
    Params params;
    int blockedNode = -1;     // station made unavailable (scenario 4)
    bool passed = false;
    std::string detail;
};
// Runs scenario #index (0-based). Graph availability flags are modified (blocked station) and restored by the caller.
ScenarioRun runScenario(Graph& g, const Params& base, int index);
