#include "Pipeline.h"

#include <chrono>
#include <fstream>
#include <sstream>

#include "ACO.h"
#include "AStar.h"
#include "CSP.h"

PipelineResult runPipeline(const Graph& g, const Params& p, int src, int dst) {
    PipelineResult r;
    r.src = src;
    r.dst = dst;

    // ---- Stage 1: A* ----
    auto t0 = std::chrono::steady_clock::now();
    AStarResult a = aStar(g, src, dst);
    double aMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!a.found) {
        r.connected = false;
        r.status = "NO FEASIBLE ROUTE";
        r.astar.name = r.csp.name = r.aco.name = "-";
        r.astar.name = "A*";
        r.csp.name = "A* + CSP";
        r.aco.name = "A* + CSP + ACO";
        r.astar.note = r.csp.note = r.aco.note = "No road connects the two locations";
        r.final = r.astar;
        return r;
    }
    r.astar = evaluatePlan(g, p, a.path, {}, "A*");
    r.astar.ms = aMs;
    r.astar.work = a.expanded;
    if (!r.astar.feasible) r.astar.note += " (shortest path ignores the battery)";

    // ---- Stage 2: CSP on the A* path ----
    r.csp = solveCSP(g, p, a.path);

    // ---- Stage 3: ACO over the whole network ----
    if (src == dst) {
        r.aco = r.astar;
        r.aco.name = "A* + CSP + ACO";
    } else {
        r.aco = solveACO(g, p, src, dst, &r.acoHistory);
    }

    // ---- choose the final route: best feasible plan ----
    const Plan* cand[3] = {&r.astar, &r.csp, &r.aco};
    int pick = 0;
    double bestScore = 1e18;
    for (int i = 0; i < 3; ++i) {
        if (!cand[i]->feasible) continue;
        // prefer later stages on ties (ACO > CSP > A*)
        if (cand[i]->score <= bestScore + 1e-9) { bestScore = cand[i]->score; pick = i + 1; }
    }
    r.finalWhich = pick;
    r.anyFeasible = pick != 0;
    if (pick) {
        r.final = *cand[pick - 1];
        r.status = r.final.stops == 0 ? "FEASIBLE - direct route"
                                       : "FEASIBLE - " + std::to_string(r.final.stops) + " charging stop(s)";
    } else {
        r.final = r.astar;
        r.status = "NO FEASIBLE ROUTE";
    }
    return r;
}

static std::vector<Scenario>& scenarioStore() {
    static std::vector<Scenario> v;
    return v;
}

const std::vector<Scenario>& defaultScenarios() { return scenarioStore(); }

bool loadScenarios(const std::string& csvPath, std::string& err) {
    std::ifstream f(csvPath);
    if (!f) { err = "cannot open " + csvPath; return false; }
    auto& v = scenarioStore();
    v.clear();
    std::string line;
    bool header = true;
    while (std::getline(f, line)) {
        size_t a = line.find_first_not_of(" \t\r\n");
        if (a == std::string::npos || line[a] == '#') continue;
        if (header) { header = false; continue; }
        std::vector<std::string> c;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            size_t x = cell.find_first_not_of(" \t\r\n"), y = cell.find_last_not_of(" \t\r\n");
            c.push_back(x == std::string::npos ? "" : cell.substr(x, y - x + 1));
        }
        if (c.size() < 6) { err = "bad scenario row: " + line; return false; }
        try {
            v.push_back({c[0], c[1], c[2], std::stod(c[3]), std::stoi(c[4]) != 0, c[5]});
        } catch (...) { err = "bad scenario row: " + line; return false; }
    }
    if (v.size() != 5) { err = "scenarios.csv must contain exactly 5 scenarios (found " + std::to_string(v.size()) + ")"; return false; }
    return true;
}

ScenarioRun runScenario(Graph& g, const Params& base, int index) {
    const Scenario& sc = defaultScenarios().at(index);
    ScenarioRun run;
    run.params = base;
    run.params.initialPct = sc.batteryPct;
    int src = g.findNode(sc.src), dst = g.findNode(sc.dst);
    if (src < 0 || dst < 0) { run.detail = "scenario locations missing from dataset"; return run; }

    PipelineResult first = runPipeline(g, run.params, src, dst);
    run.result = first;

    if (sc.blockFirstStop) {
        if (first.final.charges.empty()) { run.detail = "base route has no stop to block"; return run; }
        run.blockedNode = first.final.charges.front().node;
        g.node(run.blockedNode).available = false;
        run.result = runPipeline(g, run.params, src, dst);
    }
    const PipelineResult& r = run.result;

    switch (index) {
        case 0:
            run.passed = r.anyFeasible && r.final.stops == 0;
            run.detail = run.passed ? "direct route, 0 stops" : "expected a feasible direct route";
            break;
        case 1:
            run.passed = r.anyFeasible && r.final.stops >= 1;
            run.detail = run.passed ? std::to_string(r.final.stops) + " stop(s) at " + g.node(r.final.charges[0].node).name
                                    : "expected a feasible route with a charging stop";
            break;
        case 2:
            run.passed = r.anyFeasible && r.final.stops >= 1;   // a reachable station was found and used
            run.detail = run.passed ? "first stop: " + g.node(r.final.charges[0].node).name : "expected a reachable charging station";
            break;
        case 3: {
            bool usesBlocked = false;
            for (const auto& ev : r.final.charges) if (ev.node == run.blockedNode) usesBlocked = true;
            run.passed = r.anyFeasible && !usesBlocked && run.blockedNode >= 0;
            run.detail = run.passed ? "blocked " + g.node(run.blockedNode).name + ", alternative: " + g.node(r.final.charges[0].node).name
                                    : "expected an alternative route avoiding the blocked station";
            break;
        }
        case 4:
            run.passed = !r.anyFeasible;
            run.detail = run.passed ? "correctly reported: no feasible route" : "expected NO FEASIBLE ROUTE";
            break;
    }
    return run;
}
