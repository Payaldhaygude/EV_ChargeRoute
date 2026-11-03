// EV ChargeRoute - A* + CSP + ACO electric-vehicle route planner with a 2D simulation.
//
//   ev_chargeroute                       open the simulation window (default Thiruvananthapuram -> Kozhikode)
//   ev_chargeroute --from Kochi --to Kannur --battery 50      pre-fill the inputs
//   ev_chargeroute --cli  [...]          no window: print the algorithm comparison
//   ev_chargeroute --test                run the 5 test scenarios (no window)
//   ev_chargeroute --list                list all locations
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "Graph.h"
#include "Pipeline.h"
#include "Simulation.h"

namespace {

bool fileExists(const std::string& p) { return std::ifstream(p).good(); }

// Looks for a file/folder in a few likely places (cwd, parent, next to the executable).
std::string locate(const std::string& rel, const std::string& exeDir) {
    const std::string prefixes[] = {"", "../", "../../", exeDir + "/", exeDir + "/../", exeDir + "/../../"};
    for (const auto& pre : prefixes)
        if (fileExists(pre + rel)) return pre + rel;
    return rel;
}

std::string exeDirOf(const char* argv0) {
    std::string s = argv0 ? argv0 : "";
    size_t k = s.find_last_of("/\\");
    return k == std::string::npos ? "." : s.substr(0, k);
}

void printPlanRow(const Plan& p, bool chosen) {
    std::cout << (chosen ? " > " : "   ") << std::left << std::setw(16) << p.name << std::right << std::fixed
              << std::setw(8) << std::setprecision(1) << p.km << " km"
              << std::setw(8) << p.energyKwh << " kWh"
              << std::setw(4) << p.stops << " stops"
              << std::setw(9) << std::setprecision(1) << p.score << " cost"
              << std::setw(9) << std::setprecision(2) << p.ms << " ms   "
              << (p.feasible ? "FEASIBLE" : "INFEASIBLE") << "\n";
}

void printResult(const Graph& g, const Params& p, const PipelineResult& r) {
    std::cout << "\nTrip: " << g.node(r.src).name << " -> " << g.node(r.dst).name << "   battery " << p.capacityKwh
              << " kWh @ " << p.initialPct << "% (" << std::fixed << std::setprecision(1) << p.initialKwh()
              << " kWh), reserve " << p.reservePct << "%\n\n";
    printPlanRow(r.astar, r.finalWhich == 1);
    printPlanRow(r.csp, r.finalWhich == 2);
    printPlanRow(r.aco, r.finalWhich == 3);
    std::cout << "\nStatus: " << r.status << "\n";
    if (!r.csp.feasible) std::cout << "  CSP : " << r.csp.note << "\n";
    if (!r.aco.feasible) std::cout << "  ACO : " << r.aco.note << "\n";
    if (r.anyFeasible) {
        std::cout << "Route : ";
        for (size_t i = 0; i < r.final.path.size(); ++i) std::cout << (i ? " -> " : "") << g.node(r.final.path[i]).name;
        std::cout << "\n";
        for (const auto& ev : r.final.charges)
            std::cout << "Charge: " << std::setw(18) << std::left << g.node(ev.node).name << std::right << std::setprecision(1)
                      << ev.arriveKwh << " -> " << ev.toKwh << " kWh (+" << ev.addedKwh << " kWh, Rs "
                      << std::setprecision(0) << ev.cost << ", " << std::setprecision(0) << ev.minutes << " min)\n";
    } else if (r.connected) {
        std::cout << "(A* route would strand the EV: " << r.astar.note << ")\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string exeDir = exeDirOf(argv[0]);
    std::string dataPath, roadsPath, from = "Thiruvananthapuram", to = "Kozhikode", shot;
    std::string blockList;
    Params params;
    bool cli = false, test = false, list = false;
    int shotFrames = 0, view = 3, scenario = -1;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--cli") cli = true;
        else if (a == "--test") test = true;
        else if (a == "--list") list = true;
        else if (a == "--from") from = next();
        else if (a == "--to") to = next();
        else if (a == "--battery") params.initialPct = std::atof(next().c_str());
        else if (a == "--capacity") params.capacityKwh = std::atof(next().c_str());
        else if (a == "--reserve") params.reservePct = std::atof(next().c_str());
        else if (a == "--block") blockList = next();
        else if (a == "--seed") params.seed = static_cast<unsigned>(std::atoi(next().c_str()));
        else if (a == "--data") dataPath = next();
        else if (a == "--roads") roadsPath = next();
        else if (a == "--screenshot") shot = next();
        else if (a == "--frames") shotFrames = std::atoi(next().c_str());
        else if (a == "--view") view = std::atoi(next().c_str());
        else if (a == "--scenario") scenario = std::atoi(next().c_str());
        else if (a == "-h" || a == "--help") {
            std::cout << "usage: ev_chargeroute [--from NAME] [--to NAME] [--battery PCT] [--capacity KWH]\n"
                         "                      [--block NAME,NAME] [--cli] [--test] [--list] [--seed N]\n"
                         "                      [--data ev_data.csv --roads roads.csv] [--screenshot out.png --frames N]\n";
            return 0;
        } else { std::cerr << "unknown option " << a << " (try --help)\n"; return 2; }
    }

    if (dataPath.empty()) dataPath = locate("dataset/ev_data.csv", exeDir);
    if (roadsPath.empty()) roadsPath = locate("dataset/roads.csv", exeDir);
    std::string scenarioPath = locate("dataset/scenarios.csv", exeDir);
    std::string assetsDir = locate("assets", exeDir);

    Graph g;
    std::string err;
    if (!g.load(dataPath, roadsPath, err)) { std::cerr << "dataset error: " << err << "\n"; return 1; }

    std::string scenErr;
    if (!loadScenarios(scenarioPath, scenErr)) std::cerr << "warning: no test scenarios (" << scenErr << ")\n";

    if (list) {
        std::cout << g.size() << " locations, " << g.edgeCount() << " roads\n";
        for (int i = 0; i < g.size(); ++i) {
            const Node& n = g.node(i);
            std::cout << std::setw(3) << i << "  " << std::left << std::setw(20) << n.name << std::right
                      << (n.isCity ? " CITY    " : " STATION ") << (n.charger ? "charger " + std::to_string(static_cast<int>(n.powerKw)) + " kW  Rs " + std::to_string(static_cast<int>(n.price)) + "/kWh" : "no charger") << "\n";
        }
        return 0;
    }

    if (test) {
        std::cout << "EV ChargeRoute - test scenarios (" << g.size() << " locations, " << g.edgeCount() << " roads)\n\n";
        int failed = 0;
        for (size_t i = 0; i < defaultScenarios().size(); ++i) {
            Graph copy = g;  // scenario 4 disables a station - keep every test independent
            ScenarioRun run = runScenario(copy, params, static_cast<int>(i));
            const Scenario& sc = defaultScenarios()[i];
            std::cout << "Test " << i + 1 << " - " << sc.title << "  (" << sc.src << " -> " << sc.dst << ", "
                      << sc.batteryPct << "% battery)\n  expected: " << sc.expectation << "\n  result  : "
                      << run.result.status << " | " << run.detail << "\n  [" << (run.passed ? "PASS" : "FAIL") << "]\n\n";
            if (!run.passed) ++failed;
            if (cli) printResult(copy, run.params, run.result);
        }
        std::cout << (failed ? "SOME TESTS FAILED\n" : "ALL 5 SCENARIOS PASSED\n");
        return failed ? 1 : 0;
    }

    int src = g.findNode(from), dst = g.findNode(to);
    if (src < 0) { std::cerr << "unknown start location: " << from << " (use --list)\n"; return 1; }
    if (dst < 0) { std::cerr << "unknown destination: " << to << " (use --list)\n"; return 1; }
    {
        std::stringstream ss(blockList);
        std::string item;
        while (std::getline(ss, item, ',')) {
            int id = g.findNode(item);
            if (id >= 0) g.node(id).available = false;
            else if (!item.empty()) std::cerr << "warning: unknown station to block: " << item << "\n";
        }
    }

    if (cli) {
        PipelineResult r = runPipeline(g, params, src, dst);
        printResult(g, params, r);
        return r.anyFeasible ? 0 : 3;
    }

#ifdef EV_NO_GUI
    std::cerr << "This build has no SFML window support. Use --cli or --test, or install SFML 2.x and rebuild.\n";
    return 1;
#else
    return runSimulation(g, params, src, dst, assetsDir, shotFrames, shot, view, scenario);
#endif
}
