#include "CSP.h"

#include <chrono>
#include <limits>
#include <set>

namespace {

struct Solver {
    const Graph& g;
    const Params& p;
    const std::vector<int>& path;
    BatteryConstraints bc;

    int n = 0;
    std::vector<double> cum;              // cumulative energy from the start to path[i]
    std::vector<int> cand;                // path indices that may host a charging stop (variables)
    std::vector<std::vector<double>> dom; // domain of each variable (charge-to levels, kWh)
    double reserve = 0, cap = 0;

    int budget = 0;
    long explored = 0, backtracks = 0;
    const long limit = 3000000;
    double bestScore = std::numeric_limits<double>::infinity();
    std::map<int, double> bestAssign;
    bool found = false;
    std::vector<std::pair<int, double>> cur;

    Solver(const Graph& graph, const Params& params, const std::vector<int>& pa)
        : g(graph), p(params), path(pa), bc{graph, params} {
        n = static_cast<int>(path.size());
        reserve = p.reserveKwh();
        cap = p.capacityKwh;
        cum.assign(n, 0.0);
        for (int i = 1; i < n; ++i) cum[i] = cum[i - 1] + edgeEnergyKwh(g, p, path[i - 1], path[i]);

        // Variables: intermediate nodes with a usable charger (C3). Charging at the very start is not allowed:
        // the user supplies the current battery level, and the destination never needs a charge.
        for (int i = 1; i + 1 < n; ++i) if (bc.chargerOk(path[i])) cand.push_back(i);

        dom.resize(cand.size());
        for (size_t k = 0; k < cand.size(); ++k) {
            int ci = cand[k];
            std::set<double> d;
            for (int pct = 30; pct <= 100; pct += 10) d.insert(cap * pct / 100.0);
            double needDest = (cum[n - 1] - cum[ci]) + reserve;
            if (bc.withinCapacity(needDest)) d.insert(needDest);                    // "just enough for the destination"
            if (k + 1 < cand.size()) {
                double needNext = (cum[cand[k + 1]] - cum[ci]) + reserve;
                if (bc.withinCapacity(needNext)) d.insert(needNext);                // "just enough for the next stop"
            }
            for (double v : d) if (bc.withinCapacity(v)) dom[k].push_back(v);       // C2
        }
    }

    // Forward check: from (index li, battery sd) the car must be able to reach the next candidate OR the destination.
    bool forward(int li, double sd, size_t nk) const {
        bool dest = bc.canTravel(sd, cum[n - 1] - cum[li]);
        bool next = nk < cand.size() && bc.canTravel(sd, cum[cand[nk]] - cum[li]);
        return dest || next;
    }

    void leaf(int li, double sd) {
        if (!bc.canTravel(sd, cum[n - 1] - cum[li])) { ++backtracks; return; }   // C1 at destination
        std::map<int, double> a;
        for (auto& pr : cur) a[pr.first] = pr.second;
        Plan pl = evaluatePlan(g, p, path, a, "");
        if (pl.feasible && pl.score < bestScore) {
            bestScore = pl.score;
            bestAssign = a;
            found = true;
        }
    }

    void bt(size_t k, int li, double sd, int used) {
        if (explored > limit) return;
        ++explored;
        if (k == cand.size()) { leaf(li, sd); return; }
        int ci = cand[k];
        double arrive = sd - (cum[ci] - cum[li]);
        if (arrive < reserve - 1e-9) { ++backtracks; return; }                    // C1 violated -> backtrack

        // value 1: skip this station
        if (forward(li, sd, k + 1)) bt(k + 1, li, sd, used);
        else ++backtracks;

        // value 2..m: charge up to level L
        if (used < budget) {
            for (double L : dom[k]) {
                if (L <= arrive + 1e-9) continue;                                 // C4
                if (!forward(ci, L, k + 1)) { ++backtracks; continue; }
                cur.push_back({ci, L});
                bt(k + 1, ci, L, used + 1);
                cur.pop_back();
            }
        }
    }
};

}  // namespace

Plan solveCSP(const Graph& g, const Params& p, const std::vector<int>& path) {
    auto t0 = std::chrono::steady_clock::now();
    Solver s(g, p, path);
    Plan out;
    long totalExplored = 0, totalBack = 0;

    if (path.size() >= 2) {
        for (int b = 0; b <= p.maxStops && !s.found; ++b) {   // iterative deepening on #stops
            s.budget = b;
            s.explored = 0;
            s.backtracks = 0;
            s.cur.clear();
            s.bt(0, 0, p.initialKwh(), 0);
            totalExplored += s.explored;
            totalBack += s.backtracks;
        }
    }

    if (s.found) {
        out = evaluatePlan(g, p, path, s.bestAssign, "A* + CSP");
        out.note = (out.stops == 0 ? std::string("Shortest path is battery-feasible without charging")
                                   : std::to_string(out.stops) + " charging stop(s) needed on the shortest path");
    } else {
        out = evaluatePlan(g, p, path, {}, "A* + CSP");
        out.feasible = false;
        out.note = "No battery-feasible charging plan exists on the A* route (" +
                   std::to_string(s.cand.size()) + " candidate stations, up to " +
                   std::to_string(p.maxStops) + " stops)";
    }
    out.work = totalExplored;
    out.note += " [" + std::to_string(totalExplored) + " states, " + std::to_string(totalBack) + " backtracks]";
    out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}
