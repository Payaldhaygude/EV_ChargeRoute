#include "ACO.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <random>

#include "AStar.h"

namespace {

struct Leg {
    bool ok = false;
    double km = 0, kwh = 0;
    std::vector<int> path;
};

}  // namespace

// detourRatio > 0: only stations inside an ellipse around src-dst are considered (|src-p| + |p-dst| <= ratio*|src-dst| + 25 km).
static Plan solveACOImpl(const Graph& g, const Params& p, int src, int dst, std::vector<double>* history, double detourRatio) {
    auto t0 = std::chrono::steady_clock::now();
    BatteryConstraints bc{g, p};
    const double INF = std::numeric_limits<double>::infinity();

    // ---- points of interest: source, destination, every usable charger ----
    std::vector<int> pts{src, dst};
    const double direct = g.haversineKm(src, dst);
    for (int i = 0; i < g.size(); ++i) {
        if (i == src || i == dst || !bc.chargerOk(i)) continue;
        if (detourRatio > 0 && g.haversineKm(src, i) + g.haversineKm(i, dst) > detourRatio * direct + 25.0) continue;
        pts.push_back(i);
    }
    const int N = static_cast<int>(pts.size());
    const int S = 0, D = 1;

    // ---- pre-compute A* legs between all points ----
    std::vector<std::vector<Leg>> leg(N, std::vector<Leg>(N));
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            AStarResult r = aStar(g, pts[i], pts[j]);
            if (!r.found) continue;
            Leg l;
            l.ok = true;
            l.km = r.km;
            l.path = r.path;
            for (size_t k = 0; k + 1 < r.path.size(); ++k) l.kwh += edgeEnergyKwh(g, p, r.path[k], r.path[k + 1]);
            leg[i][j] = l;
            l.path.assign(r.path.rbegin(), r.path.rend());
            leg[j][i] = l;
        }
    }

    Plan best;
    best.score = INF;
    std::vector<std::vector<double>> tau(N, std::vector<double>(N, 1.0));
    std::mt19937 rng(p.seed);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    long trips = 0;

    struct Ant { bool ok = false; std::vector<int> seq; std::map<int, double> chargeAtPtIdx; Plan plan; };

    for (int it = 0; it < p.iterations; ++it) {
        std::vector<Ant> ants(p.ants);
        for (Ant& ant : ants) {
            ++trips;
            std::vector<char> visited(N, 0);
            std::vector<char> usedNode(g.size(), 0);   // road nodes already driven through (no doubling back)
            usedNode[pts[S]] = 1;
            int cur = S;
            double soc = p.initialKwh();
            bool canCharge = false;              // no charging at the start (battery level is given by the user)
            visited[S] = 1;
            ant.seq.push_back(S);

            for (int step = 0; step < p.maxStops + 3 && cur != D; ++step) {
                std::vector<int> options;
                std::vector<double> weight;
                for (int j = 0; j < N; ++j) {
                    if (visited[j] || !leg[cur][j].ok) continue;
                    bool revisits = false;
                    for (size_t m = 1; m < leg[cur][j].path.size(); ++m)
                        if (usedNode[leg[cur][j].path[m]]) { revisits = true; break; }
                    if (revisits) continue;
                    double need = leg[cur][j].kwh + p.reserveKwh();
                    bool reachable = canCharge ? bc.withinCapacity(need)            // can top up before leaving
                                               : bc.canTravel(soc, leg[cur][j].kwh);
                    if (!reachable) continue;

                    double extra = 0;
                    if (canCharge && need > soc) {                                   // would charge at `cur`
                        double add = need - soc;
                        extra = p.wMoney * add * g.node(pts[cur]).price + p.wStop +
                                p.wTime * (g.node(pts[cur]).powerKw > 0 ? add / g.node(pts[cur]).powerKw * 60.0 : 0);
                    }
                    double remaining = (j == D) ? 0.0 : (leg[j][D].ok ? leg[j][D].km : 1e6);
                    double eta = 1.0 / (p.wDistance * (leg[cur][j].km + remaining) + extra + 1e-6);
                    options.push_back(j);
                    weight.push_back(std::pow(tau[cur][j], p.alpha) * std::pow(eta, p.beta));
                }
                if (options.empty()) break;       // dead end

                double sum = 0;
                for (double w : weight) sum += w;
                double r = uni(rng) * sum, acc = 0;
                int pick = options.back();
                for (size_t k = 0; k < options.size(); ++k) {
                    acc += weight[k];
                    if (r <= acc) { pick = options[k]; break; }
                }

                double need = leg[cur][pick].kwh + p.reserveKwh();
                if (canCharge && need > soc) {
                    ant.chargeAtPtIdx[cur] = need;                                   // "just enough" for the next leg
                    soc = need;
                }
                soc -= leg[cur][pick].kwh;
                visited[pick] = 1;
                for (size_t m = 1; m < leg[cur][pick].path.size(); ++m) usedNode[leg[cur][pick].path[m]] = 1;
                ant.seq.push_back(pick);
                cur = pick;
                canCharge = bc.chargerOk(pts[cur]);
            }

            if (cur != D) continue;

            // ---- build the full node path + evaluate with the common battery model ----
            std::vector<int> full{pts[S]};
            std::map<int, double> charges;
            for (size_t k = 0; k + 1 < ant.seq.size(); ++k) {
                int a = ant.seq[k], b = ant.seq[k + 1];
                auto c = ant.chargeAtPtIdx.find(a);
                if (c != ant.chargeAtPtIdx.end()) charges[static_cast<int>(full.size()) - 1] = c->second;
                for (size_t m = 1; m < leg[a][b].path.size(); ++m) full.push_back(leg[a][b].path[m]);
            }
            ant.plan = evaluatePlan(g, p, full, charges, "A* + CSP + ACO");
            ant.ok = ant.plan.feasible && ant.plan.stops <= p.maxStops;
            if (ant.ok && ant.plan.score < best.score) best = ant.plan;
        }

        // ---- pheromone update: evaporate, then deposit ----
        for (auto& row : tau)
            for (double& t : row) t = std::max(0.01, t * (1.0 - p.rho));
        auto deposit = [&](const std::vector<int>& seq, double amount) {
            for (size_t k = 0; k + 1 < seq.size(); ++k) {
                tau[seq[k]][seq[k + 1]] += amount;
                tau[seq[k + 1]][seq[k]] += amount;
            }
        };
        for (const Ant& ant : ants) if (ant.ok) deposit(ant.seq, p.Q / std::max(1.0, ant.plan.score));
        if (best.score < INF) {
            // elitist bonus: re-trace the best plan's stop sequence through the point table
            std::vector<int> seq{S};
            for (const ChargeEvent& ev : best.charges)
                for (int k = 0; k < N; ++k) if (pts[k] == ev.node) { seq.push_back(k); break; }
            seq.push_back(D);
            deposit(seq, 2.0 * p.Q / std::max(1.0, best.score));
        }
        if (history) history->push_back(best.score < INF ? best.score : -1.0);
    }

    Plan out;
    if (best.score < INF) {
        out = best;
        out.note = (out.stops == 0 ? std::string("Direct route - no charging needed")
                                   : std::to_string(out.stops) + " charging stop(s), ordered by ACO");
    } else {
        AStarResult r = aStar(g, src, dst);
        out = evaluatePlan(g, p, r.path, {}, "A* + CSP + ACO");
        out.feasible = false;
        out.note = r.found ? "No battery-feasible charging sequence found in the network" : "Destination is not connected to the start";
    }
    out.work = trips;
    out.note += " [" + std::to_string(p.ants) + " ants x " + std::to_string(p.iterations) + " iterations]";
    out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

Plan solveACO(const Graph& g, const Params& p, int src, int dst, std::vector<double>* history) {
    // 1st pass: corridor around the trip (fast, focused).  2nd pass: whole network, only if the corridor has no feasible plan.
    if (history) history->clear();
    Plan r = solveACOImpl(g, p, src, dst, history, 1.35);
    if (!r.feasible) {
        double ms = r.ms;
        if (history) history->clear();
        r = solveACOImpl(g, p, src, dst, history, 0.0);
        r.ms += ms;
    }
    return r;
}
