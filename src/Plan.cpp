#include "Plan.h"

double edgeEnergyKwh(const Graph& g, const Params& p, int a, int b) {
    const Edge* e = g.findEdge(a, b);
    if (!e) return 1e9;
    return e->km * p.consumptionKwhPerKm * e->terrain;
}

double planScore(const Params& p, double km, double money, int stops, double minutes) {
    return p.wDistance * km + p.wMoney * money + p.wStop * stops + p.wTime * minutes;
}

Plan evaluatePlan(const Graph& g, const Params& p, const std::vector<int>& path,
                  const std::map<int, double>& chargeToKwh, const std::string& name) {
    Plan pl;
    pl.name = name;
    pl.path = path;
    const int n = static_cast<int>(path.size());
    pl.arriveKwh.assign(n, 0.0);
    pl.departKwh.assign(n, 0.0);
    if (n == 0) { pl.note = "empty path"; return pl; }

    std::vector<double> leg(n > 1 ? n - 1 : 0);
    for (int i = 0; i + 1 < n; ++i) {
        leg[i] = edgeEnergyKwh(g, p, path[i], path[i + 1]);
        const Edge* e = g.findEdge(path[i], path[i + 1]);
        pl.km += e ? e->km : 0;
        pl.energyKwh += leg[i];
    }

    const double reserve = p.reserveKwh(), cap = p.capacityKwh;
    double soc = p.initialKwh();
    bool ok = true;
    for (int i = 0; i < n; ++i) {
        if (i > 0) soc -= leg[i - 1];
        pl.arriveKwh[i] = soc;
        if (ok && i > 0 && soc < reserve - 1e-9) {
            ok = false;
            pl.failIdx = i;
            pl.note = "Battery drops below the " + std::to_string(static_cast<int>(p.reservePct)) +
                      "% reserve before reaching " + g.node(path[i]).name;
        }
        auto it = chargeToKwh.find(i);
        if (ok && it != chargeToKwh.end()) {
            int node = path[i];
            double to = it->second;
            if (!g.chargerUsable(node)) {
                ok = false; pl.failIdx = i;
                pl.note = g.node(node).name + " has no working charger";
            } else if (to > cap + 1e-9) {
                ok = false; pl.failIdx = i;
                pl.note = "Charge level exceeds battery capacity at " + g.node(node).name;
            } else if (to > soc + 1e-9) {
                ChargeEvent ev;
                ev.pathIdx = i;
                ev.node = node;
                ev.arriveKwh = soc;
                ev.toKwh = to;
                ev.addedKwh = to - soc;
                ev.cost = ev.addedKwh * g.node(node).price;
                ev.minutes = g.node(node).powerKw > 0 ? ev.addedKwh / g.node(node).powerKw * 60.0 : 0.0;
                pl.charges.push_back(ev);
                pl.chargeCost += ev.cost;
                pl.chargeMin += ev.minutes;
                ++pl.stops;
                soc = to;
            }
        }
        pl.departKwh[i] = soc;
    }
    pl.feasible = ok;
    if (ok) pl.note = pl.stops == 0 ? "Direct route - no charging needed"
                                    : std::to_string(pl.stops) + " charging stop(s)";
    pl.score = planScore(p, pl.km, pl.chargeCost, pl.stops, pl.chargeMin);
    return pl;
}
