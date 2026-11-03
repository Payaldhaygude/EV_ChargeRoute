#pragma once
// Shared types: EV parameters, a route "plan" (path + charging events) and the plan evaluator.
// Every algorithm (A*, CSP, ACO) produces a Plan and every Plan is checked by evaluatePlan(),
// so the numbers shown in the simulation always come from one single battery model.
#include <map>
#include <string>
#include <vector>

#include "Graph.h"

struct Params {
    // --- EV ---
    double capacityKwh = 60.0;        // battery capacity
    double initialPct = 70.0;         // current charge (% of capacity)
    double reservePct = 5.0;          // never arrive anywhere below this
    double consumptionKwhPerKm = 0.18;
    // --- planning ---
    int maxStops = 4;                 // CSP / ACO limit on charging stops
    // cost = wDistance*km + wMoney*INR + wStop*stops + wTime*minutes
    double wDistance = 1.0, wMoney = 0.1, wStop = 10.0, wTime = 0.2;
    // --- ACO ---
    int ants = 40, iterations = 80;
    double alpha = 1.0, beta = 3.0, rho = 0.30, Q = 100.0;
    unsigned seed = 42;

    double initialKwh() const { return capacityKwh * initialPct / 100.0; }
    double reserveKwh() const { return capacityKwh * reservePct / 100.0; }
};

struct ChargeEvent {
    int pathIdx = 0;       // index inside Plan::path
    int node = 0;
    double arriveKwh = 0;  // battery on arrival
    double toKwh = 0;      // battery after charging
    double addedKwh = 0;
    double cost = 0;       // INR
    double minutes = 0;
};

struct Plan {
    std::string name;
    bool feasible = false;
    std::string note;                 // human readable reason / summary
    std::vector<int> path;            // every node visited, source..destination
    std::vector<ChargeEvent> charges;
    std::vector<double> arriveKwh;    // battery on arrival at path[i]   (can go < 0 when infeasible)
    std::vector<double> departKwh;    // battery when leaving path[i]
    int failIdx = -1;                 // first path index where constraints are violated
    double km = 0, energyKwh = 0, chargeCost = 0, chargeMin = 0, score = 0;
    int stops = 0;
    double ms = 0;                    // algorithm execution time
    long work = 0;                    // search effort (nodes expanded / states explored / ant trips)
};

double edgeEnergyKwh(const Graph& g, const Params& p, int a, int b);

// Simulates the battery along `path`. `chargeToKwh` maps path index -> battery level to charge up to.
// Marks the plan infeasible when: battery < reserve on arrival, capacity exceeded, or the charger
// at that node does not exist / is unavailable.
Plan evaluatePlan(const Graph& g, const Params& p, const std::vector<int>& path,
                  const std::map<int, double>& chargeToKwh, const std::string& name);

double planScore(const Params& p, double km, double money, int stops, double minutes);

// Battery-feasibility rules from the CSP (shared by the CSP solver and the ACO ants).
struct BatteryConstraints {
    const Graph& g;
    const Params& p;
    // Remaining energy after a leg must stay above the reserve.
    bool canTravel(double socKwh, double legKwh) const { return socKwh - legKwh >= p.reserveKwh() - 1e-9; }
    // Battery capacity cannot be exceeded.
    bool withinCapacity(double kwh) const { return kwh <= p.capacityKwh + 1e-9; }
    // Charging stops must exist and be working.
    bool chargerOk(int node) const { return g.chargerUsable(node); }
};
