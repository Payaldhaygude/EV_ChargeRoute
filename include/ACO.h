#pragma once
// Assignment 5 - Ant Colony Optimization (ACO).
//
// Searches for the best SEQUENCE of charging stations over the whole network (not only along the
// A* path), so it can choose detours when they are cheaper or when the shortest path is infeasible.
//   * Each ant starts at the source and repeatedly picks the next stop among charging stations
//     (or the destination) that satisfy the battery constraints (BatteryConstraints, shared with CSP).
//   * Choice probability ~ pheromone^alpha * heuristic^beta,  heuristic = 1 / (leg cost + remaining distance).
//   * Legs between consecutive stops are A* shortest paths.
//   * After every iteration pheromone evaporates, good routes get more pheromone (best-so-far gets a bonus).
#include <vector>

#include "Plan.h"

// history (optional): best cost after each iteration, handy for plotting convergence.
Plan solveACO(const Graph& g, const Params& p, int src, int dst, std::vector<double>* history = nullptr);
