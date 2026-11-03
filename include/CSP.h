#pragma once
// Assignment 4 - Constraint Satisfaction Problem (CSP).
//
// Given the A* path, decide WHERE to charge and HOW MUCH.
//   Variables : one per charger-equipped intermediate node on the path
//   Domain    : {skip} U {charge up to level L}   (L = discrete battery levels + "just enough" levels)
//   Constraints:
//     C1  arrival battery at every node >= reserve (next point / destination must be reachable)
//     C2  battery level never exceeds capacity
//     C3  a charging stop must have a working charger
//     C4  a charge must actually add energy (L > arrival level)
//   Solved with backtracking + forward checking, using iterative deepening on the number of
//   stops so the solution with the FEWEST stops is found first (ties -> lowest cost).
#include <vector>

#include "Plan.h"

Plan solveCSP(const Graph& g, const Params& p, const std::vector<int>& path);
