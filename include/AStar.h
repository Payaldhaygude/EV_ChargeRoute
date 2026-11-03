#pragma once
// Assignment 3 - A* search. Finds the shortest road path between two locations.
#include <vector>

#include "Graph.h"

struct AStarResult {
    bool found = false;
    std::vector<int> path;  // node ids from source to destination
    double km = 0;          // g(n): actual road distance travelled
    long expanded = 0;      // nodes expanded (search effort)
};

// g(n) = road km so far, h(n) = straight-line (haversine) km to the destination.
// Road length >= straight-line length, so h never overestimates (admissible) and is consistent.
AStarResult aStar(const Graph& g, int src, int dst);
