#pragma once
// Road network: locations are nodes, roads are weighted (km) edges.
#include <string>
#include <vector>

struct Node {
    int id = 0;
    std::string name;
    bool isCity = true;      // CITY vs STATION
    double lat = 0, lon = 0;
    bool charger = false;    // does a charger exist here?
    bool available = true;   // is it currently working? (can be toggled to test "station unavailable")
    double price = 0;        // INR per kWh
    double powerKw = 0;      // charging power (fastest charger in the hub)
    int stations = 1;        // number of real charging stations merged into this hub (dataset column 'stations')
    std::string operators;   // operators present, e.g. "Tata Power/ChargeMod (IN)"
};

struct Edge {
    int to = 0;
    double km = 0;           // road distance
    double terrain = 1.0;    // energy multiplier (ghat / hilly roads > 1)
};

class Graph {
public:
    // Phase 1: load nodes + roads from CSV files. Returns false and fills err on failure.
    bool load(const std::string& nodesCsv, const std::string& roadsCsv, std::string& err);

    int size() const { return static_cast<int>(nodes_.size()); }
    const Node& node(int i) const { return nodes_[i]; }
    Node& node(int i) { return nodes_[i]; }
    const std::vector<Edge>& adj(int i) const { return adj_[i]; }
    int edgeCount() const;

    // Find a node by id ("12") or by name (case-insensitive, exact, then prefix/substring). -1 if none.
    int findNode(const std::string& text) const;
    const Edge* findEdge(int a, int b) const;

    // Straight-line distance (km) - used as the A* heuristic.
    double haversineKm(int a, int b) const;

    bool chargerUsable(int i) const { return nodes_[i].charger && nodes_[i].available; }
    std::vector<int> cityIds() const;

private:
    std::vector<Node> nodes_;
    std::vector<std::vector<Edge>> adj_;
};
