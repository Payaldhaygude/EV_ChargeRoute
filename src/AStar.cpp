#include "AStar.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>

AStarResult aStar(const Graph& g, int src, int dst) {
    AStarResult res;
    const int n = g.size();
    if (src < 0 || dst < 0 || src >= n || dst >= n) return res;

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gCost(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<char> closed(n, 0);

    using QItem = std::pair<double, int>;  // (f = g + h, node)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

    gCost[src] = 0;
    open.push({g.haversineKm(src, dst), src});

    while (!open.empty()) {
        int u = open.top().second;
        open.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        ++res.expanded;

        if (u == dst) {
            res.found = true;
            res.km = gCost[u];
            for (int v = dst; v != -1; v = parent[v]) res.path.push_back(v);
            std::reverse(res.path.begin(), res.path.end());
            return res;
        }
        for (const Edge& e : g.adj(u)) {
            if (closed[e.to]) continue;
            double ng = gCost[u] + e.km;
            if (ng < gCost[e.to]) {
                gCost[e.to] = ng;
                parent[e.to] = u;
                open.push({ng + g.haversineKm(e.to, dst), e.to});
            }
        }
    }
    return res;  // no route
}
