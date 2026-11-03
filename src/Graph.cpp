#include "Graph.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {

// Splits one CSV line; supports "quoted, fields" and trims spaces.
std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    auto flush = [&]() {
        size_t a = cur.find_first_not_of(" \t\r\n");
        size_t b = cur.find_last_not_of(" \t\r\n");
        out.push_back(a == std::string::npos ? "" : cur.substr(a, b - a + 1));
        cur.clear();
    };
    for (size_t i = 0; i < line.size(); ++i) {
        char ch = line[i];
        if (quoted) {
            if (ch == '"' && i + 1 < line.size() && line[i + 1] == '"') { cur += '"'; ++i; }
            else if (ch == '"') quoted = false;
            else cur += ch;
        } else if (ch == '"') quoted = true;
        else if (ch == ',') flush();
        else cur += ch;
    }
    flush();
    return out;
}

std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool isDataLine(const std::string& line) {
    size_t a = line.find_first_not_of(" \t\r\n");
    return a != std::string::npos && line[a] != '#';
}

}  // namespace

bool Graph::load(const std::string& nodesCsv, const std::string& roadsCsv, std::string& err) {
    nodes_.clear();
    adj_.clear();

    std::ifstream fn(nodesCsv);
    if (!fn) { err = "cannot open " + nodesCsv; return false; }
    std::string line;
    bool header = true;
    while (std::getline(fn, line)) {
        if (!isDataLine(line)) continue;
        if (header) { header = false; continue; }  // column names
        auto c = splitCsv(line);
        if (c.size() < 9) { err = "bad node row: " + line; return false; }
        Node n;
        try {
            n.id = std::stoi(c[0]);
            n.name = c[1];
            n.isCity = (c[2] == "CITY");
            n.lat = std::stod(c[3]);
            n.lon = std::stod(c[4]);
            n.charger = std::stoi(c[5]) != 0;
            n.price = std::stod(c[6]);
            n.powerKw = std::stod(c[7]);
            n.available = std::stoi(c[8]) != 0;
            if (c.size() > 9 && !c[9].empty()) n.stations = std::stoi(c[9]);
            if (c.size() > 10) n.operators = c[10];
        } catch (...) { err = "bad node row: " + line; return false; }
        if (n.id != static_cast<int>(nodes_.size())) { err = "node ids must be 0..N-1 in order (got " + c[0] + ")"; return false; }
        nodes_.push_back(n);
    }
    if (nodes_.empty()) { err = "no nodes in " + nodesCsv; return false; }
    adj_.assign(nodes_.size(), {});

    std::ifstream fr(roadsCsv);
    if (!fr) { err = "cannot open " + roadsCsv; return false; }
    header = true;
    while (std::getline(fr, line)) {
        if (!isDataLine(line)) continue;
        if (header) { header = false; continue; }
        auto c = splitCsv(line);
        if (c.size() < 4) { err = "bad road row: " + line; return false; }
        try {
            int a = std::stoi(c[0]), b = std::stoi(c[1]);
            double km = std::stod(c[2]), terrain = std::stod(c[3]);
            if (a < 0 || b < 0 || a >= size() || b >= size() || a == b) { err = "bad road endpoints: " + line; return false; }
            if (km <= 0) km = haversineKm(a, b) * 1.22;
            adj_[a].push_back({b, km, terrain});
            adj_[b].push_back({a, km, terrain});
        } catch (...) { err = "bad road row: " + line; return false; }
    }
    return true;
}

int Graph::edgeCount() const {
    size_t total = 0;
    for (const auto& a : adj_) total += a.size();
    return static_cast<int>(total / 2);
}

int Graph::findNode(const std::string& text) const {
    if (text.empty()) return -1;
    bool digits = std::all_of(text.begin(), text.end(), [](unsigned char ch) { return std::isdigit(ch); });
    if (digits) {
        int id = std::stoi(text);
        return (id >= 0 && id < size()) ? id : -1;
    }
    std::string t = lower(text);
    for (const auto& n : nodes_) if (lower(n.name) == t) return n.id;
    for (const auto& n : nodes_) if (lower(n.name).rfind(t, 0) == 0) return n.id;
    for (const auto& n : nodes_) if (lower(n.name).find(t) != std::string::npos) return n.id;
    return -1;
}

const Edge* Graph::findEdge(int a, int b) const {
    for (const auto& e : adj_[a]) if (e.to == b) return &e;
    return nullptr;
}

double Graph::haversineKm(int a, int b) const {
    const double R = 6371.0088, D = 3.14159265358979323846 / 180.0;
    double p1 = nodes_[a].lat * D, p2 = nodes_[b].lat * D;
    double dp = p2 - p1, dl = (nodes_[b].lon - nodes_[a].lon) * D;
    double h = std::sin(dp / 2) * std::sin(dp / 2) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
    return 2 * R * std::asin(std::sqrt(h));
}

std::vector<int> Graph::cityIds() const {
    std::vector<int> v;
    for (const auto& n : nodes_) if (n.isCity) v.push_back(n.id);
    return v;
}
