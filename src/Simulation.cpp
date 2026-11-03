// Phase 5 - 2D visual simulation with SFML 3.x
//   left   : user input panel (start, destination, battery, scenarios, view)
//   centre : map - roads, stations, route overlays and the animated EV
//   right  : live information + comparison of A* / A*+CSP / A*+CSP+ACO + event log
#ifndef EV_NO_GUI

#include "Simulation.h"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "Pipeline.h"

namespace {

// ------------------------------------------------------------------ constants
constexpr int WIN_W = 1300, WIN_H = 780;
sf::FloatRect FR(float x, float y, float w, float h) { return sf::FloatRect({x, y}, {w, h}); }
const sf::FloatRect LEFT_R = FR(0, 0, 300, WIN_H), MAP_R = FR(300, 0, 580, WIN_H), RIGHT_R = FR(880, 0, 420, WIN_H);

const sf::Color C_BG(16, 20, 28), C_PANEL(26, 31, 42), C_PANEL2(38, 45, 60), C_TXT(232, 236, 244), C_DIM(140, 150, 170),
    C_ACC(90, 175, 255), C_GREEN(70, 205, 125), C_ORANGE(255, 165, 45), C_RED(240, 85, 85), C_YEL(250, 215, 70),
    C_ROAD(72, 85, 104), C_SEA(18, 38, 66);

enum class AState { Idle, Moving, Charging, Arrived, Stranded, NoRoute };

enum BtnId {
    B_START_PREV = 1, B_START_NEXT, B_DEST_PREV, B_DEST_NEXT, B_CAP_MINUS, B_CAP_PLUS, B_BAT_MINUS, B_BAT_PLUS,
    B_RUN, B_PLAY, B_SPEED_MINUS, B_SPEED_PLUS, B_RESET_STATIONS,
    B_VIEW1 = 30, B_VIEW2, B_VIEW3,
    B_SCEN0 = 40
};

struct Button {
    sf::FloatRect r;
    std::string label;
    int id;
    bool small = false;
};

struct Anim {
    Plan plan;
    AState st = AState::Idle;
    int idx = 0;
    double f = 0;
    double battery = 0;
    double dist = 0, energyUsed = 0;
    int stopsDone = 0;
    double chargeT = 0, chargeDur = 1, chargeFrom = 0, chargeTo = 0;
    float angle = 0;
};

std::string fmt(double v, int prec = 1) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

std::vector<std::string> wrap(const std::string& s, size_t maxChars) {
    std::vector<std::string> lines;
    std::stringstream ss(s);
    std::string word, cur;
    while (ss >> word) {
        if (!cur.empty() && cur.size() + 1 + word.size() > maxChars) { lines.push_back(cur); cur.clear(); }
        cur += (cur.empty() ? "" : " ") + word;
    }
    if (!cur.empty()) lines.push_back(cur);
    return lines;
}

sf::Color withAlpha(sf::Color c, std::uint8_t a) { c.a = a; return c; }

// ------------------------------------------------------------------ the application
class App {
public:
    App(Graph& graph, const Params& p, int src, int dst, const std::string& assets)
        : g(graph), params(p), src(src), dst(dst), assetsDir(assets),
          win(sf::VideoMode({static_cast<unsigned>(WIN_W), static_cast<unsigned>(WIN_H)}), "EV ChargeRoute - A* + CSP + ACO", sf::Style::Titlebar | sf::Style::Close) {
        win.setFramerateLimit(60);
        loadResources();
        for (int i = 0; i < g.size(); ++i) origAvail.push_back(g.node(i).available);
        cities = g.cityIds();
        computeProjection();
        buildButtons();
        recompute();
    }

    void applyScenario(int i) { scenarioPreset(i); }
    void setView(int v) { view = v; startAnim(); }

    int run() {
        sf::Clock clock;
        while (win.isOpen()) {
            while (const std::optional<sf::Event> ev = win.pollEvent()) handleEvent(*ev);
            update(clock.restart().asSeconds());
            draw();
            win.display();
        }
        return 0;
    }

    int runShot(int frames, const std::string& path) {
        for (int i = 0; i < frames; ++i) { update(1.0 / 30.0); }
        while (win.pollEvent()) {}
        draw();
        win.display();
        sf::Texture tex;
        if (!tex.resize(win.getSize())) { std::cerr << "could not create texture\n"; return 1; }
        tex.update(win);
        if (!tex.copyToImage().saveToFile(path)) { std::cerr << "could not save " << path << "\n"; return 1; }
        std::cout << "screenshot saved: " << path << "\n";
        return 0;
    }

private:
    // ---- data
    Graph& g;
    Params params;
    int src, dst;
    std::string assetsDir;
    sf::RenderWindow win;
    sf::Font font, fontB;
    bool fontOk = false, fontBOk = false;
    sf::Texture texCar, texStation, texBg;
    bool haveCar = false, haveStation = false, haveBg = false;

    PipelineResult res;
    Anim anim;
    int view = 3;
    bool playing = true;
    double speed = 1.0;
    double clockT = 0;
    int scenarioActive = -1;
    std::vector<std::string> logLines;
    std::vector<char> origAvail;
    std::vector<int> cities;
    std::vector<sf::Vector2f> pos;
    std::vector<Button> buttons;
    double latMin = 0, latMax = 0, lonMin = 0, lonMax = 0, scale = 1, cosLat = 1;
    sf::Vector2f origin;
    sf::Vector2i mouse;

    // ---- resources
    void loadResources() {
        const std::string cand[] = {assetsDir + "/DejaVuSans.ttf", "assets/DejaVuSans.ttf",
                                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf",
                                    "/System/Library/Fonts/Supplemental/Arial.ttf", "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf"};
        for (const auto& c : cand) if (font.openFromFile(c)) { fontOk = true; break; }
        if (!fontOk) std::cerr << "warning: no font found - text will not be shown (expected assets/DejaVuSans.ttf)\n";
        const std::string candB[] = {assetsDir + "/DejaVuSans-Bold.ttf", "assets/DejaVuSans-Bold.ttf",
                                     "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "C:/Windows/Fonts/segoeuib.ttf", "C:/Windows/Fonts/arialbd.ttf"};
        for (const auto& c : candB) if (fontB.openFromFile(c)) { fontBOk = true; break; }
        haveCar = texCar.loadFromFile(assetsDir + "/car.png");
        haveStation = texStation.loadFromFile(assetsDir + "/charging_station.png");
        haveBg = texBg.loadFromFile(assetsDir + "/background.png");
        texCar.setSmooth(true);
        texStation.setSmooth(true);
        texBg.setSmooth(true);
    }

    // ---- projection: lat/lon -> pixels inside the map panel
    void computeProjection() {
        latMin = 1e9; latMax = -1e9; lonMin = 1e9; lonMax = -1e9;
        for (int i = 0; i < g.size(); ++i) {
            latMin = std::min(latMin, g.node(i).lat); latMax = std::max(latMax, g.node(i).lat);
            lonMin = std::min(lonMin, g.node(i).lon); lonMax = std::max(lonMax, g.node(i).lon);
        }
        latMin -= 0.25; latMax += 0.25; lonMin -= 0.25; lonMax += 0.25;
        cosLat = std::cos((latMin + latMax) / 2 * 3.14159265358979 / 180.0);
        const double pad = 30;
        double w = MAP_R.size.x - 2 * pad, h = MAP_R.size.y - 2 * pad;
        scale = std::min(w / ((lonMax - lonMin) * cosLat), h / (latMax - latMin));
        double cw = (lonMax - lonMin) * cosLat * scale, ch = (latMax - latMin) * scale;
        origin = sf::Vector2f(static_cast<float>(MAP_R.position.x + (MAP_R.size.x - cw) / 2), static_cast<float>((MAP_R.size.y - ch) / 2));
        pos.resize(g.size());
        for (int i = 0; i < g.size(); ++i) pos[i] = proj(g.node(i).lat, g.node(i).lon);
    }
    sf::Vector2f proj(double lat, double lon) const {
        return sf::Vector2f(static_cast<float>(origin.x + (lon - lonMin) * cosLat * scale),
                            static_cast<float>(origin.y + (latMax - lat) * scale));
    }

    // ---- text helpers
    sf::Text mk(const std::string& s, float x, float y, unsigned size, sf::Color c, bool bold = false) {
        sf::Text t(bold && fontBOk ? fontB : font);   // only called when fontOk (callers check)
        t.setString(sf::String::fromUtf8(s.begin(), s.end()));
        t.setCharacterSize(size);
        t.setFillColor(c);
        if (bold && !fontBOk) t.setStyle(sf::Text::Bold);
        t.setPosition({std::floor(x), std::floor(y)});
        return t;
    }
    void text(const std::string& s, float x, float y, unsigned size, sf::Color c, bool bold = false) {
        if (!fontOk) return;
        win.draw(mk(s, x, y, size, c, bold));
    }
    void textR(const std::string& s, float xRight, float y, unsigned size, sf::Color c, bool bold = false) {
        if (!fontOk) return;
        sf::Text t = mk(s, 0, y, size, c, bold);
        t.setPosition({std::floor(xRight - t.getLocalBounds().size.x - t.getLocalBounds().position.x), std::floor(y)});
        win.draw(t);
    }
    void textC(const std::string& s, float xCentre, float y, unsigned size, sf::Color c, bool bold = false, bool outline = false) {
        if (!fontOk) return;
        sf::Text t = mk(s, 0, y, size, c, bold);
        if (outline) { t.setOutlineColor(sf::Color(0, 0, 0, 230)); t.setOutlineThickness(2.f); }
        t.setPosition({std::floor(xCentre - (t.getLocalBounds().size.x + t.getLocalBounds().position.x) / 2), std::floor(y)});
        win.draw(t);
    }

    // ---- shape helpers
    void rect(sf::FloatRect r, sf::Color fill, sf::Color outline = sf::Color::Transparent, float ot = 0) {
        sf::RectangleShape s(r.size);
        s.setPosition(r.position);
        s.setFillColor(fill);
        s.setOutlineColor(outline);
        s.setOutlineThickness(ot);
        win.draw(s);
    }
    void line(sf::Vector2f a, sf::Vector2f b, float th, sf::Color c) {
        sf::Vector2f d = b - a;
        float len = std::sqrt(d.x * d.x + d.y * d.y);
        if (len < 0.01f) return;
        sf::Vector2f n(-d.y / len * th / 2, d.x / len * th / 2);
        sf::Vertex v[4] = {{a + n, c}, {b + n, c}, {b - n, c}, {a - n, c}};
        win.draw(v, 4, sf::PrimitiveType::TriangleFan);
    }
    void dot(sf::Vector2f p, float r, sf::Color fill, sf::Color outline = sf::Color::Transparent, float ot = 0) {
        sf::CircleShape c(r);
        c.setOrigin({r, r});
        c.setPosition(p);
        c.setFillColor(fill);
        c.setOutlineColor(outline);
        c.setOutlineThickness(ot);
        win.draw(c);
    }
    void ring(sf::Vector2f p, float r, float th, sf::Color c) { dot(p, r, sf::Color::Transparent, c, th); }
    void polyline(const std::vector<int>& path, float th, sf::Color c, size_t uptoSegments = (size_t)-1, double partial = 0) {
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (i > uptoSegments) break;
            sf::Vector2f a = pos[path[i]], b = pos[path[i + 1]];
            if (i == uptoSegments) b = a + (b - a) * static_cast<float>(partial);
            line(a, b, th, c);
            dot(a, th / 2, c);
            if (i + 1 < path.size() && i < uptoSegments) dot(pos[path[i + 1]], th / 2, c);
        }
    }

    // ---- logging
    void log(const std::string& s) {
        logLines.push_back(s);
        if (logLines.size() > 300) logLines.erase(logLines.begin(), logLines.begin() + 100);
    }

    // ---- planning
    const Plan& viewPlan() const {
        if (view == 1) return res.astar;
        if (view == 2) return res.csp;
        return res.final;
    }
    std::string viewName() const {
        if (view == 1) return "A* (shortest path only)";
        if (view == 2) return "A* + CSP (battery-feasible)";
        if (!res.anyFeasible) return "Optimised (no feasible route)";
        return "Optimised: " + res.final.name;
    }

    void recompute() {
        res = runPipeline(g, params, src, dst);
        logLines.clear();
        log("Trip " + g.node(src).name + " -> " + g.node(dst).name + " @ " + fmt(params.initialPct, 0) + "% battery");
        if (!res.connected) log("A*: no road connects these locations");
        else {
            log("A*     : " + fmt(res.astar.km) + " km, " + (res.astar.feasible ? "battery OK" : "battery NOT enough"));
            log("CSP    : " + (res.csp.feasible ? std::to_string(res.csp.stops) + " stop(s), feasible" : std::string("no feasible plan on A* route")));
            log("ACO    : " + (res.aco.feasible ? std::to_string(res.aco.stops) + " stop(s), cost " + fmt(res.aco.score, 0) : std::string("no feasible sequence")));
        }
        log("Result : " + res.status);
        view = 3;
        startAnim();
    }

    void startAnim() {
        anim = Anim();
        anim.plan = viewPlan();
        anim.battery = params.initialKwh();
        if (view == 3 && !res.anyFeasible) { anim.st = AState::NoRoute; return; }
        if (anim.plan.path.size() < 2) { anim.st = anim.plan.path.empty() ? AState::Idle : AState::Arrived; return; }
        anim.st = AState::Moving;
        sf::Vector2f d = pos[anim.plan.path[1]] - pos[anim.plan.path[0]];
        anim.angle = std::atan2(d.y, d.x) * 180.f / 3.14159265f;
        log("-- animating: " + viewName());
    }

    // ---- animation update
    void update(double dt) {
        clockT += dt;
        if (!playing) return;
        const double kmPerSec = 45.0;
        Plan& pl = anim.plan;

        if (anim.st == AState::Charging) {
            anim.chargeT += dt;
            double fr = std::min(1.0, anim.chargeT / anim.chargeDur);
            anim.battery = anim.chargeFrom + (anim.chargeTo - anim.chargeFrom) * fr;
            if (fr >= 1.0) {
                anim.battery = anim.chargeTo;
                ++anim.stopsDone;
                anim.st = AState::Moving;
                log("Charged to " + fmt(anim.battery / params.capacityKwh * 100, 0) + "% - back on the road");
            }
            return;
        }
        if (anim.st != AState::Moving) return;

        double advance = kmPerSec * speed * dt;
        while (advance > 1e-9 && anim.st == AState::Moving) {
            int a = pl.path[anim.idx], b = pl.path[anim.idx + 1];
            const Edge* e = g.findEdge(a, b);
            if (!e) { anim.st = AState::Idle; break; }
            double eseg = edgeEnergyKwh(g, params, a, b);
            double step = std::min(advance, (1.0 - anim.f) * e->km);
            anim.f += step / e->km;
            anim.dist += step;
            anim.energyUsed += eseg * step / e->km;
            advance -= step;
            anim.battery = pl.departKwh[anim.idx] - anim.f * eseg;

            if (anim.battery <= 0.0 && !pl.feasible) {
                anim.battery = 0;
                anim.st = AState::Stranded;
                log("!! BATTERY EMPTY between " + g.node(a).name + " and " + g.node(b).name);
                break;
            }
            if (anim.f >= 1.0 - 1e-9) {
                ++anim.idx;
                anim.f = 0;
                anim.battery = pl.arriveKwh[anim.idx];
                int node = pl.path[anim.idx];
                if (anim.idx >= static_cast<int>(pl.path.size()) - 1) {
                    anim.st = AState::Arrived;
                    log("ARRIVED at " + g.node(node).name + " with " + fmt(std::max(0.0, anim.battery) / params.capacityKwh * 100, 1) + "%");
                    break;
                }
                sf::Vector2f d = pos[pl.path[anim.idx + 1]] - pos[node];
                anim.angle = std::atan2(d.y, d.x) * 180.f / 3.14159265f;
                const ChargeEvent* ce = nullptr;
                for (const auto& ev : pl.charges) if (ev.pathIdx == anim.idx) ce = &ev;
                if (ce) {
                    anim.st = AState::Charging;
                    anim.chargeT = 0;
                    anim.chargeFrom = anim.battery;
                    anim.chargeTo = ce->toKwh;
                    anim.chargeDur = std::max(0.35, std::min(3.5, ce->minutes / 10.0) / std::max(0.5, speed));
                    log("CHARGING at " + g.node(node).name + ": +" + fmt(ce->addedKwh) + " kWh (Rs " + fmt(ce->cost, 0) + ", " + fmt(ce->minutes, 0) + " min)");
                    break;
                }
 if (g.node(node).isCity)
                    log("Passed " + g.node(node).name + " - battery " + fmt(anim.battery / params.capacityKwh * 100, 0) + "%");
            }
        }
    }

    // ---- UI building
    void buildButtons() {
        buttons.clear();
        auto add = [&](float x, float y, float w, float h, const std::string& l, int id, bool small = false) {
            buttons.push_back({FR(x, y, w, h), l, id, small});
        };
        add(16, 110, 30, 30, "<", B_START_PREV, true);  add(254, 110, 30, 30, ">", B_START_NEXT, true);
        add(16, 174, 30, 30, "<", B_DEST_PREV, true);   add(254, 174, 30, 30, ">", B_DEST_NEXT, true);
        add(16, 238, 30, 30, "-", B_CAP_MINUS, true);   add(254, 238, 30, 30, "+", B_CAP_PLUS, true);
        add(16, 302, 30, 30, "-", B_BAT_MINUS, true);   add(254, 302, 30, 30, "+", B_BAT_PLUS, true);
        add(16, 344, 130, 30, "RUN / RESTART", B_RUN);  add(154, 344, 130, 30, "PAUSE", B_PLAY);
        add(16, 398, 30, 28, "-", B_SPEED_MINUS, true); add(254, 398, 30, 28, "+", B_SPEED_PLUS, true);
        add(16, 466, 62, 30, "A*", B_VIEW1);            add(84, 466, 80, 30, "A*+CSP", B_VIEW2);
        add(170, 466, 114, 30, "A*+CSP+ACO", B_VIEW3);
        const auto& sc = defaultScenarios();
        for (size_t i = 0; i < sc.size(); ++i)
            add(16, 540 + 33.f * i, 268, 29, std::to_string(i + 1) + "   " + sc[i].title, B_SCEN0 + static_cast<int>(i));
        add(16, 710, 268, 26, "RESET ALL STATIONS TO AVAILABLE", B_RESET_STATIONS, true);
    }

    int stepCity(int cur, int dir) const {
        if (cities.empty()) return cur;
        int k = -1;
        for (size_t i = 0; i < cities.size(); ++i) if (cities[i] == cur) k = static_cast<int>(i);
        if (k < 0) return dir > 0 ? cities.front() : cities.back();
        k = (k + dir + static_cast<int>(cities.size())) % static_cast<int>(cities.size());
        return cities[k];
    }

    void resetStations() {
        for (int i = 0; i < g.size(); ++i) g.node(i).available = origAvail[i] != 0;
    }

    void scenarioPreset(int i) {
        const Scenario& sc = defaultScenarios().at(i);
        resetStations();
        params.initialPct = sc.batteryPct;
        int s = g.findNode(sc.src), d = g.findNode(sc.dst);
        if (s < 0 || d < 0) return;
        src = s; dst = d;
        std::string blockedMsg;
        if (sc.blockFirstStop) {
            PipelineResult base = runPipeline(g, params, src, dst);
            if (!base.final.charges.empty()) {
                int b = base.final.charges.front().node;
                g.node(b).available = false;
                blockedMsg = g.node(b).name + " is UNAVAILABLE - finding an alternative";
            }
        }
        scenarioActive = i;
        recompute();
        log("Scenario " + std::to_string(i + 1) + ": " + sc.title);
        log("Expected: " + sc.expectation);
        if (!blockedMsg.empty()) log(blockedMsg);
    }

    void onButton(int id) {
        bool shift = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift);
        auto changed = [&]() { scenarioActive = -1; recompute(); };
        switch (id) {
            case B_START_PREV: src = stepCity(src, -1); changed(); break;
            case B_START_NEXT: src = stepCity(src, +1); changed(); break;
            case B_DEST_PREV: dst = stepCity(dst, -1); changed(); break;
            case B_DEST_NEXT: dst = stepCity(dst, +1); changed(); break;
            case B_CAP_MINUS: params.capacityKwh = std::max(20.0, params.capacityKwh - (shift ? 1 : 5)); changed(); break;
            case B_CAP_PLUS: params.capacityKwh = std::min(120.0, params.capacityKwh + (shift ? 1 : 5)); changed(); break;
            case B_BAT_MINUS: params.initialPct = std::max(1.0, params.initialPct - (shift ? 1 : 5)); changed(); break;
            case B_BAT_PLUS: params.initialPct = std::min(100.0, params.initialPct + (shift ? 1 : 5)); changed(); break;
            case B_RUN: playing = true; recompute(); break;
            case B_PLAY: playing = !playing; break;
            case B_SPEED_MINUS: speed = std::max(0.25, speed / 2); break;
            case B_SPEED_PLUS: speed = std::min(8.0, speed * 2); break;
            case B_RESET_STATIONS: resetStations(); changed(); break;
            case B_VIEW1: case B_VIEW2: case B_VIEW3: view = 1 + (id - B_VIEW1); playing = true; startAnim(); break;
            default:
                if (id >= B_SCEN0 && id < B_SCEN0 + 5) { playing = true; scenarioPreset(id - B_SCEN0); }
        }
    }

    int nodeAt(sf::Vector2i m, float radius) const {
        if (!MAP_R.contains(sf::Vector2f(static_cast<float>(m.x), static_cast<float>(m.y)))) return -1;
        int best = -1;
        float bd = radius * radius;
        for (int i = 0; i < g.size(); ++i) {
            float dx = pos[i].x - m.x, dy = pos[i].y - m.y, d = dx * dx + dy * dy;
            if (d <= bd) { bd = d; best = i; }
        }
        return best;
    }

    void handleEvent(const sf::Event& ev) {
        using K = sf::Keyboard::Key;
        if (ev.is<sf::Event::Closed>()) win.close();
        else if (const auto* mm = ev.getIf<sf::Event::MouseMoved>()) mouse = mm->position;
        else if (const auto* kp = ev.getIf<sf::Event::KeyPressed>()) {
            switch (kp->code) {
                case K::Escape: win.close(); break;
                case K::Space: playing = !playing; break;
                case K::R: playing = true; recompute(); break;
                case K::Num1: onButton(B_VIEW1); break;
                case K::Num2: onButton(B_VIEW2); break;
                case K::Num3: onButton(B_VIEW3); break;
                case K::Up: onButton(B_SPEED_PLUS); break;
                case K::Down: onButton(B_SPEED_MINUS); break;
                default: break;
            }
        } else if (const auto* mb = ev.getIf<sf::Event::MouseButtonPressed>()) {
            sf::Vector2i m = mb->position;
            bool shift = sf::Keyboard::isKeyPressed(K::LShift) || sf::Keyboard::isKeyPressed(K::RShift);
            if (mb->button == sf::Mouse::Button::Left) {
                for (const auto& b : buttons)
                    if (b.r.contains(sf::Vector2f(static_cast<float>(m.x), static_cast<float>(m.y)))) { onButton(b.id); return; }
            }
            int n = nodeAt(m, 16.f);
            if (n >= 0) {
                if (mb->button == sf::Mouse::Button::Left && shift) {
                    if (g.node(n).charger) {
                        g.node(n).available = !g.node(n).available;
                        scenarioActive = -1;
                        recompute();
                        log(g.node(n).name + (g.node(n).available ? " is available again" : " marked UNAVAILABLE"));
                    }
                } else if (mb->button == sf::Mouse::Button::Left) { src = n; scenarioActive = -1; recompute(); }
                else if (mb->button == sf::Mouse::Button::Right) { dst = n; scenarioActive = -1; recompute(); }
            }
        }
    }

    // ================================================================= drawing
    void draw() {
        win.clear(C_BG);
        drawMap();
        drawLeft();
        drawRight();
        drawTooltip();
    }

    void drawMap() {
        sf::View v(MAP_R);
        v.setViewport(FR(MAP_R.position.x / WIN_W, MAP_R.position.y / WIN_H, MAP_R.size.x / WIN_W, MAP_R.size.y / WIN_H));
        win.setView(v);

        // land
        if (haveBg) {
            sf::Sprite bg(texBg);
            bg.setPosition(MAP_R.position);
            bg.setScale({MAP_R.size.x / texBg.getSize().x, MAP_R.size.y / texBg.getSize().y});
            win.draw(bg);
        } else rect(MAP_R, sf::Color(32, 42, 50));

        // Arabian Sea (west coast, real lat/lon)
        static const double coastKerala[][2] = {{12.70, 74.85}, {12.50, 74.90}, {12.10, 75.15}, {11.87, 75.30}, {11.50, 75.55},
                                                {11.26, 75.74}, {10.78, 75.90}, {10.50, 76.05}, {10.23, 76.17}, {9.93, 76.22},
                                                {9.49, 76.30}, {9.10, 76.45}, {8.89, 76.55}, {8.52, 76.90}, {8.40, 76.97}, {8.10, 77.40}};
        const double (*coast)[2] = coastKerala;                 // the shipped dataset is Kerala
        const size_t coastN = sizeof(coastKerala) / sizeof(coastKerala[0]);
        float leftX = MAP_R.position.x - 5;
        for (size_t i = 0; i + 1 < coastN; ++i) {
            sf::Vector2f a = proj(coast[i][0], coast[i][1]), b = proj(coast[i + 1][0], coast[i + 1][1]);
            sf::Vertex q[4] = {{{leftX, a.y}, C_SEA}, {a, C_SEA}, {b, C_SEA}, {{leftX, b.y}, C_SEA}};
            win.draw(q, 4, sf::PrimitiveType::TriangleFan);
            line(a, b, 2.f, sf::Color(90, 140, 190, 200));
        }
        if (fontOk) {
            sf::Text t = mk("ARABIAN  SEA", 0, 0, 13, sf::Color(70, 110, 160));
            t.setRotation(sf::degrees(-90));
            t.setPosition({MAP_R.position.x + 30, proj((latMin + latMax) / 2, 0).y});
            win.draw(t);
        }

        // roads
        for (int a = 0; a < g.size(); ++a)
            for (const Edge& e : g.adj(a))
                if (a < e.to) line(pos[a], pos[e.to], e.terrain > 1.1 ? 2.6f : 3.2f, e.terrain > 1.1 ? sf::Color(110, 92, 74) : C_ROAD);

        // route overlays
        if (res.connected) {
            polyline(res.astar.path, 3.f, withAlpha(C_ORANGE, 120));
            if (res.anyFeasible && res.final.path != res.astar.path) polyline(res.final.path, 3.f, withAlpha(C_GREEN, 130));
            sf::Color vc = view == 1 ? C_ORANGE : (view == 2 ? C_ACC : C_GREEN);
            if (anim.st != AState::NoRoute) {
                polyline(anim.plan.path, 8.f, withAlpha(vc, 70));
                if (anim.plan.path.size() > 1)
                    polyline(anim.plan.path, 5.f, vc, static_cast<size_t>(anim.idx), anim.f);
            }
        }

        // planned charging stops
        if (anim.st != AState::NoRoute)
            for (size_t k = 0; k < anim.plan.charges.size(); ++k) {
                sf::Vector2f p = pos[anim.plan.charges[k].node];
                ring(p, 15, 2.5f, C_YEL);
                dot(p + sf::Vector2f(13, -13), 8, C_YEL);
                textC(std::to_string(k + 1), p.x + 13, p.y - 21, 11, sf::Color(30, 30, 20), true);
            }

        // nodes
        std::vector<char> onPath(g.size(), 0);   // labelled: planned charging stops of the route being shown
        if (anim.st != AState::NoRoute) for (const auto& ev : anim.plan.charges) onPath[ev.node] = 1;
        for (int i = 0; i < g.size(); ++i) {
            const Node& n = g.node(i);
            sf::Vector2f p = pos[i];
            if (n.charger) {
                if (haveStation) {
                    sf::Sprite sp(texStation);
                    sp.setOrigin({texStation.getSize().x / 2.f, texStation.getSize().y / 2.f});
                    sp.setPosition(p);
                    float s = (n.isCity ? 22.f : 13.f) / texStation.getSize().x;
                    sp.setScale({s, s});
                    if (!n.available) sp.setColor(sf::Color(120, 120, 120));
                    win.draw(sp);
                } else dot(p, 7, n.available ? C_GREEN : sf::Color(120, 120, 120), sf::Color::White, 1.5f);
                if (!n.available) {
                    line(p + sf::Vector2f(-8, -8), p + sf::Vector2f(8, 8), 3, C_RED);
                    line(p + sf::Vector2f(-8, 8), p + sf::Vector2f(8, -8), 3, C_RED);
                }
            } else {
                dot(p, 6, sf::Color(230, 234, 242), sf::Color(20, 24, 32), 2);
            }
            bool showLabel = n.isCity || onPath[i];
            if (showLabel) {
                std::string label = n.name;
                if (!n.isCity && label.size() > 4 && label.substr(label.size() - 4) == " Hub") label.resize(label.size() - 4);
                text(label, p.x + 11, p.y - 8, n.isCity ? 12 : 10, n.isCity ? C_TXT : sf::Color(190, 200, 215), n.isCity);
            }
        }

        // start / destination markers
        float pulse = 1.0f + 0.12f * static_cast<float>(std::sin(clockT * 4));
        ring(pos[src], 14 * pulse, 3, C_ACC);
        text("START", pos[src].x - 17, pos[src].y + 14, 10, C_ACC, true);
        ring(pos[dst], 14 * pulse, 3, C_RED);
        text("END", pos[dst].x - 10, pos[dst].y + 14, 10, C_RED, true);

        drawCar();
        drawMapOverlay();
        win.setView(win.getDefaultView());
    }

    void drawCar() {
        if (anim.plan.path.empty()) return;
        sf::Vector2f p;
        if (anim.st == AState::NoRoute || anim.plan.path.size() < 2 || anim.idx >= static_cast<int>(anim.plan.path.size()) - 1)
            p = pos[anim.plan.path[std::min<size_t>(anim.idx, anim.plan.path.size() - 1)]];
        else
            p = pos[anim.plan.path[anim.idx]] + (pos[anim.plan.path[anim.idx + 1]] - pos[anim.plan.path[anim.idx]]) * static_cast<float>(anim.f);
        if (anim.st == AState::NoRoute) p = pos[src];

        if (anim.st == AState::Charging) {
            for (int k = 0; k < 3; ++k) {
                float ph = static_cast<float>(std::fmod(clockT * 1.6 + k / 3.0, 1.0));
                ring(p, 12 + 26 * ph, 2.5f, withAlpha(C_YEL, static_cast<std::uint8_t>(220 * (1 - ph))));
            }
        }
        if (haveCar) {
            sf::Sprite s(texCar);
            s.setOrigin({texCar.getSize().x / 2.f, texCar.getSize().y / 2.f});
            s.setPosition(p);
            s.setRotation(sf::degrees(anim.angle));
            float sc = 40.f / texCar.getSize().x;
            s.setScale({sc, sc});
            win.draw(s);
        } else {
            sf::CircleShape tri(10, 3);
            tri.setOrigin({10, 10});
            tri.setPosition(p);
            tri.setRotation(sf::degrees(anim.angle + 90));
            tri.setFillColor(C_ACC);
            win.draw(tri);
        }
        if (anim.st == AState::Stranded) {
            line(p + sf::Vector2f(-14, -14), p + sf::Vector2f(14, 14), 4, C_RED);
            line(p + sf::Vector2f(-14, 14), p + sf::Vector2f(14, -14), 4, C_RED);
            textC("OUT OF BATTERY", p.x, p.y - 34, 12, C_RED, true);
        }
        if (anim.st == AState::Charging) textC("CHARGING", p.x, p.y - 34, 12, C_YEL, true);
    }

    void drawMapOverlay() {
        // title strip
        rect(FR(MAP_R.position.x, 0, MAP_R.size.x, 34), withAlpha(C_PANEL, 220));
        textC(g.node(src).name + "  ->  " + g.node(dst).name, MAP_R.position.x + MAP_R.size.x / 2, 7, 16, C_TXT, true);

        // legend
        bool hilly = false;
        for (int a = 0; a < g.size() && !hilly; ++a) for (const Edge& e : g.adj(a)) if (e.terrain > 1.1) { hilly = true; break; }
        float lh = hilly ? 118.f : 100.f;
        float lx = MAP_R.position.x + 10, ly = WIN_H - lh - 10;
        rect(FR(lx, ly, 150, lh), withAlpha(C_PANEL, 215), sf::Color(60, 70, 90), 1);
        dot({lx + 14, ly + 14}, 5, sf::Color(230, 234, 242), sf::Color(20, 24, 32), 2); text("District town (city)", lx + 30, ly + 6, 11, C_TXT);
        dot({lx + 14, ly + 32}, 6, C_GREEN, sf::Color::White, 1.5f); text("Charging hub", lx + 30, ly + 24, 11, C_TXT);
        line({lx + 6, ly + 50}, {lx + 22, ly + 50}, 3, C_ORANGE); text("A* shortest route", lx + 30, ly + 42, 11, C_TXT);
        line({lx + 6, ly + 68}, {lx + 22, ly + 68}, 3, C_GREEN); text("Optimised route", lx + 30, ly + 60, 11, C_TXT);
        ring({lx + 14, ly + 86}, 6, 2, C_YEL); text("Planned charge stop", lx + 30, ly + 78, 11, C_TXT);
        if (hilly) { line({lx + 6, ly + 104}, {lx + 22, ly + 104}, 2.6f, sf::Color(110, 92, 74)); text("Ghat / hilly road", lx + 30, ly + 96, 11, C_TXT); }

        // no-route banner
        if (anim.st == AState::NoRoute) {
            float bw = 400, bh = 116, bx = MAP_R.position.x + (MAP_R.size.x - bw) / 2, by = 330;
            rect(FR(bx, by, bw, bh), sf::Color(60, 18, 22, 235), C_RED, 2);
            textC("NO FEASIBLE ROUTE", bx + bw / 2, by + 12, 24, C_RED, true);
            auto lines = wrap(res.connected ? "No charging station or destination is reachable with this battery. Raise the battery or pick another start. Keys 1 / 2 show where the plain A* route would strand the EV."
                                            : "No road connects these two locations.", 56);
            for (size_t i = 0; i < lines.size() && i < 4; ++i) textC(lines[i], bx + bw / 2, by + 46 + 16.f * i, 12, C_TXT);
        }
    }

    // ---- panels
    void button(const Button& b, const std::string& label, bool active, bool highlight = false) {
        bool hov = b.r.contains(sf::Vector2f(static_cast<float>(mouse.x), static_cast<float>(mouse.y)));
        sf::Color fill = active ? sf::Color(40, 92, 160) : (hov ? sf::Color(58, 68, 90) : C_PANEL2);
        if (highlight) fill = sf::Color(36, 110, 82);
        rect(b.r, fill, active ? C_ACC : sf::Color(64, 74, 96), 1);
        textC(label, b.r.position.x + b.r.size.x / 2, b.r.position.y + (b.r.size.y - 14) / 2 - 1, 12, C_TXT, !b.small);
    }

    void field(float y, const std::string& label, const std::string& value) {
        text(label, 18, y - 20, 12, C_DIM);
        rect(FR(52, y, 196, 30), sf::Color(16, 20, 28), sf::Color(64, 74, 96), 1);
        textC(value, 150, y + 6, 14, C_TXT, true);
    }

    void drawLeft() {
        rect(LEFT_R, C_PANEL);
        text("EV ChargeRoute", 16, 12, 22, C_TXT, true);
        text("A* + CSP + ACO   |   2D EV simulation", 16, 42, 11, C_DIM);
        text("USER INPUT PANEL", 16, 72, 12, C_ACC, true);

        field(110, "Starting location", g.node(src).name);
        field(174, "Destination", g.node(dst).name);
        field(238, "Battery capacity", fmt(params.capacityKwh, 0) + " kWh");
        field(302, "Current battery", fmt(params.initialPct, 0) + "%   (" + fmt(params.initialKwh()) + " kWh)");
        text("Simulation speed", 18, 379, 12, C_DIM);
        rect(FR(52, 398, 196, 28), sf::Color(16, 20, 28), sf::Color(64, 74, 96), 1);
        textC(fmt(speed, 2) + "x", 150, 403, 14, C_TXT, true);

        text("VIEW  (keys 1 / 2 / 3)", 16, 440, 12, C_ACC, true);
        text("Click map: left = start, right = destination, Shift+left = toggle station", 16, 502, 10, C_DIM);
        text("TEST SCENARIOS", 16, 518, 12, C_ACC, true);

        for (const auto& b : buttons) {
            std::string label = b.label;
            bool active = false;
            if (b.id == B_PLAY) label = playing ? "PAUSE" : "PLAY";
            if (b.id >= B_VIEW1 && b.id <= B_VIEW3) active = (view == 1 + (b.id - B_VIEW1));
            if (b.id >= B_SCEN0 && b.id < B_SCEN0 + 5) active = (scenarioActive == b.id - B_SCEN0);
            button(b, label, active, b.id == B_RUN);
        }
        text("Space = pause   R = rerun   Up/Down = speed", 16, 744, 10, C_DIM);
        text("Reserve " + fmt(params.reservePct, 0) + "% | " + fmt(params.consumptionKwhPerKm, 2) + " kWh/km base", 16, 758, 10, C_DIM);
    }

    sf::Color batteryColor(double pct) const { return pct > 40 ? C_GREEN : (pct > 20 ? C_YEL : C_RED); }

    void drawRight() {
        rect(RIGHT_R, C_PANEL);
        const float X = 898, W = 384;
        text("LIVE INFORMATION", X, 12, 14, C_ACC, true);

        double pct = std::max(0.0, anim.battery) / params.capacityKwh * 100.0;
        text("Battery level", X, 38, 12, C_DIM);
        rect(FR(X, 58, W, 26), sf::Color(14, 18, 24), sf::Color(70, 80, 100), 1);
        rect(FR(X, 58, static_cast<float>(W * std::min(1.0, pct / 100.0)), 26), batteryColor(pct));
        float rx = X + static_cast<float>(W * params.reservePct / 100.0);
        line({rx, 55}, {rx, 87}, 2, C_RED);
        textC(fmt(pct, 1) + "%   |   " + fmt(std::max(0.0, anim.battery)) + " / " + fmt(params.capacityKwh, 0) + " kWh", X + W / 2, 62, 14, sf::Color::White, true, true);
        text("reserve", rx - 18, 88, 9, C_RED);

        std::string status;
        sf::Color sc = C_TXT;
        switch (anim.st) {
            case AState::Moving: status = res.anyFeasible || view != 3 ? (anim.plan.feasible ? "FEASIBLE - driving" : "DRIVING (battery insufficient)") : "DRIVING"; sc = anim.plan.feasible ? C_GREEN : C_ORANGE; break;
            case AState::Charging: status = "CHARGING at " + g.node(anim.plan.path[anim.idx]).name; sc = C_YEL; break;
            case AState::Arrived: status = anim.plan.feasible ? "ARRIVED - " + res.status.substr(res.status.find('-') == std::string::npos ? 0 : res.status.find('-') + 2) : "ARRIVED (below reserve)"; sc = C_GREEN; break;
            case AState::Stranded: status = "STRANDED - battery empty"; sc = C_RED; break;
            case AState::NoRoute: status = "NO FEASIBLE ROUTE"; sc = C_RED; break;
            default: status = "idle"; break;
        }
        struct Row { std::string k, v; sf::Color c; };
        std::string nextName = (anim.st == AState::Moving && anim.idx + 1 < static_cast<int>(anim.plan.path.size())) ? g.node(anim.plan.path[anim.idx + 1]).name : "-";
        std::vector<Row> rows = {
            {"Distance travelled", fmt(anim.dist) + " / " + fmt(anim.plan.km) + " km", C_TXT},
            {"Energy used", fmt(anim.energyUsed) + " / " + fmt(anim.plan.energyKwh) + " kWh", C_TXT},
            {"Charging stops", std::to_string(anim.stopsDone) + " / " + std::to_string(anim.plan.stops), C_TXT},
            {"Next location", nextName, C_TXT},
            {"Route status", status, sc},
            {"Showing", viewName(), C_ACC},
        };
        float y = 108;
        for (const auto& r : rows) {
            text(r.k, X, y, 13, C_DIM);
            textR(r.v, X + W, y, 13, r.c, true);
            y += 24;
        }

        // ---- comparison table
        y = 262;
        text("ALGORITHM COMPARISON", X, y, 13, C_ACC, true);
        y += 26;
        const float cx[] = {X + 20, X + 196, X + 246, X + 292, X + 336, X + 384};
        text("Method", cx[0], y, 11, C_DIM);
        textR("km", cx[1], y, 11, C_DIM); textR("kWh", cx[2], y, 11, C_DIM); textR("Stops", cx[3], y, 11, C_DIM);
        textR("Cost", cx[4], y, 11, C_DIM); textR("ms", cx[5], y, 11, C_DIM);
        y += 20;
        const Plan* plans[3] = {&res.astar, &res.csp, &res.aco};
        for (int i = 0; i < 3; ++i) {
            const Plan& pl = *plans[i];
            bool sel = (view == 3) ? (res.finalWhich == i + 1) : (view == i + 1);   // highlight only the route being animated
            if (sel) rect(FR(X - 6, y - 3, W + 12, 24), sf::Color(44, 56, 80), C_ACC, 1);
            dot({X + 6, y + 9}, 5, pl.feasible ? C_GREEN : C_RED);
            text(pl.name, cx[0], y, 12, C_TXT, sel);
            bool ok = res.connected;
            textR(ok ? fmt(pl.km) : "-", cx[1], y, 12, C_TXT);
            textR(ok ? fmt(pl.energyKwh) : "-", cx[2], y, 12, C_TXT);
            textR(ok ? std::to_string(pl.stops) : "-", cx[3], y, 12, C_TXT);
            textR(ok ? fmt(pl.score, 0) : "-", cx[4], y, 12, C_TXT);
            textR(ok ? fmt(pl.ms, 2) : "-", cx[5], y, 12, C_TXT);
            y += 28;
        }
        text("green dot = battery-feasible, red = infeasible.  Highlighted = route being animated.", X, y - 2, 9, C_DIM);
        y += 18;
        text("Cost = km + 0.1*Rs + 10*stops + 0.2*charging-min", X, y - 2, 9, C_DIM);

        // ---- plan details
        y += 22;
        text("SELECTED PLAN", X, y, 13, C_ACC, true);
        y += 22;
        const Plan& vp = viewPlan();
        for (const auto& ln : wrap(vp.note, 62)) { text(ln, X, y, 11, vp.feasible ? C_TXT : C_ORANGE); y += 15; }
        if (res.connected && !vp.charges.empty()) {
            y += 3;
            for (size_t k = 0; k < vp.charges.size() && k < 4; ++k) {
                const ChargeEvent& ev = vp.charges[k];
                std::string nm = g.node(ev.node).name;
                text(std::to_string(k + 1) + ". " + nm, X, y, 11, C_YEL, true);
                textR(fmt(ev.arriveKwh) + " -> " + fmt(ev.toKwh) + " kWh   Rs " + fmt(ev.cost, 0) + "   " + fmt(ev.minutes, 0) + " min", X + W, y, 11, C_TXT);
                y += 17;
            }
        }

        // ---- event log
        float ly = 586;
        rect(FR(X - 8, ly - 8, W + 16, WIN_H - ly - 4), sf::Color(18, 22, 30), sf::Color(50, 60, 80), 1);
        text("EVENT LOG", X, ly - 2, 12, C_ACC, true);
        size_t maxLines = 8;
        size_t start = logLines.size() > maxLines ? logLines.size() - maxLines : 0;
        float yy = ly + 20;
        for (size_t i = start; i < logLines.size(); ++i) {
            std::string s = logLines[i];
            if (s.size() > 62) s = s.substr(0, 61) + "~";
            sf::Color c = (s.rfind("!!", 0) == 0 || s.find("NO FEASIBLE") != std::string::npos) ? C_RED : (s.rfind("CHARGING", 0) == 0 || s.rfind("Charged", 0) == 0 ? C_YEL : (s.rfind("ARRIVED", 0) == 0 || s.rfind("Result", 0) == 0 ? C_GREEN : C_TXT));
            text(s, X, yy, 11, i + 1 == logLines.size() ? c : withAlpha(c, 190));
            yy += 18;
        }
    }

    void drawTooltip() {
        int n = nodeAt(mouse, 14.f);
        if (n < 0) return;
        const Node& nd = g.node(n);
        std::vector<std::string> lines;
        lines.push_back(nd.name);
        lines.push_back(nd.isCity ? "City" : "Charging station hub");
        if (nd.charger) {
            lines.push_back(std::string(nd.available ? "Charger: available" : "Charger: UNAVAILABLE") + "  " + fmt(nd.powerKw, 1) + " kW");
            lines.push_back(std::to_string(nd.stations) + (nd.stations == 1 ? " station in this hub" : " stations in this hub"));
            if (!nd.operators.empty()) lines.push_back(nd.operators.size() > 28 ? nd.operators.substr(0, 27) + "~" : nd.operators);
            lines.push_back("Price: Rs " + fmt(nd.price, 0) + " / kWh (assumed)");
            lines.push_back("Shift+click to toggle");
        } else lines.push_back("No charger here");
        float w = 220, h = 8 + 17.f * lines.size();
        float x = std::min<float>(static_cast<float>(mouse.x) + 14, WIN_W - w - 4), y = std::min<float>(static_cast<float>(mouse.y) + 14, WIN_H - h - 4);
        rect(FR(x, y, w, h), sf::Color(10, 12, 18, 240), C_ACC, 1);
        for (size_t i = 0; i < lines.size(); ++i) text(lines[i], x + 8, y + 5 + 17.f * i, 12, i == 0 ? C_TXT : C_DIM, i == 0);
    }
};

}  // namespace

int runSimulation(Graph& g, const Params& params, int src, int dst, const std::string& assetsDir,
                  int shotFrames, const std::string& shotPath, int viewMode, int scenario) {
    App app(g, params, src, dst, assetsDir);
    if (scenario >= 1 && scenario <= 5) app.applyScenario(scenario - 1);
    if (viewMode >= 1 && viewMode <= 3 && viewMode != 3) app.setView(viewMode);
    if (shotFrames > 0 && !shotPath.empty()) return app.runShot(shotFrames, shotPath);
    return app.run();
}

#endif  // EV_NO_GUI
