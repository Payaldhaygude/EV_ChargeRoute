#!/usr/bin/env python3
"""
EV ChargeRoute - build the road network from the REAL Kaggle dataset.

    python3 tools/build_dataset.py
        reads  dataset/raw/Indian_EV_Stations_Simplified.csv   (Kaggle: EV Charging Stations in India, simplified 2025)
        writes dataset/ev_data.csv      nodes  (charging hubs + cities)
               dataset/roads.csv        edges  (road network)
               dataset/scenarios.csv    the 5 test scenarios, picked from the data

What comes from the Kaggle file (real):  station names, cities, coordinates, operators, kW ratings.
What the file does NOT contain, so it is derived / assumed (stated in README):
  * roads            -> Gabriel graph over the hub coordinates, road length = straight-line x ROAD_FACTOR
  * price per kWh    -> assumed by charger power tier (PRICE_TIERS)
  * terrain          -> 1.0 everywhere (no elevation data)

Steps: 1 clean (state typos, whitespace, bad coordinates)  2 keep Kerala  3 merge stations closer than
--radius km into one hub  4 name hubs by their most common city  5 mark Kerala's district towns as CITY nodes
6 build roads  7 write the CSVs + test scenarios.
"""
import argparse
import csv
import difflib
import math
import os
import collections

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
RAW_DEFAULT = os.path.join(ROOT, "dataset", "raw", "Indian_EV_Stations_Simplified.csv")
OUT = os.path.join(ROOT, "dataset")

ROAD_FACTOR = 1.30          # winding Kerala roads: road km = straight-line km x 1.30 (>= 1, so the A* heuristic stays admissible)
MAX_EDGE_KM = 60.0          # longest straight-line road segment kept from the Gabriel graph
PRICE_TIERS = [(50, 22), (22, 18), (15, 15), (0, 12)]   # (min kW, INR per kWh)  - ASSUMPTION, not in the dataset
KERALA_BOX = (8.1, 12.9, 74.7, 77.6)                      # lat_min, lat_max, lon_min, lon_max


def haversine(a, b):
    r = 6371.0088
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def clean_city(s):
    return " ".join((s or "").replace(" ", " ").split()).title()


def is_kerala(state):
    s = (state or "").strip().lower()
    if not s:
        return None
    return difflib.SequenceMatcher(None, s, "kerala").ratio() >= 0.7   # catches Keral, Keraka, Lerala


def price_for(kw):
    for lo, price in PRICE_TIERS:
        if kw >= lo:
            return price
    return PRICE_TIERS[-1][1]


def load_stations(path, log):
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    log(f"raw rows                      : {len(rows)}")
    out, dropped = [], collections.Counter()
    for r in rows:
        try:
            lat, lon = float(r["Latitude"]), float(r["Longitude"])
        except (TypeError, ValueError, KeyError):
            dropped["bad coordinates"] += 1
            continue
        inside = KERALA_BOX[0] < lat < KERALA_BOX[1] and KERALA_BOX[2] < lon < KERALA_BOX[3]
        k = is_kerala(r.get("State"))
        if not inside:
            dropped["outside Kerala bounding box"] += 1
            continue
        if k is False:
            dropped["other state (Tamil Nadu / Karnataka)"] += 1
            continue
        try:
            kw = float(r.get("Power (kW)") or 0)
        except ValueError:
            kw = 0.0
        out.append(dict(name=(r.get("Station Name") or "").strip(), city=clean_city(r.get("City")), lat=lat, lon=lon,
                        op=(r.get("Operator") or "").strip(), kw=kw))
    for k, v in dropped.items():
        log(f"  dropped - {k:<30}: {v}")
    log(f"stations kept (Kerala)        : {len(out)}")
    return out


def cluster(stations, radius, log):
    """Greedy leader clustering: densest areas first, every station joins the first leader within `radius` km."""
    pts = [(s["lat"], s["lon"]) for s in stations]
    n = len(pts)
    dens = [sum(1 for j in range(n) if haversine(pts[i], pts[j]) < radius) for i in range(n)]
    leaders, member_of = [], {}
    for i in sorted(range(n), key=lambda i: (-dens[i], i)):
        for li, L in enumerate(leaders):
            if haversine(pts[i], pts[L]) < radius:
                member_of[i] = li
                break
        else:
            leaders.append(i)
            member_of[i] = len(leaders) - 1
    groups = collections.defaultdict(list)
    for i, li in member_of.items():
        groups[li].append(i)
    hubs = []
    for li, members in groups.items():
        ms = [stations[i] for i in members]
        lat = sum(m["lat"] for m in ms) / len(ms)
        lon = sum(m["lon"] for m in ms) / len(ms)
        city = collections.Counter(m["city"] for m in ms if m["city"]).most_common(1)
        ops = collections.Counter(m["op"] for m in ms if m["op"] and "Unknown" not in m["op"]).most_common(2)
        hubs.append(dict(lat=lat, lon=lon, city=city[0][0] if city else "Hub", count=len(ms),
                         kw=max(m["kw"] for m in ms), ops="/".join(o.replace(",", " ") for o, _ in ops)))
    log(f"charging hubs (radius {radius:g} km)  : {len(hubs)}")
    return hubs


# Selectable start / destination towns: Kerala's district towns (+ Munnar). Only those that exist in the dataset are used.
CITY_NAMES = ["Kasaragod", "Kannur", "Kalpetta", "Kozhikode", "Malappuram", "Palakkad", "Thrissur", "Ernakulam",
              "Munnar", "Thodupuzha", "Kottayam", "Alappuzha", "Pathanamthitta", "Kollam", "Thiruvananthapuram"]


def compass(dlat, dlon):
    ang = math.degrees(math.atan2(dlat, dlon)) % 360
    return ["E", "NE", "N", "NW", "W", "SW", "S", "SE"][int((ang + 22.5) // 45) % 8]


def name_and_type(hubs, log):
    hubs.sort(key=lambda h: (-h["count"], h["city"]))
    first = {}
    used = collections.defaultdict(set)
    for h in hubs:                                   # biggest hub keeps the plain name
        c = h["city"]
        if c not in first:
            first[c] = h
            h["name"] = c
            used[c].add(c)
            continue
        base = f"{c} ({compass(h['lat'] - first[c]['lat'], h['lon'] - first[c]['lon'])})"
        name, k = base, 2
        while name in used[c]:
            name, k = f"{base} {k}", k + 1
        used[c].add(name)
        h["name"] = name
    for h in hubs:
        h["type"] = "CITY" if h["name"] in CITY_NAMES else "STATION"
    hubs.sort(key=lambda h: (-h["lat"], h["lon"]))   # stable, geographic ids: north -> south
    for i, h in enumerate(hubs):
        h["id"] = i
        h["price"] = price_for(h["kw"])
    log("CITY nodes                   : " + ", ".join(h["name"] for h in hubs if h["type"] == "CITY"))


def build_roads(hubs, log):
    n = len(hubs)
    lat0 = sum(h["lat"] for h in hubs) / n
    kx = 111.32 * math.cos(math.radians(lat0))
    xy = [(h["lon"] * kx, h["lat"] * 110.57) for h in hubs]
    d = [[math.hypot(xy[i][0] - xy[j][0], xy[i][1] - xy[j][1]) for j in range(n)] for i in range(n)]
    edges = set()
    for i in range(n):
        for j in range(i + 1, n):
            if d[i][j] > MAX_EDGE_KM:
                continue
            dij2 = d[i][j] ** 2
            # Gabriel condition: no third hub inside the circle that has i-j as diameter
            if all(d[i][k] ** 2 + d[j][k] ** 2 >= dij2 for k in range(n) if k != i and k != j):
                edges.add((i, j))
    par = list(range(n))

    def find(x):
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x
    for a, b in edges:
        par[find(a)] = find(b)
    added = 0
    while len({find(i) for i in range(n)}) > 1:      # connect any isolated groups with the shortest bridge
        best = min(((d[i][j], i, j) for i in range(n) for j in range(i + 1, n) if find(i) != find(j)))
        _, i, j = best
        edges.add((i, j))
        par[find(i)] = find(j)
        added += 1
    roads = [[a, b, round(haversine((hubs[a]["lat"], hubs[a]["lon"]), (hubs[b]["lat"], hubs[b]["lon"])) * ROAD_FACTOR, 1),
              1.0, "Road"] for a, b in sorted(edges)]
    log(f"roads (Gabriel graph)         : {len(roads)}  (+{added} bridge edges to connect the network)")
    return roads


def pick_scenarios(hubs, roads, log):
    """Pick the 5 roadmap test scenarios from the data (CITY nodes only), using approximate road distance."""
    cities = [h for h in hubs if h["type"] == "CITY"]
    approx = lambda a, b: haversine((a["lat"], a["lon"]), (b["lat"], b["lon"])) * ROAD_FACTOR
    pairs = [(approx(a, b), a, b) for a in cities for b in cities if a["id"] < b["id"]]

    def closest(target):
        return min(pairs, key=lambda p: abs(p[0] - target))
    short = closest(70)
    mid = closest(300)
    near = closest(150)
    far = max(pairs, key=lambda p: p[0])
    sc = [
        ("High battery", short[1]["name"], short[2]["name"], 90, 0, "Direct route possible"),
        ("Low battery", mid[1]["name"], mid[2]["name"], 60, 0, "Charging stop required"),
        ("Very low battery", near[1]["name"], near[2]["name"], 25, 0, "Find reachable charging station"),
        ("Charging station unavailable", mid[1]["name"], mid[2]["name"], 60, 1, "Find alternative"),
        ("No feasible route", far[1]["name"], far[2]["name"], 6, 0, "Display no feasible route"),
    ]
    log("scenarios: " + " | ".join(f"{s[0]}: {s[1]}->{s[2]} @{s[3]}%" for s in sc))
    return sc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", default=RAW_DEFAULT, help="Kaggle CSV (default: dataset/raw/Indian_EV_Stations_Simplified.csv)")
    ap.add_argument("--radius", type=float, default=6.0, help="merge stations closer than this many km into one hub")
    a = ap.parse_args()
    log = print

    stations = load_stations(a.raw, log)
    hubs = cluster(stations, a.radius, log)
    name_and_type(hubs, log)
    roads = build_roads(hubs, log)
    scen = pick_scenarios(hubs, roads, log)

    with open(os.path.join(OUT, "ev_data.csv"), "w", newline="") as f:
        f.write("# EV ChargeRoute nodes - built from dataset/raw/Indian_EV_Stations_Simplified.csv by tools/build_dataset.py\n")
        f.write("# type=CITY|STATION; charger=1; price_per_kwh is ASSUMED by power tier; power_kw = fastest charger in the hub; stations = chargers merged into the hub\n")
        w = csv.writer(f)
        w.writerow(["id", "name", "type", "lat", "lon", "charger", "price_per_kwh", "power_kw", "available", "stations", "operators"])
        for h in hubs:
            w.writerow([h["id"], h["name"], h["type"], f"{h['lat']:.5f}", f"{h['lon']:.5f}", 1, h["price"], f"{h['kw']:g}", 1, h["count"], h["ops"]])
    with open(os.path.join(OUT, "roads.csv"), "w", newline="") as f:
        f.write("# EV ChargeRoute roads (undirected). distance_km = straight-line x 1.30. terrain_factor multiplies energy use\n")
        w = csv.writer(f)
        w.writerow(["from", "to", "distance_km", "terrain_factor", "road_name"])
        w.writerows(roads)
    with open(os.path.join(OUT, "scenarios.csv"), "w", newline="") as f:
        f.write("# test scenarios (roadmap section 7). block_first_stop=1 disables the station the planner would use first\n")
        w = csv.writer(f)
        w.writerow(["title", "from", "to", "battery_pct", "block_first_stop", "expectation"])
        w.writerows(scen)
    log(f"written: {OUT}/ev_data.csv, roads.csv, scenarios.csv")


if __name__ == "__main__":
    main()
