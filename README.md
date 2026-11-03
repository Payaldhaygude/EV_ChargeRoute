# EV ChargeRoute

A C++17 mini-project: a **2D visual simulation** of an electric vehicle driving through a real road-network model of
**Kerala**, choosing where to charge so it never runs out of battery.
It combines three AI algorithms from the course assignments, each with its own job:

| Algorithm | Assignment | Job in this project | Files |
|---|---|---|---|
| **A\*** search | 3 | Shortest road path between start and destination | `AStar.h/.cpp` |
| **CSP** (backtracking + forward checking) | 4 | Decide *where* and *how much* to charge on that path so every battery constraint holds | `CSP.h/.cpp` |
| **ACO** (ant colony) | 5 | Search the network for the best *sequence* of charging stations (detours allowed) | `ACO.h/.cpp` |

> Central question: *how can an EV reach its destination safely without running out of battery, while minimising
> unnecessary charging stops and travel cost?*

## 1. Dataset (from your Kaggle zip)

`dataset/raw/Indian_EV_Stations_Simplified.csv` is the **original, unmodified file** (855 stations: name, city, state,
latitude, longitude, operator, usage type, connector, kW). The project uses it like this
(`python3 tools/build_dataset.py` does all of it and can be re-run at any time):

1. **Clean** - fixes state typos (`Keral`, `Keraka`, `Lerala` -> Kerala), trims spaces, drops bad coordinates.
2. **Keep Kerala** - 787 of 855 stations (the rest are in Tamil Nadu, Karnataka, West Bengal, ...).
3. **Merge into hubs** - stations closer than 6 km become one charging hub: **163 hubs**
   (`ev_data.csv`: real coordinates, number of stations, operators, fastest kW).
4. **Cities** - 15 district towns that exist in the data (Kasaragod, Kannur, Kalpetta, Kozhikode, Malappuram, Palakkad,
   Thrissur, Ernakulam, Munnar, Thodupuzha, Kottayam, Alappuzha, Pathanamthitta, Kollam, Thiruvananthapuram) are the
   selectable start / destination points. *Ernakulam is Kochi's district name in the file.*
5. **Roads** - **314 roads** (`roads.csv`).
6. **Test scenarios** - picked from the data and saved in `scenarios.csv`.

**What is real and what is not - please state this in your report:**

| Item | Source |
|---|---|
| station locations, names, cities, operators, kW | **real** (your Kaggle file) |
| roads between hubs | **derived**: the dataset has no roads, so hubs are joined with a Gabriel graph (a planar, road-like network) and road length = straight-line distance x 1.30 |
| price per kWh | **assumed** by power tier (>=50 kW Rs 22, >=22 kW Rs 18, >=15 kW Rs 15, otherwise Rs 12) - not in the file |
| terrain / hills | not modelled (factor 1.0 everywhere) - no elevation data |
| hub labels | taken as written in the file (a few labels do not match their coordinates, e.g. "Anthikkad") |

## 2. Build and run

You need a C++17 compiler and **SFML 3.x** (this copy was ported from SFML 2; the original SFML 2 version of the window code is not included).

| Platform | Install SFML 3 | Build |
|---|---|---|
| Ubuntu / Debian | `sudo apt install libsfml-dev cmake g++   (needs SFML 3 - on older Ubuntu build it from source)` | `cmake -S . -B build && cmake --build build` |
| macOS | `brew install sfml cmake` | same (add `-DSFML_DIR=$(brew --prefix sfml)/lib/cmake/SFML` if not found) |
| Windows (MSYS2 MinGW64) | `pacman -S mingw-w64-x86_64-sfml mingw-w64-x86_64-cmake mingw-w64-x86_64-gcc` | `cmake -S . -B build -G "MinGW Makefiles" && cmake --build build` |
| Windows (Visual Studio) | `vcpkg install sfml:x64-windows` (check it gives 2.6) | `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake` |

Linux / macOS without CMake: `make` (window + CLI) or `make cli` (algorithms only, no SFML needed).
If CMake cannot find SFML it still builds the **algorithms-only** version and prints a warning.

Run (the program finds `dataset/` and `assets/` by itself, also from `build/`):

```
./build/ev_chargeroute                                    # window, Thiruvananthapuram -> Kozhikode
./build/ev_chargeroute --from Kollam --to Kalpetta --battery 50
./build/ev_chargeroute --cli --from Thiruvananthapuram --to Kasaragod --battery 100   # text output, no window
./build/ev_chargeroute --test                             # the 5 roadmap test scenarios
./build/ev_chargeroute --list                             # all 163 hubs
./build/ev_chargeroute --from Palakkad --to Kollam --block "Thrissur"   # take a hub out of service
```

Other options: `--capacity KWH`, `--reserve PCT`, `--seed N`, `--data FILE --roads FILE`,
`--screenshot out.png --frames N` (saves an image after N simulated frames). Names are matched case-insensitively,
also by prefix (`--from thiruva`).

## 3. Using the simulation window

* **Left - user input panel:** `<` `>` cycle start / destination through the 15 district towns, `-` `+` change battery
  capacity and current charge (hold **Shift** for steps of 1). The five **TEST SCENARIO** buttons load the roadmap tests.
* **Centre - map:** all 163 hubs at their real coordinates with the Arabian Sea coast, the A\* route (orange), the
  optimised route (green), planned charge stops (yellow ring + number), the animated EV.
  **Left-click** a hub = start, **right-click** = destination, **Shift + left-click** = toggle that hub
  *unavailable* (red X) to test alternatives. Hover for hub details (kW, number of stations, operators).
* **Right - live information:** battery bar (red line = reserve), distance travelled, energy used, charging stops,
  route status, the **A\* vs A\*+CSP vs A\*+CSP+ACO comparison table** (km, kWh, stops, cost, execution time),
  selected-plan details and an event log.
* **Views:** keys `1` / `2` / `3` (or the buttons) animate A\*, A\*+CSP or the optimised route. Plain A\* on a low
  battery makes the EV **strand** ("OUT OF BATTERY") - that is the point of CSP.
* Keys: `Space` pause, `R` re-run, `Up/Down` speed, `Esc` quit. Planning takes about 0.2 s after every change.

## 4. How it works (workflow from the roadmap)

```
dataset -> graph -> A* (route) -> CSP (battery feasibility) -> ACO (charging sequence)
        -> final route -> animate (move, drain battery, stop & charge) -> results
```

**Battery model** (`Plan.cpp`, one function used by every algorithm): energy of a road =
`km x 0.18 kWh/km x terrain_factor`. Arrival charge must never fall below the **reserve (5 %)**, charge level can
never exceed capacity, and a stop needs a *working* charger. Charging at the very start is not allowed (the user
gives the current charge); the destination never needs charging. Charging time = kWh / the hub's kW rating.

**A\*** - `g` = road km, `h` = straight-line (haversine) km. Road length >= straight-line length, so `h` is
admissible and consistent -> optimal shortest path.

**CSP** - variables: charger hubs on the A\* path. Domain: skip, or charge up to a level (30...100 % of capacity,
plus "just enough for the next stop / destination"). Constraints: reserve on arrival, capacity, working charger, a
charge must add energy. Solved by backtracking with forward checking and iterative deepening on the number of stops,
so the **fewest stops** are found first (ties -> lowest cost). If the A\* path cannot be made feasible, CSP says so.

**ACO** - ants walk over {start, destination, working chargers in a corridor around the trip; the whole network if
the corridor has no feasible plan}; legs between stops are A\* paths. A move is allowed only if the CSP battery
constraints hold. Probability ~ `pheromone^1 x heuristic^3`, heuristic = 1 / (leg km + remaining km + charging
cost). Pheromone evaporates (rho 0.3), good routes deposit `Q / cost`, the best-so-far gets an elitist bonus; routes
may not revisit a place. 40 ants x 80 iterations, seeded (`--seed`). Because ACO searches beyond the A\* path it
finds **detours** when a hub is down or a better-priced fast charger exists.
ACO is stochastic: it often equals the CSP plan, sometimes beats it, occasionally lands a little worse - the final
route shown is always the best *feasible* of the three.

**Cost used to compare routes:** `km + 0.1 x charging_Rs + 10 x stops + 0.2 x charging_minutes`
(weights in `Params`, `Plan.h`).

## 5. Test scenarios (roadmap section 7)

`./ev_chargeroute --test` checks every row automatically (scenarios are read from `dataset/scenarios.csv`).

| # | Scenario | Input | Expected | Result |
|---|---|---|---|---|
| 1 | High battery | Palakkad -> Thrissur, 90 % | direct route | direct, 0 stops |
| 2 | Low battery | Kalpetta -> Kottayam, 60 % | charging stop needed | 1 stop |
| 3 | Very low battery | Thodupuzha -> Kollam, 25 % | find reachable station | 1 stop, first hub reachable |
| 4 | Station unavailable | scenario 2 with its first stop disabled | find alternative | 1 stop at a different hub |
| 5 | No feasible route | Kasaragod -> Thiruvananthapuram, 6 % | "no feasible route" | reported, EV not sent |

## 6. Project structure

```
EV_ChargeRoute/
├── dataset/   raw/Indian_EV_Stations_Simplified.csv (your original file)
│              ev_data.csv (163 hubs)  roads.csv (314 roads)  scenarios.csv (5 tests)
├── include/   Graph.h AStar.h CSP.h ACO.h Simulation.h   + Plan.h Pipeline.h
├── src/       main.cpp Graph.cpp AStar.cpp CSP.cpp ACO.cpp Simulation.cpp   + Plan.cpp Pipeline.cpp
├── assets/    car.png charging_station.png background.png + DejaVuSans fonts
├── tools/     build_dataset.py (raw Kaggle CSV -> ev_data/roads/scenarios)   make_assets.py
├── CMakeLists.txt  Makefile  README.md
```
`Plan.*` holds the shared battery model; `Pipeline.*` runs A\* -> CSP -> ACO and the scenarios (used by the window and
`--test`). Options of the builder: `python3 tools/build_dataset.py --radius 6` (hub merge distance in km; smaller = more,
denser hubs).

## 7. Limitations (worth mentioning in the report)

Straight 2D schematic, not a live map; roads are modelled (not surveyed); constant consumption per km (no speed,
temperature, elevation or regeneration); prices are assumed; "just enough" charging only; every hub is assumed free on
arrival (no queues, no connector-type matching); ACO is stochastic (fixed seed for repeatability).
