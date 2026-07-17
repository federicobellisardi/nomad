# nomad

**Network-based Open Mobility Agent Dynamics**

A research-grade agent-based urban mobility simulator written in C++20.
Simulates individual agents moving through a real road network extracted
from OpenStreetMap, with real travel demand, congestion, and multi-modal
transport, at metropolitan scale.

Validated end-to-end on Palma de Mallorca (79k nodes, 186k edges, up to 1.37M
agents/day across car, walk and bike) with **96.5% arrival rate** and a
correctly congestion-isolated multi-modal traffic model — see
[Validation](#validation--palma-de-mallorca-case-study) below.

---

## What it does

nomad simulates individual agents (people, vehicles) moving through a real
road network extracted from OpenStreetMap. It models congestion, dynamic
re-routing, and multi-modal transport at city and metropolitan scale, driven
by real origin-destination demand (e.g. Spain's MITMA mobility dataset).

**Key features**

- OSM-native ingestion — loads any city directly from a `.osm.pbf` file
- Discrete-event simulation engine (not a fixed-timestep loop), TBB-parallel pre-routing and rerouting
- Two interchangeable traffic models: BPR volume-delay queue model (default,
  fast) and a single-cell CTM-lite with real spillback (`"ltm"`)
- Contraction Hierarchies for fast car routing, A\* for multi-modal and
  traffic-aware en-route rerouting, with a shared `RouteCache`
- Multi-modal: car, walk, bike are simulated correctly today — each mode
  gets its own realistic speed cap and only cars interact with road
  congestion (see [Multi-modal support](#multi-modal-support)); transit/GTFS
  is not implemented yet
- Real-world demand pipeline: MITMA (Spain) mobile-phone mobility data →
  zone-level OD → node-level OD, with per-mode strongly-connected-component
  filtering so every generated trip is actually routable
- Stochastic per-edge route randomization for realistic route diversity
  without full traffic assignment
- All parameters controlled from a single JSON scenario file
- GeoJSON output ready for kepler.gl / QGIS / deck.gl; interactive
  Jupyter notebooks for results analysis
- Python API via pybind11 — zero-copy NumPy views over graph/edge data
- 63 unit + integration tests (Catch2)

---

## Quick start

### 1. Build

```bash
conda activate nomad          # see environment.yml

/usr/bin/cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DNOMAD_BUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/nomad" \
  -Wno-dev

cmake --build build -j$(nproc)
ctest --test-dir build         # 63/63 should pass
```

> **Note**: use `/usr/bin/cmake` (v3.28), not the conda cmake (v4.x) which
> breaks FetchContent.

Other CMake options (all `OFF`/`ON` via `-D<NAME>=...`):

| Option | Default | Purpose |
|--------|---------|---------|
| `NOMAD_BUILD_PYTHON` | `ON` | Build the `_nomad_core` pybind11 module into `python/nomad/` |
| `NOMAD_BUILD_TESTS` | `ON` | Build the Catch2 test binary |
| `NOMAD_BUILD_BENCHMARKS` | `OFF` | Build the Google Benchmark suite |
| `NOMAD_USE_ASAN` | `OFF` | AddressSanitizer + UBSan debug build (use a separate build dir, e.g. `build_asan/`) |
| `NOMAD_USE_LTO` | `OFF` | Link-time optimisation |

### 2. Download a city

```bash
python tools/nomad-dl.py Bologna
# → data/test_osm/bologna.osm.pbf
```

Works with any city: `"Rome, Italy"`, `"Firenze"`, `"Paris"`, etc. Uses
Nominatim (boundary lookup) + Overpass (PBF extract) — no API key needed,
but subject to Overpass rate limits for large cities.

### 3. Run

```bash
./build/nomad_cli --config data/schemas/scenario_bologna.json
# output → results/bologna_morning/network_state_*.geojson
```

Or quick mode (10k synthetic gravity-model agents, no OD data needed):

```bash
./build/nomad_cli --osm data/test_osm/bologna.osm.pbf
```

### 4. Inspect output

```bash
python - << 'EOF'
import geopandas as gpd, glob
files = sorted(glob.glob("results/bologna_morning/*.geojson"))
gdf = gpd.read_file(files[-1])
print(gdf[["edge_id","congestion","travel_time_s","count"]]
      .sort_values("congestion", ascending=False).head(20))
EOF
```

Or open one of the analysis notebooks in `notebooks/` (see
[`check_palma_results.ipynb`](notebooks/check_palma_results.ipynb) for a
full worked example: arrival-rate summary, time-series, static + interactive
congestion maps, OD demand breakdown).

---

## Real-world case study: Palma de Mallorca

A full pipeline from raw OSM + government mobility data to a validated
metro-scale simulation lives in `python/download_data/` and
`python/preprocessing/`. It is Palma/Spain-specific (unlike the generic
`nomad-dl.py`), but the pattern generalises to any city with an open
mobile-phone OD dataset.

```bash
# 1. Download regional OSM extract + clip to the city's Functional Urban Area
python python/download_data/download_osm.py --clip

# 2. Download MITMA (Spain) daily district-level trip matrices for the
#    period of interest (raw CSVs → data/od_raw/)
python python/download_data/download_mitma.py --start 2022-02-01 --end 2022-02-28

# 3. Build the simplified routing graph (OSM → NetworkCleaner → graph.bin +
#    nodes.parquet + edges.parquet, all under a shared node numbering)
python python/preprocessing/simplify_osm.py --city "Palma de Mallorca"

# 4. Build the node-level OD matrix from MITMA zone-level trips (average
#    weekday + weekend, all four modes, per-mode SCC-filtered node pools)
python python/preprocessing/build_od.py --city "Palma de Mallorca"

# 5. Run
./build/nomad_cli --config data/schemas/scenario_palma.json
```

**⚠️ Node-numbering coupling**: `graph.bin`'s node IDs are whatever survives
`NetworkCleaner`'s topology simplification, and `nodes.parquet` /
`edges.parquet` / the OD CSV all reference those same raw integer IDs.
**Any change to `network_cleaner.cpp` (or a `--force-clip`/re-extraction)
requires re-running step 3 *then* step 4, in that order** — skipping this
either segfaults (`CHRouter::ch_query` on an out-of-range node) or silently
routes trips to the wrong place. There is no automatic invalidation.

### Validation — Palma de Mallorca case study

Palma's FUA (79,039 nodes / 186,242 edges after simplification) was used to
validate the full pipeline against real 2022 MITMA weekday demand:

| Scenario | Agents | Arrival | Natural | Teleported | Sim time |
|----------|-------:|--------:|--------:|-----------:|---------:|
| Car only, demand_scale=0.4 | 285,098 | 96.5% | 95.0% | 1.5% | 129s |
| Car only, demand_scale=0.75 | 534,954 | 96.5% | 95.1% | 1.4% | 295s |
| Car only, demand_scale=1.0 (full MITMA) | 713,065 | 96.5% | 95.2% | 1.3% | 551s |
| Car + walk + bike, demand_scale=1.0 | 1,369,714 | 95.5% | 93.5% | 2.0% | 1,112s |

This result required fixing a structural bug: `NetworkCleaner`'s degree-2
node contraction required each neighbour to supply *both* an in-edge and an
out-edge, which only holds for bidirectional road segments. Real motorways
are almost always mapped as separate one-way carriageways in OSM, so this
check silently rejected every one-way pass-through node — leaving thousands
of unmerged micro-segments exactly at motorway interchanges, which dominated
stuck/teleported agents (~49% of both populations) regardless of traffic
model, routing algorithm, or reroute tuning. Fixing the contraction
predicate (redirect-count matching instead of a strict per-neighbour
in/out check) raised arrival from a historical 85–92% ceiling to 96.5%,
consistent across every demand scale tested.

### Multi-modal support

Car, walk and bike are all simulated with mode-specific realism:

- **Routing**: `AStarRouter` filters edges per mode (`road_class_accessible`
  in `types.hpp` — e.g. walk/bike cannot use motorways) and caps effective
  edge speed at the mode's own pace (`Graph::mode_free_flow_time` —
  `min(mode_speed, edge_speed)`), so a pedestrian routed onto a road edge
  with no separate footway mapped in OSM gets a real walking-pace travel
  time, not the road's car-calibrated speed. `CHRouter` is car-only (it
  ignores `AgentMode` at query time) — set `"routing.algorithm": "astar"`
  for any scenario using non-car modes (see
  [`scenario_palma_multimodal.json`](data/schemas/scenario_palma_multimodal.json)).
- **Traffic interaction**: only `AgentMode::Car` agents call into the
  traffic model (`QueueTrafficModel`/`LtmTrafficModel`) — walk/bike travel
  free-flow at their capped speed and never contend for car-calibrated link
  capacity. There is no pedestrian/bike congestion model.
- **Demand**: `python/preprocessing/build_od.py` computes each mode's own
  largest strongly-connected component (`walk_nodes`/`bike_nodes`, mirroring
  the pre-existing `car_nodes` logic) and restricts OD node sampling to it —
  without this, a walk/bike trip can be assigned an origin or destination
  that's topologically isolated for that mode, and routing fails outright
  (this was a real bug: fixing it took the multi-modal routing-failure rate
  from 1.8% to 0%).
- **Not implemented**: transit/GTFS (`transit/gtfs_loader.hpp` is a stub);
  the MNL mode-choice model (`demand/mode_choice.cpp` is a stub — mode is
  currently decided upstream, baked into the OD CSV's `mode` column via
  MITMA-derived or distance-band heuristics, not chosen dynamically per
  trip); `MultiLayerGraph` (unused — car/walk/bike/transit are simulated as
  filtered views over one shared graph, not separate layers).

---

## JSON scenario configuration

Every parameter is controlled from a single JSON file. Only non-default
values need to appear. Full schema reference:
[`data/schemas/scenario_example.json`](data/schemas/scenario_example.json).

```json
{
  "name": "bologna_morning",
  "simulation": {
    "start_time":   "2026-01-01 07:00:00",
    "end_time":     "2026-01-01 09:00:00",
    "traffic_model": "queue",
    "router":        "astar"
  },
  "network": {
    "osm_pbf": "data/test_osm/bologna.osm.pbf",
    "simplify_topology": true
  },
  "demand": {
    "source": "gravity",
    "modes": ["car"],
    "synthetic": { "total_agents": 50000, "seed": 42 }
  },
  "output": {
    "output_dir": "results/bologna_morning",
    "writers": ["geojson"]
  }
}
```

### Mode selection

```json
"demand": {
  "modes": ["car"],                    // car-only — CH or A* both fine
  "modes": ["car", "walk", "bike"]      // multi-modal — requires router: "astar"
}
```

Demand `source: "od_csv"` reads real mode assignments from the CSV's `mode`
column; `source: "gravity"`/`"synthetic"` samples modes from `modes_split`.

### Traffic model selection

```json
"traffic": { "model": "queue" }   // BPR volume-delay, default, fastest
"traffic": { "model": "ltm"   }   // single-cell CTM-lite with real spillback
```

Both share the same storage-capacity conventions (`kJamDensity = 1/7.5`,
50m effective-length floor) so results are directly comparable. `"ltm"`
adds real backward-propagating spillback but produced statistically
indistinguishable results from `"queue"` on Palma — the network's
structural bug (above) was the actual bottleneck, not the traffic model.
It remains available for scenarios with genuine sustained link-level
oversaturation.

---

## Architecture

```
nomad/
├── include/nomad/
│   ├── core/          graph.hpp (CSR + mode-aware travel time), agent.hpp,
│   │                   event.hpp, simulation.hpp, types.hpp (AgentMode, RoadClass)
│   ├── network/        osm_loader.hpp, network_cleaner.hpp (degree-2 contraction,
│   │                   component pruning), intersection.hpp
│   ├── traffic/        traffic_model.hpp (ITrafficModel + LinkState + congestion_ema),
│   │                   queue_model.hpp (BPR), ltm_model.hpp (CTM-lite + spillback)
│   ├── routing/        router.hpp, astar_router.hpp (mode-aware), ch_router.hpp
│   │                   (car-only), route_cache.hpp
│   ├── demand/         demand_model.hpp, od_matrix.hpp, synthetic.hpp,
│   │                   mode_choice.hpp (stub), departure_sampler.hpp, activity_plan.hpp
│   ├── transit/        gtfs_loader.hpp, transit_model.hpp (stubs — Phase 6)
│   ├── output/         writer.hpp, geojson_writer.hpp, parquet_writer.hpp
│   ├── analytics/      metrics.hpp, aggregator.hpp
│   └── config/         scenario_config.hpp
├── src/                 implementations (mirrors include/ layout)
├── bindings/            pynomad.cpp — pybind11 module (_nomad_core)
├── tools/
│   ├── nomad-cli/       CLI entry point
│   └── nomad-dl.py      generic OSM download tool (Nominatim + Overpass)
├── python/
│   ├── nomad/           Python package (scenario.py, analysis.py)
│   ├── download_data/   Palma/MITMA-specific OSM + mobility data download
│   └── preprocessing/   simplify_osm.py, build_od.py, sensitivity_analysis.py
├── tests/                63 Catch2 tests (unit + integration)
├── notebooks/            Jupyter analysis notebooks (results, network viz)
└── data/schemas/         JSON scenario examples + OSM tag config
```

**Core design decisions** (each justified in the implementation):

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Graph | CSR + geometry sidecar | Cache-friendly routing, cold geometry doesn't pollute hot path |
| Traffic | Queue model (BPR) + optional CTM-lite | O(active links) state, spillback available when needed, both fit DES |
| Routing | CH (car) + A\* (all modes, traffic-aware rerouting) | CH ~1000× faster for car-only pre-routing; A\* is the only mode-aware option today |
| Simulation | Hybrid DES + sync windows | Event-driven correctness; TBB parallelism in pre-routing/rerouting, event loop itself is sequential (per-agent/route-store mutation isn't yet lock-protected) |
| Congestion detection | EMA-smoothed tt/ff, full-route reroute scan | A single instantaneous snapshot every `reroute_interval_s` misses short, bursty congestion on fast-clearing edges |
| Config | JSON (nlohmann) | Human-readable, round-trippable, versionable |

---

## Known limitations

- **Transit/GTFS is not implemented** — `AgentMode::Transit` exists and is
  always routable (`road_class_accessible` returns `true` unconditionally),
  but there is no transit graph, schedule, or transfer model.
- **Mode choice is not dynamic** — the MNL model in `scenario_config.hpp`'s
  `mode_choice` block is parsed but never used; mode is fixed per trip
  upstream (OD CSV `mode` column).
- **CH is car-only** — `CHRouter::ch_query` accepts an `AgentMode` parameter
  but ignores it; use `"router": "astar"` for any multi-modal scenario.
- **No pedestrian/bike congestion model** — walk/bike agents always travel
  at their mode-capped free-flow speed regardless of how many other
  walk/bike agents share the same edge.
- **Sequential event loop** — TBB parallelism is used for pre-routing and
  periodic rerouting, but the main per-window event dispatch is
  single-threaded (enabling `tbb::parallel_for` there causes data races on
  `EventQueue`/`RouteStore`/`AgentHotStore` — needs per-agent locking first).
- **Parquet output** requires Apache Arrow at build time (`NOMAD_HAS_ARROW`)
  — falls back gracefully to GeoJSON-only if not found.

---

## Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | ✅ Done | OSM ingestion, A\* routing, queue traffic, DES engine, GeoJSON output |
| 2 | ✅ Done | Real-world end-to-end validation (Palma de Mallorca, MITMA demand) |
| 3 | ✅ Done | TBB parallel pre-routing/rerouting, Contraction Hierarchies (car) |
| 4 | ✅ Done | Python pybind11 bindings, Parquet output (optional) |
| 5 | ✅ Done | CTM-lite (LTM) traffic model with real spillback, dynamic re-routing |
| 6 | 🔄 Partial | Multi-modal routing (car/walk/bike ✅); GTFS integration, transit schedules, dynamic mode choice — not started |
| 7 | Planned | Mode-aware CH (or per-mode hierarchy), parallel event dispatch |

---

## Dependencies

All fetched automatically by CMake via FetchContent:

| Library | Purpose |
|---------|---------|
| libosmium + protozero | OSM PBF parsing |
| oneTBB | Task parallelism |
| Apache Arrow | Parquet output (optional) |
| nlohmann/json | JSON config |
| spdlog | Logging |
| yaml-cpp | OSM tag config |
| Catch2 | Tests |
| pybind11 | Python bindings |

System packages (conda env `nomad` — see [`environment.yml`](environment.yml)):
TBB, Arrow, yaml-cpp, pybind11, numpy, pandas, geopandas, pyarrow, pydeck,
folium (interactive maps).

---

## Citation

If you use nomad in published research, please cite:

```bibtex
@software{nomad2026,
  author  = {Bellisardi, Federico},
  title   = {nomad: Network-based Open Mobility Agent Dynamics},
  year    = {2026},
  url     = {https://github.com/bellisardi/nomad},
  version = {0.1.0}
}
```

---

## License

MIT License — see [`LICENSE`](LICENSE).
