# nomad

**Network-based Open Mobility Agent Dynamics**

A modern, research-grade agent-based urban mobility simulator written in C++20.
Designed for publication-quality transport science research.

---

## What it does

nomad simulates individual agents (people, vehicles) moving through a real road network extracted from OpenStreetMap. It models congestion, queue spillback, and multi-modal transport at city and metropolitan scale.

**Key features**

- OSM-native ingestion — loads any city directly from a `.osm.pbf` file
- Discrete-event simulation engine (not a fixed-timestep loop)
- Queue-based traffic model with spillback (MATSim-style)
- A\* routing with caching; Contraction Hierarchies (in development)
- Multi-modal: car, walk, bike, transit — configurable per scenario
- All parameters controlled from a single JSON file
- GeoJSON output ready for kepler.gl / QGIS / deck.gl
- Python API via pybind11 (Phase 4)
- 42 unit + integration tests

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
ctest --test-dir build         # 42/42 should pass
```

> **Note**: use `/usr/bin/cmake` (v3.28), not the conda cmake (v4.x) which breaks FetchContent.

### 2. Download a city

```bash
python tools/nomad-dl.py Bologna
# → data/test_osm/bologna.osm.pbf
```

Works with any city: `"Rome, Italy"`, `"Firenze"`, `"Paris"`, etc.

### 3. Run

```bash
./build/nomad_cli --config data/schemas/scenario_bologna.json
# output → results/bologna_morning/network_state_*.geojson
```

Or quick mode (10 k gravity agents, default config):

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

---

## JSON scenario configuration

Every parameter is controlled from a single JSON file. Only non-default values need to appear.

```json
{
  "name": "bologna_morning",
  "simulation": {
    "start_time_s": 25200,
    "end_time_s":   32400,
    "num_threads": 1,
    "traffic_model": "queue",
    "router": "astar"
  },
  "network": {
    "osm_pbf": "data/test_osm/bologna.osm.pbf",
    "simplify_topology": true
  },
  "demand": {
    "source": "gravity",
    "modes": ["car"],
    "modes_split": { "car": 1.0 },
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
  "modes": ["car"],                          // unimodale auto
  "modes": ["car", "walk", "bike"],          // tri-modale, proporzioni uniformi
  "modes": ["car", "walk", "bike", "transit"],
  "modes_split": { "car": 0.60, "walk": 0.15, "bike": 0.10, "transit": 0.15 }
}
```

Full schema reference: [`data/schemas/scenario_example.json`](data/schemas/scenario_example.json)

---

## Architecture

```
nomad/
├── include/nomad/
│   ├── core/          graph.hpp, agent.hpp, event.hpp, simulation.hpp
│   ├── network/       osm_loader.hpp, network_cleaner.hpp, multilayer_graph.hpp
│   ├── traffic/       traffic_model.hpp, queue_model.hpp, ltm_model.hpp
│   ├── routing/       router.hpp, astar_router.hpp, ch_router.hpp, route_cache.hpp
│   ├── demand/        demand_model.hpp, od_matrix.hpp, synthetic.hpp, mode_choice.hpp
│   ├── transit/       gtfs_loader.hpp, transit_model.hpp
│   ├── output/        writer.hpp, geojson_writer.hpp, parquet_writer.hpp
│   ├── analytics/     metrics.hpp, aggregator.hpp
│   └── config/        scenario_config.hpp
├── src/               implementations
├── tools/
│   ├── nomad-cli/     CLI entry point
│   └── nomad-dl.py    OSM download tool (Nominatim + Overpass)
├── tests/             42 Catch2 tests (unit + integration)
├── python/            Python API (Phase 4)
└── data/schemas/      JSON scenario examples + OSM tag config
```

**Core design decisions** (each justified in the implementation):

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Graph | CSR + geometry sidecar | Cache-friendly routing, cold geometry doesn't pollute hot path |
| Traffic | Queue model (MATSim-style) | O(links) state, natural spillback, fits DES |
| Routing | A\* → CH (Phase 3) | A\* correct, CH 1000× faster for 1M agents |
| Simulation | Hybrid DES + sync windows | Event-driven correctness + future parallelism |
| Config | JSON (nlohmann) | Human-readable, round-trippable, versionable |

---

## Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | ✅ **Done** | OSM ingestion, A\* routing, queue traffic, DES engine, GeoJSON output |
| 2 | 🔄 Next | End-to-end Bologna validation, GEH calibration |
| 3 | Planned | TBB parallel dispatch, Contraction Hierarchies |
| 4 | Planned | Python pybind11 bindings, Parquet output |
| 5 | Planned | LTM traffic model, dynamic re-routing |
| 6 | Planned | Multi-modal routing, GTFS integration, transit schedules |

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
TBB, Arrow, yaml-cpp, pybind11, numpy, pandas, geopandas, pyarrow, pydeck.

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
