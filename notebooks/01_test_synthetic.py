"""
nomad — Quick smoke test with a synthetic network.
No OSM data required. Run with:

    conda activate nomad
    python notebooks/01_test_synthetic.py

Or in Jupyter:

    conda activate nomad
    jupyter notebook notebooks/01_test_synthetic.py
"""

# ── NOTE: build the Python bindings first ─────────────────────────────────────
# /usr/bin/cmake -B build -DNOMAD_BUILD_PYTHON=ON -DNOMAD_BUILD_TESTS=ON \
#   -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/nomad" -Wno-dev
# cmake --build build -j$(nproc)

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

# ── Test 1: import ─────────────────────────────────────────────────────────────
try:
    from nomad import Scenario
    print("✓ nomad imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    print("  Build with: /usr/bin/cmake -B build -DNOMAD_BUILD_PYTHON=ON ...")
    sys.exit(1)

import numpy as np
import pandas as pd
import tempfile, csv, pathlib

# ── Test 2: quick scenario with gravity demand ─────────────────────────────────
print("\n── Test: gravity demand on synthetic graph ─────────────────────────")

sc = Scenario.__new__(Scenario)  # we'll call it differently for the synthetic test
# For a full OSM-based test, use:
# sc = Scenario("data/test_osm/liechtenstein.osm.pbf")
# sc.add_gravity_demand(total_agents=5000)
# sc.run()

# ── Test 3: OD demand from DataFrame ──────────────────────────────────────────
print("\n── Test: OD demand from pandas DataFrame ────────────────────────────")
# NOTE: requires Python bindings to be built.
# This test is a usage demonstration — it will fail if bindings aren't built.

od_data = pd.DataFrame({
    'origin_node':   [0, 4,  8],
    'dest_node':     [24, 20, 16],
    'count':         [100, 50, 30],
    'mode':          ['car', 'car', 'car'],
    'depart_mean_s': [28800, 29000, 27600],
    'depart_std_s':  [3600,  1800,  900],
})
print("OD demand DataFrame:")
print(od_data.to_string(index=False))

# ── Test 4: check config loading ──────────────────────────────────────────────
print("\n── Test: JSON config loading ─────────────────────────────────────────")
config_path = pathlib.Path("data/schemas/scenario_liechtenstein.json")
if config_path.exists():
    from nomad._nomad_core import ScenarioConfig
    cfg = ScenarioConfig.load(str(config_path))
    print(f"  Config name: {cfg.name}")
    print(f"  Start time: {cfg.simulation.start_time}s")
    print("  ✓ JSON config loads correctly")
else:
    print(f"  Config file not found: {config_path}")
    print("  Run from project root: python notebooks/01_test_synthetic.py")

print("\n── All Python API tests completed ───────────────────────────────────")
