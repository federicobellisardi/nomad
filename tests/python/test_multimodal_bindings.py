"""Minimal end-to-end test of the Python bindings added on
feature/python-multimodal-bindings: Simulation.set_router/set_traffic_model,
AStarRouter/CHRouter, QueueTrafficModel/LtmTrafficModel, and the explicit
error raised when either is missing (previously: a silent no-op — see
`require_ready` in bindings/pynomad.cpp and has_router()/has_traffic_model()
in include/nomad/core/simulation.hpp).

Not part of the Catch2/ctest suite — this specifically exercises the
_nomad_core Python extension, so it runs under pytest. Requires NOMAD built
with -DNOMAD_BUILD_PYTHON=ON (the default):

    cmake --build build && pytest tests/python -v

Network under test (built via the test-only `build_test_graph` binding —
see bindings/pynomad.cpp — since Graph has no other Python constructor):

    0 ──res(300m,10m/s)──► 1 ──res(300m,10m/s)──► 2 ──res(300m,10m/s)──► 3
    0 ──────────────motorway (500m, 25m/s)──────────────────────────────► 2
    3 ──res(900m,10m/s)──► 0   (return leg, for network connectivity)

Car agents 0→3 should take the motorway shortcut via node 2 (2 edges, ~50s)
since CHRouter/AStarRouter both minimize travel time and cars alone may use
Motorway-class edges (road_class_accessible in types.hpp). Walk/bike agents
cannot use Motorway at all, so they are forced onto the all-residential path
0→1→2→3 (3 edges) — this is the concrete, checkable evidence that car/walk/
bike are genuinely distinct modes in the core engine, not just distinct
labels.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

NOMAD_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NOMAD_ROOT / "python"))

core = pytest.importorskip(
    "nomad._nomad_core",
    reason="NOMAD not built with -DNOMAD_BUILD_PYTHON=ON — see tests/python/README or module docstring",
)

# ── Synthetic network ──────────────────────────────────────────────────────────
# Plain ints (not core.RoadClass members) to sidestep any pybind11 enum->int
# conversion subtlety in the tuple path — values match RoadClass in
# include/nomad/core/types.hpp exactly (asserted below as a guard rail).
RES = 10        # RoadClass::Residential
MOTORWAY = 0    # RoadClass::Motorway
assert int(core.RoadClass.Residential) == RES
assert int(core.RoadClass.Motorway) == MOTORWAY

# (from_node, to_node, length_m, free_flow_speed_ms, capacity_veh_h, road_class)
EDGES = [
    (0, 1, 300.0, 10.0, 600.0, RES),
    (1, 2, 300.0, 10.0, 600.0, RES),
    (2, 3, 300.0, 10.0, 600.0, RES),
    (0, 2, 500.0, 25.0, 2000.0, MOTORWAY),
    (3, 0, 900.0, 10.0, 600.0, RES),  # return leg, keeps the network connected
]
NUM_NODES = 4

# edge_id is assigned in the from_node-sorted order build_test_graph uses:
# node 0's edges come first (0->1, 0->2 motorway), then node 1's (1->2),
# node 2's (2->3), then node 3's (3->0). See build_test_graph docstring.
EDGE_0_1 = 0
EDGE_0_2_MOTORWAY = 1
EDGE_1_2 = 2
EDGE_2_3 = 3
EDGE_3_0 = 4


def make_graph():
    return core.build_test_graph(EDGES, NUM_NODES)


def write_od_csv(tmp_path, rows) -> str:
    path = tmp_path / "od.csv"
    lines = ["origin_node,dest_node,count,mode,depart_mean_s,depart_std_s"]
    lines += [",".join(str(v) for v in row) for row in rows]
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def build_ready_simulation(graph, od_csv_path, modes):
    """Construct a Simulation with graph/demand/router/traffic model all
    attached — the "happy path" every non-error test starts from.
    AStarRouter (not CHRouter) because walk/bike need mode-aware routing —
    see the module docstring and CHRouter's own binding docstring."""
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    sim.set_router(core.AStarRouter(graph))
    sim.set_traffic_model(core.QueueTrafficModel(graph))
    demand = core.OdMatrixDemand.from_csv(od_csv_path, 42, -1e9, 1e9, modes)
    sim.set_demand(demand)
    return sim


def track_enter_link_events(sim):
    """Returns (events list, register()) — events populated once run()."""
    events = []
    sim.register_hook(
        "AgentEnterLink", lambda t, agent_id, edge_id: events.append((t, agent_id, edge_id))
    )
    return events


def track_arrivals(sim):
    arrivals = []
    sim.register_hook("AgentArriveActivity", lambda t, agent_id, payload: arrivals.append(agent_id))
    return arrivals


# ── Tests ───────────────────────────────────────────────────────────────────────

def test_missing_router_raises_explicit_error(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 1, "car", 0, 10)])
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    sim.set_demand(core.OdMatrixDemand.from_csv(od_csv, 42, -1e9, 1e9, ["car"]))
    sim.set_traffic_model(core.QueueTrafficModel(graph))  # traffic OK, router missing

    assert sim.has_router is False
    assert sim.has_traffic_model is True
    with pytest.raises(ValueError, match="router"):
        sim.run()


def test_missing_traffic_model_raises_explicit_error(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 1, "car", 0, 10)])
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    sim.set_demand(core.OdMatrixDemand.from_csv(od_csv, 42, -1e9, 1e9, ["car"]))
    sim.set_router(core.AStarRouter(graph))  # router OK, traffic model missing

    assert sim.has_router is True
    assert sim.has_traffic_model is False
    with pytest.raises(ValueError, match="traffic model"):
        sim.run()


def test_missing_both_raises_before_running(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 1, "car", 0, 10)])
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    sim.set_demand(core.OdMatrixDemand.from_csv(od_csv, 42, -1e9, 1e9, ["car"]))
    with pytest.raises(ValueError):
        sim.run()
    # Nothing should have run — the guard fires before any event processing.
    assert sim.events_processed() == 0


def test_run_until_and_step_are_also_guarded(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 1, "car", 0, 10)])
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    sim.set_demand(core.OdMatrixDemand.from_csv(od_csv, 42, -1e9, 1e9, ["car"]))
    with pytest.raises(ValueError):
        sim.run_until(100.0)
    with pytest.raises(ValueError):
        sim.step()


def test_ch_router_binding_works_for_car_only(tmp_path):
    """CHRouter is car-only (ignores AgentMode at query time — see its
    binding docstring) but must still be attachable and produce a real
    (non-empty) run for a car-only scenario."""
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 3, "car", 0, 10)])
    sim = core.Simulation(core.SimulationConfig())
    sim.set_graph(graph)
    ch = core.CHRouter(graph)
    ch.preprocess()
    assert ch.is_preprocessed is True
    sim.set_router(ch)
    sim.set_traffic_model(core.QueueTrafficModel(graph))
    sim.set_demand(core.OdMatrixDemand.from_csv(od_csv, 42, -1e9, 1e9, ["car"]))

    sim.run()

    assert sim.events_processed() > 0
    assert sim.active_agents() == 0


def test_simulation_is_not_empty_and_conserves_agents(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 5, "car", 0, 10)])
    sim = build_ready_simulation(graph, od_csv, ["car"])
    arrivals = track_arrivals(sim)

    sim.run()

    assert sim.events_processed() > 0, "simulation must not be an empty no-op"
    assert sim.active_agents() == 0, "no agent should be left in limbo on this tiny, uncongested network"
    assert len(arrivals) == 5, "every one of the 5 generated car agents must reach AgentArriveActivity"
    assert len(set(arrivals)) == 5, "each agent must arrive exactly once (no duplicate/missing arrivals)"


def test_per_edge_flow_matches_expected_route(tmp_path):
    graph = make_graph()
    od_csv = write_od_csv(tmp_path, [(0, 3, 4, "car", 0, 10)])
    sim = build_ready_simulation(graph, od_csv, ["car"])
    events = track_enter_link_events(sim)

    sim.run()

    counts: dict[int, int] = {}
    for _, _, edge_id in events:
        counts[edge_id] = counts.get(edge_id, 0) + 1

    # All 4 car agents must take the motorway shortcut (0->2->3), never the
    # all-residential detour (0->1->2) — this is the routing engine actually
    # cost-minimizing per mode, not a fixed/hardcoded path.
    assert counts.get(EDGE_0_2_MOTORWAY, 0) == 4
    assert counts.get(EDGE_2_3, 0) == 4
    assert counts.get(EDGE_0_1, 0) == 0
    assert counts.get(EDGE_1_2, 0) == 0


def test_car_walk_bike_are_routed_as_distinct_modes(tmp_path):
    od_csv = write_od_csv(
        tmp_path,
        [
            (0, 3, 4, "car", 0, 10),
            (0, 3, 3, "walk", 0, 10),
            (0, 3, 2, "bike", 0, 10),
        ],
    )

    per_mode_edge_counts = {}
    for mode in ("car", "walk", "bike"):
        graph = make_graph()  # fresh graph per run: LinkState occupancy must not leak across runs
        sim = build_ready_simulation(graph, od_csv, [mode])
        events = track_enter_link_events(sim)
        sim.run()
        counts: dict[int, int] = {}
        for _, _, edge_id in events:
            counts[edge_id] = counts.get(edge_id, 0) + 1
        per_mode_edge_counts[mode] = counts

    # Car: takes the motorway shortcut, never touches the residential detour.
    assert per_mode_edge_counts["car"].get(EDGE_0_2_MOTORWAY, 0) == 4
    assert per_mode_edge_counts["car"].get(EDGE_0_1, 0) == 0

    # Walk/bike: Motorway is inaccessible (road_class_accessible in types.hpp)
    # — they MUST be forced onto the all-residential path instead.
    for mode, expected_count in (("walk", 3), ("bike", 2)):
        counts = per_mode_edge_counts[mode]
        assert counts.get(EDGE_0_2_MOTORWAY, 0) == 0, f"{mode} must never use the motorway edge"
        assert counts.get(EDGE_0_1, 0) == expected_count
        assert counts.get(EDGE_1_2, 0) == expected_count
        assert counts.get(EDGE_2_3, 0) == expected_count


def test_temporal_aggregation_across_hour_boundary(tmp_path):
    # Wave 1 departs at t in [0, 10) -> hour 0. Wave 2 departs at
    # t in [3700, 3710) -> hour 1. Edge traversal times here are tens of
    # seconds, so each wave's AgentEnterLink events land squarely in their
    # own hour bin with no overlap.
    od_csv = write_od_csv(
        tmp_path,
        [
            (0, 3, 3, "car", 0, 10),
            (0, 3, 3, "car", 3700, 3710),
        ],
    )
    graph = make_graph()
    sim = build_ready_simulation(graph, od_csv, ["car"])
    events = track_enter_link_events(sim)

    sim.run()

    by_hour: dict[int, int] = {}
    for t, _, _ in events:
        hour = int(t // 3600)
        by_hour[hour] = by_hour.get(hour, 0) + 1

    assert by_hour.get(0, 0) == 6, "wave 1 (3 agents x 2 edges) must land in hour 0"
    assert by_hour.get(1, 0) == 6, "wave 2 (3 agents x 2 edges) must land in hour 1"
    assert sum(by_hour.values()) == len(events)
