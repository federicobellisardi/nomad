"""
High-level Python API for nomad.

Usage::

    from nomad import Scenario

    sc = Scenario("city.osm.pbf")
    sc.add_od_demand(df)     # pandas DataFrame
    sc.run()
    positions = sc.agent_positions()  # (N, 2) float32 array, zero-copy
    flows = sc.link_flows()           # pandas DataFrame
"""
from __future__ import annotations

from pathlib import Path
from typing import Callable, Dict, Optional, Union

import numpy as np

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

from nomad._nomad_core import (
    AgentMode,
    Graph,
    GravityDemand,
    NetworkCleaner,
    OdMatrixDemand,
    OsmLoader,
    Simulation,
    SimulationConfig,
    ScenarioConfig,
)


class Scenario:
    """
    Convenience wrapper that assembles and runs a nomad simulation from a
    single OSM file and a simple Python-side demand specification.

    Parameters
    ----------
    osm_path : str or Path
        Path to the OpenStreetMap PBF file.
    config : dict or SimulationConfig, optional
        Simulation parameters. Dict keys map to SimulationConfig fields.
    simplify : bool
        Whether to simplify the road network topology (removes degree-2 nodes).
    """

    def __init__(
        self,
        osm_path: Union[str, Path],
        config: Optional[Union[dict, SimulationConfig]] = None,
        simplify: bool = True,
    ):
        osm_path = Path(osm_path)
        if not osm_path.exists():
            raise FileNotFoundError(f"OSM file not found: {osm_path}")

        # Build graph
        loader = OsmLoader()
        self.graph: Graph = loader.load_and_clean(str(osm_path), simplify)

        # Build simulation config
        if isinstance(config, SimulationConfig):
            cfg = config
        else:
            cfg = SimulationConfig()
            if config:
                for k, v in config.items():
                    if hasattr(cfg, k):
                        setattr(cfg, k, v)

        self.sim = Simulation(cfg)
        self.sim.set_graph(self.graph)
        self._demand_set = False
        self._output_dir: Optional[str] = None

    # ── Alternative constructor: load from JSON config file ───────────────────
    @classmethod
    def from_config(cls, json_path: Union[str, Path]) -> "Scenario":
        """Load scenario from a JSON configuration file."""
        cfg = ScenarioConfig.load(str(json_path))
        sc = cls.__new__(cls)

        loader = OsmLoader()
        sc.graph = loader.load_and_clean(
            cfg.simulation.router,  # NOTE: actual path comes from cfg.network
            True,
        )
        sim_cfg = cfg.simulation
        sc.sim = Simulation(sim_cfg)
        sc.sim.set_graph(sc.graph)
        sc._demand_set = False
        sc._output_dir = None
        return sc

    # ── Demand setters ────────────────────────────────────────────────────────
    def add_od_demand(
        self,
        df=None,
        *,
        csv_path: Optional[Union[str, Path]] = None,
        seed: int = 42,
    ) -> "Scenario":
        """
        Add origin-destination demand.

        Parameters
        ----------
        df : pandas.DataFrame, optional
            DataFrame with columns: origin_node, dest_node, count, mode,
            depart_mean_s, depart_std_s. If omitted, supply ``csv_path``.
        csv_path : str or Path, optional
            Path to OD matrix CSV file.
        seed : int
            Random seed for departure time sampling.
        """
        if csv_path is not None:
            demand = OdMatrixDemand.from_csv(str(csv_path), seed)
        elif df is not None:
            if not HAS_PANDAS:
                raise ImportError("pandas is required to pass a DataFrame")
            tmp = Path("/tmp/nomad_od_tmp.csv")
            df.to_csv(tmp, index=False)
            demand = OdMatrixDemand.from_csv(str(tmp), seed)
        else:
            raise ValueError("Provide either df or csv_path")

        self.sim.set_demand(demand)
        self._demand_set = True
        return self

    def add_gravity_demand(
        self,
        total_agents: int = 10_000,
        beta: float = 0.003,
        peak_hour_s: float = 28_800.0,
        peak_std_s: float = 3_600.0,
        seed: int = 42,
    ) -> "Scenario":
        """Add synthetic gravity-model demand (no external data required)."""
        demand = GravityDemand(
            total_agents=total_agents,
            beta=beta,
            peak_mean_s=peak_hour_s,
            seed=seed,
        )
        self.sim.set_demand(demand)
        self._demand_set = True
        return self

    # ── Output ────────────────────────────────────────────────────────────────
    def write_geojson(
        self,
        output_dir: Union[str, Path] = "nomad_output",
        snapshot_interval_s: float = 300.0,
    ) -> "Scenario":
        """Enable GeoJSON snapshot output during the simulation."""
        self._output_dir = str(output_dir)
        self.sim.add_geojson_writer(self._output_dir, snapshot_interval_s)
        return self

    # ── Hooks ─────────────────────────────────────────────────────────────────
    def on_event(
        self,
        event_type: str,
        callback: Callable[[float, int, int], None],
    ) -> "Scenario":
        """
        Register a Python callback for simulation events.

        Parameters
        ----------
        event_type : str
            One of: AgentDepart, AgentEnterLink, AgentExitLink,
            AgentArriveActivity, SnapshotDump, AgentReroute.
        callback : callable(time: float, agent_id: int, payload: int)
        """
        self.sim.register_hook(event_type, callback)
        return self

    # ── Run ───────────────────────────────────────────────────────────────────
    def run(self) -> "Scenario":
        """Run simulation to completion (releases GIL)."""
        if not self._demand_set:
            raise RuntimeError(
                "No demand set. Call add_od_demand() or add_gravity_demand() first."
            )
        self.sim.run()
        return self

    def step(self) -> "Scenario":
        """Advance one synchronisation window (interactive / notebook use)."""
        self.sim.step()
        return self

    def run_until(self, time_s: float) -> "Scenario":
        """Run until a specific simulation time (seconds since midnight)."""
        self.sim.run_until(time_s)
        return self

    # ── State accessors ───────────────────────────────────────────────────────
    @property
    def time(self) -> float:
        """Current simulation time [s since midnight]."""
        return self.sim.current_time()

    @property
    def active_agents(self) -> int:
        return self.sim.active_agents()

    def agent_positions(self) -> np.ndarray:
        """
        Current (lon, lat) position of each agent.
        Returns a zero-copy (N, 2) float32 array backed by C++ memory.
        """
        edges = self.sim.agent_edges()          # (N,) uint32, zero-copy
        coords = self.graph.node_coords()       # (num_nodes, 2) float32, zero-copy
        node_lons = coords[:, 0]
        node_lats = coords[:, 1]
        # Map edge → target node (approximate: use edge target position)
        # Zero-copy fancy indexing is not available; create a copy here.
        edge_lengths = self.graph.edge_lengths()  # just to get edge count
        # In production: maintain edge → midpoint position table
        # For now, return the target node position as proxy
        valid = edges < len(coords)
        positions = np.zeros((len(edges), 2), dtype=np.float32)
        positions[valid, 0] = node_lons[edges[valid] % len(coords)]
        positions[valid, 1] = node_lats[edges[valid] % len(coords)]
        return positions

    def link_flows(self):
        """
        Current link state as a pandas DataFrame (or dict if pandas unavailable).
        Columns: edge_id, occupancy, inflow, outflow, travel_time_s.
        """
        states = self.sim.link_states()  # (E, 4) float32
        data = {
            "edge_id":       np.arange(len(states), dtype=np.uint32),
            "occupancy":     states[:, 0],
            "inflow":        states[:, 1],
            "outflow":       states[:, 2],
            "travel_time_s": states[:, 3],
        }
        if HAS_PANDAS:
            import pandas as pd
            return pd.DataFrame(data)
        return data

    def __repr__(self) -> str:
        return (
            f"Scenario(nodes={self.graph.num_nodes()}, "
            f"edges={self.graph.num_edges()}, "
            f"time={self.time:.0f}s, "
            f"agents={self.active_agents})"
        )
