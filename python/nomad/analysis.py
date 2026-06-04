"""
Analysis utilities for nomad simulation outputs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Union

import numpy as np

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    import geopandas as gpd
    HAS_GEOPANDAS = True
except ImportError:
    HAS_GEOPANDAS = False


class Analysis:
    """
    Post-simulation analysis tools.
    Reads Parquet/GeoJSON outputs produced by the simulation.
    """

    def __init__(self, output_dir: Union[str, Path]):
        self.output_dir = Path(output_dir)

    def link_stats(self, time_bin_s: float = 300.0):
        """Load link_stats.parquet and return as DataFrame."""
        if not HAS_PANDAS:
            raise ImportError("pandas required for analysis")
        p = self.output_dir / "link_stats.parquet"
        if p.exists():
            return pd.read_parquet(p)
        return pd.DataFrame()

    def agent_trajectories(self):
        """Load agent_trajectories.parquet."""
        if not HAS_PANDAS:
            raise ImportError("pandas required")
        p = self.output_dir / "agent_trajectories.parquet"
        if p.exists():
            return pd.read_parquet(p)
        return pd.DataFrame()

    def congestion_map(self, time_s: float):
        """
        Load the GeoJSON snapshot nearest to time_s and return a GeoDataFrame
        with congestion level per edge.
        """
        if not HAS_GEOPANDAS:
            raise ImportError("geopandas required for congestion maps")
        import glob
        snapshots = sorted(self.output_dir.glob("network_state_*.geojson"))
        if not snapshots:
            raise FileNotFoundError(f"No GeoJSON snapshots in {self.output_dir}")
        # Find nearest snapshot
        def t_from_path(p: Path) -> float:
            return float(p.stem.split("_")[-1])
        nearest = min(snapshots, key=lambda p: abs(t_from_path(p) - time_s))
        return gpd.read_file(nearest)

    def geh_statistics(self, observed_counts: "pd.DataFrame"):
        """
        Compare simulated link volumes to observed counts.
        observed_counts must have columns: edge_id, observed_veh_h.
        Returns DataFrame with GEH values per edge.
        """
        if not HAS_PANDAS:
            raise ImportError("pandas required")
        stats = self.link_stats()
        if stats.empty:
            return pd.DataFrame()
        merged = stats.merge(observed_counts, on="edge_id")
        sim_col = "inflow"
        obs_col = "observed_veh_h"
        merged["sim_veh_h"] = merged[sim_col] * 3600.0
        diff = merged["sim_veh_h"] - merged[obs_col]
        mean = 0.5 * (merged["sim_veh_h"] + merged[obs_col])
        merged["geh"] = np.sqrt(diff**2 / mean.clip(1e-6))
        merged["geh_pass"] = merged["geh"] < 5.0
        return merged[["edge_id", "sim_veh_h", obs_col, "geh", "geh_pass"]]
