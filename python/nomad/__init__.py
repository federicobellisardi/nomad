"""
nomad — Network-based Open Mobility Agent Dynamics
A modern, research-grade urban mobility simulator.
"""
from __future__ import annotations

try:
    from nomad._nomad_core import (
        AgentMode,
        AgentState,
        Graph,
        GravityDemand,
        NetworkCleaner,
        OdMatrixDemand,
        OsmLoader,
        RouteCache,
        Simulation,
        SimulationConfig,
        ScenarioConfig,
        __version__,
    )
except ImportError as e:
    raise ImportError(
        "nomad C++ extension (_nomad_core) not found. "
        "Build the project with CMake first:\n"
        "  cmake -B build -DNOMAD_BUILD_PYTHON=ON && cmake --build build\n"
        f"Original error: {e}"
    ) from e

from nomad.scenario import Scenario
from nomad.analysis import Analysis

__all__ = [
    "Scenario",
    "Analysis",
    "Simulation",
    "SimulationConfig",
    "ScenarioConfig",
    "Graph",
    "OsmLoader",
    "NetworkCleaner",
    "OdMatrixDemand",
    "GravityDemand",
    "RouteCache",
    "AgentMode",
    "AgentState",
    "__version__",
]
