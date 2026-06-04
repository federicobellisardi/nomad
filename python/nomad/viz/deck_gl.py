"""
deck.gl / kepler.gl visualization utilities for nomad outputs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Union

import numpy as np


def animate_trajectories(
    positions: np.ndarray,
    times: np.ndarray,
    output_html: Optional[Union[str, Path]] = None,
) -> None:
    """
    Render animated agent trajectories using deck.gl TripsLayer.

    Parameters
    ----------
    positions : ndarray, shape (N_timesteps, N_agents, 2)
        Agent (lon, lat) positions over time.
    times : ndarray, shape (N_timesteps,)
        Simulation times in seconds.
    output_html : path, optional
        Write self-contained HTML file. If None, display in Jupyter.
    """
    try:
        import pydeck as pdk
    except ImportError:
        raise ImportError("pip install pydeck to use animated visualizations")

    # Build trips data: list of {path: [[lon,lat,time], ...]}
    N_t, N_a, _ = positions.shape
    trips = []
    for a in range(N_a):
        path = [
            [float(positions[t, a, 0]), float(positions[t, a, 1]), float(times[t])]
            for t in range(N_t)
        ]
        trips.append({"path": path})

    trips_layer = pdk.Layer(
        "TripsLayer",
        trips,
        get_path="path",
        get_color=[253, 128, 93],
        opacity=0.8,
        width_min_pixels=2,
        rounded=True,
        trail_length=300,
        current_time=float(times[-1]),
    )

    view = pdk.ViewState(
        longitude=float(np.nanmean(positions[:, :, 0])),
        latitude=float(np.nanmean(positions[:, :, 1])),
        zoom=12,
        pitch=45,
    )

    deck = pdk.Deck(layers=[trips_layer], initial_view_state=view,
                     map_style="mapbox://styles/mapbox/dark-v9")

    if output_html:
        deck.to_html(str(output_html))
    else:
        deck.show()


def congestion_map(
    geojson_path: Union[str, Path],
    output_html: Optional[Union[str, Path]] = None,
) -> None:
    """
    Render a congestion heatmap from a GeoJSON link-state snapshot.
    """
    try:
        import pydeck as pdk
    except ImportError:
        raise ImportError("pip install pydeck to use visualizations")
    import json

    with open(geojson_path) as f:
        data = json.load(f)

    layer = pdk.Layer(
        "GeoJsonLayer",
        data,
        pickable=True,
        stroked=True,
        filled=False,
        line_width_min_pixels=1,
        get_line_color="[255 * properties.congestion, 255 * (1 - properties.congestion), 0, 200]",
        get_line_width=2,
    )

    # Compute center from first feature
    feats = data.get("features", [])
    if feats:
        coords = feats[0]["geometry"]["coordinates"]
        lon, lat = coords[0][0], coords[0][1]
    else:
        lon, lat = 0.0, 0.0

    view = pdk.ViewState(longitude=lon, latitude=lat, zoom=12, pitch=0)
    deck = pdk.Deck(layers=[layer], initial_view_state=view,
                     map_style="mapbox://styles/mapbox/dark-v9",
                     tooltip={"text": "Edge {properties.edge_id}\nCongestion: {properties.congestion:.2f}\nSpeed: {properties.travel_time_s:.0f}s"})

    if output_html:
        deck.to_html(str(output_html))
    else:
        deck.show()
