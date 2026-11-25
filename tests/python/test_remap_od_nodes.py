"""Tests for remap_od_nodes.node_map."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python/preprocessing'))
from remap_od_nodes import node_map  # noqa: E402


def test_exact_and_nearest():
    old_lon = np.array([2.0, 2.1, 2.2], np.float32); old_lat = np.array([39.0, 39.1, 39.2], np.float32)
    new_lon = np.array([2.1, 2.0, 2.2001], np.float32); new_lat = np.array([39.1, 39.0, 39.2], np.float32)
    m, near = node_map(old_lon, old_lat, new_lon, new_lat)
    assert list(m) == [1, 0, 2]
    assert len(near) == 1 and near[0][:2] == (2, 2) and 5 < near[0][2] < 15   # ~8.6 m at 39 N
