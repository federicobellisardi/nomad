"""Tests for nomad_tools.io: graph.bin and routes.bin readers on synthetic files
written in the formats of src/core/graph_io.cpp and src/core/simulation.cpp."""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python'))
from nomad_tools import io  # noqa: E402


def vec(a):
    a = np.ascontiguousarray(a)
    return struct.pack('<I', len(a)) + a.tobytes()


def write_graph(path, version=2, trailing=b''):
    # 3 nodes, edges 0: 0->1, 1: 1->2, 2: 2->0, 3: 1->0
    row_ptr = np.array([0, 1, 3, 4], '<u4')
    col_idx = np.array([0, 1, 3, 2], '<u4')        # CSR slot -> edge id
    edges = np.zeros(4, io.EDGE_DTYPE)
    edges['target'] = [1, 2, 0, 0]
    edges['length_m'] = [10, 20, 30, 40]
    edges['capacity'] = [1000, 1800, 900, 1200]
    edges['road_class'] = [0, 4, 10, 15]
    nodes = np.zeros(3, io.NODE_DTYPE)
    nodes['lon'] = [2.0, 2.1, 2.2]; nodes['lat'] = [39.0, 39.1, 39.2]; nodes['intersection_type'] = [0, 3, 4]
    coords = np.zeros(2, io.COORD_DTYPE); coords['lon'] = [2.0, 2.1]
    names = [b'Ma-20', b'']
    b = b'NOMG' + struct.pack('<III', version, 3, 4)
    b += vec(row_ptr) + vec(col_idx) + vec(edges) + vec(nodes)
    b += vec(np.array([0, 2, 2, 2, 2], '<u4')) + vec(coords)
    b += vec(np.array([7, 8, 9], '<u8')) + vec(np.array([100, 200], '<u8'))
    b += struct.pack('<I', len(names)) + b''.join(struct.pack('<I', len(s)) + s for s in names)
    b += vec(np.zeros(1, io.SIGNAL_DTYPE))
    if version >= 2:
        b += vec(np.array([2, 1, 1, 1], '<f4'))
    path.write_bytes(b + trailing)


def test_read_graph_fields_and_sources(tmp_path):
    p = tmp_path / 'g.bin'; write_graph(p)
    g = io.read_graph(p)
    assert (g.num_nodes, g.num_edges, g.version) == (3, 4, 2)
    assert g.way_names == ['Ma-20', '']
    assert list(g.lanes) == [2, 1, 1, 1]
    assert list(g.edge_source) == [0, 1, 2, 1]      # edge 3 leaves node 1 (CSR slot 2)
    assert list(g.out_edges(1)) == [1, 3]


def test_successor_pairs(tmp_path):
    p = tmp_path / 'g.bin'; write_graph(p)
    i, o = io.read_graph(p).successor_pairs()
    pairs = set(zip(i.tolist(), o.tolist()))
    # edge 0 enters node 1 -> edges 1, 3; edge 1 enters 2 -> 2; edges 2, 3 enter 0 -> 0
    assert pairs == {(0, 1), (0, 3), (1, 2), (2, 0), (3, 0)}


def test_read_graph_version1_has_no_lanes(tmp_path):
    p = tmp_path / 'g.bin'; write_graph(p, version=1)
    assert len(io.read_graph(p).lanes) == 0


def test_read_graph_rejects_trailing_bytes(tmp_path):
    p = tmp_path / 'g.bin'; write_graph(p, trailing=b'\0\0')
    with pytest.raises(ValueError, match='trailing'):
        io.read_graph(p)


def test_read_routes(tmp_path):
    agents = [(5, 6, 3600.5, 0, [1, 2, 3]), (7, 8, 7200.0, 0, []), (1, 2, 10.0, 0, [4])]
    b = b'NRTE' + struct.pack('<III', 1, 99, len(agents))
    for o, d, t, m, e in agents:
        b += struct.pack('<IIdII', o, d, t, m, len(e)) + np.array(e, '<u4').tobytes()
    p = tmp_path / 'routes.bin'; p.write_bytes(b)
    r = io.read_routes(p)
    assert len(r) == 3 and r.num_edges == 99
    assert list(r.route(0)) == [1, 2, 3] and list(r.route(1)) == [] and list(r.route(2)) == [4]
    assert r.depart[0] == 3600.5 and list(r.origin) == [5, 7, 1]


def test_snapshots(tmp_path):
    d = tmp_path / 'run' / '2022-07-20_00-00_23-59'; d.mkdir(parents=True)
    for t, n in ((0, 1), (900, 2)):
        feats = [{'properties': {'edge_id': 3, 'count': n, 'n_enter': n, 'n_exit': 0}}]
        (d / f'network_state_{t:06d}.geojson').write_text(json.dumps({'features': feats}))
    run = tmp_path / 'run'
    assert io.snapshot_times(run) == [0, 900]
    assert list(io.read_snapshot(run, 900, 5)['count']) == [0, 0, 0, 2, 0]
    cache = tmp_path / 'daily.npy'
    assert io.daily_entries(run, 5, cache)[3] == 3 and cache.exists()
