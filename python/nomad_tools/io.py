"""Readers for nomad binary files and run outputs.

graph.bin   (src/core/graph_io.cpp)    "NOMG" u32 version, u32 N, u32 E, then
            packed vectors (u32 count + data): row_ptr, col_idx, edges, nodes,
            geom_ptr, geom_coords, osm_node_ids, way_ids, way_names (strings),
            signal_schedules, edge_lanes (version >= 2).
routes.bin  (src/core/simulation.cpp)  "NRTE" u32 version=1, u32 num_edges, u32 n_agents,
            per agent u32 origin, u32 dest, f64 depart, u32 mode, u32 len, u32 edges[len].
Run output  <run>/<date>_00-00_23-59/network_state_<t:06d>.geojson (one per snapshot)
            and static_network.geojson.
"""
from __future__ import annotations

import glob
import json
import os
import struct
from dataclasses import dataclass, field

import numpy as np

EDGE_DTYPE = np.dtype([('target', '<u4'), ('length_m', '<f4'), ('free_flow_speed', '<f4'),
                       ('capacity', '<f4'), ('road_class', 'u1'), ('flags', 'u1'),
                       ('way_meta_idx', '<u2')])
NODE_DTYPE = np.dtype([('lon', '<f4'), ('lat', '<f4'), ('intersection_type', 'u1'),
                       ('signal_phase_idx', 'u1'), ('pad', '<u2'), ('osm_node_idx', '<u4')])
SIGNAL_DTYPE = np.dtype([('cycle_s', '<u2'), ('green_pct', 'u1'), ('pad', 'u1')])
COORD_DTYPE = np.dtype([('lon', '<f4'), ('lat', '<f4')])
# RoadClass enum values (include/nomad/core/types.hpp)
ROAD_CLASS = {0: 'motorway', 1: 'motorway_link', 2: 'trunk', 3: 'trunk_link', 4: 'primary',
              5: 'primary_link', 6: 'secondary', 7: 'secondary_link', 8: 'tertiary',
              9: 'tertiary_link', 10: 'residential', 11: 'living_street', 12: 'service',
              13: 'unclassified', 14: 'track', 15: 'cycleway', 16: 'footway', 17: 'path',
              18: 'steps', 255: 'unknown'}
CAR_MAX_ROAD_CLASS = 14   # classes above track are not car-accessible
INTERSECTION_TYPE = {0: 'simple', 1: 'stop', 2: 'all_stop', 3: 'signal', 4: 'roundabout',
                     5: 'mini_roundabout', 6: 'motorway_junction'}


@dataclass
class Graph:
    version: int
    row_ptr: np.ndarray
    col_idx: np.ndarray
    edges: np.ndarray
    nodes: np.ndarray
    geom_ptr: np.ndarray
    geom_coords: np.ndarray
    osm_node_ids: np.ndarray
    way_ids: np.ndarray
    way_names: list
    signal_schedules: np.ndarray
    lanes: np.ndarray            # per edge; empty for version 1 graphs
    edge_source: np.ndarray = field(init=False)

    def __post_init__(self):
        n = len(self.row_ptr) - 1
        self.edge_source = np.empty(len(self.edges), np.int64)
        self.edge_source[self.col_idx] = np.repeat(np.arange(n), np.diff(self.row_ptr))

    @property
    def num_nodes(self):
        return len(self.row_ptr) - 1

    @property
    def num_edges(self):
        return len(self.edges)

    def out_edges(self, node):
        return self.col_idx[self.row_ptr[node]:self.row_ptr[node + 1]]

    def successor_pairs(self):
        """(i, o) arrays: edge o leaves the node that edge i enters."""
        t = self.edges['target']
        deg = np.diff(self.row_ptr)[t]
        i = np.repeat(np.arange(self.num_edges), deg)
        starts = self.row_ptr[t]
        o = self.col_idx[np.concatenate([np.arange(s, s + d) for s, d in zip(starts, deg)])] \
            if len(i) else np.empty(0, np.int64)
        return i, o


def read_graph(path) -> Graph:
    b = open(path, 'rb').read()
    if b[:4] != b'NOMG':
        raise ValueError(f'{path}: not a nomad graph (bad magic)')
    version, n_nodes, n_edges = struct.unpack_from('<III', b, 4)
    if version not in (1, 2):
        raise ValueError(f'{path}: unsupported graph version {version}')
    off = 16

    def vec(dtype):
        nonlocal off
        n = struct.unpack_from('<I', b, off)[0]
        off += 4
        a = np.frombuffer(b, dtype, n, off)
        off += a.nbytes
        return a

    def strings():
        nonlocal off
        n = struct.unpack_from('<I', b, off)[0]
        off += 4
        out = []
        for _ in range(n):
            ln = struct.unpack_from('<I', b, off)[0]
            out.append(b[off + 4:off + 4 + ln].decode('utf-8', 'replace'))
            off += 4 + ln
        return out

    row_ptr, col_idx = vec('<u4'), vec('<u4')
    edges, nodes = vec(EDGE_DTYPE), vec(NODE_DTYPE)
    geom_ptr, geom_coords = vec('<u4'), vec(COORD_DTYPE)
    osm_node_ids, way_ids = vec('<u8'), vec('<u8')
    way_names = strings()
    signals = vec(SIGNAL_DTYPE)
    lanes = vec('<f4') if version >= 2 else np.empty(0, '<f4')
    if off != len(b):
        raise ValueError(f'{path}: {len(b) - off} trailing bytes; format mismatch')
    if len(row_ptr) != n_nodes + 1 or len(edges) != n_edges or len(col_idx) != n_edges:
        raise ValueError(f'{path}: header counts do not match the vectors')
    return Graph(version, row_ptr, col_idx, edges, nodes, geom_ptr, geom_coords,
                 osm_node_ids, way_ids, way_names, signals, lanes)


@dataclass
class Routes:
    num_edges: int
    origin: np.ndarray
    dest: np.ndarray
    depart: np.ndarray
    mode: np.ndarray
    offsets: np.ndarray          # agent k's edges are edges[offsets[k]:offsets[k+1]]
    edges: np.ndarray

    def __len__(self):
        return len(self.origin)

    def route(self, k):
        return self.edges[self.offsets[k]:self.offsets[k + 1]]


def read_routes(path) -> Routes:
    b = open(path, 'rb').read()
    if b[:4] != b'NRTE':
        raise ValueError(f'{path}: not a route plan file (bad magic)')
    version, num_edges, n = struct.unpack_from('<III', b, 4)
    if version != 1:
        raise ValueError(f'{path}: unsupported routes version {version}')
    rec = struct.Struct('<IIdII')
    origin = np.empty(n, np.uint32); dest = np.empty(n, np.uint32)
    depart = np.empty(n, np.float64); mode = np.empty(n, np.uint32)
    offsets = np.zeros(n + 1, np.int64)
    starts = np.empty(n, np.int64)
    off = 16
    for k in range(n):
        origin[k], dest[k], depart[k], mode[k], ln = rec.unpack_from(b, off)
        off += rec.size
        starts[k] = off
        offsets[k + 1] = offsets[k] + ln
        off += 4 * ln
    if off != len(b):
        raise ValueError(f'{path}: {len(b) - off} trailing bytes; format mismatch')
    edges = np.empty(offsets[-1], np.uint32)
    for k in range(n):
        ln = offsets[k + 1] - offsets[k]
        if ln:
            edges[offsets[k]:offsets[k + 1]] = np.frombuffer(b, '<u4', ln, starts[k])
    return Routes(num_edges, origin, dest, depart, mode, offsets, edges)


def snapshot_dir(run_dir) -> str:
    """The folder holding network_state_*.geojson inside a run directory."""
    cands = sorted({os.path.dirname(p) for p in glob.glob(os.path.join(run_dir, '*', 'network_state_*.geojson'))})
    if len(cands) != 1:
        raise FileNotFoundError(f'{run_dir}: expected one snapshot folder, found {cands}')
    return cands[0]


def snapshot_times(run_dir):
    d = snapshot_dir(run_dir)
    return sorted(int(os.path.basename(p)[14:20]) for p in glob.glob(os.path.join(d, 'network_state_*.geojson')))


def read_snapshot(run_dir, t, num_edges=None, fields=('count', 'n_enter', 'n_exit')):
    """Per-edge arrays of the snapshot at time t (seconds). Edges absent from the
    file are 0. num_edges defaults to 1 + the largest edge id in the file."""
    feats = json.load(open(os.path.join(snapshot_dir(run_dir), f'network_state_{int(t):06d}.geojson')))['features']
    ids = np.fromiter((f['properties']['edge_id'] for f in feats), np.int64, len(feats))
    n = num_edges if num_edges is not None else (int(ids.max()) + 1 if len(ids) else 0)
    out = {}
    for k in fields:
        a = np.zeros(n)
        a[ids] = [f['properties'].get(k, 0) for f in feats]
        out[k] = a
    return out


def daily_entries(run_dir, num_edges, cache=None):
    """Daily vehicle entries per edge: sum of n_enter over all snapshots.
    cache: optional .npy path, read if present, written otherwise."""
    if cache and os.path.exists(cache):
        return np.load(cache)
    tot = np.zeros(num_edges)
    for t in snapshot_times(run_dir):
        tot += read_snapshot(run_dir, t, num_edges, ('n_enter',))['n_enter']
    if cache:
        np.save(cache, tot)
    return tot


def static_network(run_dir):
    return json.load(open(os.path.join(snapshot_dir(run_dir), 'static_network.geojson')))['features']
