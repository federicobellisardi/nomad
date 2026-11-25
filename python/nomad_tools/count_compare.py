"""Simulated daily flow against observed counts on cross-sections.

A section is a segment (lon1, lat1)-(lon2, lat2) drawn across a road. The
simulated flow is the daily number of vehicle entries (sum of n_enter over all
snapshots) on every edge whose geometry crosses the segment and whose road
class is in the section's class list, both directions summed.

Sections CSV columns: an id/count column, a name column, lon1, lat1, lon2,
lat2, classes (';'-separated road class numbers) and optionally confidence.
For the Consell de Mallorca IMD file the count column is 'imd' and the name
column 'road' (the defaults).

usage:
  python -m nomad_tools.count_compare --sections sections.csv --runs <run_dir> [...]
         [--count-col imd] [--name-col road] [--cache-dir DIR]
Prints one row per section (edges matched, simulated flow and sim/observed per
run) and the median ratio per run.
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics

import numpy as np

from . import io


def _orient(p, q, r):
    return np.sign((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))


def segments_cross(a, b, c, d):
    return _orient(a, b, c) != _orient(a, b, d) and _orient(c, d, a) != _orient(c, d, b)


def section_edges(static_features, a, b, classes):
    hit = []
    for ft in static_features:
        p = ft['properties']
        if p['road_class'] not in classes:
            continue
        c = ft['geometry']['coordinates']
        if any(segments_cross(a, b, c[i], c[i + 1]) for i in range(len(c) - 1)):
            hit.append(p['edge_id'])
    return hit


def compare(sections, run_dirs, count_col='imd', name_col='road', cache_dir=None):
    """List of (section row, edge ids, [simulated flow per run])."""
    static = io.static_network(run_dirs[0])
    n_edges = 1 + max(f['properties']['edge_id'] for f in static)
    daily = []
    for r in run_dirs:
        cache = os.path.join(cache_dir, f'daily_{os.path.basename(os.path.normpath(r))}.npy') if cache_dir else None
        daily.append(io.daily_entries(r, n_edges, cache))
    out = []
    for s in sections:
        a = (float(s['lon1']), float(s['lat1'])); b = (float(s['lon2']), float(s['lat2']))
        hit = section_edges(static, a, b, {int(c) for c in s['classes'].split(';')})
        out.append((s, hit, [float(d[hit].sum()) for d in daily]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sections', required=True)
    ap.add_argument('--runs', nargs='+', required=True, help='run output directories')
    ap.add_argument('--count-col', default='imd')
    ap.add_argument('--name-col', default='road')
    ap.add_argument('--cache-dir', help='store/reuse daily per-edge flows as daily_<run>.npy')
    a = ap.parse_args(argv)
    rows = list(csv.DictReader(open(a.sections)))
    res = compare(rows, a.runs, a.count_col, a.name_col, a.cache_dir)
    names = [os.path.basename(os.path.normpath(r)) for r in a.runs]
    print(f"{'count':>7} {'section':24} {'conf':6} {'n_e':>3}" + ''.join(f' {n:>16} {"ratio":>5}' for n in names))
    ratios = [[] for _ in a.runs]
    for s, hit, flows in res:
        obs = float(s[a.count_col])
        line = f"{s[a.count_col]:>7} {s[a.name_col]:24} {s.get('confidence', ''):6} {len(hit):>3}"
        for k, f in enumerate(flows):
            line += f' {f:16.0f} {f / obs:5.2f}'
            ratios[k].append(round(f / obs, 2))
        print(line, ' edges', hit)
    for n, r in zip(names, ratios):
        print(f'{n}: median sim/observed {statistics.median(r):.2f} over {len(r)} sections')


if __name__ == '__main__':
    main()
