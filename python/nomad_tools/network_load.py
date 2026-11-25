"""Vehicles on the network (sum of per-edge counts) at given snapshot times.

usage: python -m nomad_tools.network_load --runs <run_dir> [...] [--hours 6 7 8 ... 23.75]
Hours are decimal (23.75 = 23:45); each must match a snapshot time.
"""
from __future__ import annotations

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

from . import io

DEFAULT_HOURS = [6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20, 22, 23.75]


def vehicles_on_network(run_dir, t):
    p = os.path.join(io.snapshot_dir(run_dir), f'network_state_{int(t):06d}.geojson')
    return int(sum(f['properties']['count'] for f in json.load(open(p))['features']))


def _one(a):
    return vehicles_on_network(*a)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--hours', nargs='+', type=float, default=DEFAULT_HOURS)
    ap.add_argument('--workers', type=int, default=8)
    a = ap.parse_args(argv)
    jobs = [(r, round(h * 3600)) for r in a.runs for h in a.hours]
    with ProcessPoolExecutor(a.workers) as ex:
        v = list(ex.map(_one, jobs))
    names = [os.path.basename(os.path.normpath(r)) for r in a.runs]
    print('time  ' + ''.join(f'{n:>{max(26, len(n) + 2)}}' for n in names))
    nh = len(a.hours)
    for j, h in enumerate(a.hours):
        print(f'{int(h):02d}:{round(h % 1 * 60):02d} ' + ''.join(f'{v[i * nh + j]:{max(26, len(names[i]) + 2)}d}' for i in range(len(a.runs))))


if __name__ == '__main__':
    main()
