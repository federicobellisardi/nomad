"""Remap the node ids of an OD CSV from one node table to a graph.bin built later.

Rule (as for the Madrid OD, workspace PROTOCOL): an old node maps to the new node
with identical (lon, lat) as float32; an old node without an exact match maps to
the nearest new node (equirectangular distance). The OD rows are otherwise
copied unchanged. A JSON manifest records the counts and the unmatched nodes.

usage: python remap_od_nodes.py --od od.csv --nodes old_nodes.parquet --graph new.bin --out out.csv
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nomad_tools import io  # noqa: E402


def node_map(old_lon, old_lat, new_lon, new_lat):
    """(mapping array old id -> new id, list of (old id, new id, metres) for nearest matches)."""
    new = {(x, y): i for i, (x, y) in enumerate(zip(new_lon.tolist(), new_lat.tolist()))}
    out = np.full(len(old_lon), -1, np.int64)
    near = []
    for i, (x, y) in enumerate(zip(old_lon.tolist(), old_lat.tolist())):
        j = new.get((x, y))
        if j is not None:
            out[i] = j
            continue
        d = np.hypot((new_lon - x) * np.cos(np.radians(y)), new_lat - y) * 111320.0
        j = int(np.argmin(d)); out[i] = j; near.append((i, j, float(d[j])))
    return out, near


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--od', required=True); ap.add_argument('--nodes', required=True)
    ap.add_argument('--graph', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    if Path(a.out).exists():
        sys.exit(f'{a.out} exists: not overwritten')
    nd = pd.read_parquet(a.nodes).sort_values('node_id')
    if not (nd.node_id.to_numpy() == np.arange(len(nd))).all():
        sys.exit('old node table ids are not 0..N-1')
    g = io.read_graph(a.graph)
    m, near = node_map(nd.lon.to_numpy(np.float32), nd.lat.to_numpy(np.float32),
                       g.nodes['lon'], g.nodes['lat'])
    od = pd.read_csv(a.od)
    od['origin_node'] = m[od['origin_node'].to_numpy()]
    od['dest_node'] = m[od['dest_node'].to_numpy()]
    od.to_csv(a.out, index=False)
    md5 = lambda p: hashlib.md5(open(p, 'rb').read()).hexdigest()
    man = dict(output=a.out, source_od=a.od, source_od_md5=md5(a.od), old_nodes=a.nodes, graph=a.graph,
               graph_md5=md5(a.graph), rule='old node -> new node with identical float32 (lon, lat); '
               'otherwise the nearest new node', old_nodes_n=len(nd), new_nodes_n=g.num_nodes,
               exact=len(nd) - len(near), nearest=[dict(old=i, new=j, metres=round(d, 1)) for i, j, d in near],
               rows=len(od), md5=md5(a.out))
    json.dump(man, open(a.out + '.manifest.json', 'w'), indent=1)
    print(f'{len(nd) - len(near):,} exact, {len(near)} nearest (max {max([d for *_, d in near], default=0):.0f} m); '
          f'{len(od):,} rows -> {a.out}')


if __name__ == '__main__':
    main()
