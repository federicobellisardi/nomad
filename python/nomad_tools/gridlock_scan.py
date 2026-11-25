"""Full links and spillback cycles.

A link is full when its count is >= 0.95 x storage, with storage =
max(50 m, length) x lanes / 7.5 m (lanes from graph.bin, or capacity/1600 with
--storage capacity; diagnostic, not the LTM storage). Two full links are
connected when the second leaves the node the first enters; strongly connected
components (SCC) of size > 1 are spillback cycles, the gridlock signature.

Modes:
  time series   python -m nomad_tools.gridlock_scan --graph G --run R
                (every 30 min from 06:00: full links, links in cycles, largest cycle)
  bounding box  python -m nomad_tools.gridlock_scan --graph G --run R --bbox lat0 lat1 lon0 lon1
                --times 64800 72000 [--links]
                (cycles among car links with both ends in the box; --links prints
                 each link with count, count/storage and exit flow/capacity)
"""
from __future__ import annotations

import argparse
from collections import Counter

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from . import io

SHORT = {0: 'mw', 1: 'mwl', 2: 'tr', 3: 'trl', 4: 'pri', 5: 'pril', 6: 'sec', 7: 'secl', 8: 'ter',
         9: 'terl', 10: 'res', 11: 'liv', 12: 'svc', 13: 'unc', 14: 'trk'}


def storage(g, mode='lanes'):
    lanes = np.maximum(1, g.lanes) if mode == 'lanes' else np.maximum(1, g.edges['capacity'] / 1600)
    return np.maximum(50, g.edges['length_m']) * lanes / 7.5


def full_cycles(g, count, stor, mask=None, pairs=None):
    """(full mask, list of (size, member edge ids)) for SCCs of size > 1 among full links."""
    full = count >= 0.95 * stor
    if mask is not None:
        full &= mask
    i, o = pairs if pairs is not None else g.successor_pairs()
    m = full[i] & full[o]
    a = csr_matrix((np.ones(m.sum()), (i[m], o[m])), shape=(g.num_edges, g.num_edges))
    _, lab = connected_components(a, directed=True, connection='strong')
    sizes = Counter(lab[full])
    cycles = sorted(((s, np.nonzero((lab == l) & full)[0]) for l, s in sizes.items() if s > 1),
                    key=lambda t: -t[0])
    return full, cycles


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--graph', required=True)
    ap.add_argument('--run', required=True, help='run output directory')
    ap.add_argument('--storage', choices=['lanes', 'capacity'], default='lanes')
    ap.add_argument('--bbox', nargs=4, type=float, metavar=('LAT0', 'LAT1', 'LON0', 'LON1'))
    ap.add_argument('--times', nargs='+', type=int, help='snapshot times in seconds')
    ap.add_argument('--max-class', type=int, default=io.CAR_MAX_ROAD_CLASS,
                    help='highest road class counted as car link (default: track)')
    ap.add_argument('--links', action='store_true', help='bbox mode: print every link')
    a = ap.parse_args(argv)
    g = io.read_graph(a.graph)
    stor = storage(g, a.storage)
    pairs = g.successor_pairs()
    car = g.edges['road_class'] <= a.max_class
    nd, src, tgt = g.nodes, g.edge_source, g.edges['target']
    times = a.times or [t for t in io.snapshot_times(a.run) if t >= 6 * 3600 and t % 1800 == 0]
    if a.bbox:
        la0, la1, lo0, lo1 = a.bbox
        inb = lambda n: (nd['lat'][n] >= la0) & (nd['lat'][n] <= la1) & (nd['lon'][n] >= lo0) & (nd['lon'][n] <= lo1)
        mask = car & inb(src) & inb(tgt)
        sel = np.nonzero(mask)[0]
        print(f'{len(sel)} car links in bbox (classes <= {a.max_class}); storage {a.storage}')
        snaps = {t: io.read_snapshot(a.run, t, g.num_edges) for t in times}
        if a.links:
            print(f"{'edge':>7} {'from':>6}->{'to':<6} {'cls':4} {'L':>3} {'len':>5} {'cap':>5} {'stor':>5} {'node_in':17}"
                  + ''.join(f' | t={t:>6} n  n/st ex/cap' for t in times))
            for x in sorted(sel, key=lambda x: (round(float(nd['lat'][src[x]]), 4), x)):
                e = g.edges[x]
                line = (f"{x:7d} {src[x]:6d}->{tgt[x]:<6d} {SHORT.get(int(e['road_class']), '?'):4s} "
                        f"{(g.lanes[x] if len(g.lanes) else 0):3.1f} {e['length_m']:5.0f} {e['capacity']:5.0f} {stor[x]:5.1f} "
                        f"{io.INTERSECTION_TYPE.get(int(nd['intersection_type'][tgt[x]]), '?'):17s}")
                for t in times:
                    s = snaps[t]
                    line += f" | {'':>8} {s['count'][x]:4.0f} {s['count'][x] / stor[x]:5.2f} {s['n_exit'][x] * 4 / max(e['capacity'], 1):5.2f}"
                print(line)
        for t in times:
            cnt = snaps[t]['count']
            full, cyc = full_cycles(g, cnt, stor, mask, pairs)
            print(f'\nt={t}: full links in bbox {full.sum()}, veh on them {cnt[full].sum():.0f}; cycles (size>1): {len(cyc)}')
            for s, mem in cyc:
                print(f'  size {s}: veh {cnt[mem].sum():.0f}, links {list(map(int, mem))}')
        return
    print(f'run={a.run} storage={a.storage} car classes <= {a.max_class}')
    print('time   full  inSCC  vehSCC  nSCC   big  big_veh  big_lat  big_lon  big_classes')
    for t in times:
        cnt = io.read_snapshot(a.run, t, g.num_edges, ('count',))['count']
        full, cyc = full_cycles(g, cnt, stor, car, pairs)
        insc = np.zeros(g.num_edges, bool)
        for _, mem in cyc:
            insc[mem] = True
        if cyc:
            s, mem = cyc[0]
            cl = Counter(SHORT.get(int(r), str(r)) for r in g.edges['road_class'][mem]).most_common(3)
            bl, bo, bv = nd['lat'][src[mem]].mean(), nd['lon'][src[mem]].mean(), cnt[mem].sum()
        else:
            s, cl, bl, bo, bv = 0, [], 0, 0, 0
        print(f'{t // 3600:02d}:{t % 3600 // 60:02d} {full.sum():6d} {insc.sum():6d} {cnt[insc].sum():7.0f} '
              f'{len(cyc):5d} {s:5d} {bv:8.0f} {bl:8.4f} {bo:8.4f}  {cl}')


if __name__ == '__main__':
    main()
