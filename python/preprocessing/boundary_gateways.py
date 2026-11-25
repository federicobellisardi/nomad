"""Boundary gateways for exchange trips between a city's FUA and the rest of its region.

build_od.py keeps only MITMA trips with both ends inside the FUA. Exchange trips
(one end in the FUA, the other in a zone of the surrounding region) need a point
where they enter or leave the simulation graph. For every (outside zone, FUA
zone) pair this script:
  1. computes the fastest path on a region-wide OSM car network (class-default
     free-flow speeds, maxspeed when numeric) from the outside zone to the FUA zone;
  2. takes as gateway the first path node inside the FUA polygon, and as outside
     leg the free-flow time from the outside zone to the gateway;
  3. snaps the gateway to the simulation graph (nodes.parquet / edges.parquet,
     same ids as graph.bin):
       entry node = from-node of a car edge pointing inward (bearing within 60 deg
                    of the path direction), exit node = to-node of a car edge
                    pointing outward;
     same road class first (within --same-class-radius), then any major class,
     then any car class (400 m). A node outside the largest car SCC is moved
     along the carriageway to the nearest SCC node (clipped one-way roads at the
     graph border often dead-end outside the SCC).

Outside zones are the MITMA zones whose representative point lies in
--region-bbox and that are not FUA zones (same zone rule as build_od).

Data layout as build_od.py: <data-dir>/fua/boundaries.gpkg, <data-dir>/od_raw/
(zone .gpkg/.shp), <data-dir>/<city slug>/nodes.parquet and edges.parquet.
Needs osmium (nomad conda env).

Output CSV, one row per (out_zone, fua_zone): ok, gw_lon, gw_lat, gw_class,
outside_s, total_s, entry_node, exit_node, snap distances, walk to SCC, class
fallback flags.

usage:
  python boundary_gateways.py --city "Palma de Mallorca" --data-dir DIR --pbf region.osm.pbf
         --region-bbox 2.3 39.25 3.5 39.97 --out exchange_gateways.csv [--metric-crs 25831]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
import osmium
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, dijkstra
from shapely import prepared
from shapely.geometry import Point

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_od import CAR_MAX_ROAD_CLASS, _largest_scc, find_fua, load_mitma_zones, slugify  # noqa: E402

# OSM highway -> (nomad road_class code, default km/h). Codes as in edges.parquet.
HIGHWAY = {'motorway': (0, 100), 'motorway_link': (1, 60), 'trunk': (2, 90), 'trunk_link': (3, 50),
           'primary': (4, 80), 'primary_link': (5, 40), 'secondary': (6, 70), 'secondary_link': (7, 40),
           'tertiary': (8, 60), 'tertiary_link': (9, 40), 'unclassified': (13, 50),
           'residential': (10, 30), 'living_street': (11, 15)}
# link variants count as the same road when snapping
SAME_ROAD = {0: [0, 1], 1: [0, 1], 2: [2, 3], 3: [2, 3], 4: [4, 5], 5: [4, 5], 6: [6, 7], 7: [6, 7],
             8: [8, 9], 9: [8, 9]}


class _CarWays(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.u, self.v, self.t, self.cls, self.loc = [], [], [], [], {}

    def way(self, w):
        hw = w.tags.get('highway')
        if hw not in HIGHWAY:
            return
        cls, v = HIGHWAY[hw]
        ms = w.tags.get('maxspeed', '')
        if ms.isdigit():
            v = min(float(ms), 120.0)
        ow, junc = w.tags.get('oneway', 'no'), w.tags.get('junction', '')
        fwd = True
        bwd = not (ow in ('yes', 'true', '1') or junc in ('roundabout', 'circular') or hw == 'motorway')
        if ow == '-1':
            fwd, bwd = False, True
        nds = [(n.ref, n.location.lon, n.location.lat) for n in w.nodes if n.location.valid()]
        for (a, xa, ya), (b, xb, yb) in zip(nds, nds[1:]):
            self.loc[a] = (xa, ya); self.loc[b] = (xb, yb)
            d = 6371000 * math.hypot(math.radians(xb - xa) * math.cos(math.radians(ya)), math.radians(yb - ya))
            for s, t, ok in ((a, b, fwd), (b, a, bwd)):
                if ok:
                    self.u.append(s); self.v.append(t); self.t.append(d / (v / 3.6)); self.cls.append(cls)


def region_graph(pbf, bbox):
    """(lon, lat, csr travel-time matrix, edge-class dict, in-largest-SCC mask) of the
    car network inside bbox = (lon0, lat0, lon1, lat1)."""
    h = _CarWays(); h.apply_file(str(pbf), locations=True)
    ids = np.array(list(h.loc)); idx = {k: i for i, k in enumerate(ids)}
    lon = np.array([h.loc[k][0] for k in ids]); lat = np.array([h.loc[k][1] for k in ids])
    U = np.array([idx[x] for x in h.u]); V = np.array([idx[x] for x in h.v])
    T = np.array(h.t); C = np.array(h.cls, dtype=np.int16)
    x0, y0, x1, y1 = bbox
    inb = (lon > x0) & (lon < x1) & (lat > y0) & (lat < y1)
    keep = inb[U] & inb[V]
    U, V, T, C = U[keep], V[keep], T[keep], C[keep]
    n = len(ids)
    G = csr_matrix((T, (U, V)), shape=(n, n))
    _, lab = connected_components(G, directed=True, connection='strong')
    big = np.bincount(lab[np.unique(np.r_[U, V])]).argmax()
    print(f'region graph: {len(U):,} directed edges; largest SCC {(lab == big).sum():,} nodes')
    return lon, lat, G, {(a, b): c for a, b, c in zip(U, V, C)}, lab == big


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--city', required=True)
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--pbf', required=True, help='OSM extract covering the FUA and the region')
    ap.add_argument('--region-bbox', nargs=4, type=float, required=True, metavar=('LON0', 'LAT0', 'LON1', 'LAT1'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--metric-crs', default='auto', help="projected CRS for zone representative points "
                    "(EPSG code, or 'auto' = local UTM)")
    ap.add_argument('--same-class-radius', type=float, default=2000.0,
                    help='search radius (m) for an edge of the crossing road class')
    a = ap.parse_args(argv)
    data, slug = Path(a.data_dir), slugify(a.city)

    lon, lat, G, ecls, inscc = region_graph(a.pbf, a.region_bbox)
    fua = find_fua(a.city, data / 'fua' / 'boundaries.gpkg').geometry
    P = prepared.prep(fua)
    zp = sorted(list((data / 'od_raw').glob('*.gpkg')) + list((data / 'od_raw').glob('*.shp')))[0]
    fz = load_mitma_zones(zp, fua); fz_ids = set(fz['id'].astype(str))
    allz = gpd.read_file(zp)
    idc = [c for c in allz.columns if c.lower() == 'id'][0]
    allz['id'] = allz[idc].astype(str)
    crs = allz.estimate_utm_crs() if a.metric_crs == 'auto' else int(a.metric_crs)
    rp = allz.to_crs(crs).representative_point().to_crs(4326)
    allz['lon'], allz['lat'] = rp.x, rp.y
    x0, y0, x1, y1 = a.region_bbox
    rest = allz[(allz.lon > x0) & (allz.lon < x1) & (allz.lat > y0) & (allz.lat < y1) & ~allz.id.isin(fz_ids)]
    print(f'FUA zones {len(fz_ids)}, outside zones in the region {len(rest)}')

    nd = pd.read_parquet(data / slug / 'nodes.parquet')
    ed = pd.read_parquet(data / slug / 'edges.parquet')
    ced = ed[ed.road_class <= CAR_MAX_ROAD_CLASS].copy()
    car_scc = _largest_scc(ced.from_node.to_numpy(), ced.to_node.to_numpy())
    out_e = ced.groupby('from_node').apply(lambda d: list(zip(d.to_node, d.road_class, d.length_m))).to_dict()
    in_e = ced.groupby('to_node').apply(lambda d: list(zip(d.from_node, d.road_class, d.length_m))).to_dict()

    def walk_to_scc(node, allowed, downstream):
        walked = 0.0
        adj = out_e if downstream else in_e
        for _ in range(200):
            if node in car_scc:
                return node, walked
            nxt = [t for t in adj.get(node, []) if t[1] in allowed] or adj.get(node, [])
            if not nxt:
                return None, walked
            node, _, L = nxt[0]; walked += L
        return None, walked

    nlon, nlat = nd.set_index('node_id').lon, nd.set_index('node_id').lat
    ced['x0'], ced['y0'] = nlon[ced.from_node].values, nlat[ced.from_node].values
    ced['x1'], ced['y1'] = nlon[ced.to_node].values, nlat[ced.to_node].values
    ced['bx'] = (ced.x1 - ced.x0) * np.cos(np.radians(ced.y0)); ced['by'] = ced.y1 - ced.y0

    # FUA zone targets: mean position of the zone's car-SCC nodes (a zone centroid can lie at sea)
    ng = gpd.GeoDataFrame(nd, geometry=gpd.points_from_xy(nd.lon, nd.lat), crs=4326)
    ng = ng[ng.node_id.isin(car_scc)]
    j = gpd.sjoin(ng, fz.to_crs(4326)[['id', 'geometry']], predicate='within')
    ftarget = j.groupby('id')[['lon', 'lat']].mean()

    def nearest_region_node(x, y):
        d = (lon - x) ** 2 * math.cos(math.radians(y)) ** 2 + (lat - y) ** 2
        d[~inscc] = np.inf
        return int(np.argmin(d))

    def snap(x, y, dirx, diry, cls, inward):
        tiers = [SAME_ROAD.get(cls, [cls]), list(range(0, 10)), list(range(0, 14))]
        radius = {0: a.same_class_radius, 1: 400.0, 2: 400.0}
        for ti, allowed in enumerate(tiers):
            c = ced[ced.road_class.isin(allowed)]
            # point-to-segment distance (m, local equirectangular frame): long motorway edges
            # have midpoints far from the crossing
            kx, ky = 111320 * math.cos(math.radians(y)), 111320
            ax, ay = (c.x0 - x) * kx, (c.y0 - y) * ky; bx, by = (c.x1 - x) * kx, (c.y1 - y) * ky
            dx, dy = bx - ax, by - ay
            tt = np.clip(-(ax * dx + ay * dy) / (dx * dx + dy * dy + 1e-9), 0, 1)
            dist = np.hypot(ax + tt * dx, ay + tt * dy)
            sgn = 1 if inward else -1
            cosang = sgn * (c.bx * dirx + c.by * diry) / (np.hypot(c.bx, c.by) * math.hypot(dirx, diry) + 1e-12)
            ok = (dist < radius[ti]) & (cosang > 0.5)
            if ok.any():
                k = dist[ok].idxmin(); e = c.loc[k]
                node, walked = walk_to_scc(int(e.from_node if inward else e.to_node), allowed, downstream=inward)
                if node is None:
                    continue
                return node, float(dist[k]), int(e.road_class), allowed is not tiers[0], walked
        return None, None, None, None, None

    src = {z: nearest_region_node(x, y) for z, x, y in zip(rest.id, rest.lon, rest.lat)}
    tgt = {z: nearest_region_node(r.lon, r.lat) for z, r in ftarget.iterrows()}
    inside = np.array([P.contains(Point(x, y)) for x, y in zip(lon, lat)])
    rows = []
    for oz, s in src.items():
        if inside[s]:
            print(f'  WARN outside zone {oz} snaps inside the FUA'); continue
        dist, pred = dijkstra(G, directed=True, indices=s, return_predecessors=True)
        for fzid, t in tgt.items():
            if not np.isfinite(dist[t]):
                rows.append(dict(out_zone=oz, fua_zone=fzid, ok=False)); continue
            path = [t]
            while path[-1] != s:
                path.append(pred[path[-1]])
            path = path[::-1]
            k = next(i for i, p in enumerate(path) if inside[p])
            g, gp = path[k], path[k - 1]
            cls = ecls[(gp, g)]
            dirx = (lon[g] - lon[gp]) * math.cos(math.radians(lat[g])); diry = lat[g] - lat[gp]
            en, en_d, en_c, en_fb, en_w = snap(lon[g], lat[g], dirx, diry, cls, True)
            ex, ex_d, ex_c, ex_fb, ex_w = snap(lon[g], lat[g], dirx, diry, cls, False)
            rows.append(dict(out_zone=oz, fua_zone=fzid, ok=en is not None and ex is not None,
                             gw_lon=lon[g], gw_lat=lat[g], gw_class=cls, outside_s=dist[g], total_s=dist[t],
                             entry_node=en, entry_snap_m=en_d, entry_walk_m=en_w, entry_class=en_c, entry_fallback=en_fb,
                             exit_node=ex, exit_snap_m=ex_d, exit_walk_m=ex_w, exit_class=ex_c, exit_fallback=ex_fb))
    gw = pd.DataFrame(rows)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    gw.to_csv(a.out, index=False)
    print(f'pairs {len(gw)}, snapped {gw.ok.sum()}, failed {(~gw.ok).sum()}')
    print('class fallback: entry %d exit %d' % (gw.entry_fallback.fillna(False).sum(), gw.exit_fallback.fillna(False).sum()))
    print('snap distance m (entry) p50 %.0f p95 %.0f max %.0f' % tuple(gw.entry_snap_m.quantile([.5, .95, 1])))
    print('walk to SCC m: entry max %.0f, exit max %.0f' % (gw.entry_walk_m.max(), gw.exit_walk_m.max()))
    print('outside leg min: p50 %.1f p95 %.1f max %.1f' % tuple(gw.outside_s.quantile([.5, .95, 1]) / 60))
    g2 = gw[gw.ok].groupby('entry_node').agg(lon=('gw_lon', 'first'), lat=('gw_lat', 'first'),
                                             cls=('gw_class', 'first'), pairs=('fua_zone', 'size'))
    print('distinct entry gateways:', len(g2)); print(g2.sort_values('pairs', ascending=False).to_string())


if __name__ == '__main__':
    main()
