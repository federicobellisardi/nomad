"""Exchange trips between a city's FUA and the rest of its region, as OD rows.

Companion of build_od.py for trips with one end in the FUA and the other in an
outside zone of the region (boundary_gateways.py). Generation follows build_od
exactly (same MITMA files, weekday mean, noise, occupancy, mode choice, trip-end
draw and optional node weights). Differences:
- only MITMA rows between an outside zone and a FUA zone are kept;
- the outside end is the gateway of that (outside zone, FUA zone) pair: its
  entry node when the trip starts outside, its exit node when it ends outside;
- only car rows are kept (walk/bike/transit legs from a road gateway are
  meaningless in a car simulation);
- inbound departure windows are shifted by the free-flow outside leg
  (gateway outside_s), so the vehicle appears at the gateway when it would reach
  the FUA border; shifted windows are clipped at --day-end (with nested sampling
  the two depart columns are a window [start, end]);
- own RNG stream (--seed), so the internal rows of the base OD are untouched.

Outputs:
- --out-rows: the exchange rows (OD_ROW_COLUMNS) and a <stem>_cells.parquet
  sidecar with (orig_zone, dest_zone, period, distance_band) per row;
- --out-combined (optional, with --base-od): base OD + exchange rows, plus a
  .manifest.json. Existing files are never overwritten.

Mode choice: by default build_od's distance-band fractions. With
--green-mobility the green_mobility ModeChoiceModel and city transit share of
the city slug are used (as green_mobility run_build_od does).

usage:
  python exchange_trips.py --city "Palma de Mallorca" --data-dir DIR --months 2022-02
         --gateways exchange_gateways.csv --out-rows exchange_trips.csv
         [--node-weights W.parquet] [--base-od base.csv --out-combined combined.csv]
         [--green-mobility] [--seed 43]
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_od as B  # noqa: E402


def _md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()


def gateway_offsets(rows, gw, day_end):
    """Shift inbound windows (orig_zone 'XO|out|fua') by outside_s and clip their end
    at day_end. Returns (rows, number of clipped rows, clipped vehicles)."""
    g = gw.set_index(['out_zone', 'fua_zone'])
    inbound = rows.orig_zone.str.startswith('XO|')
    key = rows.loc[inbound, 'orig_zone'].str.split('|')
    off = np.array([g.loc[(k[1], k[2]), 'outside_s'] for k in key])
    rows.loc[inbound, 'depart_mean_s'] += off
    rows.loc[inbound, 'depart_std_s'] += off
    if (rows.depart_mean_s >= day_end).any():
        raise ValueError('a shifted departure window starts after the end of the day')
    late = rows.depart_std_s > day_end
    n_late, v_late = int(late.sum()), int(rows.loc[late, 'count'].sum())
    rows.loc[late, 'depart_std_s'] = day_end
    return rows, n_late, v_late, off


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--city', required=True)
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--months', required=True, help='comma-separated od_raw month folders, e.g. 2022-02')
    ap.add_argument('--gateways', required=True)
    ap.add_argument('--out-rows', required=True)
    ap.add_argument('--node-weights')
    ap.add_argument('--base-od')
    ap.add_argument('--out-combined')
    ap.add_argument('--green-mobility', action='store_true')
    ap.add_argument('--seed', type=int, default=43)
    ap.add_argument('--noise-sigma', type=float, default=0.08)
    ap.add_argument('--occupancy-factor', type=float, default=1.2)
    ap.add_argument('--country', default='ES')
    ap.add_argument('--day-end', type=float, default=86400.0)
    a = ap.parse_args(argv)
    if bool(a.base_od) != bool(a.out_combined):
        sys.exit('--base-od and --out-combined go together')
    for p in (a.out_rows, a.out_combined):
        if p and Path(p).exists():
            sys.exit(f'{p} exists: not overwritten')
    data, slug = Path(a.data_dir), B.slugify(a.city)

    fua = B.find_fua(a.city, data / 'fua' / 'boundaries.gpkg').geometry
    zp = sorted(list((data / 'od_raw').glob('*.gpkg')) + list((data / 'od_raw').glob('*.shp')))[0]
    fz = B.load_mitma_zones(zp, fua); fz_ids = set(fz['id'].astype(str))
    gw = pd.read_csv(a.gateways, dtype={'out_zone': str, 'fua_zone': str})
    if not gw.ok.all():
        sys.exit('gateway table has unsnapped pairs')
    rest_ids = set(gw.out_zone)
    print(f'FUA zones {len(fz_ids)}, outside zones {len(rest_ids)}, gateway pairs {len(gw)}')

    nodes = pd.read_parquet(data / slug / 'nodes.parquet'); edges = pd.read_parquet(data / slug / 'edges.parquet')
    ce = edges[edges.road_class <= B.CAR_MAX_ROAD_CLASS]
    car_nodes = B._largest_scc(ce.from_node.to_numpy(), ce.to_node.to_numpy())
    ng = gpd.GeoDataFrame({'node_id': nodes.node_id}, geometry=gpd.points_from_xy(nodes.lon, nodes.lat), crs='EPSG:4326')
    j = gpd.sjoin(ng, fz[['id', 'geometry']].rename(columns={'id': 'mitma_zone'}), how='inner', predicate='within')
    zone_to_nodes = j.groupby('mitma_zone')['node_id'].apply(list).to_dict()

    files = sorted(f for m in a.months.split(',') for f in (data / 'od_raw' / m.strip()).glob('*iajes*.csv.gz'))
    if not files:
        sys.exit('no MITMA viajes files for the given months')
    cols = B.detect_columns(files[0])
    hol = B.build_holidays_set({int(f.name[:4]) for f in files}, a.country) if B._HOLIDAYS_AVAILABLE else set()
    if not hol:
        sys.exit('holidays package missing: weekday selection would differ from build_od')
    od = B.stream_viajes(files, cols, fz_ids | rest_ids, False, hol)
    O, D = od[cols['orig']].astype(str), od[cols['dest']].astype(str)
    inb = O.isin(rest_ids) & D.isin(fz_ids); outb = O.isin(fz_ids) & D.isin(rest_ids)
    ex = od[inb | outb].copy()
    print(f'exchange MITMA rows {len(ex):,}, person trips/day {ex[cols["trips"]].sum():,.0f} '
          f'(inbound {od[inb][cols["trips"]].sum():,.0f}, outbound {od[outb][cols["trips"]].sum():,.0f})')

    # pseudo-zones: the outside end of each pair maps to its gateway node
    g = gw.set_index(['out_zone', 'fua_zone'])
    ztn = dict(zone_to_nodes)

    def pseudo(row):
        o, d = str(row[cols['orig']]), str(row[cols['dest']])
        if o in rest_ids:
            k = f'XO|{o}|{d}'; ztn[k] = [int(g.loc[(o, d), 'entry_node'])]; return k, d
        k = f'XD|{d}|{o}'; ztn[k] = [int(g.loc[(d, o), 'exit_node'])]; return o, k
    pz = ex.apply(pseudo, axis=1, result_type='expand')
    ex[cols['orig']], ex[cols['dest']] = pz[0].values, pz[1].values

    mode_choice_fn = transit = None
    if a.green_mobility:
        from green_mobility.demand.mode_choice import CITY_TRANSIT_SHARE, ModeChoiceModel
        mode_choice_fn, transit = ModeChoiceModel(slug).predict_proba, CITY_TRANSIT_SHARE.get(slug)
    node_weight = None
    if a.node_weights:
        nw = pd.read_parquet(a.node_weights)
        node_weight = dict(zip(nw['node_id'].astype(int), nw['weight'].astype(float)))
    rng = np.random.default_rng(a.seed)
    rows = B.build_od_rows(ex, ztn, cols, rng, a.noise_sigma, car_nodes=car_nodes, scale=1.0,
                           occupancy_factor=a.occupancy_factor, mode_choice_fn=mode_choice_fn,
                           transit_frac_override=transit, with_cells=True, node_weight=node_weight)
    rows = rows[rows['mode'] == 'car'].copy()
    rows, n_late, v_late, off = gateway_offsets(rows, gw, a.day_end)
    inbound = rows.orig_zone.str.startswith('XO|')
    print(f'inbound windows clipped at {a.day_end:.0f} s: {n_late} rows, {v_late} cars')
    print(f'exchange car vehicles/day {rows["count"].sum():,} (inbound {rows.loc[inbound, "count"].sum():,}, '
          f'outbound {rows.loc[~inbound, "count"].sum():,}); inbound offset min p50 {np.median(off) / 60:.1f} '
          f'max {off.max() / 60:.1f}')
    out_rows = Path(a.out_rows); out_rows.parent.mkdir(parents=True, exist_ok=True)
    rows[B.OD_ROW_COLUMNS].to_csv(out_rows, index=False)
    rows[['orig_zone', 'dest_zone', 'period', 'distance_band']].to_parquet(
        out_rows.with_name(out_rows.stem + '_cells.parquet'), index=False)
    if not a.base_od:
        return
    base = pd.read_csv(a.base_od)
    if list(base.columns) != B.OD_ROW_COLUMNS:
        sys.exit('base OD columns differ from OD_ROW_COLUMNS')
    comb = pd.concat([base, rows[B.OD_ROW_COLUMNS]], ignore_index=True)
    comb.to_csv(a.out_combined, index=False)
    base_car = int(base.loc[base['mode'] == 'car', 'count'].sum()); ex_car = int(rows['count'].sum())
    here = str(Path(__file__).resolve().parent)
    man = dict(output=str(a.out_combined), created=dt.datetime.now().isoformat(timespec='seconds'),
               base_od=str(a.base_od), base_md5=_md5(a.base_od), gateways=str(a.gateways), gateways_md5=_md5(a.gateways),
               exchange_rows=str(out_rows), node_weights=a.node_weights, seed_exchange=a.seed, months=a.months,
               mode_choice='green_mobility' if a.green_mobility else 'build_od distance bands',
               builder=f'{here}/exchange_trips.py',
               nomad_walk_commit=subprocess.run(['git', '-C', here, 'rev-parse', '--short', 'HEAD'],
                                                capture_output=True, text=True).stdout.strip(),
               base_car_agents=base_car, exchange_car_agents=ex_car, total_car_agents=base_car + ex_car,
               clipped_rows=n_late, clipped_cars=v_late, day_end_s=a.day_end, md5=_md5(a.out_combined))
    json.dump(man, open(str(a.out_combined) + '.manifest.json', 'w'), indent=1)
    print(f'base car {base_car:,} + exchange {ex_car:,} = {base_car + ex_car:,} -> {a.out_combined}')


if __name__ == '__main__':
    main()
