#!/usr/bin/env python3
"""Build nomad OD matrices from MITMA viajes data.

Generates two files (average weekday and average weekend/holiday):
  data/{city_slug}/od_{city_slug}_weekday.csv
  data/{city_slug}/od_{city_slug}_weekend.csv

Both share the same columns:
  origin_node, dest_node, count, mode, depart_mean_s, depart_std_s

Usage:
  python preprocessing/build_od.py --city "Palma de Mallorca"
"""

import argparse
import datetime as dt
import os
import re
import sys
import unicodedata
from pathlib import Path

os.environ["SHAPE_RESTORE_SHX"] = "YES"

import geopandas as gpd
import numpy as np
import pandas as pd

try:
    import holidays as _holidays_lib
    _HOLIDAYS_AVAILABLE = True
except ImportError:
    _HOLIDAYS_AVAILABLE = False

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))


# ── helpers ───────────────────────────────────────────────────────────────────

def slugify(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_",
                  unicodedata.normalize("NFKD", name.lower())
                  .encode("ascii", "ignore").decode()).strip("_")


def build_holidays_set(years: set, country: str) -> set:
    if not _HOLIDAYS_AVAILABLE:
        return set()
    h = set()
    for y in years:
        try:
            h |= set(_holidays_lib.country_holidays(country, years=y).keys())
        except Exception:
            pass
    return h


def is_weekend(d: dt.date, holidays_set: set) -> bool:
    """Returns True for Saturdays, Sundays, and public holidays."""
    return d.weekday() >= 5 or d in holidays_set


def find_fua(city_name: str, fua_file: Path):
    fuas = gpd.read_file(fua_file, layer="FUAs").to_crs("EPSG:4326")
    lo = city_name.lower().strip()
    match = fuas[fuas["fuaname"].str.lower() == lo]
    if match.empty:
        match = fuas[fuas["fuaname"].str.lower().str.contains(lo, regex=False)]
    if match.empty:
        from difflib import get_close_matches
        suggestions = get_close_matches(lo, fuas["fuaname"].str.lower().tolist(),
                                        n=5, cutoff=0.4)
        raise SystemExit(f"'{city_name}' non trovata.\nSuggerimenti: {suggestions}")
    return match.iloc[0]


def load_mitma_zones(zones_path: Path, fua_poly) -> gpd.GeoDataFrame:
    zones_crs = gpd.read_file(zones_path, rows=1).crs
    fua_proj  = gpd.GeoDataFrame(geometry=[fua_poly], crs="EPSG:4326").to_crs(zones_crs)
    bbox      = tuple(fua_proj.total_bounds)
    zones     = gpd.read_file(zones_path, bbox=bbox).to_crs("EPSG:4326")
    if zones.empty:
        raise RuntimeError("Nessuna zona MITMA nella bbox FUA.")
    id_col = next(
        (c for c in zones.columns if c.lower() in ("id", "codzona", "cod_zona", "zona", "id_zona")),
        [c for c in zones.columns if c.lower() != "geometry"][0],
    )
    zones = zones.rename(columns={id_col: "id"})
    centroids_proj = zones.to_crs(zones_crs).centroid.to_crs("EPSG:4326")
    return zones[centroids_proj.within(fua_poly)].copy()


def detect_columns(sample_path: Path) -> dict:
    gz = str(sample_path).endswith(".gz")
    df = pd.read_csv(sample_path, sep="|", nrows=3,
                     compression="gzip" if gz else None)
    lc = {c.lower(): c for c in df.columns}
    def pick(*names):
        for n in names:
            if n in lc:
                return lc[n]
        return None
    return {
        "orig":     pick("origen", "origin")      or "origen",
        "dest":     pick("destino", "destination") or "destino",
        "period":   pick("periodo", "hora")        or "periodo",
        "trips":    pick("viajes", "trips")        or "viajes",
        "mode":     pick("modo", "mode", "medio"),
        "distance": pick("distancia", "distance"),
    }


MITMA_MODE_MAP = {
    "1": "car", "2": "transit", "3": "walk", "4": "bike",
    "PRIVADO_REGULAR": "car", "PRIVADO_OCASIONAL": "car",
    "PUBLICO": "transit", "A_PIE": "walk", "BICI": "bike", "OTRO": "car",
}

# Mode fractions per distance band (proxy Movilia/MITMA when no mode column).
# Source: Encuesta de Movilidad de las Personas Residentes en España (Movilia 2006,
# updated with MITMA 2022 report). Rows sum to 1.0.
DISTANCE_MODE_FRACTIONS: dict[str, dict[str, float]] = {
    #           walk   bike   car   transit
    "0.5-2":  {"walk": 0.70, "bike": 0.05, "car": 0.05, "transit": 0.20},
    "2-10":   {"walk": 0.10, "bike": 0.05, "car": 0.50, "transit": 0.35},
    "10-50":  {"walk": 0.01, "bike": 0.01, "car": 0.80, "transit": 0.18},
    ">50":    {"walk": 0.00, "bike": 0.00, "car": 0.90, "transit": 0.10},
}


def stream_viajes(viajes_files: list, cols: dict,
                  fua_zone_ids: set, weekend: bool,
                  holidays_set: set) -> "pd.DataFrame | None":
    """Stream viajes files for one day-type and return the mean OD.

    Uses a dict accumulator (key → [sum, count]) instead of DataFrame merges
    to keep memory proportional to the number of unique OD pairs, not to
    the product of files × pairs.
    """
    COL_ORIG   = cols["orig"]
    COL_DEST   = cols["dest"]
    COL_PERIOD = cols["period"]
    COL_TRIPS  = cols["trips"]
    COL_MODE   = cols["mode"]
    COL_DIST   = cols["distance"]
    group_cols = (
        [COL_ORIG, COL_DEST, COL_PERIOD]
        + ([COL_MODE] if COL_MODE else [])
        + ([COL_DIST] if COL_DIST else [])
    )

    # key → [trips_sum, days_seen]
    acc: dict = {}
    n_used = 0

    for fpath in viajes_files:
        m = re.search(r"(\d{8})", fpath.name)
        if not m:
            continue
        file_date = dt.datetime.strptime(m.group(1), "%Y%m%d").date()
        if is_weekend(file_date, holidays_set) != weekend:
            continue

        gz = str(fpath).endswith(".gz")
        df = pd.read_csv(fpath, sep="|",
                         compression="gzip" if gz else None,
                         dtype={COL_ORIG: str, COL_DEST: str},
                         usecols=group_cols + [COL_TRIPS])
        df = df[df[COL_ORIG].isin(fua_zone_ids) & df[COL_DEST].isin(fua_zone_ids)]
        df[COL_PERIOD] = pd.to_numeric(df[COL_PERIOD], errors="coerce")
        df[COL_TRIPS]  = pd.to_numeric(df[COL_TRIPS],  errors="coerce").fillna(0)

        day_sum = df.groupby(group_cols)[COL_TRIPS].sum()
        for key, val in day_sum.items():
            if key in acc:
                acc[key][0] += val
                acc[key][1] += 1
            else:
                acc[key] = [val, 1]

        n_used += 1
        print(f"  [{n_used}] {fpath.name}", end="\r")

    if not acc:
        return None

    print(f"\n  file usati: {n_used}")

    # Rebuild DataFrame from accumulator
    if len(group_cols) == 1:
        keys = [(k,) for k in acc]
    else:
        keys = list(acc.keys())

    rows = {col: [] for col in group_cols}
    rows[COL_TRIPS] = []
    for key, (s, c) in zip(keys, acc.values()):
        for col, val in zip(group_cols, key):
            rows[col].append(val)
        rows[COL_TRIPS].append(s / c)

    return pd.DataFrame(rows)


CAR_MAX_ROAD_CLASS = 13   # RoadClass.Unclassified — cars cannot use Track or above


def _largest_scc(from_nodes: "np.ndarray", to_nodes: "np.ndarray") -> set:
    """Kosaraju's algorithm (iterative) — returns node set of the largest SCC."""
    from collections import defaultdict, deque

    fwd: dict = defaultdict(list)
    rev: dict = defaultdict(list)
    all_nodes: set = set()
    for u, v in zip(from_nodes.tolist(), to_nodes.tolist()):
        fwd[u].append(v)
        rev[v].append(u)
        all_nodes.add(u); all_nodes.add(v)

    # Pass 1: finish-time order on forward graph
    visited: set = set()
    finish: list = []
    for start in all_nodes:
        if start in visited:
            continue
        stack = [(start, iter(fwd[start]))]
        visited.add(start)
        while stack:
            node, it = stack[-1]
            try:
                nb = next(it)
                if nb not in visited:
                    visited.add(nb)
                    stack.append((nb, iter(fwd[nb])))
            except StopIteration:
                finish.append(node)
                stack.pop()

    # Pass 2: SCCs on reverse graph in reverse finish order
    visited2: set = set()
    best: set = set()
    for start in reversed(finish):
        if start in visited2:
            continue
        comp: list = []
        stack2 = [start]
        visited2.add(start)
        while stack2:
            node = stack2.pop()
            comp.append(node)
            for nb in rev[node]:
                if nb not in visited2:
                    visited2.add(nb)
                    stack2.append(nb)
        if len(comp) > len(best):
            best = set(comp)
    return best

# Road-class accessibility per mode (mirrors C++ road_class_accessible in types.hpp)
def mode_node_pool(mode: str,
                   all_pool: list,
                   car_nodes: "set | None") -> list:
    """Return the node pool appropriate for this transport mode."""
    if mode == "car" and car_nodes is not None:
        return [n for n in all_pool if n in car_nodes]
    return all_pool


def build_od_rows(od_fua: pd.DataFrame,
                  zone_to_nodes: dict,
                  cols: dict,
                  rng: np.random.Generator,
                  noise_sigma: float,
                  max_pairs: int = 20,
                  car_nodes: "set | None" = None,
                  scale: float = 1.0,
                  occupancy_factor: float = 1.0) -> pd.DataFrame:
    """Convert zone-level MITMA OD to node-level nomad OD rows.

    occupancy_factor: average persons per car trip.  Divides car person-trips
    to obtain vehicle-trips (e.g. 1.20 for Spain average).  Set to 1.0 to
    keep raw person-trips as vehicle-trips.
    """
    COL_ORIG   = cols["orig"]
    COL_DEST   = cols["dest"]
    COL_PERIOD = cols["period"]
    COL_TRIPS  = cols["trips"]
    COL_MODE   = cols["mode"]
    COL_DIST   = cols["distance"]

    rows = []
    skipped = 0.0
    n_intra_person = 0.0   # intrazonal person-trips entering the loop
    n_inter_person = 0.0   # interzonal person-trips
    n_intra_veh    = 0.0   # intrazonal vehicle-trips after occupancy correction
    n_inter_veh    = 0.0

    for _, r in od_fua.iterrows():
        mean_count = r[COL_TRIPS] * scale
        if mean_count < 0.5:
            continue
        all_orig = zone_to_nodes.get(str(r[COL_ORIG]), [])
        all_dest = zone_to_nodes.get(str(r[COL_DEST]), [])
        if not all_orig or not all_dest:
            skipped += mean_count
            continue

        is_intrazonal = str(r[COL_ORIG]) == str(r[COL_DEST])

        if COL_MODE:
            mode_fracs = {MITMA_MODE_MAP.get(str(r[COL_MODE]), "car"): 1.0}
        elif COL_DIST:
            mode_fracs = DISTANCE_MODE_FRACTIONS.get(str(r[COL_DIST]),
                                                      {"car": 0.5, "walk": 0.3,
                                                       "bike": 0.1, "transit": 0.1})
        else:
            mode_fracs = {"car": 1.0}

        periodo = int(r[COL_PERIOD])
        for mode, frac in mode_fracs.items():
            mode_trips_person = mean_count * frac
            if mode_trips_person < 0.5:
                continue

            # Convert person-trips → vehicle-trips for car mode
            if mode == "car" and occupancy_factor > 1.0:
                mode_trips = mode_trips_person / occupancy_factor
            else:
                mode_trips = mode_trips_person

            if is_intrazonal:
                n_intra_person += mode_trips_person
                n_intra_veh    += mode_trips
            else:
                n_inter_person += mode_trips_person
                n_inter_veh    += mode_trips

            count = int(round(mode_trips * rng.lognormal(0.0, noise_sigma)))
            if count < 1:
                continue

            orig_pool = mode_node_pool(mode, all_orig, car_nodes)
            dest_pool = mode_node_pool(mode, all_dest, car_nodes)
            if not orig_pool or not dest_pool:
                skipped += mode_trips
                continue

            n_pairs  = min(count, max_pairs)
            base     = count // n_pairs
            rem      = count % n_pairs

            origs = rng.choice(orig_pool, n_pairs, replace=True)
            dests = rng.choice(dest_pool, n_pairs, replace=True)
            for i, (o, d) in enumerate(zip(origs, dests)):
                pair_count = base + (1 if i < rem else 0)
                if int(o) != int(d) and pair_count > 0:
                    rows.append((int(o), int(d), pair_count, mode,
                                 float(periodo * 3600), float((periodo + 1) * 3600)))

    total_person = n_intra_person + n_inter_person
    total_veh    = n_intra_veh    + n_inter_veh
    if total_person > 0:
        print(f"  intrazonali: {n_intra_person:.0f} persone → {n_intra_veh:.0f} veicoli "
              f"({100*n_intra_person/total_person:.1f}%)")
        print(f"  interzonali: {n_inter_person:.0f} persone → {n_inter_veh:.0f} veicoli "
              f"({100*n_inter_person/total_person:.1f}%)")
        print(f"  totale:      {total_person:.0f} persone → {total_veh:.0f} veicoli "
              f"(riduzione occupancy {100*(1-total_veh/total_person):.1f}%)")
    if skipped > 0:
        print(f"  viaggi senza nodi nel distretto: {skipped:.0f}")
    return pd.DataFrame(rows, columns=[
        "origin_node", "dest_node", "count", "mode",
        "depart_mean_s", "depart_std_s",
    ])


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Genera matrici OD nomad da dati MITMA")
    parser.add_argument("--city", required=True,
                        help="Nome città (es. 'Palma de Mallorca')")
    parser.add_argument("--data-dir", default=str(ROOT / "data"))
    parser.add_argument("--noise-sigma", type=float, default=0.08,
                        help="LogNormal σ per la variabilità giornaliera (default: 0.08)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--country", default="ES",
                        help="Codice paese ISO per i festivi (default: ES)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Fattore di scala sulla domanda [0-1] (default: 1.0)")
    parser.add_argument("--occupancy-factor", type=float, default=1.20,
                        help="Persone per viaggio in auto (default: 1.20, media Spagna). "
                             "Converte person-trips MITMA → vehicle-trips. Usa 1.0 per disabilitare.")
    args = parser.parse_args()

    data_dir  = Path(args.data_dir)
    city_slug = slugify(args.city)
    city_dir  = data_dir / city_slug
    fua_file  = data_dir / "fua" / "boundaries.gpkg"
    od_raw    = data_dir / "od_raw"

    # ── FUA ──────────────────────────────────────────────────────────────────
    print(f"FUA: '{args.city}'")
    fua_row  = find_fua(args.city, fua_file)
    fua_poly = fua_row.geometry
    print(f"  {fua_row['fuaname']}  ({fua_row['fuacode']})")

    # ── Nodes ────────────────────────────────────────────────────────────────
    nodes_path = city_dir / "nodes.parquet"
    if not nodes_path.exists():
        raise SystemExit(
            f"nodes.parquet non trovato: {nodes_path}\n"
            "Esegui prima:  python preprocessing/simplify_osm.py --city '...'"
        )
    nodes_df = pd.read_parquet(nodes_path)
    print(f"Nodi: {len(nodes_df):,}")

    # Build car_nodes = largest strongly connected component of the car graph.
    # Using the full SCC guarantees that every OD pair is mutually reachable,
    # eliminating routing failures from disconnected fragments (boundary clips,
    # mis-tagged private roads, one-way dead-ends).
    car_nodes: set | None = None
    edges_path = city_dir / "edges.parquet"
    if edges_path.exists():
        edges_df = pd.read_parquet(edges_path)
        if "road_class" in edges_df.columns and "from_node" in edges_df.columns:
            car_edges = edges_df[edges_df["road_class"] <= CAR_MAX_ROAD_CLASS]
            car_nodes = _largest_scc(car_edges["from_node"].to_numpy(),
                                     car_edges["to_node"].to_numpy())
            all_car = set(car_edges["from_node"]) | set(car_edges["to_node"])
            print(f"Nodi car-accessibili: {len(all_car):,}  "
                  f"SCC principale: {len(car_nodes):,} ({100*len(car_nodes)/len(all_car):.1f}%)")
    if car_nodes is None:
        print("  ⚠ edges.parquet non trovato o senza road_class — uso tutti i nodi per car")

    # ── MITMA zones ──────────────────────────────────────────────────────────
    zone_cands = list(od_raw.glob("*.gpkg")) + list(od_raw.glob("*.shp"))
    if not zone_cands:
        raise SystemExit(f"Nessun file zone MITMA in {od_raw}")
    fua_zones    = load_mitma_zones(zone_cands[0], fua_poly)
    fua_zone_ids = set(fua_zones["id"].astype(str))
    print(f"Zone MITMA: {len(fua_zones)} distretti nella FUA")

    # ── Spatial join nodes → districts ───────────────────────────────────────
    print("Spatial join nodi → distretti …")
    nodes_gdf = gpd.GeoDataFrame(
        {"node_id": nodes_df["node_id"]},
        geometry=gpd.points_from_xy(nodes_df["lon"], nodes_df["lat"]),
        crs="EPSG:4326",
    )
    joined = gpd.sjoin(
        nodes_gdf,
        fua_zones[["id", "geometry"]].rename(columns={"id": "mitma_zone"}),
        how="inner", predicate="within",
    ).drop(columns="index_right")
    zone_to_nodes = joined.groupby("mitma_zone")["node_id"].apply(list).to_dict()
    print(f"  {len(joined):,} nodi nella FUA  |  {len(zone_to_nodes)} distretti con nodi")

    # ── Viajes files + festivi ────────────────────────────────────────────────
    viajes_files = (sorted(od_raw.glob("**/*iajes*.csv.gz")) +
                    sorted(od_raw.glob("**/*iajes*.csv")))
    if not viajes_files:
        raise SystemExit(f"Nessun file viajes in {od_raw}")

    cols = detect_columns(viajes_files[0])
    print(f"Colonne MITMA: {cols}")

    file_years = set()
    for fp in viajes_files:
        mm = re.search(r"(\d{8})", fp.name)
        if mm:
            file_years.add(int(mm.group(1)[:4]))

    if _HOLIDAYS_AVAILABLE:
        holidays_set = build_holidays_set(file_years, args.country)
        print(f"Festivi {args.country}: {len(holidays_set)} date "
              f"({min(file_years)}–{max(file_years)})")
    else:
        holidays_set = set()
        print("  ⚠ 'holidays' non installato — festivi non rilevati (pip install holidays)")

    # Conta file per tipo
    n_weekday = sum(1 for fp in viajes_files
                    if (mm := re.search(r"(\d{8})", fp.name)) and
                    not is_weekend(dt.datetime.strptime(mm.group(1), "%Y%m%d").date(),
                                   holidays_set))
    n_weekend = sum(1 for fp in viajes_files
                    if (mm := re.search(r"(\d{8})", fp.name)) and
                    is_weekend(dt.datetime.strptime(mm.group(1), "%Y%m%d").date(),
                               holidays_set))
    print(f"File disponibili: {n_weekday} feriali  {n_weekend} weekend/festivi")

    city_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    OD_COLUMNS = ["origin_node", "dest_node", "count", "mode",
                  "depart_mean_s", "depart_std_s"]

    for label, weekend in [("weekday", False), ("weekend", True)]:
        n_files = n_weekday if not weekend else n_weekend
        if n_files == 0:
            print(f"\n[{label}] nessun file disponibile — salto")
            continue

        print(f"\n[{label}] Caricamento viajes ({n_files} file) …")
        od_fua = stream_viajes(viajes_files, cols, fua_zone_ids, weekend, holidays_set)
        if od_fua is None:
            print(f"  nessun dato — salto")
            continue

        total_trips = od_fua[cols["trips"]].sum()
        intra_mask   = od_fua[cols["orig"]].astype(str) == od_fua[cols["dest"]].astype(str)
        intra_trips  = od_fua.loc[intra_mask,  cols["trips"]].sum()
        inter_trips  = od_fua.loc[~intra_mask, cols["trips"]].sum()
        print(f"  righe OD media: {len(od_fua):,}   viaggi totali: {total_trips:.0f}")
        print(f"    intrazonali (zona O=D): {intra_trips:.0f} ({100*intra_trips/total_trips:.1f}%)")
        print(f"    interzonali (zona O≠D): {inter_trips:.0f} ({100*inter_trips/total_trips:.1f}%)")
        print(f"  occupancy_factor: {args.occupancy_factor} (solo car → vehicle-trips)")

        print(f"  Generazione OD nomad …")
        od_nomad = build_od_rows(od_fua, zone_to_nodes, cols, rng, args.noise_sigma,
                                 car_nodes=car_nodes, scale=args.scale,
                                 occupancy_factor=args.occupancy_factor)
        print(f"  righe: {len(od_nomad):,}   agenti: {od_nomad['count'].sum():,}")

        od_path = city_dir / f"od_{city_slug}_{label}.csv"
        od_nomad.to_csv(od_path, index=False)
        assert list(pd.read_csv(od_path, nrows=1).columns) == OD_COLUMNS
        print(f"  → {od_path}  ({od_path.stat().st_size/1e6:.1f} MB)")

    print("\nFatto.")


if __name__ == "__main__":
    main()
