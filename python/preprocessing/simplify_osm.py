#!/usr/bin/env python3
"""Clip a regional OSM PBF to a city FUA and export the simplified graph.

Outputs:
  data/{city_slug}/nodes.parquet   — node_id, lon, lat
  data/{city_slug}/edges.parquet   — edge_id, from_node, to_node,
                                     length_m, speed_ms, road_class

Usage:
  python preprocessing/simplify_osm.py \\
      --city "Palma de Mallorca" \\
      --pbf  data/osm/islas-baleares.osm.pbf

The FUA polygon is read from data/fua/boundaries.gpkg (layer 'FUAs').
The clipped PBF is cached at data/osm/{city_slug}_fua.osm.pbf.
"""

import argparse
import json
import re
import subprocess
import sys
import unicodedata
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))


def slugify(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_",
                  unicodedata.normalize("NFKD", name.lower())
                  .encode("ascii", "ignore").decode()).strip("_")


def find_fua(city_name: str, fua_file: Path) -> object:
    fuas = gpd.read_file(fua_file, layer="FUAs").to_crs("EPSG:4326")
    lo = city_name.lower().strip()
    match = fuas[fuas["fuaname"].str.lower() == lo]
    if match.empty:
        match = fuas[fuas["fuaname"].str.lower().str.contains(lo, regex=False)]
    if match.empty:
        from difflib import get_close_matches
        suggestions = get_close_matches(lo, fuas["fuaname"].str.lower().tolist(),
                                        n=5, cutoff=0.4)
        raise SystemExit(
            f"'{city_name}' non trovata in {fua_file}\n"
            f"Suggerimenti: {suggestions}"
        )
    if len(match) > 1:
        print(f"  trovate {len(match)} FUA, uso la prima")
    return match.iloc[0]


def clip_osm(pbf_in: Path, pbf_out: Path, fua_poly, tmp_dir: Path) -> Path:
    if pbf_out.exists():
        print(f"  clip già presente: {pbf_out.name}  ({pbf_out.stat().st_size/1e6:.1f} MB)")
        return pbf_out
    geojson = tmp_dir / "_fua_clip.geojson"
    geojson.write_text(json.dumps({
        "type": "FeatureCollection",
        "features": [{"type": "Feature",
                       "geometry": fua_poly.__geo_interface__,
                       "properties": {}}],
    }))
    result = subprocess.run(
        ["osmium", "extract", "--polygon", str(geojson),
         str(pbf_in), "-o", str(pbf_out), "--overwrite"],
        capture_output=True, text=True,
    )
    geojson.unlink(missing_ok=True)
    if result.returncode != 0:
        print(f"  osmium fallito: {result.stderr[:300]}")
        print("  uso PBF originale come fallback")
        return pbf_in
    print(f"  clipped: {pbf_out.name}  ({pbf_out.stat().st_size/1e6:.1f} MB)")
    return pbf_out


def load_graph(pbf: Path):
    from nomad._nomad_core import OsmLoader
    return OsmLoader().load_and_clean(str(pbf), True)


def export_graph(graph, out_dir: Path) -> None:
    N = graph.num_nodes
    E = graph.num_edges

    # ── Nodes ─────────────────────────────────────────────────────────────────
    coords = graph.node_coords()          # (N, 2) float32  [lon, lat]
    nodes_df = pd.DataFrame({
        "node_id": np.arange(N, dtype=np.int32),
        "lon":     coords[:, 0].astype(np.float64),
        "lat":     coords[:, 1].astype(np.float64),
    })
    nodes_path = out_dir / "nodes.parquet"
    nodes_df.to_parquet(nodes_path, index=False)
    print(f"  nodes: {N:,} righe  → {nodes_path}")

    # ── Edges ─────────────────────────────────────────────────────────────────
    # Reconstruct from_node from CSR row_ptr: edge e belongs to node u where
    # row_ptr[u] <= e < row_ptr[u+1].
    rp          = graph.row_ptr()         # (N+1,) uint32
    from_nodes  = np.repeat(np.arange(N, dtype=np.uint32), np.diff(rp))
    to_nodes    = graph.edge_targets()    # (E,)   uint32
    lengths     = graph.edge_lengths()    # (E,)   float32  [m]
    speeds      = graph.edge_speeds()     # (E,)   float32  [m/s]
    road_class  = graph.edge_road_class() # (E,)   uint8

    edges_df = pd.DataFrame({
        "edge_id":    np.arange(E, dtype=np.int32),
        "from_node":  from_nodes.astype(np.int32),
        "to_node":    to_nodes.astype(np.int32),
        "length_m":   lengths.astype(np.float32),
        "speed_ms":   speeds.astype(np.float32),
        "road_class": road_class.astype(np.uint8),
    })
    edges_path = out_dir / "edges.parquet"
    edges_df.to_parquet(edges_path, index=False)
    print(f"  edges: {E:,} righe  → {edges_path}")

    # ── Binary graph cache ────────────────────────────────────────────────────
    bin_path = out_dir / "graph.bin"
    graph.save(str(bin_path))
    print(f"  cache: {bin_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Clip OSM + export simplified graph")
    parser.add_argument("--city", required=True,
                        help="Nome città (es. 'Palma de Mallorca')")
    parser.add_argument("--pbf", default=None,
                        help="PBF regionale di input (default: primo *.osm.pbf in data/osm/)")
    parser.add_argument("--data-dir", default=str(ROOT / "data"),
                        help="Directory dati (default: data/)")
    parser.add_argument("--force-clip", action="store_true",
                        help="Rigenera il PBF clippato anche se esiste")
    args = parser.parse_args()

    data_dir  = Path(args.data_dir)
    fua_file  = data_dir / "fua" / "boundaries.gpkg"
    osm_dir   = data_dir / "osm"
    city_slug = slugify(args.city)
    out_dir   = data_dir / city_slug
    out_dir.mkdir(parents=True, exist_ok=True)

    # PBF di input
    if args.pbf:
        pbf_in = Path(args.pbf)
    else:
        cands = sorted(osm_dir.glob("*.osm.pbf"))
        cands = [p for p in cands if "_fua" not in p.name]
        if not cands:
            raise SystemExit(f"Nessun PBF trovato in {osm_dir}")
        pbf_in = cands[0]
    print(f"PBF input   : {pbf_in}")

    # FUA
    print(f"Ricerca FUA : '{args.city}'")
    fua_row  = find_fua(args.city, fua_file)
    fua_poly = fua_row.geometry
    print(f"  trovata   : {fua_row['fuaname']}  ({fua_row['fuacode']})")

    # Clip OSM
    pbf_fua = osm_dir / f"{city_slug}_fua.osm.pbf"
    if args.force_clip and pbf_fua.exists():
        pbf_fua.unlink()
    pbf_use = clip_osm(pbf_in, pbf_fua, fua_poly, out_dir)

    # Carica grafo
    print("Caricamento grafo …")
    graph = load_graph(pbf_use)
    print(f"  nodi: {graph.num_nodes:,}   archi: {graph.num_edges:,}")

    # Esporta
    print("Esportazione …")
    export_graph(graph, out_dir)
    print("Fatto.")


if __name__ == "__main__":
    main()
