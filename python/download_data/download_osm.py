#!/usr/bin/env python3
"""Download Baleares OSM PBF from Geofabrik, optionally clip to Palma FUA.

Usage:
    python download_osm.py                     # download only
    python download_osm.py --clip              # download + clip to FUA bbox
    python download_osm.py --output-dir data/osm --clip
"""

import argparse
import shutil
import subprocess
import urllib.request
from pathlib import Path

GEOFABRIK_URL = "https://download.geofabrik.de/europe/spain/islas-baleares-260101.osm.pbf"
PALMA_FUA_BBOX = "2.35,39.38,3.00,39.85"   # lon_min,lat_min,lon_max,lat_max


def download_pbf(url: str, dest: Path, force: bool = False) -> None:
    if dest.exists() and not force:
        print(f"✓  Già presente: {dest}  ({dest.stat().st_size / 1e6:.0f} MB)")
        return
    print(f"Downloading {url} …")
    with urllib.request.urlopen(url) as resp, open(dest, "wb") as f:
        shutil.copyfileobj(resp, f)
    print(f"✓  {dest}  ({dest.stat().st_size / 1e6:.0f} MB)")


def clip_pbf(src: Path, dest: Path, bbox: str, force: bool = False) -> Path:
    if dest.exists() and not force:
        print(f"✓  Già presente: {dest}  ({dest.stat().st_size / 1e6:.0f} MB)")
        return dest
    try:
        subprocess.run(
            ["osmium", "extract", "--bbox", bbox, "-o", str(dest), str(src)],
            check=True,
        )
        print(f"✓  Clipped → {dest}  ({dest.stat().st_size / 1e6:.0f} MB)")
    except FileNotFoundError:
        print("⚠  osmium-tool non trovato.")
        print("   Installa con: conda install -c conda-forge osmium-tool")
        print(f"   Verrà usato il PBF completo: {src}")
        return src
    return dest


def main() -> None:
    parser = argparse.ArgumentParser(description="Download Baleares OSM PBF da Geofabrik")
    parser.add_argument(
        "--output-dir", default="data/osm",
        help="Directory di destinazione (default: data/osm)",
    )
    parser.add_argument(
        "--clip", action="store_true",
        help="Ritaglia il PBF sulla bounding box della FUA di Palma con osmium-tool",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Re-scarica anche se il file esiste già",
    )
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    raw = out_dir / "islas-baleares.osm.pbf"
    download_pbf(GEOFABRIK_URL, raw, force=args.force)

    if args.clip:
        clipped = out_dir / "palma_fua.osm.pbf"
        clip_pbf(raw, clipped, PALMA_FUA_BBOX, force=args.force)


if __name__ == "__main__":
    main()
