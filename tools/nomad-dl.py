#!/usr/bin/env python3
"""
nomad-dl — Download an OSM road network for a city and save as .osm.pbf.

Usage:
    python tools/nomad-dl.py <city> [output.osm.pbf]
    python tools/nomad-dl.py --bbox lon_min,lat_min,lon_max,lat_max [output.osm.pbf]

Examples:
    python tools/nomad-dl.py Bologna
    python tools/nomad-dl.py "Rome, Italy"
    python tools/nomad-dl.py --bbox 11.20,44.45,11.45,44.55 data/test_osm/bologna.osm.pbf

Requirements:
    sudo apt install osmium-tool

How it works:
    1. Nominatim resolves the city name into a bounding box.
    2. Overpass downloads the selected OSM data as XML.
    3. osmium converts the temporary .osm file into .osm.pbf.
"""

import sys
import time
import shutil
import argparse
import pathlib
import subprocess
import urllib.request
import urllib.parse
import json


NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"
OVERPASS_URL = "https://overpass-api.de/api/interpreter"
USER_AGENT = "nomad-simulator/0.1 (urban mobility research)"


def geocode(city_name: str):
    """Geocode city name -> (min_lat, min_lon, max_lat, max_lon, display)."""
    params = urllib.parse.urlencode({
        "q": city_name,
        "format": "json",
        "limit": 1,
    })

    url = f"{NOMINATIM_URL}?{params}"
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})

    with urllib.request.urlopen(req, timeout=15) as resp:
        data = json.loads(resp.read())

    if not data:
        raise ValueError(f"City not found: '{city_name}'")

    bb = data[0]["boundingbox"]  # [min_lat, max_lat, min_lon, max_lon]
    display = data[0]["display_name"]

    min_lat, max_lat = float(bb[0]), float(bb[1])
    min_lon, max_lon = float(bb[2]), float(bb[3])

    lat_span = max_lat - min_lat
    lon_span = max_lon - min_lon

    if lat_span > 0.8 or lon_span > 0.8:
        lat_c = (min_lat + max_lat) / 2.0
        lon_c = (min_lon + max_lon) / 2.0

        print(
            f"  Bounding box too large ({lat_span:.2f}° × {lon_span:.2f}°), "
            "clipping to 0.3° × 0.3° around centroid."
        )

        min_lat, max_lat = lat_c - 0.15, lat_c + 0.15
        min_lon, max_lon = lon_c - 0.15, lon_c + 0.15

    return min_lat, min_lon, max_lat, max_lon, display


def build_overpass_query(min_lat, min_lon, max_lat, max_lon) -> str:
    """
    Build Overpass QL query.

    Important:
    Overpass bbox order is:
        south, west, north, east

    Here:
        min_lat, min_lon, max_lat, max_lon
    """

    bbox = f"{min_lat:.6f},{min_lon:.6f},{max_lat:.6f},{max_lon:.6f}"

    return f"""
[out:xml][timeout:300][bbox:{bbox}];
(
  way["highway"];
  relation["type"="restriction"];
);
(._;>;);
out body;
"""


def download_osm_xml(query: str, tmp_osm_path: pathlib.Path) -> None:
    """POST the Overpass query and save XML .osm."""
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")

    req = urllib.request.Request(
        OVERPASS_URL,
        data=data,
        headers={
            "User-Agent": USER_AGENT,
            "Content-Type": "application/x-www-form-urlencoded",
        },
    )

    tmp_osm_path.parent.mkdir(parents=True, exist_ok=True)

    chunk_size = 65536
    bytes_so_far = 0
    t0 = time.time()

    with urllib.request.urlopen(req, timeout=600) as resp, open(tmp_osm_path, "wb") as f:
        while True:
            chunk = resp.read(chunk_size)
            if not chunk:
                break

            f.write(chunk)
            bytes_so_far += len(chunk)

            elapsed = max(time.time() - t0, 1e-9)
            speed = bytes_so_far / elapsed / 1024.0

            print(
                f"\r  XML: {bytes_so_far / 1_048_576:.1f} MB "
                f"({speed:.0f} KB/s)",
                end="",
                flush=True,
            )

    print()


def convert_osm_to_pbf(tmp_osm_path: pathlib.Path, output_path: pathlib.Path) -> None:
    """Convert temporary .osm XML file to .osm.pbf using osmium."""
    if shutil.which("osmium") is None:
        raise RuntimeError(
            "osmium not found. Install it with:\n"
            "  sudo apt install osmium-tool"
        )

    print("Converting XML .osm -> .osm.pbf with osmium...")

    subprocess.run(
        [
            "osmium",
            "cat",
            str(tmp_osm_path),
            "-o",
            str(output_path),
            "--overwrite",
        ],
        check=True,
    )


def download_pbf(query: str, output_path: pathlib.Path, keep_xml: bool = False) -> None:
    """Download OSM XML from Overpass, then convert it to PBF."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    tmp_osm_path = output_path.with_suffix(".osm")

    print(f"  Temporary XML: {tmp_osm_path}")

    try:
        download_osm_xml(query, tmp_osm_path)
        convert_osm_to_pbf(tmp_osm_path, output_path)
    finally:
        if tmp_osm_path.exists() and not keep_xml:
            tmp_osm_path.unlink()


def parse_bbox(raw_bbox: str):
    """Parse lon_min,lat_min,lon_max,lat_max from CLI."""
    try:
        coords = [float(x.strip()) for x in raw_bbox.split(",")]
    except ValueError:
        raise ValueError("Invalid bbox. Use: lon_min,lat_min,lon_max,lat_max")

    if len(coords) != 4:
        raise ValueError("Invalid bbox. Use: lon_min,lat_min,lon_max,lat_max")

    lon_min, lat_min, lon_max, lat_max = coords

    if lon_min >= lon_max or lat_min >= lat_max:
        raise ValueError("Invalid bbox ordering. Use: lon_min,lat_min,lon_max,lat_max")

    return lat_min, lon_min, lat_max, lon_max


def main():
    parser = argparse.ArgumentParser(
        description="Download an OSM road network for a city as .osm.pbf",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "city",
        nargs="?",
        help="City name, e.g. 'Bologna'. Not needed if --bbox is used.",
    )

    parser.add_argument(
        "output",
        nargs="?",
        help="Output .osm.pbf path. Default: data/test_osm/<city>.osm.pbf",
    )

    parser.add_argument(
        "--bbox",
        metavar="LON_MIN,LAT_MIN,LON_MAX,LAT_MAX",
        help="Manual bounding box instead of city name lookup.",
    )

    parser.add_argument(
        "--keep-xml",
        action="store_true",
        help="Keep temporary .osm XML file after conversion.",
    )

    args = parser.parse_args()

    if args.bbox:
        min_lat, min_lon, max_lat, max_lon = parse_bbox(args.bbox)
        display = f"custom bbox ({args.bbox})"
        city_slug = "custom"
        out_arg = args.output

    else:
        if not args.city:
            parser.error("You must provide either a city name or --bbox.")

        city_name = args.city
        print(f"Geocoding '{city_name}' via Nominatim...")

        min_lat, min_lon, max_lat, max_lon, display = geocode(city_name)
        city_slug = city_name.lower().split(",")[0].strip().replace(" ", "_")
        out_arg = args.output

    print(f"  → {display}")
    print(
        f"  Bounding box: lon [{min_lon:.4f}, {max_lon:.4f}]  "
        f"lat [{min_lat:.4f}, {max_lat:.4f}]"
    )

    if out_arg:
        output_path = pathlib.Path(out_arg)
    else:
        output_path = pathlib.Path("data/test_osm") / f"{city_slug}.osm.pbf"

    print(f"  Output: {output_path}")

    query = build_overpass_query(min_lat, min_lon, max_lat, max_lon)

    print("\nDownloading road network from Overpass API...")
    print("  This downloads XML first, then converts it to PBF.\n")

    download_pbf(query, output_path, keep_xml=args.keep_xml)

    size_mb = output_path.stat().st_size / 1_048_576

    print(f"\n✓ Saved {size_mb:.1f} MB → {output_path}")
    print("\nRun with:")
    print(f"  ./build/nomad_cli --osm {output_path}")
    print("  ./build/nomad_cli --config data/schemas/scenario_bologna.json")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        sys.exit(1)