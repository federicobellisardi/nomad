#!/bin/bash
# Download small OSM extracts for integration testing.
# Liechtenstein (~2 MB) is the smallest readily available country extract.
# For a city-scale test, use a specific bounding box extract from BBBike.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="$SCRIPT_DIR/../../data/test_osm"
mkdir -p "$DATA_DIR"

# ── Liechtenstein (smallest country, ~2 MB PBF) ────────────────────────────────
LIECHT_URL="https://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf"
LIECHT_FILE="$DATA_DIR/liechtenstein.osm.pbf"
if [ ! -f "$LIECHT_FILE" ]; then
    echo "Downloading Liechtenstein OSM extract (~2 MB)..."
    curl -L -o "$LIECHT_FILE" "$LIECHT_URL"
    echo "Saved to $LIECHT_FILE"
else
    echo "Liechtenstein already downloaded: $LIECHT_FILE"
fi

# ── Optional: Venice (relevant to minimocas heritage, ~5 MB) ─────────────────
# Uncomment to download:
# VENICE_URL="https://download.geofabrik.de/europe/italy/nord-est-latest.osm.pbf"
# Nord-est is large (1.4 GB). Use osmium to extract Venice:
# osmium extract -b 12.2,45.3,12.5,45.6 nord-est-latest.osm.pbf -o venice.osm.pbf

echo ""
echo "Test data ready. Run the nomad CLI:"
echo "  ./build/nomad_cli --osm $LIECHT_FILE"
echo ""
echo "Or with a JSON config:"
echo "  ./build/nomad_cli --config data/schemas/scenario_liechtenstein.json"
