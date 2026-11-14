"""Tests for the FUA zone selection in load_mitma_zones() (preprocessing/build_od.py).

A zone belongs to the FUA when at least half of its area lies inside the FUA
polygon. The old rule (zone centroid inside the FUA) dropped multipolygon zones
whose parts are all inside the FUA but whose centroid falls in the gap between
them: MITMA district 0704001 (Palma centre + Cabrera archipelago) has its
centroid in the sea, so the whole city centre was missing from the Palma OD.
"""
from __future__ import annotations

import sys
from pathlib import Path

import geopandas as gpd
from shapely.geometry import MultiPolygon, box

NOMAD_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NOMAD_ROOT / "python/preprocessing"))

from build_od import load_mitma_zones  # noqa: E402

# Two FUA "islands" 0.1 deg apart; the gap between them is outside the FUA.
FUA = MultiPolygon([box(0.0, 0.0, 0.1, 0.1), box(0.2, 0.0, 0.3, 0.1)])


def _write_zones(tmp_path: Path, zones: dict) -> Path:
    gdf = gpd.GeoDataFrame({"ID": list(zones)}, geometry=list(zones.values()), crs="EPSG:4326")
    p = tmp_path / "zones.gpkg"
    gdf.to_file(p, driver="GPKG")
    return p


def _selected(tmp_path, zones):
    return set(load_mitma_zones(_write_zones(tmp_path, zones), FUA)["id"])


def test_multipolygon_with_centroid_in_gap_is_selected(tmp_path):
    split = MultiPolygon([box(0.05, 0.02, 0.09, 0.06), box(0.21, 0.02, 0.25, 0.06)])
    assert not split.centroid.within(FUA)          # the old rule would drop it
    assert _selected(tmp_path, {"split": split}) == {"split"}


def test_plain_zone_inside_selected_and_outside_dropped(tmp_path):
    zones = {"inside": box(0.01, 0.01, 0.05, 0.05), "outside": box(0.5, 0.5, 0.6, 0.6)}
    assert _selected(tmp_path, zones) == {"inside"}


def test_zone_mostly_outside_is_dropped(tmp_path):
    # 30% of the area inside the FUA, centroid outside: dropped (unchanged behaviour)
    mostly_out = box(0.07, 0.02, 0.17, 0.06)
    assert _selected(tmp_path, {"m": mostly_out}) == set()


def test_centroid_inside_kept_even_if_minor_area_share(tmp_path):
    # Tall zone: centroid (0.05, 0.05) inside the FUA, 38% of its area inside.
    # Kept, so no zone selected by the original centroid rule is ever removed.
    z = box(0.02, -0.08, 0.08, 0.18)
    assert z.centroid.within(FUA)
    assert _selected(tmp_path, {"z": z}) == {"z"}


def test_invalid_zone_geometry_does_not_crash(tmp_path):
    # self-intersecting "bowtie" ring, as found in the MITMA shapefile (Madrid)
    from shapely.geometry import Polygon
    bowtie = Polygon([(0.01, 0.01), (0.05, 0.05), (0.05, 0.01), (0.01, 0.05)])
    assert not bowtie.is_valid
    load_mitma_zones(_write_zones(tmp_path, {"b": bowtie}), FUA)


def test_zone_mostly_inside_is_selected(tmp_path):
    # 70% inside the FUA (centroid inside as well): selected (unchanged behaviour)
    mostly_in = box(0.03, 0.02, 0.13, 0.06)
    assert _selected(tmp_path, {"m": mostly_in}) == {"m"}
