"""Tests for the optional mode_choice_fn hook added to build_od_rows() and
the optional viajes_km tracking added to detect_columns()/stream_viajes()
(preprocessing/build_od.py), on branch feature/logit-mode-choice.

Goal: replace the flat, national, distance-banded DISTANCE_MODE_FRACTIONS
lookup with a per-city calibrated model, WITHOUT changing behaviour for any
caller that doesn't pass mode_choice_fn (backward compatibility is the main
thing under test here, since this file is a general-purpose NOMAD script
used outside this one project too).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

NOMAD_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NOMAD_ROOT / "python/preprocessing"))

from build_od import (  # noqa: E402
    DISTANCE_BAND_FALLBACK_KM,
    DISTANCE_MODE_FRACTIONS,
    build_od_rows,
    detect_columns,
    stream_viajes,
)


def _base_od_fua(distancia="2-10", viajes_km=None):
    row = {"origen": "Z1", "destino": "Z2", "periodo": 8, "viajes": 100.0, "distancia": distancia}
    if viajes_km is not None:
        row["viajes_km"] = viajes_km
    return pd.DataFrame([row])


def _cols(with_km: bool):
    return {"orig": "origen", "dest": "destino", "period": "periodo", "trips": "viajes",
            "mode": None, "distance": "distancia", "km": "viajes_km" if with_km else None}


def _zone_to_nodes():
    return {"Z1": [1, 2, 3], "Z2": [4, 5, 6]}


class TestBackwardCompatibility:
    def test_no_mode_choice_fn_matches_distance_mode_fractions_exactly(self):
        """mode_choice_fn=None (default) must reproduce the pre-existing
        DISTANCE_MODE_FRACTIONS-only behaviour byte-for-byte -- this is the
        main safety property for a generic preprocessing script used by
        other projects too, not just this one."""
        od_fua = _base_od_fua(distancia="2-10", viajes_km=400.0)
        cols = _cols(with_km=True)
        rng = np.random.default_rng(42)
        result = build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                               occupancy_factor=1.2)  # mode_choice_fn omitted

        counts = result.groupby("mode")["count"].sum()
        frac = DISTANCE_MODE_FRACTIONS["2-10"]
        # car e' l'unico modo diviso per occupancy_factor
        expected_car = int(round(100.0 * frac["car"] / 1.2))
        expected_walk = int(round(100.0 * frac["walk"]))
        expected_bike = int(round(100.0 * frac["bike"]))
        expected_transit = int(round(100.0 * frac["transit"]))
        assert counts.get("car", 0) == expected_car
        assert counts.get("walk", 0) == expected_walk
        assert counts.get("bike", 0) == expected_bike
        assert counts.get("transit", 0) == expected_transit

    def test_detect_columns_km_absent_is_none_not_error(self, tmp_path):
        """A viajes file with no viajes_km column must not break column
        detection -- cols['km'] should just be None, matching the existing
        optional-column pattern already used for mode/distance."""
        f = tmp_path / "sample.csv"
        f.write_text("origen|destino|periodo|distancia|viajes\n1|2|8|2-10|10\n")
        cols = detect_columns(f)
        assert cols["km"] is None
        assert cols["distance"] == "distancia"


class TestModeChoiceHook:
    def test_transit_fraction_unchanged_car_walk_bike_redistributed(self):
        """mode_choice_fn must NOT touch transit's share (this function has
        no real data source for transit itself) -- only car/walk/bike are
        replaced, rescaled to fill (1 - transit_frac)."""
        od_fua = _base_od_fua(distancia="2-10", viajes_km=400.0)  # avg 4.0 km/trip
        cols = _cols(with_km=True)

        def fake_model(distance_km):
            assert distance_km == pytest.approx(4.0)
            return {"car": 0.2, "walk": 0.7, "bike": 0.1}

        rng = np.random.default_rng(42)
        result = build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                               occupancy_factor=1.2, mode_choice_fn=fake_model)
        counts = result.groupby("mode")["count"].sum()

        transit_frac_old = DISTANCE_MODE_FRACTIONS["2-10"]["transit"]
        assert counts.get("transit", 0) == int(round(100.0 * transit_frac_old))

        # tolleranza di 1 sui singoli conteggi: ogni modo viene arrotondato
        # indipendentemente (int(round(...))), quindi un valore esattamente
        # a meta' (es. 45.5) puo' arrotondare in un verso o nell'altro a
        # seconda di rumore in virgola mobile -- non e' un bug della logica
        # di redistribuzione, che e' invece verificata qui sulla massa totale.
        non_transit_mass = 100.0 * (1.0 - transit_frac_old)
        expected_car = non_transit_mass * 0.2 / 1.2
        expected_walk = non_transit_mass * 0.7
        expected_bike = non_transit_mass * 0.1
        assert abs(counts.get("car", 0) - expected_car) <= 1
        assert abs(counts.get("walk", 0) - expected_walk) <= 1
        assert abs(counts.get("bike", 0) - expected_bike) <= 1

    def test_falls_back_to_band_midpoint_when_viajes_km_missing(self):
        """If viajes_km is absent/zero for a row, use
        DISTANCE_BAND_FALLBACK_KM for that band instead of skipping the row
        or silently defaulting to 0 -- must never fabricate a different
        distance than the documented band fallback."""
        od_fua = _base_od_fua(distancia="10-50", viajes_km=None)
        cols = _cols(with_km=False)  # niente colonna km disponibile affatto

        seen_distances = []

        def fake_model(distance_km):
            seen_distances.append(distance_km)
            return {"car": 1.0, "walk": 0.0, "bike": 0.0}

        rng = np.random.default_rng(42)
        build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                     occupancy_factor=1.2, mode_choice_fn=fake_model)
        assert seen_distances == [DISTANCE_BAND_FALLBACK_KM["10-50"]]

    def test_period_passed_when_mode_choice_fn_declares_it(self):
        """If mode_choice_fn's signature has a `period` parameter,
        build_od_rows must call it with period=<MITMA hour from the row> --
        lets a caller apply time-of-day-dependent adjustments (e.g. a
        peak-hour congestion bonus) without a separate code path."""
        od_fua = _base_od_fua(distancia="2-10", viajes_km=400.0)
        cols = _cols(with_km=True)
        seen_periods = []

        def fake_model(distance_km, period=None):
            seen_periods.append(period)
            return {"car": 1.0, "walk": 0.0, "bike": 0.0}

        rng = np.random.default_rng(42)
        build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                     occupancy_factor=1.2, mode_choice_fn=fake_model)
        assert seen_periods == [8]  # "periodo": 8 in _base_od_fua

    def test_mode_choice_fn_without_period_param_still_works(self):
        """A mode_choice_fn with the pre-existing single-arg signature (no
        `period`) must keep working unchanged -- inspect.signature() must
        not force a period kwarg onto callers that never declared one."""
        od_fua = _base_od_fua(distancia="2-10", viajes_km=400.0)
        cols = _cols(with_km=True)

        def fake_model(distance_km):
            return {"car": 1.0, "walk": 0.0, "bike": 0.0}

        rng = np.random.default_rng(42)
        result = build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                               occupancy_factor=1.2, mode_choice_fn=fake_model)
        assert result["count"].sum() > 0

    def test_mode_choice_fn_output_still_sums_with_transit_to_one(self):
        od_fua = _base_od_fua(distancia="0.5-2", viajes_km=125.0)  # avg 1.25 km/trip
        cols = _cols(with_km=True)

        def fake_model(distance_km):
            return {"car": 0.3, "walk": 0.5, "bike": 0.2}

        rng = np.random.default_rng(0)
        result = build_od_rows(od_fua, _zone_to_nodes(), cols, rng, noise_sigma=0.0,
                               occupancy_factor=1.0, mode_choice_fn=fake_model)
        # senza noise e senza conversione occupancy, la somma dei conteggi
        # (arrotondati) deve restare vicina a 100 (tolleranza per arrotondamento
        # indipendente per ciascun modo)
        total = result["count"].sum()
        assert abs(total - 100) <= 4


class TestStreamViajesKmTracking:
    def test_viajes_km_averaged_alongside_viajes_when_present(self, tmp_path):
        f1 = tmp_path / "20220207_viajes.csv.gz"
        f2 = tmp_path / "20220214_viajes.csv.gz"
        header = "origen|destino|periodo|distancia|viajes|viajes_km\n"
        # stessa coppia OD, due lunedi' diversi, viajes_km diversi -> media attesa
        pd.DataFrame([{"origen": "1", "destino": "2", "periodo": 8,
                       "distancia": "2-10", "viajes": 10.0, "viajes_km": 40.0}]).to_csv(
            f1, sep="|", index=False, compression="gzip")
        pd.DataFrame([{"origen": "1", "destino": "2", "periodo": 8,
                       "distancia": "2-10", "viajes": 20.0, "viajes_km": 100.0}]).to_csv(
            f2, sep="|", index=False, compression="gzip")

        cols = {"orig": "origen", "dest": "destino", "period": "periodo",
                "trips": "viajes", "mode": None, "distance": "distancia", "km": "viajes_km"}
        out = stream_viajes([f1, f2], cols, fua_zone_ids={"1", "2"}, weekend=False, holidays_set=set())
        assert out is not None
        assert "viajes_km" in out.columns
        row = out.iloc[0]
        assert row["viajes"] == pytest.approx((10.0 + 20.0) / 2)
        assert row["viajes_km"] == pytest.approx((40.0 + 100.0) / 2)
