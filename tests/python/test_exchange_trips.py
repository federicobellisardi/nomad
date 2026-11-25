"""Tests for exchange_trips.gateway_offsets: inbound shift by the outside leg and
clipping of departure windows at the end of the day."""
from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python/preprocessing'))
from exchange_trips import gateway_offsets  # noqa: E402

GW = pd.DataFrame({'out_zone': ['A', 'A'], 'fua_zone': ['f1', 'f2'], 'outside_s': [600.0, 1800.0]})


def rows(starts):
    return pd.DataFrame({'orig_zone': ['XO|A|f1', 'XO|A|f2', 'f1'], 'dest_zone': ['f1', 'f2', 'XD|f1|A'],
                         'count': [1, 2, 3], 'depart_mean_s': starts, 'depart_std_s': [s + 3600 for s in starts]})


def test_inbound_shift_outbound_untouched():
    r, n, v, _, ng, vg = gateway_offsets(rows([3600.0, 7200.0, 3600.0]), GW, 86400.0)
    assert list(r.depart_mean_s) == [4200.0, 9000.0, 3600.0]
    assert list(r.depart_std_s) == [7800.0, 12600.0, 7200.0]
    assert (n, v, ng, vg) == (0, 0, 0, 0)


def test_clip_late_windows_keeps_count():
    r, n, v, _, ng, _ = gateway_offsets(rows([82800.0, 82800.0, 82800.0]), GW, 86400.0)
    # f2 window 84600..88200 -> end clipped; f1 window 83400..87000 -> end clipped; outbound unchanged
    assert list(r.depart_std_s) == [86400.0, 86400.0, 86400.0]
    assert (n, v, ng) == (2, 3, 0) and list(r['count']) == [1, 2, 3]


def test_window_starting_after_day_end_is_dropped():
    r, n, v, _, ng, vg = gateway_offsets(rows([85000.0, 85000.0, 0.0]), GW, 86400.0)
    # f1: 85600..89200 kept and clipped; f2: 86800.. starts after midnight -> dropped
    assert (ng, vg) == (1, 2) and list(r.orig_zone) == ['XO|A|f1', 'f1'] and r.depart_std_s.iloc[0] == 86400.0
