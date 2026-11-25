"""Tests for the nomad_tools analysis modules on small synthetic inputs."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from nomad_tools import chain_summary, count_compare, gridlock_scan, io  # noqa: E402
from test_nomad_tools_io import write_graph  # noqa: E402


def test_parse_log(tmp_path):
    p = tmp_path / 'r.o'
    p.write_text('[info] replanning: RG=0.0175 12688 switched\n'
                 '[info] Final states: Waiting=70 OnLink=2535 AtActivity=0 Arrived=1013679\n')
    s = chain_summary.parse_log(p)
    assert s['finished'] and s['OnLink'] == 2535 and s['Waiting'] == 70 and s['Arrived'] == 1013679
    assert s['rg'] == 0.0175 and s['switched'] == 12688
    assert chain_summary.parse_log(tmp_path / 'missing.o') is None
    p.write_text('[info] still running\n')
    assert chain_summary.parse_log(p)['finished'] is False


def test_section_edges_crossing_and_class_filter():
    feats = [{'properties': {'edge_id': 0, 'road_class': 0}, 'geometry': {'coordinates': [[0, -1], [0, 1]]}},
             {'properties': {'edge_id': 1, 'road_class': 4}, 'geometry': {'coordinates': [[0.1, -1], [0.1, 1]]}},
             {'properties': {'edge_id': 2, 'road_class': 0}, 'geometry': {'coordinates': [[5, -1], [5, 1]]}},
             {'properties': {'edge_id': 3, 'road_class': 0}, 'geometry': {'coordinates': [[0.2, -1], [0.2, -0.5], [0.3, 1]]}}]
    # horizontal section from x=-1 to x=1 at y=0, motorway only
    assert count_compare.section_edges(feats, (-1, 0), (1, 0), {0}) == [0, 3]
    assert count_compare.section_edges(feats, (-1, 0), (1, 0), {0, 4}) == [0, 1, 3]


def test_full_cycles_detects_two_link_loop(tmp_path):
    p = tmp_path / 'g.bin'; write_graph(p)
    g = io.read_graph(p)
    stor = gridlock_scan.storage(g)            # lanes 2,1,1,1 and length < 50 -> 13.3, 6.7, 6.7, 6.7
    cnt = np.zeros(4)
    cnt[[0, 3]] = stor[[0, 3]]                 # 0: 0->1 and 3: 1->0 full: a 2-link cycle
    full, cyc = gridlock_scan.full_cycles(g, cnt, stor)
    assert full.sum() == 2 and len(cyc) == 1 and sorted(cyc[0][1].tolist()) == [0, 3]
    cnt[3] = 0.5 * stor[3]                     # break the cycle
    assert gridlock_scan.full_cycles(g, cnt, stor)[1] == []
    cnt[3] = stor[3]
    mask = np.array([True, True, True, False]) # masking a member also removes the cycle
    assert gridlock_scan.full_cycles(g, cnt, stor, mask)[1] == []
