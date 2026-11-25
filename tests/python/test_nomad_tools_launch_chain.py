"""Tests for nomad_tools.launch_chain.build_chain on a temporary workspace."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python'))
from nomad_tools import launch_chain  # noqa: E402


@pytest.fixture
def ws(tmp_path):
    for d in ('configs', 'jobs', 'runs', 'inputs'):
        (tmp_path / d).mkdir()
    for f in ('inputs/g.bin', 'inputs/t.yaml', 'inputs/od.csv', 'nomad_cli_abc'):
        (tmp_path / f).write_text('x')
    base = {'name': 'x', 'simulation': {'num_threads': 4}, 'network': {'graph_bin': 'old'},
            'demand': {'od_csv': 'old'}, 'traffic': {'ltm_priority_service': True, 'ltm_stuck_time_s': 30},
            'routing': {'td_costs_in': 'old', 'replan_rule': 'gap'}, 'output': {'td_smoothing': 0.5}}
    (tmp_path / 'configs/t0.json').write_text(json.dumps(base))
    (tmp_path / 'configs/t1.json').write_text(json.dumps(base))
    preset = {'workspace': str(tmp_path), 'run_prefix': 'city_', 'template_iter0': 'configs/t0.json',
              'template_warm': 'configs/t1.json', 'binary': str(tmp_path / 'nomad_cli_{sha}'),
              'demands': {'v1': 'inputs/od.csv'}, 'networks': {'07': {'graph': 'inputs/g.bin', 'tags': 'inputs/t.yaml'}},
              'node_rules': {'ltm_priority_service': True, 'ltm_priority_cycle_max_m': 500.0},
              'config': {'simulation.num_threads': 16}}
    return tmp_path, preset


def test_chain_structure(ws):
    root, preset = ws
    chain, names = launch_chain.build_chain(preset, 'exp', 'v1', '07', 2, True, {'ltm_stuck_time_s': 10.0}, 'abc')
    assert names == ['city_exp_iter0', 'city_exp_iter1', 'city_exp_iter2']
    assert Path(chain).read_text().split() == names
    c0 = json.loads((root / 'configs/city_exp_iter0.json').read_text())
    c2 = json.loads((root / 'configs/city_exp_iter2.json').read_text())
    assert 'td_costs_in' not in c0['routing'] and 'td_smoothing' not in c0['output']
    assert c2['routing']['routes_in'] == str(root / 'runs/city_exp_iter1/routes.bin')
    assert c2['routing']['replan_seed'] == 102 and c2['routing']['replan_rule'] == 'gap'
    assert c2['output']['td_smoothing'] == 0.5
    assert c0['traffic'] == {'ltm_priority_service': True, 'ltm_priority_cycle_max_m': 500.0, 'ltm_stuck_time_s': 10.0}
    assert c0['simulation']['num_threads'] == 16 and c0['demand']['od_csv'] == str(root / 'inputs/od.csv')
    assert c0['network']['osm_tag_config'] == str(root / 'inputs/t.yaml')
    job = (root / 'jobs/city_exp_iter1.sh').read_text()
    assert 'nomad_cli_abc --config' in job and '.provenance.json' in job


def test_no_node_rules_removes_them(ws):
    root, preset = ws
    launch_chain.build_chain(preset, 'off', 'v1', '07', 0, False, {}, 'abc')
    c0 = json.loads((root / 'configs/city_off_iter0.json').read_text())
    assert 'ltm_priority_service' not in c0['traffic'] and c0['traffic']['ltm_stuck_time_s'] == 30


def test_refuses_existing_and_bad_inputs(ws):
    root, preset = ws
    launch_chain.build_chain(preset, 'exp', 'v1', '07', 0, True, {}, 'abc')
    with pytest.raises(FileExistsError):
        launch_chain.build_chain(preset, 'exp', 'v1', '07', 0, True, {}, 'abc')
    with pytest.raises(KeyError):
        launch_chain.build_chain(preset, 'new', 'v9', '07', 0, True, {}, 'abc')
    with pytest.raises(FileNotFoundError):
        launch_chain.build_chain(preset, 'new2', 'v1', '07', 0, True, {}, 'missing')
    with pytest.raises(ValueError):
        launch_chain.build_chain(preset, 'Bad-Name', 'v1', '07', 0, True, {}, 'abc')
