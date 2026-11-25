"""Generate (and optionally submit) a day-to-day assignment chain from a city preset.

A chain is iter0 (free-flow routing, no warm start) followed by iter1..iterN of
the gap replanning rule (alpha 1, band 0.05, link-cost smoothing 0.5), each
iteration warm-started from the previous one's routes.bin and td_costs.bin.

The city preset (TOML) names the workspace, the run prefix, the two template
configs (one free-flow, one warm-started), the available networks and demands,
the default node rules, the binary and the Slurm settings. Example:

    workspace = "/data/workspaces/me/nomad"
    run_prefix = "palma_feb2022_"
    template_iter0 = "configs/palma_feb2022_baseline_iter0.json"   # relative to workspace
    template_warm = "configs/palma_feb2022_baseline_iter1.json"
    binary = "/home/me/nomad/build-cluster-{sha}/nomad_cli"
    default_binary = "a857621"
    default_demand = "v4"
    default_network = "07"
    [demands]
    v4 = "inputs/palma/palma_demand_v4.csv"
    [networks.07]
    graph = "inputs/palma/palma_network_07.bin"
    tags = "inputs/palma/palma_osm_tags.yaml"
    [node_rules]                      # applied unless --no-node-rules
    ltm_priority_service = true
    [config]                          # optional dotted overrides of the templates
    "simulation.num_threads" = 16
    [slurm]
    host = "nuredduna"
    path = "$HOME/bin:/common/slurm_22.04/bin"
    time = 300                        # minutes per iteration
    cores = 4
    mem = 8                           # GB

Writes, for chain <name> (run names <run_prefix><name>_iter<k>):
  configs/<run>.json, jobs/<run>.sh, jobs/<run_prefix><name>_chain.txt
Each job writes runs/<run>/.provenance.json before running nomad_cli.

usage:
  python -m nomad_tools.launch_chain <preset.toml> <name> [--demand D] [--network N]
         [--iters 8] [--no-node-rules] [--traffic key=value ...] [--binary SHA]
         [--submit] [--time MIN] [--cores N] [--mem GB]
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import re
import stat
import subprocess
import sys
import tomllib

GAP = dict(replan_rule='gap', replan_gap_alpha=1.0, replan_gap_band=0.05, replan_fraction=1.0)
WARM_KEYS = ('td_costs_in', 'routes_in', 'replan_fraction', 'replan_seed', 'replan_rule',
             'replan_gap_alpha', 'replan_gap_band')
NODE_RULE_KEYS = ('ltm_priority_service', 'ltm_priority_cycle_max_m', 'ltm_minor_squeeze_stuck_receiver')


def value(v):
    if v.lower() in ('true', 'false'):
        return v.lower() == 'true'
    try:
        return float(v)
    except ValueError:
        return v


def set_dotted(cfg, key, val):
    *path, last = key.split('.')
    d = cfg
    for k in path:
        d = d.setdefault(k, {})
    d[last] = val


def build_chain(preset, name, demand, network, iters, node_rules, traffic, sha):
    """Write configs, jobs and chain list; return (chain file, run names)."""
    W = preset['workspace']
    ab = lambda p: p if os.path.isabs(p) else os.path.join(W, p)
    P = preset['run_prefix']
    if not re.fullmatch(r'[a-z0-9_]+', name):
        raise ValueError('name: use lowercase letters, digits and _ only')
    chain = ab(f'jobs/{P}{name}_chain.txt')
    if os.path.exists(chain) or os.path.exists(ab(f'runs/{P}{name}_iter0')):
        raise FileExistsError(f'chain {name} already exists: pick another name')
    if demand not in preset['demands']:
        raise KeyError(f'demand {demand!r} not in preset (have {sorted(preset["demands"])})')
    if network not in preset['networks']:
        raise KeyError(f'network {network!r} not in preset (have {sorted(preset["networks"])})')
    binary = preset['binary'].format(sha=sha)
    od, net = ab(preset['demands'][demand]), preset['networks'][network]
    graph, tags = ab(net['graph']), ab(net['tags']) if 'tags' in net else None
    t0, t1 = ab(preset['template_iter0']), ab(preset['template_warm'])
    for p in (binary, od, graph, t0, t1) + ((tags,) if tags else ()):
        if not os.path.exists(p):
            raise FileNotFoundError(p)
    tr = dict(preset.get('node_rules', {})) if node_rules else {}
    tr.update(traffic)
    t0, t1 = json.load(open(t0)), json.load(open(t1))
    names, prev = [], None
    for it in range(iters + 1):
        run = f'{P}{name}_iter{it}'
        out = ab(f'runs/{run}')
        c = copy.deepcopy(t1 if it else t0)
        for k, v in preset.get('config', {}).items():
            set_dotted(c, k, v)
        c['name'] = run
        c['network']['graph_bin'] = graph
        if tags:
            c['network']['osm_tag_config'] = tags
        c['demand']['od_csv'] = od
        c.setdefault('traffic', {})
        for k in NODE_RULE_KEYS:
            c['traffic'].pop(k, None)
        c['traffic'].update(tr)
        c['output'].update(output_dir=out, td_costs_out=f'{out}/td_costs.bin', routes_out=f'{out}/routes.bin')
        r = c['routing']
        if prev:
            r.update(td_costs_in=ab(f'runs/{prev}/td_costs.bin'), routes_in=ab(f'runs/{prev}/routes.bin'),
                     replan_seed=100 + it, **GAP)
            c['output']['td_smoothing'] = 0.5
        else:
            for k in WARM_KEYS:
                r.pop(k, None)
            c['output'].pop('td_smoothing', None)
        cfg = ab(f'configs/{run}.json')
        json.dump(c, open(cfg, 'w'), indent=2)
        prov = {'config': cfg, 'binary': binary, 'nomad_commit': sha, 'graph': graph, 'od': od,
                'demand': demand, 'network': network, 'traffic_overrides': tr, 'warm_start_from': prev,
                'generator': 'nomad_tools.launch_chain',
                'replanning': {**GAP, 'seed': 100 + it, 'td_smoothing': 0.5} if prev else None}
        job = ab(f'jobs/{run}.sh')
        with open(job, 'w') as fh:
            fh.write(f"#!/bin/bash\nset -euo pipefail\nmkdir -p {out}\n"
                     f"echo '{json.dumps(prov)}' > {out}/.provenance.json\n"
                     f"/usr/bin/time -v {binary} --config {cfg}\n")
        os.chmod(job, os.stat(job).st_mode | stat.S_IXUSR | stat.S_IXGRP)
        names.append(run)
        prev = run
    with open(chain, 'w') as fh:
        fh.write('\n'.join(names) + '\n')
    return chain, names


def submit(preset, chain, time, cores, mem):
    s = preset['slurm']
    script = f"""export PATH={s.get('path', '$PATH')}:$PATH; cd {preset['workspace']}
prev=""; ids=""
while read n; do
  dep=""; [ -n "$prev" ] && dep="-d $prev"
  j=$(run -t {time} -c {cores} -m {mem} -j $n -o logs/$n.o -e logs/$n.e $dep jobs/$n.sh 2>&1 | grep -o "[0-9]\\{{6,\\}}" | tail -1)
  [ -n "$j" ] || {{ echo "SUBMIT FAIL $n"; exit 1; }}; prev=$j; ids="$ids $j"
done < {chain}
echo "job ids:$ids"
"""
    return subprocess.run(['ssh', '-o', 'ConnectTimeout=20', s['host'], script], capture_output=True, text=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('preset')
    ap.add_argument('name')
    ap.add_argument('--demand')
    ap.add_argument('--network')
    ap.add_argument('--iters', type=int, default=8)
    ap.add_argument('--no-node-rules', action='store_true')
    ap.add_argument('--traffic', nargs='*', default=[], help='extra traffic settings key=value')
    ap.add_argument('--binary', help='commit sha of the build (default: preset default_binary)')
    ap.add_argument('--submit', action='store_true')
    ap.add_argument('--time', type=int)
    ap.add_argument('--cores', type=int)
    ap.add_argument('--mem', type=int)
    a = ap.parse_args(argv)
    preset = tomllib.load(open(a.preset, 'rb'))
    demand, network = a.demand or preset['default_demand'], a.network or preset['default_network']
    sha = a.binary or preset['default_binary']
    traffic = {k: value(v) for k, v in (x.split('=', 1) for x in a.traffic)}
    try:
        chain, names = build_chain(preset, a.name, demand, network, a.iters, not a.no_node_rules, traffic, sha)
    except (ValueError, KeyError, FileExistsError, FileNotFoundError) as e:
        sys.exit(f'error: {e}')
    print(f'{len(names)} runs -> {chain}\n  demand {demand}, network {network}, binary {sha}, '
          f'node rules {"off" if a.no_node_rules else "on"}, extra traffic {traffic}')
    if not a.submit:
        print('not submitted (add --submit)')
        return
    s = preset.get('slurm', {})
    res = submit(preset, chain, a.time or s.get('time', 300), a.cores or s.get('cores', 4), a.mem or s.get('mem', 8))
    print(res.stdout.strip() or res.stderr.strip())
    sys.exit(res.returncode)


if __name__ == '__main__':
    main()
