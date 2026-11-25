"""Per-iteration status of a day-to-day chain, read from the simulator logs.

For each run: agents still on links and still waiting at the end of the day
("Final states"), the relative gap of the replanning step (RG) and the number of
agents that switched plan.

usage:
  python -m nomad_tools.chain_summary --chain jobs/<chain>_chain.txt --logs logs
  python -m nomad_tools.chain_summary --runs <run> [<run> ...] --logs logs
The log of run <run> is <logs>/<run>.o (stdout of nomad_cli).
"""
from __future__ import annotations

import argparse
import os
import re

FINAL = re.compile(r'Final states: (.*)')
RG = re.compile(r'RG=([0-9.]+)')
SWITCHED = re.compile(r'(\d+) switched')


def parse_log(path):
    """dict with final-state counts (Waiting, OnLink, Arrived, ...), rg, switched;
    None if the log does not exist. Missing fields are None (e.g. RG at iter0)."""
    if not os.path.exists(path):
        return None
    txt = open(path, errors='replace').read()
    out = {'finished': False, 'rg': None, 'switched': None}
    m = FINAL.search(txt)
    if m:
        out['finished'] = True
        out.update({k: int(v) for k, v in re.findall(r'(\w+)=(\d+)', m.group(1))})
    m = RG.search(txt)
    if m:
        out['rg'] = float(m.group(1))
    m = SWITCHED.search(txt)
    if m:
        out['switched'] = int(m.group(1))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--chain', help='file with one run name per line')
    g.add_argument('--runs', nargs='+')
    ap.add_argument('--logs', required=True, help='folder with <run>.o')
    a = ap.parse_args(argv)
    runs = [l.strip() for l in open(a.chain) if l.strip()] if a.chain else a.runs
    print(f"{'run':40s} {'OnLink':>8} {'Waiting':>8} {'Arrived':>9} {'RG':>7} {'switched':>9}")
    for r in runs:
        s = parse_log(os.path.join(a.logs, f'{r}.o'))
        if s is None:
            print(f'{r:40s} not run'); continue
        if not s['finished']:
            print(f'{r:40s} running or failed (no Final states)'); continue
        f = lambda v, fmt: format(v, fmt) if v is not None else '-'
        print(f"{r:40s} {s.get('OnLink', 0):8d} {s.get('Waiting', 0):8d} {s.get('Arrived', 0):9d} "
              f"{f(s['rg'], '7.4f'):>7} {f(s['switched'], '9d'):>9}")


if __name__ == '__main__':
    main()
