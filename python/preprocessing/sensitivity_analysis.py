#!/usr/bin/env python3
"""Demand-scale sensitivity analysis for nomad.

Runs nomad_cli at multiple demand_scale values and extracts key metrics
(arrival rate, stuck agents, teleported, simulation wall time) from the logs.

Usage:
  python preprocessing/sensitivity_analysis.py \
      --config data/schemas/scenario_palma.json \
      --binary build/nomad_cli \
      --scales 0.6 0.7 0.8 0.9 1.0
"""

import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def run_simulation(binary: Path, config_path: Path,
                   scale: float, timeout_s: int) -> "tuple[str, float]":
    """Write a patched config JSON with demand_scale=scale and run nomad_cli.

    Returns (log_output, wall_time_s).
    """
    with config_path.open() as f:
        cfg = json.load(f)

    cfg.setdefault("demand", {})["demand_scale"] = scale

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False, prefix=f"nomad_s{scale:.2f}_"
    ) as tf:
        json.dump(cfg, tf, indent=2)
        tmp_path = Path(tf.name)

    try:
        import time
        t0 = time.perf_counter()
        result = subprocess.run(
            [str(binary), "--config", str(tmp_path)],
            capture_output=True, text=True, timeout=timeout_s,
            cwd=str(ROOT),
        )
        wall = time.perf_counter() - t0
        log = result.stdout + "\n" + result.stderr
        return log, wall
    finally:
        tmp_path.unlink(missing_ok=True)


def parse_metrics(log: str, scale: float) -> dict:
    """Extract key metrics from nomad_cli spdlog output."""
    m: dict = {"scale": scale}

    # Total agents injected
    hit = re.search(r"Pre-routing\s+(\d+)\s+agents", log)
    if hit:
        m["agents"] = int(hit.group(1))

    # Final states: Waiting=X OnLink=X AtActivity=X Arrived=X
    hit = re.search(
        r"Final states:\s+Waiting=(\d+)\s+OnLink=(\d+)\s+AtActivity=(\d+)\s+Arrived=(\d+)",
        log,
    )
    if hit:
        waiting, onlink, atact, arrived = (int(x) for x in hit.groups())
        total = waiting + onlink + atact + arrived
        m["arrived"]    = arrived
        m["stuck"]      = onlink + waiting
        m["arrival_pct"] = 100.0 * arrived / total if total else 0.0

    # Teleported
    hit = re.search(r"Teleported=(\d+)", log)
    if hit:
        m["teleported"] = int(hit.group(1))

    # Simulation complete in X.Xs
    hit = re.search(r"Simulation complete in ([\d.]+)s", log)
    if hit:
        m["sim_wall_s"] = float(hit.group(1))

    # Aggregate V/C
    hit = re.search(r"Aggregate V/C:\s*([\d.]+)", log)
    if hit:
        m["vc_ratio"] = float(hit.group(1))

    return m


def main() -> None:
    parser = argparse.ArgumentParser(description="Sensitivity analysis on demand_scale")
    parser.add_argument("--config",  required=True,
                        help="Path to scenario JSON (e.g. data/schemas/scenario_palma.json)")
    parser.add_argument("--binary",  default=str(ROOT / "build" / "nomad_cli"),
                        help="Path to nomad_cli binary (default: build/nomad_cli)")
    parser.add_argument("--scales",  nargs="+", type=float,
                        default=[0.6, 0.7, 0.8, 0.9, 1.0],
                        help="demand_scale values to test (default: 0.6 0.7 0.8 0.9 1.0)")
    parser.add_argument("--timeout", type=int, default=3600,
                        help="Timeout per run in seconds (default: 3600)")
    parser.add_argument("--log-dir", default=None,
                        help="Directory to save per-run log files (optional)")
    args = parser.parse_args()

    binary     = Path(args.binary)
    config_path = Path(args.config)

    if not binary.exists():
        sys.exit(f"nomad_cli not found: {binary}\n"
                 "Build first:  cmake --build build --target nomad_cli -j$(nproc)")
    if not config_path.exists():
        sys.exit(f"Config not found: {config_path}")

    log_dir = Path(args.log_dir) if args.log_dir else None
    if log_dir:
        log_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for scale in sorted(args.scales):
        print(f"\n{'='*60}")
        print(f"  demand_scale = {scale:.2f}")
        print(f"{'='*60}")
        try:
            log, wall = run_simulation(binary, config_path, scale, args.timeout)
        except subprocess.TimeoutExpired:
            print(f"  TIMEOUT after {args.timeout}s")
            results.append({"scale": scale, "timeout": True})
            continue

        if log_dir:
            (log_dir / f"run_scale_{scale:.2f}.log").write_text(log)

        metrics = parse_metrics(log, scale)
        metrics["wall_s"] = wall
        results.append(metrics)

        # Print a brief summary
        agents   = metrics.get("agents",   "?")
        arrived  = metrics.get("arrived",  "?")
        pct      = metrics.get("arrival_pct", "?")
        stuck    = metrics.get("stuck",    "?")
        tele     = metrics.get("teleported","?")
        vc       = metrics.get("vc_ratio", "?")
        sim_w    = metrics.get("sim_wall_s","?")
        print(f"  agents={agents}  arrived={arrived} ({pct:.1f}% if numeric)")
        print(f"  stuck={stuck}  teleported={tele}  V/C={vc}")
        print(f"  wall={wall:.0f}s  (sim_reported={sim_w}s)")

    # ── Summary table ────────────────────────────────────────────────────────
    print(f"\n{'='*80}")
    print("SENSITIVITY ANALYSIS SUMMARY")
    print(f"{'='*80}")
    hdr = f"{'scale':>6} {'agents':>8} {'arrived':>8} {'arr%':>6} "
    hdr += f"{'stuck':>7} {'tele':>7} {'V/C':>6} {'wall_s':>7}"
    print(hdr)
    print("-" * 60)
    for r in results:
        if r.get("timeout"):
            print(f"  {r['scale']:>4.2f}   TIMEOUT")
            continue
        s    = r.get("scale",       "?")
        ag   = r.get("agents",      "?")
        arr  = r.get("arrived",     "?")
        pct  = r.get("arrival_pct", float("nan"))
        stk  = r.get("stuck",       "?")
        tel  = r.get("teleported",  "?")
        vc   = r.get("vc_ratio",    float("nan"))
        wl   = r.get("wall_s",      float("nan"))
        print(f"  {s:>4.2f} {ag:>8} {arr:>8} {pct:>6.1f} "
              f"{stk:>7} {tel:>7} {vc:>6.3f} {wl:>7.0f}")

    # Optionally save results as JSON
    out_path = config_path.parent / "sensitivity_results.json"
    with out_path.open("w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nResults saved to: {out_path}")


if __name__ == "__main__":
    main()
