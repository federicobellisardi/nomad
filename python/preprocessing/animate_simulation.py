#!/usr/bin/env python3
"""
animate_simulation.py — Video animato dello stato della simulazione nomad.

Mostra la rete stradale di Bologna frame per frame, colorando gli archi
per congestione o flusso ad ogni snapshot GeoJSON.

Utilizzo:
    # Finestra interattiva (città obbligatoria)
    python python/preprocessing/animate_simulation.py --city "Palma de Mallorca"

    # Salva GIF (nessun codec necessario)
    python python/preprocessing/animate_simulation.py --city "Palma de Mallorca" --save results/palma_animation.gif

    # Salva MP4 (richiede ffmpeg, qualità migliore)
    python python/preprocessing/animate_simulation.py --city "Palma de Mallorca" --save results/palma_animation.mp4

    # Colorare per flusso (numero agenti) invece che congestione
    python python/preprocessing/animate_simulation.py --city "Palma de Mallorca" --metric flow

    # Intervallo tra frame in ms (default 800)
    python python/preprocessing/animate_simulation.py --city "Palma de Mallorca" --interval 600

    # Cartella personalizzata (override)
    python python/preprocessing/animate_simulation.py --result-dir results/my_run

    # In Jupyter:
    import sys; sys.path.insert(0, "python/preprocessing")
    from animate_simulation import build_animation
    from IPython.display import HTML
    ani = build_animation(result_dir=..., city_name="Palma de Mallorca")
    HTML(ani.to_jshtml())
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import unicodedata
import colorsys
from typing import Optional


def slugify(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_",
                  unicodedata.normalize("NFKD", name.lower())
                  .encode("ascii", "ignore").decode()).strip("_")

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import numpy as np
import contextily as ctx

ROOT = pathlib.Path(__file__).parent.parent.parent

# ── Color palettes ─────────────────────────────────────────────────────────────
CMAP_CONGESTION = matplotlib.colormaps["RdBu_r"]    # blue→red
CMAP_FLOW       = matplotlib.colormaps["YlOrRd"]    # yellow→red

BASEMAP_LIGHT = ctx.providers.CartoDB.Positron
BASEMAP_DARK  = ctx.providers.CartoDB.DarkMatter


# ── Data loading ───────────────────────────────────────────────────────────────

def load_static_network(result_dir: pathlib.Path) -> list:
    """Load static_network.geojson as background segments. Returns [[p0,p1], ...] list."""
    path = result_dir / "static_network.geojson"
    if not path.exists():
        print("static_network.geojson non trovato — sfondo rete omesso.")
        return []
    size_mb = path.stat().st_size / 1e6
    print(f"Carico rete statica ({size_mb:.0f} MB) ...")
    data = json.loads(path.read_text())
    segments = []
    for feat in data.get("features", []):
        coords = feat["geometry"]["coordinates"]
        for i in range(len(coords) - 1):
            segments.append([
                [coords[i][0],   coords[i][1]],
                [coords[i+1][0], coords[i+1][1]],
            ])
    print(f"  {len(segments):,} segmenti rete statica caricati.")
    return segments


def load_snapshots(result_dir: pathlib.Path) -> list[dict]:
    """Return snapshot metadata (path, time, label) — features loaded lazily per frame."""
    files = sorted(result_dir.glob("network_state_*.geojson"))
    snapshots = []
    for path in files:
        m = re.search(r"(\d+)\.geojson$", path.name)
        if not m:
            continue
        if path.stat().st_size < 200:   # skip empty files without parsing
            continue
        t = int(m.group(1))
        h, mi = t // 3600, (t % 3600) // 60
        snapshots.append({
            "time_s": t,
            "label":  f"{h:02d}:{mi:02d}",
            "path":   path,
        })
    return snapshots


def load_frame(snap: dict) -> list[dict]:
    """Load feature list for a single snapshot (called per-frame during animation)."""
    data = json.loads(snap["path"].read_text())
    return data.get("features", [])


def extract_segments(
    features: list[dict],
    metric: str = "congestion",
) -> tuple[list, np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns:
        segments    — list of [[x0,y0],[x1,y1]] pairs (LineCollection format)
        values      — float array of the requested metric per segment
        counts      — int array of agent counts per segment
        flows       — float array of flow_veh_h per segment
    """
    segments, values, counts, flows = [], [], [], []
    for feat in features:
        coords = feat["geometry"]["coordinates"]
        props  = feat["properties"]
        val    = float(props.get(metric, 0))
        cnt    = int(props.get("count", 1))
        flw    = float(props.get("flow_veh_h", 0))
        for i in range(len(coords) - 1):
            segments.append([
                [coords[i][0],   coords[i][1]],
                [coords[i+1][0], coords[i+1][1]],
            ])
            values.append(val)
            counts.append(cnt)
            flows.append(flw)
    return (segments,
            np.array(values, dtype=float),
            np.array(counts, dtype=int),
            np.array(flows,  dtype=float))


# ── Bounding box ───────────────────────────────────────────────────────────────

def network_bbox(snapshots: list[dict],
                 static_segs: list | None = None) -> tuple[float, float, float, float]:
    """Return (min_lon, min_lat, max_lon, max_lat).

    If static_segs is provided uses it (already loaded, cheapest).
    Otherwise samples three snapshot files to keep memory low.
    """
    pad = 0.008
    if static_segs:
        lons = [p[0] for seg in static_segs for p in seg]
        lats = [p[1] for seg in static_segs for p in seg]
        return min(lons)-pad, min(lats)-pad, max(lons)+pad, max(lats)+pad

    # Fallback: sample first, middle and last non-empty snapshot
    lons, lats = [], []
    indices = {0, len(snapshots) // 2, len(snapshots) - 1}
    for i in sorted(indices):
        for feat in load_frame(snapshots[i]):
            for c in feat["geometry"]["coordinates"]:
                lons.append(c[0])
                lats.append(c[1])
    return min(lons)-pad, min(lats)-pad, max(lons)+pad, max(lats)+pad


# ── Animation builder ──────────────────────────────────────────────────────────

def build_animation(
    result_dir: pathlib.Path,
    city_name: str = "",
    metric: str = "congestion",      # "congestion" | "flow"
    interval_ms: int = 800,
    figsize: tuple = (11, 10),
    dark_mode: bool = False,
) -> animation.FuncAnimation:
    """Build and return the FuncAnimation object."""

    snapshots = load_snapshots(result_dir)
    if not snapshots:
        raise FileNotFoundError(
            f"Nessuno snapshot non vuoto trovato in {result_dir}.\n"
            "Lancia prima la simulazione:\n"
            "  ./build/nomad_cli --config data/schemas/scenario_<city>.json"
        )
    display_name = city_name or result_dir.name

    cmap   = CMAP_CONGESTION if metric == "congestion" else CMAP_FLOW
    basemap = BASEMAP_DARK if dark_mode else BASEMAP_LIGHT
    bg     = "#1a1a2e" if dark_mode else "#f8f9fa"
    fg     = "#e0e0e0" if dark_mode else "#222222"

    # Load static network first so we can reuse it for bbox (avoids scanning all snapshots)
    static_segs = load_static_network(result_dir)
    min_lon, min_lat, max_lon, max_lat = network_bbox(snapshots, static_segs)

    # ── Figure setup ──────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=figsize, facecolor=bg)
    ax.set_facecolor(bg)
    ax.set_xlim(min_lon, max_lon)
    ax.set_ylim(min_lat, max_lat)
    ax.set_aspect("equal")
    ax.axis("off")

    # CartoDB basemap
    try:
        ctx.add_basemap(
            ax,
            crs="EPSG:4326",
            source=basemap,
            alpha=0.6 if dark_mode else 0.8,
            attribution=False,
        )
    except Exception as e:
        print(f"Basemap non disponibile ({e}) — sfondo vuoto")

    # ── Static network background ─────────────────────────────────────────────
    if static_segs:
        net_color = "#555555" if dark_mode else "#c8c8c8"
        bg_lc = mc.LineCollection(
            static_segs,
            colors=net_color,
            linewidths=0.35,
            alpha=0.55 if dark_mode else 0.45,
            zorder=1,
        )
        ax.add_collection(bg_lc)

    # ── Static elements ───────────────────────────────────────────────────────
    title_text = ax.text(
        0.5, 0.97, "", transform=ax.transAxes,
        ha="center", va="top", fontsize=16, fontweight="bold",
        color=fg,
        bbox=dict(boxstyle="round,pad=0.3", facecolor=bg, alpha=0.7, edgecolor="none"),
    )

    subtitle_text = ax.text(
        0.5, 0.92, "", transform=ax.transAxes,
        ha="center", va="top", fontsize=10,
        color=fg, alpha=0.8,
    )

    # ── LineCollection (updated each frame) ───────────────────────────────────
    # flow_veh_h: cap at 2000 veh/h on the colormap (most edges stay below this)
    FLOW_MAX = 2000.0
    if metric == "flow":
        norm = mcolors.Normalize(vmin=0, vmax=FLOW_MAX)
    else:
        norm = mcolors.Normalize(vmin=0, vmax=1)
    lc    = mc.LineCollection([], cmap=cmap, norm=norm, linewidths=2.5, alpha=0.85, zorder=2)
    ax.add_collection(lc)

    # ── Colorbar ──────────────────────────────────────────────────────────────
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    if metric == "congestion":
        cbar_label = "Congestione [0=free-flow → 1=congested]"
    else:
        cbar_label = f"Flusso [veh/h, saturato a {FLOW_MAX:.0f}]"
    cbar = fig.colorbar(
        sm, ax=ax,
        fraction=0.025, pad=0.01, shrink=0.45, aspect=20,
    )
    cbar.set_label(cbar_label, color=fg, fontsize=9)
    cbar.ax.yaxis.set_tick_params(color=fg, labelcolor=fg)

    # ── Timestamp counter ─────────────────────────────────────────────────────
    progress_bar = ax.axhline(y=min_lat + 0.001, xmin=0, xmax=0,
                               color="#4fc3f7", linewidth=3, alpha=0.7, zorder=5)

    # ── Frame progress indicator (dots) ───────────────────────────────────────
    n = len(snapshots)
    dot_xs = np.linspace(min_lon + 0.01, max_lon - 0.01, n)
    dot_y  = min_lat + 0.003
    ax.scatter(dot_xs, [dot_y]*n, s=40, c=[[0.5,0.5,0.5,0.4]]*n, zorder=4,
               transform=ax.transData)
    active_dot, = ax.plot([], [], 'o', ms=8, color="#4fc3f7", zorder=5)

    # ── Animation function ────────────────────────────────────────────────────
    def update(frame_idx: int):
        snap = snapshots[frame_idx]
        features = load_frame(snap)          # load one file at a time from disk
        segs, vals, counts, flows = extract_segments(features, metric=metric)

        if metric == "flow":
            vals = flows   # use flow_veh_h [veh/h], colormap clipped at FLOW_MAX

        # Line widths: thicker for more congested/busy edges
        widths = 1.5 + np.clip(vals / (FLOW_MAX if metric == "flow" else 1.0), 0, 1) * 4.0

        lc.set_segments(segs)
        lc.set_array(vals)
        lc.set_linewidths(widths)

        # Title
        n_active = len(features)
        title_text.set_text(f"{display_name} — {snap['label']}")
        if metric == "flow":
            mean_v, max_v = np.mean(vals), np.max(vals)
            subtitle_text.set_text(
                f"{n_active} archi attivi  |  "
                f"flusso medio={mean_v:.0f} veh/h  |  max={max_v:.0f} veh/h"
            )
        else:
            subtitle_text.set_text(
                f"{n_active} archi attivi  |  "
                f"congestione media={np.mean(vals):.2f}  |  max={np.max(vals):.2f}"
            )

        # Progress bar
        frac = (frame_idx + 1) / n
        progress_bar.set_xdata([min_lon, min_lon + (max_lon - min_lon) * frac])

        # Active dot
        active_dot.set_data([dot_xs[frame_idx]], [dot_y])

        return lc, title_text, subtitle_text, progress_bar, active_dot

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(snapshots),
        interval=interval_ms,
        blit=False,
        repeat=True,
        repeat_delay=1500,
    )

    # Tight layout
    fig.tight_layout(pad=0.5)

    return ani


# ── Export helper ──────────────────────────────────────────────────────────────

def save_animation(
    ani: animation.FuncAnimation,
    output_path: pathlib.Path,
    fps: int = 2,
    dpi: int = 130,
) -> None:
    """Save animation as GIF or MP4."""
    output_path = pathlib.Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    suffix = output_path.suffix.lower()
    if suffix == ".gif":
        writer = animation.PillowWriter(fps=fps)
    elif suffix in (".mp4", ".avi"):
        writer = animation.FFMpegWriter(fps=fps, bitrate=1800,
                                         extra_args=["-vcodec", "libx264"])
    else:
        raise ValueError(f"Formato non supportato: {suffix}. Usa .gif o .mp4")

    print(f"Salvataggio {output_path} ...")
    ani.save(str(output_path), writer=writer, dpi=dpi)
    size_kb = output_path.stat().st_size / 1024
    print(f"✓  Salvato {output_path} ({size_kb:.0f} KB)")


# ── Jupyter helper ─────────────────────────────────────────────────────────────

def show_in_notebook(metric: str = "congestion", interval_ms: int = 800):
    """Convenience function for Jupyter use. Returns HTML object."""
    from IPython.display import HTML
    plt.ioff()
    ani = build_animation(metric=metric, interval_ms=interval_ms)
    plt.close("all")
    return HTML(ani.to_jshtml(fps=None, default_mode="loop"))


# ── CLI entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Video animato della simulazione nomad",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--city",     metavar="CITY",
                        help="Nome città (es. 'Palma de Mallorca'). "
                             "Deriva automaticamente result-dir da results/<slug>/")
    parser.add_argument("--save",     metavar="PATH",
                        help="Salva in FILE.gif o FILE.mp4 invece di mostrare")
    parser.add_argument("--metric",   default="congestion",
                        choices=["congestion", "flow"],
                        help="Metrica: congestion (default) o flow (veh/h)")
    parser.add_argument("--interval", type=int, default=800,
                        help="Millisecondi per frame (default: 800)")
    parser.add_argument("--dark",     action="store_true",
                        help="Usa CartoDB DarkMatter come sfondo")
    parser.add_argument("--fps",      type=int, default=2,
                        help="FPS per il file salvato (default: 2)")
    parser.add_argument("--dpi",      type=int, default=130,
                        help="DPI per il file salvato (default: 130)")
    parser.add_argument("--result-dir", metavar="PATH",
                        help="Cartella risultati (override rispetto a --city)")
    args = parser.parse_args()

    if args.result_dir:
        result_dir  = pathlib.Path(args.result_dir)
        city_name   = args.city or result_dir.name
    elif args.city:
        result_dir  = ROOT / "results" / slugify(args.city)
        city_name   = args.city
    else:
        parser.error("Specifica --city oppure --result-dir")

    if not result_dir.exists():
        parser.error(f"Cartella risultati non trovata: {result_dir}\n"
                     "Lancia prima la simulazione con nomad_cli.")

    print(f"Carico dati da {result_dir} ...")
    snapshots = load_snapshots(result_dir)
    print(f"  {len(snapshots)} snapshot trovati: "
          f"{[s['label'] for s in snapshots]}")

    ani = build_animation(
        result_dir  = result_dir,
        city_name   = city_name,
        metric      = args.metric,
        interval_ms = args.interval,
        dark_mode   = args.dark,
    )

    if args.save:
        save_animation(ani, pathlib.Path(args.save), fps=args.fps, dpi=args.dpi)
    else:
        print("Premi Q nella finestra per chiudere. Ctrl-C per uscire.")
        plt.show()


if __name__ == "__main__":
    main()
