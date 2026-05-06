"""
Generate top-down PNG previews for DEM .npy files.

Quick usage:
    python scripts/generate_dem_previews.py

    Generate preview PNGs for all DEMs under assets/Terrains/SouthPole, saving preview.png next to each dem.npy.
    python scripts/generate_dem_previews.py
        
    Generate a mosaic preview of all DEMs under assets/Terrains/SouthPole, saving to preview_mosaic.png:
    python scripts/generate_dem_previews.py \
        --mode mosaic \

    Generate a Markdown table of DEM metadata and elevation ranges for all DEMs under assets/Terrains/SouthPole, saving to preview_statistics.md:
    python scripts/generate_dem_previews.py \
        --mode preview_statistics \

Notes:
- The script can process one file with --dem-path, or recursively find files under --root.
- It processes one DEM at a time and saves preview.png next to each dem.npy.
- With --mode mosaic, it renders all selected DEMs in one figure (2 columns, rows as needed).
- With --mode preview_statistics, it writes a Markdown table of DEM metadata and elevation ranges.
- The output image is exactly size x size pixels.
- Colormap is fixed to jet with vmin/vmax set to each DEM min/max so blue=low and red=high.
- A colorbar and in-plot legend show min/max elevations (meters) to anchor the color scale.
- If a sibling dem.yaml exists, the figure includes center coordinates, pixel size, and terrain size.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import ticker
from matplotlib.patches import Patch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate PNG previews for DEM arrays.")
    parser.add_argument(
        "--dem-path",
        type=Path,
        default=None,
        help="Optional explicit path to a single DEM .npy file to process.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("assets/Terrains/SouthPole"),
        help="Root directory to recursively search for DEM files.",
    )
    parser.add_argument(
        "--dem-name",
        type=str,
        default="dem.npy",
        help="DEM filename to search for in each subdirectory.",
    )
    parser.add_argument(
        "--output-name",
        type=str,
        default="preview.png",
        help="Output PNG filename saved next to each DEM.",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=2048,
        help="Output PNG width and height in pixels.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=("preview", "mosaic", "preview_statistics"),
        default="preview",
        help="Processing mode: preview, mosaic, or preview_statistics.",
    )
    parser.add_argument(
        "--mosaic-output",
        type=Path,
        default=None,
        help="Output PNG path for mosaic mode. Defaults to <root>/preview_mosaic.png.",
    )
    parser.add_argument(
        "--mosaic-tile-size",
        type=int,
        default=1024,
        help="Per-tile pixel size used for sampling in mosaic mode.",
    )
    parser.add_argument(
        "--statistics-output",
        type=Path,
        default=None,
        help="Output Markdown path for preview_statistics mode. Defaults to <root>/preview_statistics.md.",
    )
    return parser.parse_args()


def _build_ticks(length: int, max_ticks: int = 10) -> np.ndarray:
    if length <= 1:
        return np.array([0], dtype=int)
    step = max(1, int(np.ceil((length - 1) / max(1, max_ticks - 1))))
    ticks = list(range(0, length, step))
    if ticks[-1] != length - 1:
        ticks.append(length - 1)
    return np.array(ticks, dtype=int)


def _sample_indices(length: int, target: int) -> np.ndarray:
    if length <= target:
        return np.arange(length, dtype=int)
    return np.linspace(0, length - 1, target, dtype=int)


def _compute_min_max(dem: np.ndarray, chunk_rows: int = 2048) -> tuple[float, float]:
    min_elev = np.inf
    max_elev = -np.inf

    total_rows = dem.shape[0]
    for start in range(0, total_rows, chunk_rows):
        end = min(start + chunk_rows, total_rows)
        chunk = dem[start:end, :]
        finite_chunk = np.isfinite(chunk)
        if not np.any(finite_chunk):
            continue
        valid_values = chunk[finite_chunk]
        min_elev = min(min_elev, float(valid_values.min()))
        max_elev = max(max_elev, float(valid_values.max()))

    if not np.isfinite(min_elev) or not np.isfinite(max_elev):
        raise ValueError("DEM has no finite values")

    return min_elev, max_elev


def _load_dem_metadata(dem_path: Path) -> dict[str, Any] | None:
    metadata_path = dem_path.with_suffix(".yaml")
    if not metadata_path.exists() or not metadata_path.is_file():
        return None

    metadata: dict[str, Any] = {}
    current_key: str | None = None

    with metadata_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            if line.endswith(":") and not line.startswith("-"):
                current_key = line[:-1].strip()
                metadata[current_key] = []
                continue

            if line.startswith("-") and current_key is not None:
                value_str = line[1:].strip()
                try:
                    value: Any = float(value_str)
                except ValueError:
                    value = value_str
                metadata[current_key].append(value)

    return metadata if metadata else None


def _build_metadata_lines(metadata: dict[str, Any] | None) -> list[str]:
    if metadata is None:
        return ["Metadata: dem.yaml not found"]

    lines: list[str] = []

    center = metadata.get("center_coordinates")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        lon = float(center[0])
        lat = float(center[1])
        lines.append(f"{'Center Lon/Lat (deg)':<22}{lon:>7.2f}{lat:>7.2f}")
    else:
        lines.append(f"{'Center Lon/Lat (deg)':<22}{'N/A':>7}{'N/A':>7}")

    pixel_size = metadata.get("pixel_size")
    if isinstance(pixel_size, (list, tuple)) and len(pixel_size) >= 2:
        px_x = abs(float(pixel_size[0]))
        px_y = abs(float(pixel_size[1]))
        lines.append(f"{'Pixel size (m)':<22}{px_x:>7.1f}{px_y:>7.1f}")
    else:
        lines.append(f"{'Pixel size (m)':<22}{'N/A':>7}{'N/A':>7}")

    size = metadata.get("size")
    if (
        isinstance(pixel_size, (list, tuple))
        and len(pixel_size) >= 2
        and isinstance(size, (list, tuple))
        and len(size) >= 2
    ):
        terrain_x_km = abs(float(pixel_size[0])) * float(size[0]) / 1000.0
        terrain_y_km = abs(float(pixel_size[1])) * float(size[1]) / 1000.0
        lines.append(f"{'Terrain size (km)':<22}{terrain_x_km:>7.1f}{terrain_y_km:>7.1f}")
    else:
        lines.append(f"{'Terrain size (km)':<22}{'N/A':>7}{'N/A':>7}")

    return lines


def _extract_metadata_values(metadata: dict[str, Any] | None) -> dict[str, str]:
    values = {
        "center": "N/A",
        "pixel_size": "N/A",
        "terrain_size": "N/A",
    }

    if metadata is None:
        return values

    center = metadata.get("center_coordinates")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        lon = float(center[0])
        lat = float(center[1])
        values["center"] = f"{lon:.2f}, {lat:.2f}"

    pixel_size = metadata.get("pixel_size")
    if isinstance(pixel_size, (list, tuple)) and len(pixel_size) >= 2:
        px_x = abs(float(pixel_size[0]))
        px_y = abs(float(pixel_size[1]))
        values["pixel_size"] = f"{px_x:.0f}, {px_y:.0f}"

    size = metadata.get("size")
    if (
        isinstance(pixel_size, (list, tuple))
        and len(pixel_size) >= 2
        and isinstance(size, (list, tuple))
        and len(size) >= 2
    ):
        terrain_x_km = abs(float(pixel_size[0])) * float(size[0]) / 1000.0
        terrain_y_km = abs(float(pixel_size[1])) * float(size[1]) / 1000.0
        values["terrain_size"] = f"{terrain_x_km:.0f}, {terrain_y_km:.0f}"

    return values


def _prepare_dem_preview_data(dem_path: Path, size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, dict[str, Any] | None]:
    dem = np.load(dem_path, mmap_mode="r")
    if dem.ndim != 2:
        raise ValueError(f"Expected 2D DEM array, got shape {dem.shape}")

    min_elev, max_elev = _compute_min_max(dem)
    metadata = _load_dem_metadata(dem_path)

    y_len, x_len = dem.shape
    x_idx = _sample_indices(x_len, size)
    y_idx = _sample_indices(y_len, size)
    dem_preview = dem[np.ix_(y_idx, x_idx)]
    masked_dem = np.ma.masked_invalid(dem_preview)

    return masked_dem, x_idx, y_idx, min_elev, max_elev, metadata


def render_dem_preview(dem_path: Path, output_path: Path, size: int) -> None:
    masked_dem, x_idx, y_idx, min_elev, max_elev, metadata = _prepare_dem_preview_data(dem_path, size)

    # Avoid singular normalization when DEM is constant.
    vmax_for_plot = max_elev if max_elev > min_elev else min_elev + 1e-6

    dpi = 128
    fig = plt.figure(figsize=(size / dpi, size / dpi), dpi=dpi)
    ax = fig.add_subplot(111)

    image = ax.imshow(
        masked_dem,
        cmap="jet",
        origin="lower",
        vmin=min_elev,
        vmax=vmax_for_plot,
        interpolation="nearest",
    )

    ax.set_title(dem_path.parent.name)
    ax.set_xlabel("Pixel X")
    ax.set_ylabel("Pixel Y")

    preview_y_len, preview_x_len = masked_dem.shape
    x_tick_pos = _build_ticks(preview_x_len)
    y_tick_pos = _build_ticks(preview_y_len)
    ax.set_xticks(x_tick_pos)
    ax.set_yticks(y_tick_pos)
    ax.set_xticklabels(x_idx[x_tick_pos])
    ax.set_yticklabels(y_idx[y_tick_pos])

    cbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Elevation (m)")
    cbar.set_ticks([min_elev, max_elev])
    cbar.ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.0f"))

    legend_items = [
        Patch(color=plt.get_cmap("jet")(1.0), label=f"Max: {max_elev:.0f} m"),
        Patch(color=plt.get_cmap("jet")(0.0), label=f"Min: {min_elev:.0f} m"),
    ]
    ax.legend(handles=legend_items, title="Elevation", loc="upper right", framealpha=0.9)

    metadata_text = "\n".join(_build_metadata_lines(metadata))
    ax.text(
        0.01,
        0.01,
        metadata_text,
        transform=ax.transAxes,
        fontsize=9,
        family="monospace",
        va="bottom",
        ha="left",
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "black"},
    )

    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)


def render_dem_mosaic(dem_paths: list[Path], output_path: Path, tile_size: int, columns: int = 2) -> None:
    if tile_size <= 0:
        raise ValueError("--mosaic-tile-size must be a positive integer")
    if columns <= 0:
        raise ValueError("columns must be positive")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    count = len(dem_paths)
    rows = int(math.ceil(count / columns))
    dpi = 128
    fig_width = columns * tile_size / dpi
    fig_height = rows * tile_size / dpi

    fig, axes = plt.subplots(rows, columns, figsize=(fig_width, fig_height), dpi=dpi, squeeze=False)

    for idx, dem_path in enumerate(dem_paths):
        row = idx // columns
        col = idx % columns
        ax = axes[row][col]

        try:
            masked_dem, x_idx, y_idx, min_elev, max_elev, metadata = _prepare_dem_preview_data(dem_path, tile_size)
            vmax_for_plot = max_elev if max_elev > min_elev else min_elev + 1e-6
            ax.imshow(
                masked_dem,
                cmap="jet",
                origin="lower",
                vmin=min_elev,
                vmax=vmax_for_plot,
                interpolation="nearest",
            )

            ax.set_title(dem_path.parent.name, fontsize=10)
            ax.set_xlabel("Pixel X", fontsize=8)
            ax.set_ylabel("Pixel Y", fontsize=8)
            ax.tick_params(labelsize=7)

            preview_y_len, preview_x_len = masked_dem.shape
            x_tick_pos = _build_ticks(preview_x_len, max_ticks=5)
            y_tick_pos = _build_ticks(preview_y_len, max_ticks=5)
            ax.set_xticks(x_tick_pos)
            ax.set_yticks(y_tick_pos)
            ax.set_xticklabels(x_idx[x_tick_pos])
            ax.set_yticklabels(y_idx[y_tick_pos])

            ax.text(
                0.99,
                0.99,
                f"Max: {max_elev:.0f} m\nMin: {min_elev:.0f} m",
                transform=ax.transAxes,
                fontsize=8,
                va="top",
                ha="right",
                bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "black"},
            )

            metadata_text = "\n".join(_build_metadata_lines(metadata))
            ax.text(
                0.01,
                0.01,
                metadata_text,
                transform=ax.transAxes,
                fontsize=6,
                family="monospace",
                va="bottom",
                ha="left",
                bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "black"},
            )
        except Exception as exc:  # noqa: BLE001
            ax.set_axis_off()
            ax.text(
                0.5,
                0.5,
                f"{dem_path.parent.name}\nERROR\n{exc}",
                transform=ax.transAxes,
                fontsize=9,
                va="center",
                ha="center",
            )

    for idx in range(count, rows * columns):
        row = idx // columns
        col = idx % columns
        axes[row][col].set_axis_off()

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)


def write_preview_statistics(dem_paths: list[Path], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "# DEM Preview Statistics",
        "",
        "| Name | Center Lon / Lat (deg) | Pixel Size (m) | Terrain Size (km) | Elevation Min / Max (m) | Size (MB) |",
        "| --- | --- | --- | --- | --- | ---: |",
    ]

    for dem_path in dem_paths:
        metadata = _load_dem_metadata(dem_path)
        metadata_values = _extract_metadata_values(metadata)
        min_elev, max_elev = _compute_min_max(np.load(dem_path, mmap_mode="r"))
        size_mb = dem_path.stat().st_size / 1_000_000.0

        lines.append(
            "| "
            + " | ".join(
                [
                    dem_path.parent.name,
                    metadata_values["center"],
                    metadata_values["pixel_size"],
                    metadata_values["terrain_size"],
                    f"{min_elev:.0f}, {max_elev:.0f}",
                    f"{size_mb:.1f}",
                ]
            )
            + " |"
        )

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()

    if args.size <= 0:
        raise ValueError("--size must be a positive integer")
    if args.mosaic_tile_size <= 0:
        raise ValueError("--mosaic-tile-size must be a positive integer")

    if args.dem_path is not None:
        if not args.dem_path.exists() or not args.dem_path.is_file():
            raise ValueError(f"Invalid --dem-path file: {args.dem_path}")
        dem_paths = [args.dem_path]
    else:
        if not args.root.exists() or not args.root.is_dir():
            raise ValueError(f"Invalid --root directory: {args.root}")
        dem_paths = sorted(args.root.rglob(args.dem_name))

    if not dem_paths:
        print(f"No DEM files named '{args.dem_name}' found under {args.root}")
        return 0

    if args.mode == "preview_statistics":
        if args.statistics_output is not None:
            statistics_output = args.statistics_output
        elif args.dem_path is not None:
            statistics_output = args.dem_path.parent / "preview_statistics.md"
        else:
            statistics_output = args.root / "preview_statistics.md"

        write_preview_statistics(dem_paths, statistics_output)
        print(f"OK    Statistics -> {statistics_output}")
        print("\nSummary")
        print(f"  Found:      {len(dem_paths)}")
        print("  Statistics: generated")
        return 0

    if args.mode == "mosaic":
        if args.mosaic_output is not None:
            mosaic_output = args.mosaic_output
        elif args.dem_path is not None:
            mosaic_output = args.dem_path.parent / "preview_mosaic.png"
        else:
            mosaic_output = args.root / "preview_mosaic.png"

        render_dem_mosaic(dem_paths, mosaic_output, tile_size=args.mosaic_tile_size, columns=2)
        print(f"OK    Mosaic -> {mosaic_output}")
        print("\nSummary")
        print(f"  Found:     {len(dem_paths)}")
        print("  Mosaic:    generated")
        return 0

    processed = 0
    failed = 0

    for dem_path in dem_paths:
        output_path = dem_path.parent / args.output_name


        try:
            render_dem_preview(dem_path, output_path, args.size)
            processed += 1
            print(f"OK    {dem_path} -> {output_path}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"ERROR {dem_path}: {exc}")

    total = len(dem_paths)
    print("\nSummary")
    print(f"  Found:     {total}")
    print(f"  Processed: {processed}")
    print(f"  Failed:    {failed}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())