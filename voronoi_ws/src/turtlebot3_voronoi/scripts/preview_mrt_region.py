#!/usr/bin/env python3
"""
Utility script to preview MRT data regions before running the simulation.

Helps identify good extraction coordinates by displaying the MRT data
around specified pixel coordinates. Uses interactive matplotlib for
exploration.

Usage:
    python3 preview_mrt_region.py /path/to/mrt_file.tif
    python3 preview_mrt_region.py /path/to/mrt_file.tif --center 46160 29736 --size 50

Defaults to Gammage Center coordinates (46160, 29736) with 50m region.
"""

import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


def load_tif_region(filepath, center_x, center_y, size, context_size=200):
    """Load a region from a TIF file with surrounding context."""
    try:
        import tifffile
        import zarr
    except ImportError:
        print("Error: tifffile and zarr packages required.")
        print("Install with: pip install tifffile zarr")
        sys.exit(1)

    store = tifffile.imread(filepath, aszarr=True)
    z = zarr.open(store, mode='r')

    full_h, full_w = z.shape
    print(f"Full TIF dimensions: {full_w} x {full_h} pixels")

    # Extract larger context region for exploration
    half_ctx = context_size // 2
    ctx_y_start = max(0, center_y - half_ctx)
    ctx_y_end = min(full_h, center_y + half_ctx)
    ctx_x_start = max(0, center_x - half_ctx)
    ctx_x_end = min(full_w, center_x + half_ctx)

    context_data = z[ctx_y_start:ctx_y_end, ctx_x_start:ctx_x_end].astype(float)
    store.close()

    return context_data, (ctx_x_start, ctx_y_start, ctx_x_end, ctx_y_end)


def preview_region(filepath, center_x, center_y, extract_size):
    """Display interactive preview of MRT region."""

    print(f"\nLoading MRT data from: {filepath}")
    print(f"Center: ({center_x}, {center_y})")
    print(f"Extract size: {extract_size}m x {extract_size}m")

    # Load with larger context for exploration
    context_size = max(300, extract_size * 4)
    data, (ctx_x0, ctx_y0, ctx_x1, ctx_y1) = load_tif_region(
        filepath, center_x, center_y, extract_size, context_size
    )

    # Calculate the extraction region within context
    local_cx = center_x - ctx_x0
    local_cy = center_y - ctx_y0
    half_size = extract_size // 2

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Left: Full context with extraction box
    ax1 = axes[0]
    im1 = ax1.imshow(data, cmap='hot', origin='upper',
                     extent=[ctx_x0, ctx_x1, ctx_y1, ctx_y0])
    ax1.set_title(f'Context View ({context_size}m around center)')
    ax1.set_xlabel('X pixel')
    ax1.set_ylabel('Y pixel')
    plt.colorbar(im1, ax=ax1, label='MRT Value')

    # Draw extraction rectangle
    rect = plt.Rectangle(
        (center_x - half_size, center_y - half_size),
        extract_size, extract_size,
        fill=False, edgecolor='cyan', linewidth=2, linestyle='--'
    )
    ax1.add_patch(rect)
    ax1.plot(center_x, center_y, 'c+', markersize=15, markeredgewidth=2)

    # Right: Extracted region (what simulation will see)
    ax2 = axes[1]
    y0 = max(0, local_cy - half_size)
    y1 = min(data.shape[0], local_cy + half_size)
    x0 = max(0, local_cx - half_size)
    x1 = min(data.shape[1], local_cx + half_size)

    extracted = data[y0:y1, x0:x1]

    # Normalize like density_field.py does
    valid = extracted > 0
    if valid.any():
        gmin = extracted[valid].min()
        gmax = extracted[valid].max()
        if gmax > gmin:
            normalized = np.where(
                valid,
                (extracted - gmin) / (gmax - gmin) * 0.9 + 0.1,
                0.1
            )
        else:
            normalized = np.where(valid, 1.0, 0.1)
    else:
        normalized = np.full_like(extracted, 0.1)

    im2 = ax2.imshow(normalized, cmap='hot', origin='upper',
                     extent=[-3, 3, 3, -3])  # Simulation coordinates
    ax2.set_title(f'Extracted Region ({extract_size}m) - Simulation View')
    ax2.set_xlabel('Sim X (meters)')
    ax2.set_ylabel('Sim Y (meters)')
    plt.colorbar(im2, ax=ax2, label='Normalized Density')

    # Add grid to show simulation scale
    ax2.set_xticks([-3, -2, -1, 0, 1, 2, 3])
    ax2.set_yticks([-3, -2, -1, 0, 1, 2, 3])
    ax2.grid(True, alpha=0.3)

    # Print statistics
    print(f"\n--- Extracted Region Statistics ---")
    print(f"Raw MRT range: {extracted.min():.1f} - {extracted.max():.1f}")
    print(f"Normalized range: {normalized.min():.2f} - {normalized.max():.2f}")
    print(f"Hotspot (max) location in sim coords: ", end="")

    # Find hotspot in normalized data
    hot_idx = np.unravel_index(normalized.argmax(), normalized.shape)
    hot_sim_x = -3 + (hot_idx[1] / normalized.shape[1]) * 6
    hot_sim_y = 3 - (hot_idx[0] / normalized.shape[0]) * 6
    print(f"({hot_sim_x:.2f}, {hot_sim_y:.2f})")

    ax2.plot(hot_sim_x, hot_sim_y, 'g*', markersize=15, label='Hotspot')
    ax2.legend()

    print(f"\n--- Launch Command ---")
    print(f"ros2 launch turtlebot3_voronoi bringup.launch.py \\")
    print(f"  density_type:=mrt_file \\")
    print(f"  mrt_file:={filepath} \\")
    print(f"  mrt_center_x:={center_x} \\")
    print(f"  mrt_center_y:={center_y} \\")
    print(f"  mrt_extract_size:={extract_size}")

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Preview MRT data regions for Voronoi simulation'
    )
    parser.add_argument('mrt_file', help='Path to MRT .tif file')
    parser.add_argument('--center', nargs=2, type=int, default=[46160, 29736],
                        metavar=('X', 'Y'),
                        help='Center pixel coordinates (default: Gammage 46160 29736)')
    parser.add_argument('--size', type=int, default=50,
                        help='Extraction size in meters (default: 50)')

    args = parser.parse_args()

    preview_region(
        args.mrt_file,
        args.center[0],
        args.center[1],
        args.size
    )


if __name__ == '__main__':
    main()
