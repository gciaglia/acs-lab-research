#!/usr/bin/env python3
"""
MRT and NDVI Data Analysis and Visualization Script

This script demonstrates how to load and visualize large GeoTIFF files
containing Mean Radiant Temperature (MRT) and NDVI data for the Phoenix
metropolitan area, specifically focused on the ASU Gammage Auditorium area.

Data Sources:
- MRT Data: Simulated Mean Radiant Temperature from urban heat modeling
- NDVI Data: https://portal.edirepository.org/nis/mapbrowse?packageid=knb-lter-cap.708.1

Requirements:
    pip install numpy matplotlib tifffile zarr imagecodecs

"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for saving plots

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import os

# Optional imports for GeoTIFF support
try:
    import tifffile
    import zarr
    GEOTIFF_SUPPORT = True
except ImportError:
    GEOTIFF_SUPPORT = False
    print("Warning: tifffile/zarr not installed. Install with: pip install tifffile zarr imagecodecs")


# =============================================================================
# CONFIGURATION
# =============================================================================

# File paths (adjust these to your data locations)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MRT_FILES = {
    '0700': os.path.join(BASE_DIR, 'mrt_20120627_0700.tif'),
    '1200': os.path.join(BASE_DIR, 'mrt_20120627_1200.tif'),
    '1700': os.path.join(BASE_DIR, 'mrt_20120627_1700.tif'),
}
NDVI_FILE = os.path.join(BASE_DIR, 'NDVIData',
                         'NAIP_NDVI_CAP2021-0000032768-0000098304.TIF')

# Gammage Auditorium center coordinates (in pixels for each dataset)
# These were calculated from UTM coordinates of Gammage (33.4146°N, -111.9396°W)
MRT_GAMMAGE_CENTER = (46160, 29736)  # (x, y) in MRT pixel coordinates
NDVI_GAMMAGE_CENTER = (1245, 21008)  # (x, y) in NDVI pixel coordinates

# Output directory for plots
OUTPUT_DIR = BASE_DIR


# =============================================================================
# DATA LOADING FUNCTIONS
# =============================================================================

def load_mrt_region(tif_path, center_x, center_y, window_size=500):
    """
    Load a region from a large MRT GeoTIFF file.

    Uses zarr for memory-efficient chunked reading, which allows loading
    specific regions from multi-GB files without loading the entire file.

    Args:
        tif_path: Path to the GeoTIFF file
        center_x, center_y: Center pixel coordinates
        window_size: Size of the square region to extract (in pixels/meters)

    Returns:
        numpy array of MRT values, or None if loading fails
    """
    if not GEOTIFF_SUPPORT:
        print("GeoTIFF support not available")
        return None

    if not os.path.exists(tif_path):
        print(f"File not found: {tif_path}")
        return None

    half = window_size // 2

    store = tifffile.imread(tif_path, aszarr=True)
    z = zarr.open(store, mode='r')

    # Extract region (zarr only loads the chunks needed)
    y_start = center_y - half
    y_end = center_y + half
    x_start = center_x - half
    x_end = center_x + half

    data = z[y_start:y_end, x_start:x_end]
    store.close()

    return data


def load_mrt_subsampled(tif_path, step=30):
    """
    Load an entire MRT file with subsampling for overview visualization.

    Args:
        tif_path: Path to the GeoTIFF file
        step: Subsampling step (e.g., 30 means every 30th pixel)

    Returns:
        Subsampled numpy array
    """
    if not GEOTIFF_SUPPORT:
        return None

    if not os.path.exists(tif_path):
        print(f"File not found: {tif_path}")
        return None

    store = tifffile.imread(tif_path, aszarr=True)
    z = zarr.open(store, mode='r')

    data = z[::step, ::step]
    store.close()

    return data


def load_ndvi_region(tif_path, center_x, center_y, window_size=500):
    """
    Load a region from the NDVI GeoTIFF file.

    Args:
        tif_path: Path to the NDVI GeoTIFF file
        center_x, center_y: Center pixel coordinates
        window_size: Size of the square region to extract

    Returns:
        numpy array of NDVI values
    """
    if not GEOTIFF_SUPPORT:
        return None

    if not os.path.exists(tif_path):
        print(f"File not found: {tif_path}")
        return None

    half = window_size // 2

    store = tifffile.imread(tif_path, aszarr=True)
    z = zarr.open(store, mode='r')

    y_start = center_y - half
    y_end = center_y + half
    x_start = center_x - half
    x_end = center_x + half

    data = z[y_start:y_end, x_start:x_end]
    store.close()

    return data


# =============================================================================
# PLOTTING FUNCTIONS
# =============================================================================

def plot_mrt_overview(step=30, output_file='mrt_overview.png'):
    """
    Plot overview of all three MRT time periods with shared color scale.

    Creates a 3-panel plot showing MRT at 07:00, 12:00, and 17:00 with
    a shared horizontal colorbar for comparison.
    """
    print("Creating MRT overview plot...")

    datasets = []
    global_min = float('inf')
    global_max = float('-inf')

    for time_label, tif_path in MRT_FILES.items():
        print(f"  Loading {time_label}...")
        data = load_mrt_subsampled(tif_path, step=step)
        if data is None:
            continue

        valid = data[data > 0]
        if len(valid) > 0:
            global_min = min(global_min, valid.min())
            global_max = max(global_max, valid.max())
        datasets.append((data, time_label))

    if not datasets:
        print("No data loaded")
        return

    # Create figure
    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    extent = [0, datasets[0][0].shape[1], datasets[0][0].shape[0], 0]

    for ax, (data, label) in zip(axes, datasets):
        masked_data = np.ma.masked_where(data == 0, data)
        im = ax.imshow(masked_data, cmap='inferno', origin='upper',
                       vmin=global_min, vmax=global_max)
        ax.set_title(f'{label[:2]}:{label[2:]}', fontsize=14)
        ax.set_xlabel('pixels')
        ax.set_ylabel('pixels')

    # Shared colorbar
    cbar_ax = fig.add_axes([0.15, 0.08, 0.7, 0.03])
    cbar = fig.colorbar(im, cax=cbar_ax, orientation='horizontal')
    cbar.set_label('Mean Radiant Temperature (°C)', fontsize=12)

    plt.suptitle('Mean Radiant Temperature - June 27, 2012', fontsize=14)
    plt.subplots_adjust(bottom=0.2, top=0.9, wspace=0.25)

    output_path = os.path.join(OUTPUT_DIR, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_path}")
    plt.close()


def plot_mrt_individual_scales(step=30, output_file='mrt_individual_scales.png'):
    """
    Plot MRT with individual color scales per time period.

    This view maximizes contrast within each time period, useful for
    seeing spatial patterns that might be washed out with a shared scale.
    """
    print("Creating MRT plot with individual scales...")

    datasets = []
    for time_label, tif_path in MRT_FILES.items():
        print(f"  Loading {time_label}...")
        data = load_mrt_subsampled(tif_path, step=step)
        if data is not None:
            datasets.append((data, time_label))

    if not datasets:
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))

    for ax, (data, label) in zip(axes, datasets):
        masked_data = np.ma.masked_where(data == 0, data)
        valid = data[data > 0]

        im = ax.imshow(masked_data, cmap='inferno', origin='upper',
                       vmin=valid.min(), vmax=valid.max())
        ax.set_title(f'{label[:2]}:{label[2:]}', fontsize=14)
        ax.set_xlabel('pixels')
        ax.set_ylabel('pixels')

        cbar = fig.colorbar(im, ax=ax, orientation='horizontal', pad=0.12, shrink=0.9)
        cbar.set_label('°C', fontsize=10)

    plt.suptitle('Mean Radiant Temperature - Individual Scales', fontsize=14)
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)

    output_path = os.path.join(OUTPUT_DIR, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_path}")
    plt.close()


def plot_gammage_mrt_closeup(window_size=500, output_file='mrt_gammage_closeup.png'):
    """
    Plot MRT closeup of Gammage Auditorium area with shared scale.

    Shows a 500m x 500m area at full 1-meter resolution centered on
    ASU's Gammage Auditorium.
    """
    print("Creating Gammage closeup plot...")

    center_x, center_y = MRT_GAMMAGE_CENTER
    half = window_size // 2
    extent = [0, window_size, window_size, 0]

    datasets = []
    global_min = float('inf')
    global_max = float('-inf')

    for time_label, tif_path in MRT_FILES.items():
        print(f"  Loading {time_label}...")
        data = load_mrt_region(tif_path, center_x, center_y, window_size)
        if data is None:
            continue

        valid = data[data > 0]
        if len(valid) > 0:
            global_min = min(global_min, valid.min())
            global_max = max(global_max, valid.max())
        datasets.append((data, time_label))

    if not datasets:
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 6))

    for ax, (data, label) in zip(axes, datasets):
        masked_data = np.ma.masked_where(data == 0, data)
        im = ax.imshow(masked_data, cmap='inferno', origin='upper',
                       vmin=global_min, vmax=global_max, extent=extent)
        ax.set_title(f'{label[:2]}:{label[2:]}', fontsize=14)
        ax.set_xlabel('meters')
        ax.set_ylabel('meters')
        ax.plot(half, half, 'c*', markersize=15, markeredgecolor='white', markeredgewidth=1)

    cbar_ax = fig.add_axes([0.15, 0.08, 0.7, 0.03])
    cbar = fig.colorbar(im, cax=cbar_ax, orientation='horizontal')
    cbar.set_label('Mean Radiant Temperature (°C)', fontsize=12)

    plt.suptitle('Gammage Auditorium Area - 500m × 500m', fontsize=14)
    plt.subplots_adjust(bottom=0.2, top=0.9, wspace=0.25)

    output_path = os.path.join(OUTPUT_DIR, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_path}")
    plt.close()


def plot_ndvi_mrt_comparison(window_size=500, output_file='ndvi_mrt_comparison.png'):
    """
    Plot NDVI and MRT side-by-side comparison for Gammage area.

    Shows the relationship between vegetation (NDVI) and temperature (MRT).
    Note: NDVI is from 2021, MRT from 2012.
    """
    print("Creating NDVI vs MRT comparison...")

    if not os.path.exists(NDVI_FILE):
        print(f"NDVI file not found: {NDVI_FILE}")
        return

    half = window_size // 2
    extent = [0, window_size, window_size, 0]

    # Load MRT 12:00 data
    mrt_x, mrt_y = MRT_GAMMAGE_CENTER
    mrt_data = load_mrt_region(MRT_FILES['1200'], mrt_x, mrt_y, window_size)

    # Load NDVI data
    ndvi_x, ndvi_y = NDVI_GAMMAGE_CENTER
    ndvi_data = load_ndvi_region(NDVI_FILE, ndvi_x, ndvi_y, window_size)

    if mrt_data is None or ndvi_data is None:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # NDVI plot
    ax1 = axes[0]
    ndvi_im = ax1.imshow(ndvi_data, cmap='RdYlGn', origin='upper',
                         vmin=-0.2, vmax=0.8, extent=extent)
    ax1.set_title('NDVI (2021)\nVegetation Index', fontsize=14)
    ax1.set_xlabel('meters')
    ax1.set_ylabel('meters')
    ax1.plot(half, half, 'c*', markersize=15, markeredgecolor='black', markeredgewidth=1)
    cbar1 = fig.colorbar(ndvi_im, ax=ax1, orientation='horizontal', pad=0.1, shrink=0.8)
    cbar1.set_label('NDVI')

    # MRT plot
    ax2 = axes[1]
    mrt_masked = np.ma.masked_where(mrt_data == 0, mrt_data)
    mrt_im = ax2.imshow(mrt_masked, cmap='inferno', origin='upper', extent=extent)
    ax2.set_title('MRT 12:00 (2012)\nMean Radiant Temperature', fontsize=14)
    ax2.set_xlabel('meters')
    ax2.set_ylabel('meters')
    ax2.plot(half, half, 'c*', markersize=15, markeredgecolor='white', markeredgewidth=1)
    cbar2 = fig.colorbar(mrt_im, ax=ax2, orientation='horizontal', pad=0.1, shrink=0.8)
    cbar2.set_label('Temperature (°C)')

    plt.suptitle('Gammage Auditorium Area - NDVI vs MRT Comparison\n500m × 500m',
                 fontsize=14, y=1.02)
    plt.tight_layout()

    output_path = os.path.join(OUTPUT_DIR, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_path}")
    plt.close()


def plot_thermal_comfort_zones(window_size=500, output_file='thermal_comfort_zones.png'):
    """
    Create bivariate analysis combining NDVI and MRT data.

    Classifies areas into thermal comfort zones:
    - Hot + Bare (red): Heat stress zones
    - Hot + Vegetated (orange): Partial relief
    - Cool + Bare (blue): Shaded structures
    - Cool + Vegetated (green): Comfort zones
    """
    print("Creating thermal comfort zones analysis...")

    if not os.path.exists(NDVI_FILE):
        print(f"NDVI file not found: {NDVI_FILE}")
        return

    half = window_size // 2
    extent = [0, window_size, window_size, 0]

    # Load data
    mrt_x, mrt_y = MRT_GAMMAGE_CENTER
    mrt_data = load_mrt_region(MRT_FILES['1200'], mrt_x, mrt_y, window_size)

    ndvi_x, ndvi_y = NDVI_GAMMAGE_CENTER
    ndvi_data = load_ndvi_region(NDVI_FILE, ndvi_x, ndvi_y, window_size)

    if mrt_data is None or ndvi_data is None:
        return

    mrt_data = mrt_data.astype(float)
    mrt_valid = mrt_data > 0

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # 1. Bivariate map
    ax1 = axes[0]
    mrt_norm = np.zeros_like(mrt_data)
    mrt_norm[mrt_valid] = (mrt_data[mrt_valid] - mrt_data[mrt_valid].min()) / \
                          (mrt_data[mrt_valid].max() - mrt_data[mrt_valid].min())
    ndvi_norm = (ndvi_data - (-0.2)) / (0.8 - (-0.2))
    ndvi_norm = np.clip(ndvi_norm, 0, 1)

    rgb = np.zeros((window_size, window_size, 3))
    rgb[:,:,0] = mrt_norm * (1 - ndvi_norm)  # Red: hot AND low vegetation
    rgb[:,:,1] = ndvi_norm * (1 - mrt_norm * 0.5)  # Green: vegetation
    rgb[:,:,2] = (1 - mrt_norm) * (1 - ndvi_norm)  # Blue: cool AND low vegetation
    rgb = np.clip(rgb, 0, 1)
    rgb[~mrt_valid] = 0

    ax1.imshow(rgb, origin='upper', extent=extent)
    ax1.set_title('Bivariate Map\nRed=Hot+Bare | Green=Vegetated | Dark=Cool+Bare', fontsize=12)
    ax1.set_xlabel('meters')
    ax1.set_ylabel('meters')
    ax1.plot(half, half, 'w*', markersize=15, markeredgecolor='black', markeredgewidth=1)

    # 2. Classified thermal comfort zones
    ax2 = axes[1]

    # Classification thresholds
    high_veg = ndvi_data > 0.3  # NDVI > 0.3 = vegetated
    high_mrt = mrt_data > 55    # MRT > 55°C = hot

    classified = np.zeros_like(mrt_data)
    classified[(~high_veg) & (high_mrt)] = 1   # Hot + bare (heat stress)
    classified[(high_veg) & (high_mrt)] = 2    # Hot + vegetated
    classified[(~high_veg) & (~high_mrt)] = 3  # Cool + bare (shaded)
    classified[(high_veg) & (~high_mrt)] = 4   # Cool + vegetated (comfort)
    classified[~mrt_valid] = 0

    cmap = plt.cm.colors.ListedColormap(['black', 'red', 'orange', 'lightblue', 'green'])
    bounds = [0, 0.5, 1.5, 2.5, 3.5, 4.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    ax2.imshow(classified, cmap=cmap, norm=norm, origin='upper', extent=extent)
    ax2.set_title('Thermal Comfort Zones', fontsize=12)
    ax2.set_xlabel('meters')
    ax2.set_ylabel('meters')
    ax2.plot(half, half, 'w*', markersize=15, markeredgecolor='black', markeredgewidth=1)

    # Legend
    legend_elements = [
        Patch(facecolor='red', label='Hot + Bare (Heat stress)'),
        Patch(facecolor='orange', label='Hot + Vegetated'),
        Patch(facecolor='lightblue', label='Cool + Bare (Shaded)'),
        Patch(facecolor='green', label='Cool + Vegetated (Comfort)')
    ]
    ax2.legend(handles=legend_elements, loc='lower right', fontsize=9)

    plt.suptitle('NDVI + MRT Combined Analysis - Gammage Area (12:00)', fontsize=14)
    plt.tight_layout()

    output_path = os.path.join(OUTPUT_DIR, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_path}")
    plt.close()


def print_data_summary():
    """Print summary information about the available data files."""
    print("\n" + "="*60)
    print("DATA SUMMARY")
    print("="*60)

    print("\nMRT Files:")
    for label, path in MRT_FILES.items():
        exists = "✓" if os.path.exists(path) else "✗"
        print(f"  [{exists}] {label}: {path}")

    print(f"\nNDVI File:")
    exists = "✓" if os.path.exists(NDVI_FILE) else "✗"
    print(f"  [{exists}] {NDVI_FILE}")

    print(f"\nGammage Center Coordinates:")
    print(f"  MRT pixel coords: {MRT_GAMMAGE_CENTER}")
    print(f"  NDVI pixel coords: {NDVI_GAMMAGE_CENTER}")
    print(f"  Geographic: 33.4146°N, -111.9396°W")

    print(f"\nOutput Directory: {OUTPUT_DIR}")
    print("="*60 + "\n")


# =============================================================================
# MAIN EXECUTION
# =============================================================================

def main():
    """Generate all analysis plots."""

    print_data_summary()

    if not GEOTIFF_SUPPORT:
        print("ERROR: Required packages not installed.")
        print("Run: pip install tifffile zarr imagecodecs")
        return

    print("Generating all plots...")
    print("-" * 40)

    # Generate all plots
    plot_mrt_overview(step=30)
    plot_mrt_individual_scales(step=30)
    plot_gammage_mrt_closeup(window_size=500)
    plot_ndvi_mrt_comparison(window_size=500)
    plot_thermal_comfort_zones(window_size=500)

    print("-" * 40)
    print("All plots generated successfully!")
    print(f"Output directory: {OUTPUT_DIR}")


if __name__ == '__main__':
    main()
