"""
Pluggable density fields for weighted Voronoi centroid computation.

Pure Python module (no ROS imports). Provides a callable interface that
voronoi_math.compute_centroid() uses to weight samples. Swap density
strategies by changing the config — no controller code changes needed.

Extensibility:
    - ShadowDensity: future subclass for shadow-sensing robots
    - StigmergyDensity: future subclass for pheromone-like fields
    - FokkerPlanckDensity: future subclass for heat diffusion dynamics
"""

from abc import ABC, abstractmethod
import numpy as np


class DensityField(ABC):
    """Abstract base for density fields.

    Subclasses must implement __call__ to return per-point density weights.
    These weights bias Voronoi centroid computation toward higher-density
    areas.
    """

    @abstractmethod
    def __call__(self, points):
        """Evaluate density at given points.

        Args:
            points: (K, 2) array of (x, y) positions.

        Returns:
            (K,) array of non-negative density weights.
        """
        raise NotImplementedError


class UniformDensity(DensityField):
    """Returns 1.0 everywhere. Default for geometric Voronoi centroids."""

    def __call__(self, points):
        return np.ones(len(points))


class GridDensity(DensityField):
    """Density from a 2D grid mapped to arena coordinates.

    Performs bilinear-style lookup by mapping arena (x, y) positions to grid
    indices. Useful for MRT data, NDVI, or any raster-based density field.

    Args:
        grid: 2D numpy array of density values.
        region_bounds: (min_x, max_x, min_y, max_y) arena coordinates.
    """

    def __init__(self, grid, region_bounds):
        self._grid = grid
        self._min_x, self._max_x, self._min_y, self._max_y = region_bounds
        self._arena_w = self._max_x - self._min_x
        self._arena_h = self._max_y - self._min_y
        self._grid_h, self._grid_w = grid.shape

    @property
    def grid(self):
        """Return the underlying density grid array."""
        return self._grid

    @property
    def bounds(self):
        """Return (min_x, max_x, min_y, max_y) arena bounds."""
        return (self._min_x, self._max_x, self._min_y, self._max_y)

    def __call__(self, points):
        weights = np.ones(len(points))

        for i, (x, y) in enumerate(points):
            norm_x = (x - self._min_x) / self._arena_w
            norm_y = (y - self._min_y) / self._arena_h

            # Convert to grid indices (matches matplotlib extent=[-3,3,3,-3] with origin='upper')
            gx = int(np.clip(norm_x * (self._grid_w - 1), 0, self._grid_w - 1))
            gy = int(np.clip(norm_y * (self._grid_h - 1), 0, self._grid_h - 1))

            weights[i] = self._grid[gy, gx]

        return weights

    @classmethod
    def from_npy(cls, filepath, region_bounds):
        """Load density grid from a .npy file.

        Args:
            filepath: Path to numpy array file.
            region_bounds: (min_x, max_x, min_y, max_y) arena coordinates.

        Returns:
            GridDensity instance.
        """
        grid = np.load(filepath)
        # Normalize to [0.1, 1.0] range (avoid zero weights)
        valid = grid > 0
        if valid.any():
            gmin = grid[valid].min()
            gmax = grid[valid].max()
            if gmax > gmin:
                grid = np.where(valid, (grid - gmin) / (gmax - gmin) * 0.9 + 0.1, 0.1)
            else:
                grid = np.where(valid, 1.0, 0.1)
        else:
            grid = np.full_like(grid, 0.1, dtype=float)
        return cls(grid, region_bounds)

    @classmethod
    def from_geotiff(cls, filepath, region_bounds, target_size=100,
                     center_x=46160, center_y=29736, extract_size=50):
        """Load density grid from a GeoTIFF file with region extraction.

        Uses tifffile + zarr for memory-efficient reading. Extracts a region
        centered on the specified pixel coordinates (default: Gammage Center).

        Args:
            filepath: Path to .tif file.
            region_bounds: (min_x, max_x, min_y, max_y) arena coordinates.
            target_size: Approximate grid dimension after subsampling.
            center_x: X pixel coordinate for extraction center (default: Gammage).
            center_y: Y pixel coordinate for extraction center (default: Gammage).
            extract_size: Size in pixels to extract (e.g., 50 for 50x50m region).
                         Since MRT data is 1 pixel = 1 meter, this equals meters.

        Returns:
            GridDensity instance.
        """
        import tifffile
        import zarr

        store = tifffile.imread(filepath, aszarr=True)
        z = zarr.open(store, mode='r')

        # Calculate extraction bounds (centered on specified coordinates)
        half_size = extract_size // 2
        y_start = max(0, center_y - half_size)
        y_end = min(z.shape[0], center_y + half_size)
        x_start = max(0, center_x - half_size)
        x_end = min(z.shape[1], center_x + half_size)

        # Extract the region of interest
        grid = z[y_start:y_end, x_start:x_end].astype(float)
        store.close()

        # Subsample if larger than target_size
        if grid.shape[0] > target_size or grid.shape[1] > target_size:
            step = max(1, min(grid.shape[0], grid.shape[1]) // target_size)
            grid = grid[::step, ::step]

        # Normalize to [0.1, 1.0] range (avoid zero weights)
        valid = grid > 0
        if valid.any():
            gmin = grid[valid].min()
            gmax = grid[valid].max()
            if gmax > gmin:
                grid = np.where(valid, (grid - gmin) / (gmax - gmin) * 0.9 + 0.1, 0.1)
            else:
                grid = np.where(valid, 1.0, 0.1)
        else:
            grid = np.full_like(grid, 0.1)

        return cls(grid, region_bounds)
