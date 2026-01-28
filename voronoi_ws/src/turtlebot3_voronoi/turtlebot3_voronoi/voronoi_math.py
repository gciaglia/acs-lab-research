"""
Bounded Voronoi partitioning and centroid computation.

Pure math module with no ROS dependencies. Uses the mirror boundary technique
to compute Voronoi cells clipped to a rectangular region, and Monte Carlo
sampling for centroid calculation (optionally density-weighted).
"""

import numpy as np
from scipy.spatial import Voronoi


def points_in_polygon(points, polygon):
    """Ray-casting point-in-polygon test.

    Args:
        points: (K, 2) array of query points.
        polygon: (M, 2) array of polygon vertices (ordered).

    Returns:
        (K,) boolean array — True for points inside the polygon.
    """
    n = len(polygon)
    inside = np.zeros(len(points), dtype=bool)

    for i, (px, py) in enumerate(points):
        j = n - 1
        for k in range(n):
            xi, yi = polygon[k]
            xj, yj = polygon[j]
            if ((yi > py) != (yj > py)) and \
               (px < (xj - xi) * (py - yi) / (yj - yi + 1e-10) + xi):
                inside[i] = not inside[i]
            j = k

    return inside


def get_bounded_voronoi_cells(positions, region_bounds):
    """Compute bounded Voronoi cells using the mirror boundary technique.

    Mirrors each robot position across all four region boundaries to create
    virtual generators, computes the full Voronoi diagram, then clips the
    resulting cells to the bounded region.

    Args:
        positions: (N, 2) array of robot positions.
        region_bounds: (min_x, max_x, min_y, max_y) tuple defining the
            rectangular region.

    Returns:
        List of N polygon arrays (each shape (M, 2)), or None on failure.
    """
    min_x, max_x, min_y, max_y = region_bounds

    if len(positions) < 2:
        return [np.array([
            [min_x, min_y], [max_x, min_y],
            [max_x, max_y], [min_x, max_y]
        ])]

    # Create extended point set with mirrors across all four boundaries
    ext_region = list(positions)
    for p in positions:
        ext_region.append([2 * min_x - p[0], p[1]])   # left mirror
        ext_region.append([2 * max_x - p[0], p[1]])   # right mirror
        ext_region.append([p[0], 2 * min_y - p[1]])    # bottom mirror
        ext_region.append([p[0], 2 * max_y - p[1]])    # top mirror

    ext_region = np.array(ext_region)

    try:
        vor = Voronoi(ext_region)
    except Exception:
        return None

    cells = []
    for i in range(len(positions)):
        region_idx = vor.point_region[i]
        region = vor.regions[region_idx]

        if -1 not in region and len(region) > 0:
            polygon = vor.vertices[region].copy()
            polygon[:, 0] = np.clip(polygon[:, 0], min_x, max_x)
            polygon[:, 1] = np.clip(polygon[:, 1], min_y, max_y)
            cells.append(polygon)
        else:
            # Fallback: assign the entire region
            cells.append(np.array([
                [min_x, min_y], [max_x, min_y],
                [max_x, max_y], [min_x, max_y]
            ]))

    return cells


def compute_centroid(cell_vertices, density_fn=None, n_samples=500):
    """Compute the centroid of a polygon via Monte Carlo sampling.

    If a density function is provided, computes the density-weighted centroid
    (biased toward higher-density areas). Otherwise returns the geometric
    centroid.

    Args:
        cell_vertices: (M, 2) array of polygon vertices.
        density_fn: Optional callable mapping (K, 2) points to (K,) weights.
            Pass None for uniform (geometric) centroid.
        n_samples: Number of random samples for Monte Carlo integration.

    Returns:
        (2,) array — the centroid position.
    """
    if len(cell_vertices) < 3:
        return cell_vertices.mean(axis=0)

    min_xy = cell_vertices.min(axis=0)
    max_xy = cell_vertices.max(axis=0)

    samples = np.random.uniform(min_xy, max_xy, size=(n_samples, 2))
    inside = points_in_polygon(samples, cell_vertices)
    samples_inside = samples[inside]

    if len(samples_inside) == 0:
        return cell_vertices.mean(axis=0)

    if density_fn is not None:
        weights = density_fn(samples_inside)
        total_weight = weights.sum()
        if total_weight > 0:
            weighted_x = (samples_inside[:, 0] * weights).sum() / total_weight
            weighted_y = (samples_inside[:, 1] * weights).sum() / total_weight
            return np.array([weighted_x, weighted_y])

    return samples_inside.mean(axis=0)
