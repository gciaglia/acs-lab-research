# DataSimMRT - Mean Radiant Temperature & NDVI Analysis

This folder contains a Python script and generated plots for analyzing simulated Mean Radiant Temperature (MRT) and Normalized Difference Vegetation Index (NDVI) data over the Phoenix metropolitan area, centered on ASU's Gammage Auditorium (33.4146°N, 111.9396°W).

## Data Requirements

The GeoTIFF data files must be downloaded before running the script. They are not included in this repository due to their size.

**MRT Data** — Simulated Mean Radiant Temperature for June 27, 2012 at three time periods (07:00, 12:00, 17:00). Place the following files in this directory:
- `mrt_20120627_0700.tif`
- `mrt_20120627_1200.tif`
- `mrt_20120627_1700.tif`

Download from: https://portal.edirepository.org/nis/mapbrowse?packageid=knb-lter-cap.707.1

**NDVI Data** — NAIP-derived NDVI for the CAP LTER study area (2021). Place the following file in the `NDVIData/` subdirectory:
- `NDVIData/NAIP_NDVI_CAP2021-0000032768-0000098304.TIF`

Download from: https://portal.edirepository.org/nis/mapbrowse?packageid=knb-lter-cap.708.1

## Script

### `plot_mrt_analysis.py`

Loads large GeoTIFF files using memory-efficient chunked reading (via `tifffile` and `zarr`) and generates five analysis plots. The script can extract specific regions at full 1-meter resolution or subsample entire files for overview visualizations.

**Dependencies:**
```
pip install numpy matplotlib tifffile zarr imagecodecs
```

**Usage:**
```
python plot_mrt_analysis.py
```

## Output Plots

### `mrt_overview.png`
Three-panel overview of MRT across the full study area at 07:00, 12:00, and 17:00 on June 27, 2012. All panels share a single color scale (°C), making it easy to compare how radiant temperature increases from morning to afternoon across the region.

### `mrt_individual_scales.png`
Same three-panel MRT layout, but each time period uses its own color scale. This maximizes contrast within each panel, revealing spatial patterns (e.g., roads, buildings, vegetation) that may be washed out when using a shared scale.

### `mrt_gammage_closeup.png`
A 500m x 500m closeup centered on Gammage Auditorium at full 1-meter resolution for all three time periods. A cyan star marks the Gammage location. Uses a shared color scale across panels for direct comparison of how MRT changes throughout the day at the building level.

### `ndvi_mrt_comparison.png`
Side-by-side comparison of NDVI (2021) and MRT at 12:00 (2012) for the 500m x 500m Gammage area. The NDVI panel uses a red-yellow-green color map where green indicates dense vegetation. The MRT panel uses an inferno color map where brighter colors indicate higher temperatures. This highlights the spatial relationship between vegetation coverage and radiant temperature.

### `thermal_comfort_zones.png`
Combined NDVI and MRT analysis for the Gammage area at 12:00, presented in two panels:
- **Bivariate Map** — RGB composite encoding both MRT and NDVI simultaneously. Red indicates hot and bare areas, green indicates vegetated areas, and dark tones indicate cool and bare areas.
- **Thermal Comfort Zones** — Classified map using thresholds (NDVI > 0.3 for vegetated, MRT > 55°C for hot) to categorize areas into four zones:
  - Red: Hot + Bare (heat stress zones)
  - Orange: Hot + Vegetated (partial relief)
  - Light Blue: Cool + Bare (shaded structures)
  - Green: Cool + Vegetated (comfort zones)
