# GNSS Planner Python Visualization

This directory contains offline plotting utilities for the `gnss_global_path_planner` experiment. The scripts are not ROS 2 nodes and are not part of the main vehicle launch flow.

## Purpose

- Load a GeoJSON route graph.
- Interpolate long graph edges and add nearby connections.
- Read calibrated GNSS trajectory logs.
- Read A* output paths.
- Plot the map, GNSS trace, and planned path with Matplotlib.

## Main Script

```bash
python3 superviser_2.py
```

Dependencies include `geojson`, `matplotlib`, and `numpy`.

## Current Limitations

- `superviser_2.py` still contains old hard-coded input paths from an earlier workspace.
- Update the input paths before using it with current `runtime-data/` files.
- This tool is for offline visualization only; it does not publish ROS topics or affect vehicle runtime behavior.
