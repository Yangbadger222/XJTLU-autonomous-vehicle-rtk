#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${FYP_RUNTIME_ROOT:-$HOME/XJTLU-autonomous-vehicle/runtime-data}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$HOME/XJTLU-autonomous-vehicle}"
LEGACY_ROOT="${LEGACY_ROOT:-$HOME/2025_FYP}"

mkdir -p "$RUNTIME_ROOT/config" "$RUNTIME_ROOT/gnss" "$RUNTIME_ROOT/planning" "$RUNTIME_ROOT/maps"
mkdir -p "$RUNTIME_ROOT/frc"/{models,routes,trials,events,anchor_log}

install -m 644 "$LEGACY_ROOT/all_kind_output_file/Other_File/manual_config/log_switch.yaml" "$RUNTIME_ROOT/config/log_switch.yaml"
install -m 644 "$LEGACY_ROOT/car_ws/src/Sensor_Driver_layer/GNSS/gnss_calibration/gnss_calibration/gnss_offset.txt" "$RUNTIME_ROOT/gnss/gnss_offset.txt"
install -m 644 "$LEGACY_ROOT/car_ws/src/Sensor_Driver_layer/GNSS/gnss_calibration/gnss_calibration/startid.txt" "$RUNTIME_ROOT/gnss/startid.txt"
install -m 644 "$WORKSPACE_ROOT/src/planning/global2local_tf/global2local_tf/angle_offset.txt" "$RUNTIME_ROOT/planning/angle_offset.txt"
install -m 644 "$WORKSPACE_ROOT"/src/planning/gnss_global_path_planner/map/*.geojson "$RUNTIME_ROOT/maps/"

printf 'Runtime data initialized under %s
' "$RUNTIME_ROOT"
find "$RUNTIME_ROOT" -maxdepth 2 -type f | sort
