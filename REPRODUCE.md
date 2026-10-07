# Reproduce the isolated research delivery

Use the published research branch and the source commit recorded in audit/remote_humble/clean-final-build.json. Do not start again from main or replace the production workspace.

```bash
git clone --branch codex/superlio-ego-active-road https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk.git research-checkout
cd research-checkout
vcs import src < dependencies.research.repos
bash scripts/apply_super_lio_patch.sh
bash scripts/apply_ego_vehicle_patch.sh
bash scripts/build_active_road_research.sh /absolute/isolated/research-ws
source /opt/ros/humble/setup.bash
source /absolute/isolated/research-ws/install/setup.bash
```

The build clears inherited overlays, compiles/installs a task-local Livox SDK and explicitly selects the 14-package source allowlist with one worker. There must be a real `Summary: 14 packages finished`; `0 packages finished` is not compilation evidence. Neither whole-repository colcon discovery nor a production install is a dependency shortcut.

Executed host: non-Jetson x86_64 Ubuntu 22.04.5 / ROS2 Humble. Install the declared ROS dependencies before running an asset-backed entry; active_road_mapping declares python3-rasterio and python3-pyproj. The experiment used a task-only Python venv with rasterio 1.3.11, pyproj 3.6.1 and numpy 1.26.4 with system ROS packages. Invoke geospatial Python probes with that venv interpreter; no global pip installation is needed. The system-shebang launch requires its declared runtime dependencies when a real prior manifest is supplied; the no-manifest default audit is not proof of asset-backed target deployment. Builds, source and current explicit session logs are under /home/badger/codex-research/superlio-ego-20261007. Earlier unchanged serial binaries also wrote native logs to their default /home/badger/XJTLU-autonomous-vehicle/runtime-data/logs/twist_log; that fallback root is not a Git checkout and contains only runtime-data. Those historical artifacts are preserved. Set the FYP and ROS logging roots below for isolated reproductions. Lightweight summaries are checked in; bags, GeoTIFF fixtures and binaries are not.

```bash
PYTHONPATH=src/research_runtime:src/active_road_mapping:src/super_lio_vehicle_adapter python3 -m pytest -q src/research_runtime/test src/super_lio_vehicle_adapter/test
python3 scripts/generate_research_parameter_lock.py --check
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91
export FYP_RUNTIME_ROOT=/absolute/isolated/logs/runtime-data
export FYP_LOG_SESSION_DIR=/absolute/isolated/logs/native-serial
export ROS_LOG_DIR=/absolute/isolated/logs/ros
python3 scripts/validate_ego_native.py --repo "$PWD" --workspace /absolute/isolated/research-ws --output /absolute/logs/native.json
python3 scripts/validate_ego_vehicle_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/logs/ego-interface.json
python3 scripts/validate_mock_serial_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/logs/faults.json
python3 scripts/validate_mock_serial_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/logs/straight.json --ego-loop --loop-budget-s 65
python3 scripts/validate_mock_serial_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/logs/arc.json --arc-loop --loop-budget-s 65
```

The probe allocates its own PTY and feeds the unchanged serial executable. Curvature=1/m, lateral tolerance=.05 m/s, jerk=3 m/s³ and model braking are labelled synthetic settings, never physical acceptance. Additional switches `--rtk-classifier` and `--tf-static-fault` test original GNSS quality classification and static TF competition. Physical manual/KEY/firmware e-stop is outside their scope.

```bash
python3 scripts/validate_route_prefix_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --input /absolute/retained-measured-failures.json --output /absolute/logs/route-prefix.json
python3 scripts/validate_ego_cache_progress_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --input /absolute/retained-measured-failures.json --output /absolute/logs/cache-progress.json
export ROS_DOMAIN_ID=96
python3 scripts/validate_evidence_anchor_ros.py --install /absolute/isolated/research-ws/install --output /absolute/logs/anchors.json
export ROS_DOMAIN_ID=97
python3 scripts/validate_lio_adapter_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/logs/adapter.json
export ROS_DOMAIN_ID=98
/path/to/research-venv/bin/python scripts/validate_observer_session_ros.py --install /absolute/isolated/research-ws/install --output /absolute/logs/observer-session.json
export ROS_DOMAIN_ID=93
/path/to/research-venv/bin/python scripts/validate_research_launch.py --repo "$PWD" --output /absolute/logs/entry.json --serial-evidence /absolute/logs/faults.json --override-probes
python3 scripts/audit_research_runtime_parameters.py --root "$PWD" --default-entry /absolute/logs/entry.json --serial /absolute/logs/faults.json --output /absolute/logs/three-layer.json
export ROS_DOMAIN_ID=94
/path/to/research-venv/bin/python scripts/validate_restricted_policy_ros.py --repo "$PWD" --install /absolute/isolated/research-ws/install --output /absolute/new-trial-directory/comparison.json --budget-s 60
```

Use a fresh trial directory and run comparisons without other ROS/build jobs. The retained c5dfeec measured-failure input is under the isolated remote logs/qualified-final-policy/PASSIVE/task-2/failed-measured-inputs.json; the prefix regression does not use truth or a full world map. Its core query records old endpoint rejection and shortened endpoint feasibility. The cache-progress probe uses the distinct already-shortened partial-map input at logs/delivery-final-policy/PERIODIC_LOOK/task-2/failed-measured-inputs.json. The simulator spawns only task-owned processes; no real actuator or production domain is connected. For sensor-loss tests add `--strategies PASSIVE --tasks 1 --budget-s 25 --sensor-fault imu_lost --fault-after-s 12`, and similarly lidar_lost/depth_invalid. Inspect fault-phase wire bytes captured before cleanup denies RTK.

Raw bag identity and one-time alignment rules are in audit/remote_humble/lio-paired-metrics.json and lio-bag-fields.json. The raw July 15 bag contains 2867 scans / 57008 IMU samples over 287.37 s. Existing legacy odometry is not an estimator input. Replay scripts audit input identity, source measurement time and binary SHA. The paired discrepancy is not ground-truth error.

Ordinary entry is `ros2 launch bringup system_active_road_research.launch.py`, default replay with simulated time and no drivers/serial. A new launch creates a fresh localization session ID; providers use its latched identity/submap namespace. Independent LIO reinitialization requires a new session. Default physical curvature/jerk/sensor model and verified extrinsics/health remain unresolved and block motion. Jetson deployment is not performed by these commands. Follow LIVE_ACCEPTANCE_CHECKLIST.md before any human-controlled live test.
