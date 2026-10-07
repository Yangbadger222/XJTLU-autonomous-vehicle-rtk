# Current source qualification

Goal is active. Compile source da361f24b15f417787647c224cabde22d404fd73; subsequent test/report commits leave runtime sources unchanged. Exact pins and sequential Super2/EGO6 patches are checked by scripts/verify_current_upstream_patches.py; use a NEW verification-root and the actual patched dependency checkout. Fresh build recipe scripts/build_active_road_research.sh clears inherited overlays and compiles the original SDK plus14 allowlisted packages with one worker into an independent absolute workspace.

Current non-Jetson workspace: /home/badger/codex-research/superlio-ego-20261007/ground-da361f2-ws. Source checkout vehicle-lio-ready; original dirty checkout and historical preview/install remain preserved. No physical devices or drivers are opened by default replay.

Executed receipts and repeatable scripts:

- Three full raw LIO/source-reference trials at69b9b75: scripts/validate_lio_reference_replay.py, domain104/localhost-only, raw /livox/lidar and /livox/imu plus clock. Actual metadata/EOF/span/gap/health checks, no duration prefix relabeled as full replay.
- Current installed ground fixtures: scripts/validate_observed_ground_ros.py, domain106/localhost-only, actual16 ROS cases, no serial sink.
- Current EGO interface and final serial faults: scripts/validate_ego_vehicle_ros.py and scripts/validate_mock_serial_ros.py, domain91/localhost-only, explicit analytical grid/settings; serial only allocated PTY.
- Current default entry: scripts/validate_research_launch.py, domain93/localhost-only, --console-port8876 --override-probes, preserved preview port8765 untouched. Actual graph/owners/parameters; deliberately invalid overrides must fail.
- Portable tests: PYTHONPATH=src/research_runtime:src/active_road_mapping:src/super_lio_vehicle_adapter python3 -m pytest -q src/research_runtime/test src/super_lio_vehicle_adapter/test scripts/test_research_bag_contract.py scripts/test_recorded_chassis_response.py scripts/test_replay_acceptance.py. Current174 checks pass; scripts/generate_research_parameter_lock.py --check verifies generated source constants.

Full observed-road/default control/Jetson shadow and physical acceptance remain pending. Historical command/evidence details are preserved under audit/vehicle_ready/history_pre_qualification/REPRODUCE.md; their older runtime hashes must not be applied to current sources.
