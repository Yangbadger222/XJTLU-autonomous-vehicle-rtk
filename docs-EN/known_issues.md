# Known Issues Tracker

## Current Blockers

34. **[Important] GPS Corridor v3 ordinary GNSS accuracy is insufficient; Nav2 targets do not match physical points**
   - Description: During the 2026-05-08 v3 on-vehicle tests, the system passed the startup guard and entered `RUNNING_ROUTE`, but the user observed that GPS-converted coordinates in Nav2 did not correspond to the intended physical points.
   - Direct evidence:
     - `2026-05-08-17-43-03`: startup `distance_to_start_ref=1.83m`, `spread=0.06m`, entered `RUNNING_ROUTE`, then repeatedly triggered alignment preemption and later reported `Failed to make progress`
     - `2026-05-08-17-50-10`: startup `distance_to_start_ref=3.25m`, `spread=0.36m`, entered `RUNNING_ROUTE`; the first-segment subgoal moved from `x=33.03 y=-0.67` to `x=46.69/-0.96`, `x=48.64/-1.46`, and `x=48.35/-1.92` after alignment shifts
     - PGO/FAST-LIO2 continuously received cloud + odom; no evidence of point cloud map, odometry, or `map -> odom` divergence
     - The 100x100 global costmap was deployed, and the previous v2 `goal off the global costmap` / `worldToMap failed` issue did not recur in v3
   - Status: Recorded on 2026-05-08. Runner, costmap, and RViz deployment fixes are closed; the remaining issue is attributed to ordinary `/fix` absolute positioning and ENU -> map anchoring/conversion accuracy.
   - Impact: The current ordinary-GNSS corridor cannot be claimed as physically precise. The next phase should move to RTK, `robot_localization`, multi-point registration, or map-anchored physical waypoint routing.

1. **[Fatal] First on-vehicle `nav-gps` navigation run on `feature/gps-route-ready-v2` fails during execution**
   - Description: User has collected a real `ls-building` scene and completed the `current_scene/` compilation; `nav_gps_menu.py` can successfully enter `GOAL_REQUESTED -> COMPUTING_ROUTE -> FOLLOWING_ROUTE`, but the vehicle only moves briefly before stopping.
   - Direct evidence:
     - `goal_manager`: `FAILED; follow_path_status=6`
     - `controller_server`: `Controller patience exceeded`
     - launch log: `pgo_node ... process has died ... exit code -11`
   - Confirmed impact chain:
     - `pgo_node` exits with segfault
     - `map -> odom` TF disappears, only `odom -> base_link` remains
     - `/gps_system/status` regresses from `NAV_READY` back to `GNSS_READY`
     - Nav2 `follow_path` is aborted
   - Status: Reproduced on 2026-03-20 and identified as the current highest-priority blocker
   - Impact: The new GPS route-graph navigation mode cannot pass on-vehicle acceptance

3. **[Important] `nav-gps` software is deployed but outdoor on-vehicle verification is incomplete**
   - Description: `feature/gps-navigation-v4` has completed `gps_waypoint_dispatcher`, `nav2_gps.yaml`, fixed ENU origin, and `system_nav_gps.launch.py`; indoor smoke test passed; however, actual outdoor operation, route-graph expansion, and tuning are not yet done.
   - Status: Software complete, awaiting outdoor verification
   - Impact: Cannot claim GPS goal navigation capability as a final on-vehicle stable feature

4. **[Fatal] Velocity smoother closed-loop control failure**
   - Description: C Board odometry closed-loop causes abnormal vehicle spinning; FAST-LIO2 odometry closed-loop also did not achieve stable path tracking.
   - Status: Still using `OPEN_LOOP`
   - Source: 2025-11 series of on-vehicle tests

## Important Issues

5. **[Important] Vehicle stops after prolonged operation**
   - Description: Nav2 paths and costmaps still update, but the chassis no longer moves; suspected serial output or controller chain issue.
   - Status: Investigation incomplete

6. **[Important] SLAM Toolbox high memory pressure in long corridors**
   - Description: Significant memory growth risk exists during prolonged mapping sessions.
   - Status: Not permanently resolved

7. **[Important] LiDAR degradation near walls**
   - Description: MID360 point cloud degrades when near walls, unfriendly for narrow corridor obstacle avoidance.
   - Status: Only empirical avoidance measures; no definitive solution yet

8. **[Important] FAST-LIO2 + PGO memory growth**
   - Description: During prolonged operation, FAST-LIO2, PGO keyframes, and related caches push up memory usage.
   - Status: System-level service trimming done, but algorithm-level handling not addressed

31. **[Resolved] MPPI straight path tracking serpentine corrections**
    - Description: Evening high-speed test on 2026-04-01 revealed left-right-left-right serpentine corrections on clear straight path tracking
    - Solution: 2026-04-02 introduced Savitzky-Golay path smoothing (inserted SmoothPath in BT) + MPPI critic tuning (PathAlignCritic earlier intervention, PathFollowCritic delayed exit)
    - Status: Resolved, user on-vehicle test evaluation "perfectly successful" (commit `15d4cac`)

## Medium Issues

35. **[Medium] GPS Noise parameters outdated**
    - Description: The GPS parameters in `src/bringup/config/master_params.yaml` such as `gps.noise_xy` and `gps.noise_z` are outdated. They belong to the previous innaccurate GPS antenna, and have not been adapted for the new RTK.
    - Status: Awaiting tuning

10. **[Medium] Travel prior-map mode still needs on-vehicle validation**
    - Description: `system_travel.launch.py` uses a prior-map chain: the 2D `map.yaml` feeds Nav2/AMCL, the 3D `map.pcd` feeds initial localizer ICP, and `prior_map_tf_authority` gates both candidates while exclusively owning `map -> odom`.
    - Status: Software wiring restored and built on the Jetson; further indoor prior-map navigation validation is still needed.

11. **[Medium] GPS route-graph still needs further collection and refinement**
    - Description: Fixed ENU origin and dispatcher route planning are implemented in software, but `campus_road_network.yaml` is still only a bootstrap graph that needs continued on-vehicle `/gnss` waypoint collection to expand.
    - Status: Awaiting further collection

12. **[Medium] GPS drift still affects global pose and endpoint convergence**
    - Description: During long-distance unidirectional runs, loop closures alone are insufficient to constrain global drift, so the system still relies on GPS factor quality, valid offsets, and outdoor data quality.
    - Status: GPS factor is online; insufficient field data

14. **[Medium] In-place rotation trajectory is not circular**
    - Description: Mechanical asymmetry in left/right wheel output creates higher risk during rotation in tight spaces.
    - Status: Hardware limitation

29. **[Medium] Rosbag recording early termination**
    - Description: During 2026-04-01 GPS corridor test, rosbag stopped at 11:22:15 but system ran until 11:31:13; second half cannot be replayed
    - Status: Recorded, root cause TBD

## Low Priority / Toolchain Issues

16. **[Low] PGO global point cloud RViz visualization still incomplete**
    - Description: The current main chain emphasizes `map -> odom` and `/pgo/optimized_odom`, but the global point cloud display experience is not a primary maintenance target.
    - Status: Low priority

17. **[Low] FAST-LIO2 `world_cloud` has limited practical utility**
    - Description: This output is not a core dependency in the current main chain and can be re-evaluated during future resource optimization.
    - Status: Low priority

18. **[Low] Lower-level controller PID emergency stop causes wheel reversal**
    - Description: The old KEY/B/X stop paths could stop CAN output or use PID zero-speed braking, causing noticeable wheel reversal.
    - Status: Updated further on 2026-06-14 to separate stop semantics: KEY/X/gamepad-loss keep sustained zero-current coast-stop output, while `B` now performs damped active braking with a high-speed current limit, low-speed current tapering, current-rate limiting, and zero-current release near stop; the `B` latch initializes only on the first trigger, holding `B` no longer resets the current ramp, and the indicator is now non-blocking solid pink; bench and vehicle validation are still required.

## Recently Fixed
15. **[Low] Risk of `gh` token expiration on Jetson**
    - Description: Jetson-side `gh` has previously experienced login expiration; if it recurs, `gh pr create` / `gh pr merge` can be run on the logged-in Windows workstation as a workaround.
    - Status: Changed to a non-expiring token (2026-07-09)

13. **[Fixed] No URDF**
    - Description: A complete URDF / simulation chain is currently missing, so parameter validation primarily relies on the physical vehicle.
    - Fix: Added simplified URDF to identify the estimated robot dimensions and direction
    - Status: Fixed and verified on the Jetson (2026-07-07)

36. **[Fixed] Jetson Board Out of storage**
    - Description: During the LiDAR on-vehicle tests in 2026-06-25, the rosbag record failed and a "Low Disk Space" warning appeared, pointing there is only 733.9 MB of storage left.
    - Direct evidence: `OSError: I/O error: I/O error: No space left on device (os error 28)`
    - Fix: Some logs were storing cloudpoints that consumed most of the space, over 30 GB per log. Removing them has solved the problem.
    - Status: Fixed, over 64 Gb available

37. **[Fixed] make launch-nav-gps not launching URDF properly**
   - Description: Launching the robot with `make launch-nav-gps` would not render properly in Foxglove, with missing grid and frame connections.
   - Fix:
        - **Step 1: Verify the parameters on the robot**
          Run this command in the terminal on the vehicle to check if the running parameter is indeed stale:
          ```bash
          cat ~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/master_params_scene.yaml | grep body_frame
          ```
        - **Step 2: Regenerate the scene parameters**
          Compile your scene bundle to update `master_params_scene.yaml` from your updated `master_params.yaml` template:
          ```bash
          cd ~/XJTLU-autonomous-vehicle
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          python3 scripts/build_scene_runtime.py
          ```
        - **Step 3: Launch the navigation stack**
          Ensure the URDF launch is uncommented in [system_nav_gps.launch.py](file:///wsl.localhost/Ubuntu-22.04/home/yoonzh22/robotrepo/XJTLU-autonomous-vehicle-rtk/src/bringup/launch/system_nav_gps.launch.py#L137-L141) and run:
          ```bash
          make launch-nav-gps
          ```

          Once launched, the TF tree will correctly flow:
          $$\text{map} \rightarrow \text{odom} \rightarrow \text{base\_footprint} \rightarrow \text{base\_link} \rightarrow \text{sensor\_links...}$$
          All nodes will be able to resolve transforms correctly, and Foxglove will render the grid and robot model.
   - Status: 2026-07-02, fixed and proven successful in on-vehicle testing
   - Impact: No longer a blocker

2. **[Fixed] Outdoor GNSS RF / fix quality**
   - Description: GPS antenna feed cable has been replaced; device enumeration is normal.
   - Status: 2026-03-22, across multiple corridor v2 outdoor on-vehicle runs, GPS fix worked reliably; startup positioning and PGO alignment both used `/fix` normally
   - Impact: No longer a blocker

19. **[Fixed] USB 2.0 interface limitation**
    - Description: Unfriendly for certain high-bandwidth peripheral expansion.
        - S350 V1.1 Hardware Profile:
        - Compute Connector: Standard 260-pin SO-DIMM for NVIDIA Jetson modules.
        - Power: 1x yellow XT30 connector (DCIN) for direct LiPo battery input.
        - Networking & Display: 1x Gigabit Ethernet (RJ45) and 1x HDMI port.
        - Camera Interface: 2x MIPI-CSI FPC connectors.
        - USB: 2x USB Type-C ports (used for data and module flashing/recovery).
    - Fix:
        - Removed the previous `S350 V1.1 OEM Jetson Carrier Board`
        - Changed previous board to a `Seeed reComputer carrier board (4×USB 3.0 + GbE + HDMI)`, with 4 ports for USB 3.0
        - Item link: https://e.tb.cn/h.RZusJWI?tk=QTX75qlJplP
    - Status: Jetson is attached to the Seeed board, working correctly (2026-06-25)

32. **[Fixed] runtime-data Hugging Face remote synchronization blocked**
   - Description: Attempts to archive and push on-vehicle test runtime-data to the Hugging Face dataset failed.
   - Symptom:
     - Git/Xet push throws `gnutls_handshake() failed`
     - HF API fallback fails with `SSL EOF` due to lack of token under proxy
     - SSH fallback fails with `Permission denied (publickey)` due to unconfigured keys
   - Root cause: Access keys were unconfigured, and the Jetson could not reach Huggingface servers due to local firewall issues
   - Fix:
     - Installed Huggingface CLI with `pip install -U "huggingface_hub[cli]"`
     - Changed the endpoint to use a mirror with `export HF_ENDPOINT=https://hf-mirror.com`
     - Re-authenticated with `hf auth login` by creating an organization for the team and using a shared access token
     - Created an organization repo with `hf repos create my-rtk-data --type dataset`
     - Can now properly upload rosbags with `hf upload frogcar/rtk-data-2026-surf ./runtime-data/ --repo-type dataset` and access the data in the shared database at https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main (only team members)
   - Status: Fixed and verified on Jetson (2026-06-25)

9. **[Fixed] Costmap obstacle residuals / slow clearing**
   - Description: After obstacles are removed, cost values on the costmap clear too slowly.
   - Status: 2026-03-26, STVL `clear_after_reading` changed from `true` to `false` (local + global); obstacles are now managed naturally by `voxel_decay` instead of being cleared every cycle
   - 2026-03-31 update (`gps-mppi`): Height window narrowed to `-0.33~0.30m` (vehicle body height range), inflation radius local `0.43` / global `0.63`, obstacle map expanded to 15m. Indoor testing confirmed false obstacles eliminated
   - Impact: No longer an issue

33. **[Fixed] Missing optional directories caused bringup build failure**
    - Symptom: During a full build on Jetson, the `bringup` package failed, reporting that `maps/` or `urdf/` directories were not found.
    - Root cause: `src/bringup/CMakeLists.txt` lacked existence checks for these directories before the `install` command.
    - Fix: commit `fe7e546` -- wrapped `install` commands with `if(EXISTS ...)` guards.
    - Status: Fixed and verified on Jetson (2026-04-15)

30. **[Fixed] GPS corridor alignment mechanism issues**
    - Symptom: Morning test on 2026-04-01 exposed three core issues: (1) startup stable GPS offset (~4.11m) not absorbed into bootstrap (2) waypoint calibration rotation flip (176.95deg) (3) per-waypoint frozen alignment caused live alignment rejection by guard
    - Fix: Evening 2026-04-01 completed four changes (commit `ebc26e2` + `fe3933e` + `e73c2bf` + `c0ea847`):
      - Calibration switched to translation-only, avoiding rotation flips
      - Startup directly absorbs stable GPS offset into bootstrap alignment
      - Runner uses real-time alignment to recompute subgoals, removed per-waypoint frozen mechanism
      - Speed cap raised to 1.5 m/s
    - Verification: Evening indoor high-speed test passed, high-speed obstacle avoidance normal
    - Status: Fixed and deployed (2026-04-01)

27. **[Fixed] syncPackage empty point cloud segfault (Issue #4)**
    - Symptom: When LiDAR is occluded or all points are out of range, `livox2PCL()` returns an empty cloud; `syncPackage()` calls `.back()` on empty `points`, triggering a segfault
    - Impact chain: FAST-LIO2 crash -> `odom -> base_link` TF lost -> system localization failure
    - Fix: commit `9a193af` -- added empty point cloud guard in `syncPackage()`; empty frames are discarded
    - Status: Fixed, deployed, GitHub Issue #4 closed (2026-03-31)

28. **[Fixed] Startup GPS spread threshold too strict, preventing corridor startup**
    - Symptom: In first on-vehicle re-test, `gps_global_aligner` / `gps_route_runner` stuck at `WAITING_FOR_STABLE_FIX`; rest of system running normally
    - Root cause: Route default `startup_fix_spread_max_m: 3.0`, but current GPS device's 60-point window spread is typically ~4.8m, failing to meet the threshold
    - Fix: commit `d9b63dc` -- route collection script default relaxed from `2.0` to `5.0`; current running route changed from `3.0` to `5.0`
    - Status: Fixed, awaiting next clear-weather re-test for verification (2026-03-31)
      - 2026-04-15 update: During the outdoor corridor smoke test, the code chain launched normally, but the system remained stuck at `WAITING_FOR_STABLE_FIX` due to the lack of a valid GNSS fix in the environment (continuously outputting empty NMEA `$GNGGA,,,,,,0,00,25.5,,,,,,*64`). This is a physical signal issue, not a code blocker.

1. **[Fixed] PGO does not establish `map -> odom` after startup**
   - Symptom: Continuously prints `Received out of order message` after startup, no keyframes, no stable `map -> odom`; RViz under the `map` fixed frame appears blank.
   - Root cause: `last_message_time` sync state in `pgo_node.cpp` was uninitialized; the first pair of synced messages was incorrectly flagged as out-of-order.
   - Fix: Initialized `last_message_time` to `0.0` to ensure the first matching message pair is accepted.
   - Status: Merged to `main` on 2026-03-18

2. **[Fixed] GPS navigation main software chain was missing**
   - Symptom: Historical versions only had `GNSS -> PGO GPS factor` without a formal execution chain to pass GPS goals to Nav2.
   - Fix: Added `gps_waypoint_dispatcher`, `nav2_gps.yaml`, fixed ENU origin, `system_nav_gps.launch.py`, and the `nav-gps` mode on `feature/gps-navigation-v4`.
   - Status: Indoor software smoke test passed; awaiting final outdoor verification

20. **[Upgraded] Corridor v1 multi-meter endpoint residual -> v2 standalone aligner deployed**
    - Description: v1 used body_vector straight-line navigation, with endpoint deviation of ~4 m due to yaw0 uncertainty.
    - Status: Upgraded to corridor v2 (standalone global aligner architecture); v1 retained as baseline
    - v2 current status (closed out 2026-03-26):
      - Waypoint 1 reached reliably
      - Waypoint boundary alignment guard is active; bad alignments are no longer swapped in
      - **Primary issue has converged to GPS route collection/anchoring method**: startup GPS has ~2.5 m error; single-point anchoring cannot guarantee stable arrival at the intended physical location
      - Further tuning of Nav2 YAML or minor runner fixes has hit the ceiling; anchoring approach redesign needed

21. **[Resolved] Corridor v2 PGO handoff threshold does not match field frequency**
    - Description: v2 initial version used PGO live handoff, but PGO's ~1 Hz update rate could not meet the switching threshold.
    - Status: **Resolved via architectural change** -- standalone global aligner replaced PGO handoff; switching threshold no longer needed
    - Impact: No longer applicable

22. **[Fixed] Controller / Planner loop frequency below target**
    - Description: Controller repeatedly reports `Control loop missed its desired rate`; Planner drops to ~2 Hz.
    - Status: On `gps-mppi`, MPPI runs at 20Hz; indoor testing passed with no frequency shortfall reports
    - Impact: No longer an issue

23. **[Fixed] Corridor BT file had Spin removed but runtime still executed spin**
    - Description: Both local and Jetson corridor BT XML files did not contain `Spin`, but `behavior_server` logs still printed `Running spin`
    - Root cause: `default_nav_to_pose_bt_xml` parameter was placed under `bt_navigator_navigate_to_pose_rclcpp_node` instead of `bt_navigator`, causing the launch injection to have no effect
    - Fix: Commit `d075c6b` moved the parameter to the correct `bt_navigator` node
    - Status: Fixed (2026-03-26)
    - 2026-07-06 update: RTK corridor testing hit recovery `/cmd_vel` contention again. This time the cause was not the parameter namespace; `system_explore.launch.py` injected the same recovery BT for both explore and corridor, and `default_server_timeout=20ms` let a slightly slow FollowPath acknowledgement trigger recovery. Corridor now uses dedicated NavigateToPose / NavigateThroughPoses BTs without `Spin`/`BackUp`, and its BT action timeout is raised to 1000ms.
    - 2026-07-06 22:33 update: the through-poses BT injection still did not take effect because `nav2_explore.yaml` lacked the empty `default_nav_through_poses_bt_xml` default slot. RewrittenYaml cannot replace a missing key, so Nav2 fell back to the upstream recovery tree and failed startup because the `spin` action server was absent. The slot is now present and covered by a regression test.

24. **[Root Cause Fixed] Late-segment `lio_odom` / `odom->base_link` divergence**
    - Description: In multiple on-vehicle runs, `odom->base_link` starts making continuous large jumps in the second half of the second segment, causing pose divergence and navigation failure
    - **Root cause confirmed (2026-03-30)**: `lidar_processor.cpp:245` point-to-plane Jacobian `hat()` argument incorrectly uses `state.t_wi` (world position, ~50m) instead of `state.t_il` (extrinsic offset, ~0.04m)
      - Origin: fork changed original FAST-LIO2 `SKEW_SYM_MATRX` macro to `Sophus::SO3d::hat()` with wrong variable name
      - Impact: rotation Jacobian scales with distance from origin (hundreds of times too large) -> incorrect IESKF rotation correction -> map rotation -> odom divergence
    - Fix: commit `e4945f4` -- one-line fix `state.t_wi` -> `state.t_il`
    - Mitigations retained (commit `308fe77`): odom watchdog + ESKF degradation protection kept as general safety measures
    - 2026-03-31 update: `gps-mppi` indoor full-loop corridor verification showed FAST-LIO2 stable throughout, no odom divergence, no IMU drift, no point cloud drift. Jacobian fix introduced no regressions on the current baseline
    - Status: root cause fixed; indoor verification passed; awaiting outdoor long-distance verification

25. **[Important] GPS route collection/anchoring method insufficient for physical precision**
    - Description: Current corridor GPS route relies on a single `start_ref` + `launch_yaw_deg` for anchoring. Startup GPS itself has ~2.5 m error, and route geometry is defined only in the ENU domain, causing map-projected targets to systematically deviate from the user's intended physical path
    - Latest evidence (session `2026-03-27-13-43-46`):
      - `distance_to_start_ref=4.75m` (tolerance 15.0 m allowed an excessively large deviation at start)
      - Under frozen alignment, second segment map projection dx=+3.28 m lateral offset
      - Translation-only aligner continuously rejected corrections (delta 8.88~9.50 m > 8.0 m threshold)
    - Calibration handshake attempt (commit `308fe77`):
      - Deployed waypoint progressive calibration mechanism: runner requests aligner to recalibrate using static GPS samples upon reaching a waypoint
      - Session `2026-03-27-18-22-31` result: wp1 calibration failed; GPS mean was 30.35m from recorded waypoint; executed `CALIBRATION_FALLBACK`
      - Conclusion: wp1 in the current route does not serve as a reliable calibration anchor
    - Status: 2026-03-27, re-confirmed as the current primary bottleneck; needs to return to Step 8 for anchoring approach re-review
    - Candidate directions: multi-point rigid-body registration / map physical waypoint route / continuous trajectory collection
    - 2026-04-01 GPS corridor regression test confirmed again: startup offset not absorbed (~4.11m), calibration theta flip (176.95deg), live alignment rejected by guard (4.96m > 3.0m). User decided to close current round as pass; issues deferred to next round
    - 2026-06-18 mitigation: UM982 dual-antenna `/heading` is now used by `gps_global_aligner_node` bootstrap, so the initial rotation prefers live RTK heading; `launch_yaw_deg` remains as fallback and consistency warning. The stronger remaining direction is still multi-point rigid-body registration or map-physical waypoint routes.

26. **[Known] Translation-only aligner failed to correct startup anchoring error**
    - Description: Commit `94862d7` changed the global aligner to fix the bootstrap rotation and only estimate translation, intending to progressively correct startup anchoring deviation during operation
    - On-vehicle result (session `2026-03-27-13-43-46`):
      - Continuously printed `Rejecting raw GPS alignment: bootstrap translation delta 8.88m > 8.00m` during operation
      - No successful publication of a new trusted runtime translation correction
    - Root cause: The startup anchoring deviation itself exceeded the aligner's `max_bootstrap_translation_delta_m: 8.0` threshold
    - Status: Recorded; will not be fixed independently -- this is a downstream manifestation of #25 GPS anchoring primary issue
