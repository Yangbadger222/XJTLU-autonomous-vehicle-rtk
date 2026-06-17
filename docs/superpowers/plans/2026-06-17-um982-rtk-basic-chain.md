# UM982 RTK C++ Basic Chain Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prepare the current `frc` branch for first UM982 RTK rover bring-up on the Jetson with a C++ ROS 2 driver that publishes `/fix` and `/heading` and can optionally feed CORS/NTRIP RTCM back to the receiver.

**Architecture:** Add a new `ament_cmake` package, `um982_rtk_driver`, under `src/sensor_drivers/gnss/`. The package owns `/dev/rtk_um982` in one C++ node, parses UM982 NMEA/heading sentences, publishes `/fix`, `/heading`, raw NMEA, and a text status topic, and optionally connects to an NTRIP caster so no second process competes for the same serial device. Existing GPS launch paths are switched to this C++ node while keeping the old Python `nmea_navsat_driver` in-tree for compatibility.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, C++17, in-repo `serial` C++ library, `sensor_msgs`, `geometry_msgs`, `nmea_msgs`, POSIX sockets for non-TLS NTRIP, `ament_cmake_gtest`.

---

## Chunk 1: C++ Parser And Driver

### Task 1: Add UM982 Parser Tests

**Files:**
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/test/test_nmea_parser.cpp`
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/include/um982_rtk_driver/nmea_parser.hpp`
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/src/nmea_parser.cpp`

- [ ] **Step 1: Write the failing tests**

Add gtest cases that construct valid-checksum NMEA sentences and assert:
- `$GPGGA` with GGA quality `4` parses as RTK Fixed.
- `$GNTHS` parses heading plus mode.
- `$GNHPR` parses heading, pitch, roll, fix type, satellite count, differential age, and station id.
- bad checksum sentences are rejected.

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select um982_rtk_driver --event-handlers console_direct+
```

Expected: FAIL before implementation because the package/parser does not exist.

- [ ] **Step 3: Implement parser support**

Implement pure C++ parser helpers for:
- checksum validation;
- comma splitting before `*`;
- NMEA coordinate conversion;
- GGA, RMC, THS, HPR;
- RTK quality labels for status output.

- [ ] **Step 4: Run tests to verify pass**

Run the same colcon test command. Expected: PASS.

### Task 2: Add C++ ROS Driver Node

**Files:**
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/src/um982_rtk_node.cpp`
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/CMakeLists.txt`
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/package.xml`

- [ ] **Step 1: Implement node parameters**

Parameters: serial `port`, `baud`, `frame_id`, covariance defaults, `publish_raw`, `status_period_s`, `ntrip.enabled`, `ntrip.host`, `ntrip.port`, `ntrip.mountpoint`, `ntrip.username`, `ntrip.password`, `ntrip.password_env`, `ntrip.send_gga_interval_s`, `ntrip.connect_requires_valid_gga`.

- [ ] **Step 2: Implement serial ownership**

Use the in-repo `serial` C++ library to open `/dev/rtk_um982` and read newline-delimited NMEA.

- [ ] **Step 3: Publish ROS topics**

Publish:
- `/fix` as `sensor_msgs/msg/NavSatFix` from GGA;
- `/heading` as `geometry_msgs/msg/QuaternionStamped` from valid THS/HPR;
- `rtk/nmea_sentence` as `nmea_msgs/msg/Sentence`;
- `rtk/status` as `std_msgs/msg/String`.

- [ ] **Step 4: Implement optional NTRIP**

Use POSIX sockets for non-TLS NTRIP. Feed latest GGA to caster and write RTCM bytes back to the same serial port. Keep credentials in ROS parameters or environment only; never commit CORS credentials.

## Chunk 2: Bring-Up Entrypoints

### Task 3: Add RTK Parameter Profile And Launch Wiring

**Files:**
- Create: `src/sensor_drivers/gnss/um982_rtk_driver/config/um982_rtk.yaml`
- Create: `src/bringup/launch/system_rtk_basic.launch.py`
- Modify: `src/sensor_drivers/gnss/gnss_calibration/launch/gnss_calibration_launch.py`
- Modify: `src/bringup/launch/system_gps_corridor.launch.py`
- Modify: `src/bringup/launch/system_nav_gps.launch.py`
- Modify: `src/bringup/package.xml`
- Modify: `src/bringup/config/master_params.yaml`
- Modify: `src/sensor_drivers/gnss/nmea_navsat_driver/config/nmea_serial_driver.yaml`
- Modify: `Makefile`
- Modify: `scripts/launch_with_logs.sh`

- [ ] **Step 1: Add standalone RTK config**

Create a config that points `um982_rtk_driver` to `/dev/rtk_um982` at `115200`, frame `gps`, with NTRIP disabled by default.

- [ ] **Step 2: Switch GPS launch paths**

Use the C++ UM982 driver in GNSS calibration, corridor, and nav-gps launch paths so downstream `/fix` behavior stays the same.

- [ ] **Step 3: Add convenience targets and cleanup**

Add `make launch-rtk-basic`, include `um982_rtk_driver` in `make build-sensor`, and clean up `/dev/rtk_um982` plus `um982_rtk_node` in runtime cleanup.

## Chunk 3: Docs, Verification, Jetson Deploy

### Task 4: Document First RTK Test

**Files:**
- Create: `docs-CN/handoff/rtk_um982_basic_chain.md`
- Create: `docs-EN/handoff/rtk_um982_basic_chain.md`
- Modify: `docs-CN/devlog/2026-06.md`
- Modify: `docs-EN/devlog/2026-06.md`

- [ ] **Step 1: Add bilingual operator guide**

Document build command, `launch-rtk-basic`, topic checks, CORS parameter pattern, expected indoor invalid fix, outdoor Fixed/Float acceptance, and no-credential rule.

- [ ] **Step 2: Add bilingual devlog entry**

Record File / Change / Reason / Effect.

### Task 5: Verify, Commit, Push, Deploy

**Files:**
- All files changed in Tasks 1-4.

- [ ] **Step 1: Run local tests**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select um982_rtk_driver --event-handlers console_direct+
```

- [ ] **Step 2: Build packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select serial nmea_msgs um982_rtk_driver bringup gnss_calibration --symlink-install --parallel-workers 1
```

- [ ] **Step 3: Commit explicit files only**

Use explicit `git add <file>...`, never `git add -A` or `git add .`.

- [ ] **Step 4: Push `frc`**

```bash
git push origin frc
```

- [ ] **Step 5: Fix Jetson checkout**

On Jetson, backup the currently empty non-Git `~/XJTLU-autonomous-vehicle`, clone `https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk.git`, checkout `frc`, build with `--parallel-workers 1`, source install, and run `make launch-rtk-basic` if the UM982 is connected.
