# RTK FGO Localizer Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first shadow-mode `rtk_fgo_localizer` experiment so the vehicle can fuse FAST-LIO2 odometry, IMU, wheel/chassis odometry, RTK position, and RTK heading in a gated sliding-window factor graph without changing production navigation modes.

**Architecture:** Add a new `ament_cmake` package under `src/perception/rtk_fgo_localizer/` with ROS-free core classes for RTK gating, state transitions, correction smoothing, and GTSAM window optimization. Add a ROS node that subscribes to the existing sensor topics, publishes `/rtk_fgo/*` diagnostics and odometry, and initially runs beside the current PGO/corridor stack without publishing production `map -> odom`. Add an experimental launch and parameter profile only after the core is tested.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, C++17, GTSAM, Eigen3, GeographicLib, `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `nmea_msgs`, `std_msgs`, `diagnostic_msgs`, `tf2_ros`, `ament_cmake_gtest`.

---

## Scope Check

This plan implements the first working shadow-mode RTK FGO localizer. It does not replace `corridor`, `explore-gps`, or `nav-gps`, and it does not make Nav2 consume FGO output by default.

Guarded Nav2 control is included only as launch/parameter scaffolding with defaults disabled. Closed-loop behavior must wait for shadow-mode replay and vehicle validation.

## File Structure

Create:

- `src/perception/rtk_fgo_localizer/CMakeLists.txt` - package build, libraries, executable, tests
- `src/perception/rtk_fgo_localizer/package.xml` - ROS and system dependencies
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/rtk_quality.hpp` - raw GGA/RTK quality parsing and gate decisions
- `src/perception/rtk_fgo_localizer/src/rtk_quality.cpp`
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/state_machine.hpp` - localization mode transitions
- `src/perception/rtk_fgo_localizer/src/state_machine.cpp`
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/correction_smoother.hpp` - bounded correction release
- `src/perception/rtk_fgo_localizer/src/correction_smoother.cpp`
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/fgo_graph.hpp` - sliding-window GTSAM graph API
- `src/perception/rtk_fgo_localizer/src/fgo_graph.cpp`
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/yaw_factor.hpp` - yaw-only Pose3 factor for RTK heading
- `src/perception/rtk_fgo_localizer/src/yaw_factor.cpp`
- `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/topic_buffers.hpp` - timestamped sensor sample buffers
- `src/perception/rtk_fgo_localizer/src/topic_buffers.cpp`
- `src/perception/rtk_fgo_localizer/src/rtk_fgo_node.cpp` - ROS node and publishers
- `src/perception/rtk_fgo_localizer/test/test_rtk_quality.cpp`
- `src/perception/rtk_fgo_localizer/test/test_state_machine.cpp`
- `src/perception/rtk_fgo_localizer/test/test_correction_smoother.cpp`
- `src/perception/rtk_fgo_localizer/test/test_fgo_graph.cpp`
- `src/bringup/config/rtk_fgo.yaml` - experimental parameters, disabled control defaults
- `src/bringup/launch/system_tightly_coupled.launch.py` - experimental launch

Modify:

- `Makefile` - add `rtk_fgo_localizer` to `build-perception` and add `launch-tightly-coupled`
- `scripts/launch_with_logs.sh` - route `tightly-coupled` mode and record `/rtk_fgo/*`
- `src/bringup/package.xml` - add runtime dependency on `rtk_fgo_localizer`
- `docs-EN/knowledge/rtk_fgo.md` and `docs-CN/knowledge/rtk_fgo.md` - update from design-only to shadow-mode implementation notes
- `docs-EN/architecture.md` and `docs-CN/architecture.md` - add experimental mode after implementation exists
- `docs-EN/commands.md` and `docs-CN/commands.md` - add command only after launch exists
- `docs-EN/devlog/2026-06.md` and `docs-CN/devlog/2026-06.md` - File / Change / Reason / Effect entry

Do not modify:

- Existing `system_gps_corridor.launch.py`, `system_explore_gps.launch.py`, or `system_nav_gps.launch.py` behavior
- Existing production `map -> odom` publishing
- Existing tuned Nav2 YAML profiles

---

## Chunk 1: Package Skeleton And ROS-Free RTK Gate

### Task 1: Add Package Skeleton

**Files:**
- Create: `src/perception/rtk_fgo_localizer/CMakeLists.txt`
- Create: `src/perception/rtk_fgo_localizer/package.xml`
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/rtk_quality.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/rtk_quality.cpp`
- Create: `src/perception/rtk_fgo_localizer/test/test_rtk_quality.cpp`

- [ ] **Step 1: Write the failing RTK quality tests**

Add gtest cases:

```cpp
TEST(RtkQuality, ParsesGgaQualityFourAsFixed) {
  auto parsed = rtk_fgo_localizer::parseGgaQuality(
    "$GNGGA,123519,3116.4956,N,12044.2528,E,4,18,0.6,12.3,M,0.0,M,,*00");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->quality, 4);
  EXPECT_TRUE(parsed->is_fixed());
}

TEST(RtkQuality, GatesFixedSampleAsStrongCandidate) {
  rtk_fgo_localizer::RtkQuality q;
  q.quality = 4;
  q.hdop = 0.6;
  q.satellites = 18;
  auto decision = rtk_fgo_localizer::evaluateRtkGate(q, 0.4, 0.1);
  EXPECT_EQ(decision.mode, rtk_fgo_localizer::RtkGateMode::StrongCandidate);
}

TEST(RtkQuality, FloatIsWeakCandidate) {
  rtk_fgo_localizer::RtkQuality q;
  q.quality = 5;
  q.hdop = 0.8;
  q.satellites = 16;
  auto decision = rtk_fgo_localizer::evaluateRtkGate(q, 0.5, 0.2);
  EXPECT_EQ(decision.mode, rtk_fgo_localizer::RtkGateMode::WeakCandidate);
}

TEST(RtkQuality, LargeInnovationRejectsEvenFixed) {
  rtk_fgo_localizer::RtkQuality q;
  q.quality = 4;
  q.hdop = 0.6;
  q.satellites = 18;
  auto decision = rtk_fgo_localizer::evaluateRtkGate(q, 12.0, 0.1);
  EXPECT_EQ(decision.mode, rtk_fgo_localizer::RtkGateMode::Rejected);
}
```

- [ ] **Step 2: Run the test and verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
```

Expected: FAIL because the package and symbols do not exist.

- [ ] **Step 3: Implement the package skeleton**

Use `ament_cmake`, C++17, `ament_cmake_gtest`, and dependencies:

```xml
<depend>diagnostic_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>nmea_msgs</depend>
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<build_depend>eigen</build_depend>
<build_depend>libgeographic-dev</build_depend>
```

In CMake, create a core library:

```cmake
add_library(rtk_fgo_core
  src/rtk_quality.cpp
)
target_include_directories(rtk_fgo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtk_fgo_core std_msgs)
```

- [ ] **Step 4: Implement `rtk_quality` minimally**

Provide:

```cpp
namespace rtk_fgo_localizer {
enum class RtkGateMode { Rejected, DiagnosticOnly, WeakCandidate, StrongCandidate };

struct RtkQuality {
  int quality = 0;
  int satellites = 0;
  double hdop = 99.0;
  bool heading_stable = false;
  bool is_fixed() const { return quality == 4; }
  bool is_float() const { return quality == 5; }
};

struct RtkGateDecision {
  RtkGateMode mode = RtkGateMode::Rejected;
  std::string reason;
};

std::optional<RtkQuality> parseGgaQuality(const std::string& sentence);
RtkGateDecision evaluateRtkGate(
  const RtkQuality& quality,
  double position_innovation_m,
  double heading_innovation_rad);
}
```

The GGA parser only needs field splitting and quality/satellite/HDOP extraction. It does not need checksum validation because `um982_rtk_driver` already publishes checksum-valid raw NMEA.

- [ ] **Step 5: Run the test and verify pass**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
colcon test-result --verbose
```

Expected: PASS.

- [ ] **Step 6: Commit chunk 1**

```bash
git add src/perception/rtk_fgo_localizer/CMakeLists.txt \
  src/perception/rtk_fgo_localizer/package.xml \
  src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/rtk_quality.hpp \
  src/perception/rtk_fgo_localizer/src/rtk_quality.cpp \
  src/perception/rtk_fgo_localizer/test/test_rtk_quality.cpp
git commit -m "Add RTK FGO localizer package skeleton"
```

---

## Chunk 2: State Machine And Correction Smoother

### Task 2: Add Localization State Machine

**Files:**
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/state_machine.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/state_machine.cpp`
- Create: `src/perception/rtk_fgo_localizer/test/test_state_machine.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`

- [ ] **Step 1: Write failing state-machine tests**

Test:

```cpp
TEST(StateMachine, StartsLocalOnly) {
  rtk_fgo_localizer::LocalizationStateMachine sm;
  EXPECT_EQ(sm.state(), rtk_fgo_localizer::LocalizationState::LocalOnly);
}

TEST(StateMachine, RequiresPersistentFixedBeforeRecovery) {
  rtk_fgo_localizer::LocalizationStateMachine sm;
  for (int i = 0; i < 7; ++i) {
    sm.update(rtk_fgo_localizer::RtkGateMode::StrongCandidate, true);
  }
  EXPECT_EQ(sm.state(), rtk_fgo_localizer::LocalizationState::RtkCandidate);
  sm.update(rtk_fgo_localizer::RtkGateMode::StrongCandidate, true);
  EXPECT_EQ(sm.state(), rtk_fgo_localizer::LocalizationState::RtkRecovery);
}

TEST(StateMachine, DegradesWhenLockedRtkDrops) {
  rtk_fgo_localizer::LocalizationStateMachine sm;
  for (int i = 0; i < 9; ++i) {
    sm.update(rtk_fgo_localizer::RtkGateMode::StrongCandidate, true);
  }
  sm.markRecoveryCommitted();
  EXPECT_EQ(sm.state(), rtk_fgo_localizer::LocalizationState::RtkLocked);
  sm.update(rtk_fgo_localizer::RtkGateMode::Rejected, false);
  EXPECT_EQ(sm.state(), rtk_fgo_localizer::LocalizationState::RtkDegraded);
}
```

- [ ] **Step 2: Run and verify failure**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
```

Expected: FAIL for missing state machine.

- [ ] **Step 3: Implement the state machine**

Implement enum values:

```cpp
enum class LocalizationState {
  LocalOnly,
  RtkCandidate,
  RtkLocked,
  RtkDegraded,
  RtkRecovery,
  FaultHold,
};
```

Rules:

- `LocalOnly` + strong candidate samples -> `RtkCandidate`
- `RtkCandidate` reaches `recovery_min_samples` -> `RtkRecovery`
- `RtkRecovery` commits -> `RtkLocked`
- `RtkLocked` + rejected gate -> `RtkDegraded`
- repeated invalid samples -> `LocalOnly`
- impossible jumps or graph commit rejection -> `FaultHold`

- [ ] **Step 4: Run and verify pass**

Run the same colcon test command. Expected: PASS.

### Task 3: Add Correction Smoother

**Files:**
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/correction_smoother.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/correction_smoother.cpp`
- Create: `src/perception/rtk_fgo_localizer/test/test_correction_smoother.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`

- [ ] **Step 1: Write failing smoother tests**

Test:

```cpp
TEST(CorrectionSmoother, LimitsTranslationStep) {
  rtk_fgo_localizer::CorrectionSmoother smoother(0.15, 0.3 * M_PI / 180.0);
  auto out = smoother.step({1.0, 0.0, 0.0});
  EXPECT_NEAR(out.dx, 0.15, 1e-6);
  EXPECT_NEAR(out.dy, 0.0, 1e-6);
}

TEST(CorrectionSmoother, LimitsYawStep) {
  rtk_fgo_localizer::CorrectionSmoother smoother(0.15, 0.3 * M_PI / 180.0);
  auto out = smoother.step({0.0, 0.0, 10.0 * M_PI / 180.0});
  EXPECT_NEAR(out.dyaw, 0.3 * M_PI / 180.0, 1e-6);
}
```

- [ ] **Step 2: Run and verify failure**

Expected: FAIL for missing smoother.

- [ ] **Step 3: Implement minimal smoother**

Use a simple bounded delta structure:

```cpp
struct Correction2D {
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
};
```

Clamp translation vector norm and yaw independently. Normalize yaw with `atan2(sin, cos)`.

- [ ] **Step 4: Run all chunk tests**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
colcon test-result --verbose
```

Expected: PASS.

- [ ] **Step 5: Commit chunk 2**

```bash
git add src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/state_machine.hpp \
  src/perception/rtk_fgo_localizer/src/state_machine.cpp \
  src/perception/rtk_fgo_localizer/test/test_state_machine.cpp \
  src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/correction_smoother.hpp \
  src/perception/rtk_fgo_localizer/src/correction_smoother.cpp \
  src/perception/rtk_fgo_localizer/test/test_correction_smoother.cpp \
  src/perception/rtk_fgo_localizer/CMakeLists.txt
git commit -m "Add RTK FGO gating state machine"
```

---

## Chunk 3: GTSAM Window Core

### Task 4: Add Yaw-Only RTK Heading Factor

**Files:**
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/yaw_factor.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/yaw_factor.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`
- Test: `src/perception/rtk_fgo_localizer/test/test_fgo_graph.cpp`

- [ ] **Step 1: Write failing yaw factor test**

Add a gtest that creates a Pose3 with yaw `10 deg`, evaluates a yaw measurement of `12 deg`, and expects a residual near `-2 deg` or `2 deg` depending on residual convention. Document the convention in the test name.

- [ ] **Step 2: Run and verify failure**

Expected: FAIL for missing factor.

- [ ] **Step 3: Implement `YawFactor`**

Implement a custom factor derived from:

```cpp
gtsam::NoiseModelFactor1<gtsam::Pose3>
```

The factor should:

- extract yaw from `pose.rotation().yaw()`
- compute normalized residual `yaw - measured_yaw`
- provide numerical derivative first if analytic derivative is too risky
- use a 1D diagonal noise model

- [ ] **Step 4: Run and verify pass**

Expected: PASS.

### Task 5: Add Sliding-Window FGO Graph API

**Files:**
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/fgo_graph.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/fgo_graph.cpp`
- Modify: `src/perception/rtk_fgo_localizer/test/test_fgo_graph.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`

- [ ] **Step 1: Write failing graph tests**

Add tests:

```cpp
TEST(FgoGraph, AddsInitialStateAndReturnsEstimate) {
  rtk_fgo_localizer::FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  auto estimate = graph.latestEstimate();
  ASSERT_TRUE(estimate.has_value());
}

TEST(FgoGraph, RejectsShadowCommitForHugeRtkCorrection) {
  rtk_fgo_localizer::FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addFastLioBetween(1.0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  auto result = graph.tryShadowRtkCommit(1.0, gtsam::Point3(50.0, 0.0, 0.0), 0.02);
  EXPECT_FALSE(result.committed);
}

TEST(FgoGraph, CommitsSmallRtkRecovery) {
  rtk_fgo_localizer::FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addFastLioBetween(1.0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  auto result = graph.tryShadowRtkCommit(1.0, gtsam::Point3(1.05, 0.0, 0.0), 0.02);
  EXPECT_TRUE(result.committed);
}
```

- [ ] **Step 2: Run and verify failure**

Expected: FAIL for missing graph API.

- [ ] **Step 3: Implement minimal graph with full-state keys**

Use GTSAM keys:

```cpp
using gtsam::symbol_shorthand::X;  // Pose3
using gtsam::symbol_shorthand::V;  // Vector3 velocity
using gtsam::symbol_shorthand::B;  // imuBias::ConstantBias
```

First implementation requirements:

- Insert pose, velocity, and bias states for each keyframe.
- Add a prior on the initial pose, velocity, and bias.
- Add FAST-LIO relative pose as `BetweenFactor<Pose3>`.
- Add wheel planar odometry as a weak `BetweenFactor<Pose3>` with z/roll/pitch loose covariance.
- Add RTK position as `GPSFactor`.
- Add RTK heading using `YawFactor`.
- Keep IMU preintegration API present, but allow tests to pass with no IMU samples.

Do not publish or broadcast anything from this core class.

- [ ] **Step 4: Add IMU preintegration support**

Add methods:

```cpp
void addImuSample(double stamp_s, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);
void closeImuFactorBetween(size_t from_index, size_t to_index);
```

Use GTSAM IMU preintegration classes. If `CombinedImuFactor` is available in the Jetson GTSAM install, use it. If not, fall back to `ImuFactor` plus bias between factor and document the fallback in the code comment.

- [ ] **Step 5: Implement fixed-window pruning**

Keep at most `max_states`. When states leave the window, preserve a prior on the first remaining state from the latest estimate. Do not attempt full dense marginalization in the first version unless GTSAM marginal covariance is stable in tests.

- [ ] **Step 6: Run graph tests**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
colcon test-result --verbose
```

Expected: PASS.

- [ ] **Step 7: Commit chunk 3**

```bash
git add src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/yaw_factor.hpp \
  src/perception/rtk_fgo_localizer/src/yaw_factor.cpp \
  src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/fgo_graph.hpp \
  src/perception/rtk_fgo_localizer/src/fgo_graph.cpp \
  src/perception/rtk_fgo_localizer/test/test_fgo_graph.cpp \
  src/perception/rtk_fgo_localizer/CMakeLists.txt
git commit -m "Add RTK FGO sliding window graph core"
```

---

## Chunk 4: ROS Node Shadow Mode

### Task 6: Add Timestamped Topic Buffers

**Files:**
- Create: `src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/topic_buffers.hpp`
- Create: `src/perception/rtk_fgo_localizer/src/topic_buffers.cpp`
- Test: add buffer tests to `src/perception/rtk_fgo_localizer/test/test_fgo_graph.cpp` or create `test_topic_buffers.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`

- [ ] **Step 1: Write failing buffer tests**

Test that a buffer:

- keeps only samples inside `max_age_s`
- returns the closest sample within tolerance
- returns no sample outside tolerance

- [ ] **Step 2: Implement generic timestamped buffer**

Use a template class for simple sample structs. Keep it ROS-free.

- [ ] **Step 3: Run tests and commit**

Run colcon tests and commit explicit files:

```bash
git commit -m "Add RTK FGO sensor sample buffers"
```

### Task 7: Add `rtk_fgo_node`

**Files:**
- Create: `src/perception/rtk_fgo_localizer/src/rtk_fgo_node.cpp`
- Modify: `src/perception/rtk_fgo_localizer/CMakeLists.txt`
- Modify: `src/perception/rtk_fgo_localizer/package.xml`

- [ ] **Step 1: Add node parameters**

Declare:

```yaml
topics.fastlio_odom
topics.imu
topics.wheel_odom
topics.fix
topics.heading
topics.rtk_status
topics.raw_nmea
frames.map
frames.base_link
frames.odom_fgo
publish_tf
nav2_use_fgo
window.duration_s
window.keyframe_rate_hz
window.max_states
rtk_gating.*
correction_smoother.*
```

- [ ] **Step 2: Add subscriptions**

Subscribe to:

```text
/fastlio2/lio_odom
/livox/imu
/odom_CBoar
/fix
/heading
/rtk/status
/rtk/nmea_sentence
```

All topic names must come from parameters.

- [ ] **Step 3: Add publishers**

Publish:

```text
/rtk_fgo/odom
/rtk_fgo/path
/rtk_fgo/status
/rtk_fgo/rtk_gate
/rtk_fgo/correction_status
/rtk_fgo/factor_diagnostics
```

For `factor_diagnostics`, prefer `diagnostic_msgs/msg/DiagnosticArray`.

- [ ] **Step 4: Implement shadow output behavior**

Rules:

- If graph has no estimate, publish no `/rtk_fgo/odom`.
- If graph estimate exists, publish it under `map` frame with child `base_link`.
- If `publish_tf=false`, never broadcast TF.
- If `publish_tf=true`, broadcast only `map -> odom_fgo`, not production `map -> odom`.
- If RTK quality is invalid, status must report `LOCAL_ONLY` or degraded state.

- [ ] **Step 5: Compile**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_fgo_localizer --symlink-install --parallel-workers 1
source install/setup.bash
```

Expected: build succeeds.

- [ ] **Step 6: Commit chunk 4**

```bash
git add src/perception/rtk_fgo_localizer/src/rtk_fgo_node.cpp \
  src/perception/rtk_fgo_localizer/include/rtk_fgo_localizer/topic_buffers.hpp \
  src/perception/rtk_fgo_localizer/src/topic_buffers.cpp \
  src/perception/rtk_fgo_localizer/CMakeLists.txt \
  src/perception/rtk_fgo_localizer/package.xml
git commit -m "Add RTK FGO shadow ROS node"
```

---

## Chunk 5: Experimental Launch And Runtime Recording

### Task 8: Add Parameter Profile And Launch

**Files:**
- Create: `src/bringup/config/rtk_fgo.yaml`
- Create: `src/bringup/launch/system_tightly_coupled.launch.py`
- Modify: `src/bringup/package.xml`
- Modify: `Makefile`
- Modify: `scripts/launch_with_logs.sh`

- [ ] **Step 1: Add `rtk_fgo.yaml`**

Use defaults:

```yaml
/rtk_fgo_localizer:
  ros__parameters:
    publish_tf: false
    nav2_use_fgo: false
    topics.fastlio_odom: /fastlio2/lio_odom
    topics.imu: /livox/imu
    topics.wheel_odom: /odom_CBoar
    topics.fix: /fix
    topics.heading: /heading
    topics.rtk_status: /rtk/status
    topics.raw_nmea: /rtk/nmea_sentence
```

Add the window, factor, gate, and smoother parameters from `docs-EN/knowledge/rtk_fgo.md`.

- [ ] **Step 2: Add launch file**

`system_tightly_coupled.launch.py` should include:

- `system_explore.launch.py` with RViz optional
- `um982_rtk.launch.py`
- `rtk_fgo_localizer` node
- rosbag record process for source topics and `/rtk_fgo/*`

Do not remap Nav2 to FGO output in this task.

- [ ] **Step 3: Wire launch script**

In `scripts/launch_with_logs.sh`, add mode:

```bash
tightly-coupled) LAUNCH_FILE="system_tightly_coupled.launch.py" ;;
```

Pass `FYP_RTK_PARAMS_FILE` as `rtk_params_file` for this mode, same as other GPS modes.

- [ ] **Step 4: Add Make targets**

Add:

```make
launch-tightly-coupled:
	bash scripts/launch_with_logs.sh tightly-coupled
```

Add `rtk_fgo_localizer` to `build-perception`.

- [ ] **Step 5: Build launch dependencies**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_fgo_localizer bringup --symlink-install --parallel-workers 1
source install/setup.bash
```

Expected: build succeeds.

- [ ] **Step 6: Commit chunk 5**

```bash
git add src/bringup/config/rtk_fgo.yaml \
  src/bringup/launch/system_tightly_coupled.launch.py \
  src/bringup/package.xml \
  Makefile \
  scripts/launch_with_logs.sh
git commit -m "Add RTK FGO experimental launch mode"
```

---

## Chunk 6: Documentation And Verification

### Task 9: Update Bilingual Documentation

**Files:**
- Modify: `docs-EN/knowledge/rtk_fgo.md`
- Modify: `docs-CN/knowledge/rtk_fgo.md`
- Modify: `docs-EN/architecture.md`
- Modify: `docs-CN/architecture.md`
- Modify: `docs-EN/commands.md`
- Modify: `docs-CN/commands.md`
- Modify: `docs-EN/devlog/2026-06.md`
- Modify: `docs-CN/devlog/2026-06.md`

- [ ] **Step 1: Update knowledge docs**

Change "planned" language only for pieces that now exist. Keep guarded Nav2 as future work unless implemented.

- [ ] **Step 2: Update architecture docs**

Add a clearly labeled experimental mode section. State:

- it is shadow by default
- it does not replace production `map -> odom`
- it publishes `/rtk_fgo/*`

- [ ] **Step 3: Update commands docs**

Add:

```bash
make build-perception
make launch-tightly-coupled
```

Mark it as experimental shadow mode.

- [ ] **Step 4: Update devlog**

Use File / Change / Reason / Effect. Include tests run and note whether Jetson build was performed.

- [ ] **Step 5: Commit docs**

```bash
git add docs-EN/knowledge/rtk_fgo.md docs-CN/knowledge/rtk_fgo.md \
  docs-EN/architecture.md docs-CN/architecture.md \
  docs-EN/commands.md docs-CN/commands.md \
  docs-EN/devlog/2026-06.md docs-CN/devlog/2026-06.md
git commit -m "Document RTK FGO experimental shadow mode"
```

### Task 10: Final Verification

**Files:**
- All files changed in this plan.

- [ ] **Step 1: Run unit tests**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rtk_fgo_localizer --event-handlers console_direct+
colcon test-result --verbose
```

Expected: PASS.

- [ ] **Step 2: Build selected packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_fgo_localizer bringup --symlink-install --parallel-workers 1
source install/setup.bash
```

Expected: PASS.

- [ ] **Step 3: Check launch syntax without starting hardware**

```bash
python3 -m py_compile src/bringup/launch/system_tightly_coupled.launch.py
```

Expected: PASS.

- [ ] **Step 4: Check docs and whitespace**

```bash
git diff --check
```

Expected: no output.

- [ ] **Step 5: Confirm no production launch behavior changed**

Review diffs and confirm no functional edits to:

```text
src/bringup/launch/system_gps_corridor.launch.py
src/bringup/launch/system_explore_gps.launch.py
src/bringup/launch/system_nav_gps.launch.py
src/bringup/config/nav2_explore.yaml
src/bringup/config/nav2_gps.yaml
```

- [ ] **Step 6: Push branch**

```bash
git status --short --branch
git push origin Tightly-coupled
```

Expected: branch pushed. Do not open a PR until the user asks or the branch is ready for review.

---

## Execution Notes

- Use explicit `git add <path>...` only.
- Never use `git add -A` or `git add .`.
- Every Jetson or ROS build command must include `--parallel-workers 1`.
- Re-source `install/setup.bash` after every build.
- Do not commit runtime data, bags, CORS credentials, or files under `src/third_party/`.
- If GTSAM IMU APIs differ on the Jetson, keep the compile fallback small and document the exact API used in the devlog.
