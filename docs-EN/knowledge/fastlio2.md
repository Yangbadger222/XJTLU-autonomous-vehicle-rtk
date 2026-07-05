# FASTLIO2 Algorithm In-Depth

## Current Location in the Project

- Package name: `fastlio2`
- Current primary outputs (when height filtering is enabled, point cloud outputs are height-filtered before publishing; see below):
  - `/fastlio2/lio_odom`
  - `/fastlio2/body_cloud`
  - `/fastlio2/body_cloud_nav2_obstacles` (optional taller Nav2 obstacle cloud)
- Current primary entry point:

```bash
ros2 launch fastlio2 lio_no_rviz.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml
```

- Current parameter source is primarily `src/bringup/config/master_params.yaml`; `lio_no_rviz.py` retains legacy `fastlio2/config/lio.yaml` fallback capability.
- The downstream `pgo` node directly consumes `/fastlio2/lio_odom` and `/fastlio2/body_cloud`.
- This document mainly explains the algorithm principles; for the full vehicle pipeline, also refer to `docs/architecture.md` and `docs/knowledge/pgo.md`.

## Publish Cloud Pre-Height-Filtering (2026-04-01)

FAST-LIO2 now includes publish-time point cloud height filtering in `lio_node.cpp` (commit `f619fa6`):

- Before publishing `body_cloud` and `world_cloud` to downstream consumers (PGO, localizer, etc.), points are filtered by their relative height along gravity with respect to the body origin
- Filter window controlled via `master_params.yaml`:
  - `publish_cloud_height_filter_enabled: true` (toggle)
  - `publish_cloud_min_z: -0.33` (minimum relative height)
  - `publish_cloud_max_z: 0.30` (maximum relative height)
- Relative height calculation: `relative_height = world_cloud.z - body_origin_z_in_world`
- Effect: ground-level low points and above-vehicle-height noise are eliminated at the source; downstream PGO/localizer consumers keep using the stable low-window cloud
- This filter does not affect FAST-LIO2's internal SLAM mapping and state estimation; it only affects the point clouds published to external consumers
- Travel/Nav2 has a separate obstacle cloud:
  - `nav2_obstacle_cloud_enabled: true`
  - `nav2_obstacle_cloud_min_z: -0.30`
  - `nav2_obstacle_cloud_max_z: 1.20`
  - Publishes `/fastlio2/body_cloud_nav2_obstacles`, which `nav2_cloud_retime.py` republishes as `/fastlio2/body_cloud_nav2`
  - This topic lets tables, table edges, and other obstacles above 0.30 m enter the local costmap without changing the original `/fastlio2/body_cloud`

## Nav2-Dedicated Obstacle Cloud (2026-07-09)

RTK corridor now branches an additional `/fastlio2/body_cloud_nav2_obstacles` stream at the FAST-LIO2 publish layer, used only by the Nav2 local costmap:

- `/fastlio2/body_cloud` keeps the low `[-0.33, 0.30]m` publish window so PGO / LIO downstream consumers retain the existing lightweight structure-point input
- `/fastlio2/body_cloud_nav2_obstacles` uses a wider `[-0.20, 1.20]m` height window, covering standing pedestrians, upper-body returns, and common outdoor barriers
- This Nav2-only cloud does not feed PGO and does not change FAST-LIO2 internal state estimation; it isolates obstacle perception from the PGO/LIO cloud tune
- Filtering runs only when the topic has subscribers, avoiding extra CPU cost outside corridor / Nav2 runs

If future field tests still show unstable pedestrian marking, inspect whether `/fastlio2/body_cloud_nav2_obstacles` contains points, whether the local costmap STVL voxel map marks them, and whether the Livox blind zone / mounting angle is the limiting factor before widening the PGO-facing `/fastlio2/body_cloud`.

## LiDAR / IMU Sync Guard (2026-07-06)

FAST-LIO2 now drops a LiDAR frame before the IESKF update when its synchronized package contains fewer than `min_imu_samples_per_lidar` IMU samples. The current corridor acceptance value is:

- `min_imu_samples_per_lidar: 3`

Reason: the 2026-07-06 21:02 corridor bag showed the robot stationary while FAST-LIO2 processed packages such as `0 IMU samples, 925 LIDAR points` and `1 IMU samples, 1086 LIDAR points`; the odom then jumped tens of meters before Nav2 received the route goal. Dropping under-constrained LiDAR frames is safer than allowing a bad IMU/LiDAR sync window to advance the filter.

The guard has a second layer for low-feature outdoor startup: if the IESKF update has no valid LiDAR correction, FAST-LIO2 restores the pre-prediction state, skips incremental map insertion, and does not publish TF/odom for that frame. This prevents IMU-only prediction from being exposed as a valid localization estimate when the LiDAR matcher reports `NO Effective Points`.

## Outdoor Point-Structure Retention Profile (2026-07-08)

RTK corridor currently uses a CPU-conscious outdoor structure-retention profile:

- `lidar_filter_num: 4`
- `lidar_max_range: 25.0`

The old `lidar_filter_num: 6` and `lidar_max_range: 15.0` profile was cheaper, but in plaza-like areas it can discard distant walls, trees, poles, and other matchable structure too early, increasing the chance of `NO Effective Points` and degeneracy regularization. The `4/25m` profile is a compromise: it keeps more structure than the old profile without immediately dropping to `3`, which could roughly double the point-processing load on the Jetson.

If future bags show CPU headroom while `/fastlio2/degeneracy` still reports frequent degeneracy, try `lidar_filter_num: 3`; if IMU/LiDAR sync windows start dropping samples or FAST-LIO2 latency rises, prefer returning to `4` or reducing `lidar_max_range`.

## 1. Odom (Odometry) Data Interpretation

The odom output from FASTLIO2 (`nav_msgs/Odometry`) contains two core data components:

- **`pose.pose.position`**: `(x, y, z)` -- vehicle position in the world coordinate frame
- **`pose.pose.orientation`**: `(x, y, z, w)` -- vehicle attitude in the world coordinate frame (quaternion representation)

These data are not raw measurements from any single sensor. Instead, they are optimal estimates obtained by fusing IMU forward integration with LiDAR point cloud matching through **IESKF (Iterated Error-State Kalman Filter)**. Specifically, the IMU provides high-frequency (200Hz) acceleration and angular velocity data for short-term prediction, while the LiDAR provides per-frame (10Hz) point cloud data to correct accumulated errors.

---

## 2. Position Data Computation

### 2.1 Initial State

At system startup, the initial state is:

```
Position: t_wi = [0, 0, 0]^T
Velocity: v = [0, 0, 0]^T
Rotation: R_wi = I (identity matrix)
```

### 2.2 IMU Data Arrival

When the first IMU data frame arrives, it contains:

- **acc**: accelerometer measurement (in body frame)
- **gyro**: gyroscope measurement (in body frame)
- **dt**: sampling interval, approximately **5ms** (200Hz)

### 2.3 Computing Acceleration in the World Frame

The IMU-measured acceleration `acc` is in the body frame and needs to be transformed to the world frame:

```
a_world = R_wi × (acc - ba) + g
```

Where:
- **`R_wi`**: rotation matrix at the current time (body -> world rotation)
- **`acc`**: raw accelerometer measurement
- **`ba`**: accelerometer bias, which drifts slowly over time and is estimated online by the filter
- **`g`**: gravity acceleration vector `[0, 0, -9.81]^T`

### 2.4 Velocity and Position Update

After obtaining the acceleration in the world frame, velocity and position are updated through numerical integration:

```
v_new = v + a_world × dt
t_wi_new = t_wi + v × dt + 0.5 × a_world × dt²
```

Each time an IMU data frame arrives (every 5ms), the above integration is performed once, continuously accumulating to produce the current position estimate.

---

## 3. Computing the Rotation Matrix R_wi

Updating the rotation matrix `R_wi` is one of the core operations in FASTLIO2, broken down into the following 7 steps:

### Step 1: Remove Gyroscope Bias

```
ω_true = gyro - bg
```

- **`gyro`**: raw gyroscope measurement (rad/s)
- **`bg`**: gyroscope bias, estimated online by IESKF
- **`ω_true`**: true angular velocity

### Step 2: Compute Angular Increment

```
Δθ = ω_true × dt
```

- **`dt`**: IMU sampling interval (approximately 5ms)
- **`Δθ`**: angular change during this time interval (3D vector, in radians)

### Step 3: Exponential Map (Rotation Vector -> Rotation Matrix)

```
ΔR = Sophus::SO3d::exp(Δθ).matrix()
```

The Sophus library's `SO3::exp()` function maps the angular increment vector to a rotation matrix. This is the **exponential map of the Lie group SO(3)**:
- Input: 3D rotation vector `Δθ`
- Output: 3x3 rotation matrix `ΔR`
- The internal implementation is equivalent to the Rodrigues formula

### Step 4: Update the Rotation Matrix

```
R_wi = R_wi × ΔR
```

The incremental rotation is right-multiplied onto the current rotation matrix, achieving cumulative rotation update.

**Error analysis**: Each integration step introduces small errors (gyroscope noise, discretization error), which accumulate over time. If relying solely on IMU integration, the rotation estimate would diverge quickly. Therefore, LiDAR data is needed for correction (Steps 5-6).

### Step 5: LiDAR Data Residual Computation

When a LiDAR point cloud frame arrives, the point cloud is matched against the existing map to compute residuals, which are used to correct the accumulated IMU integration errors.

**Point cloud coordinate transformation**:

Points captured by the LiDAR are in the LiDAR coordinate frame and need to be transformed to the world frame:

```
p_world = R_wi × p_lidar + t_wi
```

- **`p_lidar`**: point coordinates in the LiDAR frame
- **`R_wi`**: currently estimated rotation matrix
- **`t_wi`**: currently estimated position
- **`p_world`**: transformed world coordinates

**ikd-Tree nearest neighbor search**:

For each transformed point, the 5 nearest neighbors are searched in the ikd-Tree (incremental KD-tree) for local plane fitting.

**Plane fitting**:

Least-squares plane fitting is performed on the 5 nearest neighbors, yielding the plane equation:

```
n^T × p + d = 0
```

Where `n` is the plane normal vector and `d` is the distance parameter.

**Residual computation**:

The transformed point is substituted into the plane equation to obtain the point-to-plane distance as the residual:

```
residual = n^T × p_world + d
```

If the current `R_wi` and `t_wi` are very accurate, the residual should be close to 0. Larger residuals indicate less accurate pose estimates.

### Step 6: IESKF Pose Correction

After collecting residuals from all valid points, the optimal correction is solved through **Iterated Error-State Kalman Filter (IESKF)**.

**Jacobian matrix H**:

For each residual, partial derivatives with respect to state variables (position, rotation, velocity, biases, etc.) are computed to form the observation Jacobian matrix `H`.

**Normal equation**:

```
(H^T × H) × Δx = H^T × residuals
```

**Pseudo-inverse solution**:

```
Δx = (H^T × H)^(-1) × H^T × residuals
```

Where `Δx` contains corrections for rotation, position, velocity, biases, and other state variables.

**Rotation correction**:

```
R_wi = R_wi × Exp(Δθ_correction)
```

- **`Δθ_correction`**: rotation correction vector extracted from `Δx`
- **`Exp()`**: SO(3) exponential map

### Step 7: Final Rotation Matrix Update

After multiple iterations (typically 3-5), convergence is reached. The final `R_wi` is the optimal rotation estimate at the current time, and `t_wi` has also been corrected. These results are packaged into the odom message for publication.

---

## 4. Quaternion Data Interpretation

### 4.1 Meaning of Quaternions

The `orientation` in odom represents rotation as a quaternion `(x, y, z, w)`, where:

- **`w`**: scalar part, equal to `cos(θ/2)`, where `θ` is the rotation angle
- **`(x, y, z)`**: vector part, equal to `sin(θ/2) × (nx, ny, nz)`, where `(nx, ny, nz)` is the unit vector of the rotation axis

Quaternions satisfy the constraint: `x² + y² + z² + w² = 1`

### 4.2 Rodrigues Rotation Formula

Given a rotation axis `n` (unit vector) and rotation angle `θ`, the formula for rotating a vector `v`:

```
v_rotated = v × cos(θ) + (n × v) × sin(θ) + n × (n · v) × (1 - cos(θ))
```

This is the fundamental formula for understanding rotations; both quaternions and rotation matrices can be derived from it.

### 4.3 Quaternion -> Rotation Matrix

```
R = | 1-2(y²+z²)   2(xy-wz)     2(xz+wy)   |
    | 2(xy+wz)     1-2(x²+z²)   2(yz-wx)   |
    | 2(xz-wy)     2(yz+wx)     1-2(x²+y²) |
```

### 4.4 Rotation Matrix -> Quaternion

Reverse conversion formula:

```
w = 0.5 × √(1 + R[0][0] + R[1][1] + R[2][2])
x = (R[2][1] - R[1][2]) / (4w)
y = (R[0][2] - R[2][0]) / (4w)
z = (R[1][0] - R[0][1]) / (4w)
```

Note: When `w` is close to 0, alternative formula branches must be used to avoid numerical instability.

### 4.5 Euler Angle Representation (ZYX Order)

Quaternions can also be converted to Euler angles. FASTLIO2 uses ZYX order (i.e., first rotate around Z-axis for yaw, then around Y-axis for pitch, finally around X-axis for roll):

- **Yaw**: rotation around the Z-axis, representing vehicle heading
- **Pitch**: rotation around the Y-axis, representing forward/backward tilt
- **Roll**: rotation around the X-axis, representing lateral tilt

**Gimbal Lock**:

When Pitch approaches +/-90 degrees, the rotation axes of Yaw and Roll coincide, causing one degree of freedom to be lost. This is the gimbal lock problem. This is why FASTLIO2 internally uses rotation matrices or quaternions rather than Euler angles to represent rotations -- quaternions and rotation matrices do not suffer from gimbal lock.

---

## 5. Detailed FASTLIO2 Algorithm Pipeline

### Stage 0: System Initialization

After startup, the system does not begin localization immediately. Instead, it first collects **20 IMU data frames** for initialization:

1. **Collect stationary IMU data**: Assuming the vehicle is stationary, collect approximately 20 frames of acceleration and angular velocity data
2. **Gravity estimation**: Average the acceleration readings to estimate the gravity direction `g_est`
3. **Initial bias estimation**: The mean angular velocity serves as the initial gyroscope bias `bg_init`
4. **Initial covariance setup**: Set the initial uncertainty for state variables
5. **State initialization**: `R_wi = I`, `t_wi = 0`, `v = 0`

After initialization is complete, the system enters normal operation.

### Stage 1: Data Reception and Synchronization (SyncPackage)

```
SyncPackage() → Align IMU data and LiDAR point clouds by timestamp
```

1. Receive LiDAR point cloud (10Hz), record its start and end timestamps
2. Collect all IMU data within that time window (200Hz, approximately 20 frames)
3. Package the paired {IMU sequence + LiDAR point cloud} as one processing unit
4. Ensure the IMU data time coverage spans the entire LiDAR scan period

### Stage 2: IMU Forward Integration + Pose Sequence Generation

For each IMU data frame collected in Stage 1, forward integration is performed frame by frame:

1. **Bias removal**: `ω_true = gyro - bg`, `a_true = acc - ba`
2. **Rotation update**: `R_wi = R_wi × Exp(ω_true × dt)`
3. **Acceleration transformation**: `a_world = R_wi × a_true + g`
4. **Velocity update**: `v = v + a_world × dt`
5. **Position update**: `t_wi = t_wi + v × dt + 0.5 × a_world × dt²`
6. **Save intermediate poses**: Each IMU frame's corresponding `{R_wi, t_wi}` is saved as a pose sequence

The pose sequence generated in this step has two purposes:
- The last pose serves as the IESKF prediction
- Intermediate poses are used for point cloud de-distortion in the next step

### Stage 3: Point Cloud De-distortion

**Why is de-distortion needed?**

A LiDAR scan takes approximately 100ms to complete one frame, during which the vehicle is continuously moving. Therefore, different points within the same frame are captured at different vehicle poses. Without correction, the point cloud would exhibit motion blur distortion.

**De-distortion steps**:

1. **Obtain point timestamps**: Each point carries a time offset `t_offset` relative to the frame start
2. **Look up corresponding pose**: Based on `t_offset`, interpolate within the pose sequence from Stage 2 to get the vehicle pose `{R_i, t_i}` at the time the point was captured
3. **Transform to a unified coordinate frame**: Transform each point from its capture-time coordinate frame to the frame-end coordinate frame
4. **Formula**: `p_corrected = R_end^(-1) × (R_i × p_lidar_i + t_i - t_end)`
5. **Output**: All points are unified to the same time instant's coordinate frame, eliminating motion distortion

### Stage 4: IESKF Iterative Optimization (Core)

This is the most critical step in FASTLIO2, correcting accumulated IMU integration errors through iterative optimization:

#### Sub-step 1: Transform Point Cloud to the World Frame

```
p_world = R_wi × p_lidar + t_wi
```

Using the current best pose estimate (the first iteration uses the IMU integration prediction), transform the de-distorted point cloud to the world frame.

#### Sub-step 2: KNN Nearest Neighbor Search

For each transformed point, search for the **5 nearest neighbors** in the ikd-Tree. The ikd-Tree is an incremental KD-tree that supports efficient dynamic insertion and deletion operations.

#### Sub-step 3: Plane Fitting

Least-squares plane fitting on the 5 nearest neighbors:

```
n^T × p + d = 0
```

If the fitting quality is insufficient (e.g., neighbors are too scattered), the point is marked as invalid and excluded from subsequent optimization.

#### Sub-step 4: Compute Jacobian Matrix

For the residual `r = n^T × (R_wi × p_lidar + t_wi) + d`, compute partial derivatives with respect to state variables:

- Partial derivative with respect to position `t_wi`: `∂r/∂t = n^T`
- Partial derivative with respect to rotation `R_wi`: `∂r/∂θ = -n^T × R_wi × [p_lidar]×` (where `[p_lidar]×` is the skew-symmetric matrix)

These form the observation Jacobian matrix `H`.

#### Sub-step 5: Solve for the Correction

```
K = P × H^T × (H × P × H^T + R_noise)^(-1)
Δx = K × residuals
```

Or equivalently via the normal equation pseudo-inverse:

```
Δx = (H^T × R_noise^(-1) × H + P^(-1))^(-1) × H^T × R_noise^(-1) × residuals
```

#### Sub-step 6: State Update

```
R_wi = R_wi × Exp(Δθ)
t_wi = t_wi + Δt
v = v + Δv
bg = bg + Δbg
ba = ba + Δba
```

#### Sub-step 7: Convergence Check

Check whether the norm of the correction `Δx` is below the threshold:
- If `||Δx|| < threshold`, converged -- exit the iteration
- Otherwise, return to Sub-step 1 and re-perform point cloud transformation and residual computation with the updated pose
- Typically **3-5 iterations** are sufficient for convergence

**Key insight: Why only optimize the final pose?**

FASTLIO2 does not go back to optimize intermediate IMU integration poses; it only optimizes the pose at the LiDAR frame end time. The reasons are:
- The relative rotation accuracy of IMU integration is very high (gyroscope drift is minimal over short time spans)
- The main errors come from long-term accumulation, which can be effectively eliminated by correcting the final pose
- Per-frame correction is far more efficient than global optimization
- The corrected bias estimates `ba` and `bg` propagate to the next frame's IMU integration, indirectly improving subsequent integration accuracy

### Stage 5: Map Update

After optimization converges, the current frame's point cloud is added to the global map:

1. **ikd-Tree pruning**: Delete points too far from the current position (maintaining local map size)
2. **Incremental addition**: Incrementally insert the current frame's de-distorted points (transformed to the world frame) into the ikd-Tree
3. The ikd-Tree automatically rebalances to maintain query efficiency

### Stage 6: Result Publication

1. **TF publication**: Publish the `odom -> base_link` coordinate transform
2. **Odom publication**: Publish the `nav_msgs/Odometry` message containing position and attitude
3. **Point cloud publication**: Publish the current frame point cloud (in the world frame) and the global map point cloud
4. **Path publication**: Publish the historical trajectory (`nav_msgs/Path`)

---

## 6. Algorithm Core Summary

```
FASTLIO2 = High-frequency IMU integration prediction + Per-frame LiDAR correction + Iterative optimization + Incremental mapping
```

- **High-frequency IMU integration prediction**: 200Hz provides continuous pose estimates, filling the gaps between LiDAR frames
- **Per-frame LiDAR correction**: 10Hz corrects accumulated IMU integration errors through point cloud matching
- **Iterative optimization (IESKF)**: Multiple iterations drive the pose estimate to converge to the optimal value
- **Incremental mapping (ikd-Tree)**: Real-time maintenance of a local map with efficient nearest-neighbor queries

---

## 7. Corridor Mode ESKF Safety Guards (commit `308fe77`)

During GPS corridor on-vehicle testing, FAST-LIO2 local odom was found to diverge after sustained recovery/backup maneuvers (`odom->base_link` single-step jumps of up to 2.43m/0.11s). The following ESKF-level safety guards were deployed:

## 7. Corridor Mode ESKF Safety Guards and Jacobian Bug Fix

### 7.0 Fatal Jacobian Bug Fix (commit `e4945f4`)

**This is the root cause of odom divergence**, taking priority over all safety guards below.

**Bug location**: `lidar_processor.cpp:245`

```cpp
// Before fix (wrong):
hat(state.r_il * laser_p_vec + state.t_wi)   // t_wi = world position, ~50m
// After fix (correct):
hat(state.r_il * laser_p_vec + state.t_il)   // t_il = extrinsic offset, ~0.04m
```

**Root cause**: fork changed original HKU-Mars FAST-LIO2 `SKEW_SYM_MATRX` macro to `Sophus::SO3d::hat()` with wrong variable — `state.t_wi` (IMU world position) instead of `state.t_il` (LiDAR-to-IMU extrinsic translation).

**Mathematical derivation**: point-to-plane residual Jacobian w.r.t. rotation (right perturbation):
```
dh/dtheta = -n^T * R_wi * hat(R_il * p_lidar + t_il)
```
The `hat()` argument is the point position in IMU frame (the rotation "lever arm"), unrelated to world position `t_wi`. Using `t_wi` causes the lever arm to grow linearly with travel distance, amplifying the rotation Jacobian by hundreds of times.

**Impact chain**: wrong Jacobian -> incorrect IESKF rotation correction -> `r_wi` estimate drifts -> accumulated point cloud map rotates with vehicle -> odom divergence

### 7.1 Effective Feature Point Threshold Check

**Files**: `lidar_processor.cpp`, `ieskf.cpp`

When the number of effectively matched feature points in a frame `effect_feat_num < 50`, the entire IESKF update step is skipped. IMU integration continues to provide pose prediction, but low-quality point cloud observations are not used to correct the state.

### 7.2 Invalid Measurement Protection

**File**: `ieskf.cpp`

In the original code, even when observations were invalid (NaN / Inf), they would still overwrite the covariance matrix `m_P`. After modification, invalid measurements no longer update `m_P`, preventing covariance pollution that could cause rapid divergence in subsequent frames.

### 7.3 Covariance Matrix Diagonal Clamping

**File**: `ieskf.cpp`

After each IESKF update, diagonal elements of `m_P` are clamped:
- Lower bound: prevents covariance from becoming too small, which would make the filter overconfident and unresponsive to new observations
- Upper bound: prevents covariance explosion (e.g., in degenerate scenarios)

### 7.4 Degenerate Direction Regularization

**File**: `ieskf.cpp`

Eigenvalue analysis is performed on `shared_data.H` (the information matrix of the observation Jacobian). When an eigenvalue in a certain direction is too small (degenerate direction), a regularization term is injected in that direction to prevent excessively large corrections that would cause pose jumps.

### 7.5 Detection Range Correction

**File**: `lio.yaml`

`det_range: 60 -> 30`. Reduces the effective detection range, decreasing the participation of low-quality distant points in matching and improving near-range matching quality.

### 7.6 Odom Divergence Watchdog

**File**: `gps_route_runner_node.py`

Added odom divergence detection on the runner side:
- Each control cycle queries the `odom->base_link` TF and compares with the previous frame
- Single-step displacement > 0.5m: `WARNING` log
- Single-step displacement > 1.0m: executes `ODOM_DIVERGENCE_ABORT`, safely terminating navigation

On-vehicle verification (session `2026-03-27-18-43-20`): the watchdog detected a 2.45m jump within 22ms and correctly triggered safe termination.
