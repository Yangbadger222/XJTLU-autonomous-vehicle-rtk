# Prior Pointcloud Travel Navigation Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the paused `travel` runtime as an experimental prior-map navigation chain that uses a 2D Nav2 map for planning and a prior PCD point-cloud map for ICP relocalization.

**Architecture:** `system_travel.launch.py` will launch Livox, FAST-LIO2, optional PGO without TF, `localizer`, pointcloud-to-laserscan, serial nodes, Nav2 localization/navigation, and RViz. The `localizer` node owns `map -> odom` after loading the prior `map.pcd`; Nav2 localization publishes the required 2D map while AMCL TF output stays disabled, and Nav2 navigation uses live point-cloud/scan data for local obstacle handling.

**Tech Stack:** ROS 2 Humble launch Python, Nav2, FAST-LIO2, PGO, PCL ICP localizer, ament/pytest launch-config tests, bilingual Markdown documentation.

---

## Files

- Modify: `src/bringup/launch/system_travel.launch.py` - add prior map arguments, include localizer, inject Nav2 map YAML, control TF publisher ownership.
- Modify: `src/bringup/config/nav2_travel.yaml` - make static-map planning coherent and disable AMCL TF ownership when the point-cloud localizer owns `map -> odom`.
- Modify: `src/bringup/package.xml` - add the missing runtime dependency on `localizer`.
- Modify: `src/perception/localizer/launch/localizer_launch.py` - expose reusable launch arguments for standalone ICP relocalization.
- Create: `src/bringup/test/test_system_travel_launch.py` - launch-description tests for the restored travel entry.
- Create or modify docs: `docs-CN/commands.md`, `docs-EN/commands.md`, `docs-CN/architecture.md`, `docs-EN/architecture.md`, `docs-CN/known_issues.md`, `docs-EN/known_issues.md`, `docs-CN/devlog/2026-07.md`, `docs-EN/devlog/2026-07.md`.

## Chunk 1: Launch Contract Tests

### Task 1: Add failing travel launch tests

**Files:**
- Create: `src/bringup/test/test_system_travel_launch.py`

- [ ] **Step 1: Write the failing tests**

```python
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from bringup_launch_test_utils import load_launch_description


def test_travel_launch_exposes_prior_map_arguments():
    launch_description = load_launch_description(
        Path("src/bringup/launch/system_travel.launch.py")
    )
    argument_names = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {"map_yaml", "pcd_map", "use_rviz", "use_pgo"}.issubset(argument_names)


def test_travel_launch_includes_localizer_and_nav2():
    launch_description = load_launch_description(
        Path("src/bringup/launch/system_travel.launch.py")
    )
    nodes = [action for action in launch_description.entities if isinstance(action, Node)]
    includes = [
        action
        for action in launch_description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert any(getattr(node, "node_package", None) == "localizer" for node in nodes)
    assert includes, "Nav2 localization/navigation should still be included through nav2_bringup"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest src/bringup/test/test_system_travel_launch.py -q`

Expected: FAIL because the test file/helper and/or new launch arguments do not exist yet.

### Task 2: Add a tiny launch test helper if needed

**Files:**
- Create: `src/bringup/test/bringup_launch_test_utils.py`

- [ ] **Step 1: Write the helper**

```python
import importlib.util
from pathlib import Path


def load_launch_description(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()
```

- [ ] **Step 2: Re-run tests**

Run: `python3 -m pytest src/bringup/test/test_system_travel_launch.py -q`

Expected: FAIL on missing `map_yaml`, `pcd_map`, `use_rviz`, `use_pgo`, and missing localizer wiring.

## Chunk 2: Travel Launch Wiring

### Task 3: Restore `system_travel.launch.py`

**Files:**
- Modify: `src/bringup/launch/system_travel.launch.py`
- Modify: `src/bringup/package.xml`

- [ ] **Step 1: Add launch arguments**

Add:

```python
map_yaml_arg = DeclareLaunchArgument(
    "map_yaml",
    default_value="",
    description="Absolute path to the Nav2 2D occupancy-grid map YAML.",
)
pcd_map_arg = DeclareLaunchArgument(
    "pcd_map",
    default_value="",
    description="Absolute path to the prior PCD map used by /localizer/relocalize.",
)
use_rviz_arg = DeclareLaunchArgument(
    "use_rviz",
    default_value="true",
    description="Whether to launch RViz with the travel stack.",
)
use_pgo_arg = DeclareLaunchArgument(
    "use_pgo",
    default_value="false",
    description="Whether to launch PGO without TF for visualization/save-map support.",
)
```

- [ ] **Step 2: Inject `map_yaml` into Nav2 parameters**

Use `nav2_common.launch.RewrittenYaml` to set:

```python
param_rewrites={"yaml_filename": LaunchConfiguration("map_yaml")}
```

and pass the rewritten params file to both `nav2_bringup/launch/localization_launch.py` and `nav2_bringup/launch/navigation_launch.py`.

- [ ] **Step 3: Add the localizer node**

Launch `localizer/localizer_node` with:

```python
parameters=[{"config_path": localizer_config_path}]
```

The service `/localizer/relocalize` remains the explicit operator action that loads `pcd_map` and supplies the initial guess.

- [ ] **Step 4: Keep TF ownership singular**

Do not let AMCL or PGO own `map -> odom` in prior point-cloud travel mode. PGO may be launched only when `use_pgo:=true` and should use the non-TF config already present at `pgo/config/pgo_slam.yaml`.

- [ ] **Step 5: Add the missing dependency**

Add `<exec_depend>localizer</exec_depend>` to `src/bringup/package.xml`.

- [ ] **Step 6: Run tests**

Run: `python3 -m pytest src/bringup/test/test_system_travel_launch.py -q`

Expected: PASS.

## Chunk 3: Nav2 Travel Configuration

### Task 4: Make `nav2_travel.yaml` match point-cloud relocalized static-map navigation

**Files:**
- Modify: `src/bringup/config/nav2_travel.yaml`

- [ ] **Step 1: Disable AMCL TF ownership**

Set:

```yaml
amcl:
  ros__parameters:
    tf_broadcast: false
```

AMCL can remain configured for future 2D fallback, but the restored PCD chain uses `localizer` for `map -> odom`.

- [ ] **Step 2: Make the global costmap consume the static map**

Set:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

Keep live point-cloud obstacles for dynamic updates, but ensure prior-map obstacles are part of global planning.

- [ ] **Step 3: Keep map filename injectable**

Leave:

```yaml
map_server:
  ros__parameters:
    yaml_filename: ""
```

The launch file owns runtime injection.

- [ ] **Step 4: Run focused config checks**

Run: `python3 -m pytest src/bringup/test/test_system_travel_launch.py -q`

Expected: PASS.

## Chunk 4: Localizer Launch Reuse

### Task 5: Make standalone localizer launch configurable

**Files:**
- Modify: `src/perception/localizer/launch/localizer_launch.py`

- [ ] **Step 1: Add launch arguments**

Expose:

```python
config_path
use_rviz
use_lio
```

Use defaults matching the existing behavior.

- [ ] **Step 2: Gate RViz and FAST-LIO2**

Use `IfCondition(use_rviz)` and `IfCondition(use_lio)` so bringup can use the node without duplicate LIO/RViz.

- [ ] **Step 3: Run Python syntax checks**

Run: `python3 -m py_compile src/perception/localizer/launch/localizer_launch.py`

Expected: PASS.

## Chunk 5: Documentation

### Task 6: Document the restored experimental chain bilingually

**Files:**
- Modify: `docs-CN/commands.md`
- Modify: `docs-EN/commands.md`
- Modify: `docs-CN/architecture.md`
- Modify: `docs-EN/architecture.md`
- Modify: `docs-CN/known_issues.md`
- Modify: `docs-EN/known_issues.md`
- Modify: `docs-CN/devlog/2026-07.md`
- Modify: `docs-EN/devlog/2026-07.md`

- [ ] **Step 1: Add operator commands**

Document:

```bash
cd ~/XJTLU-autonomous-vehicle
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh travel \
  map_yaml:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/2d/<map_name>/map.yaml \
  pcd_map:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd
```

Also document the explicit relocalization service call:

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

- [ ] **Step 2: Document TF ownership**

State that `localizer` owns `map -> odom`, FAST-LIO2 owns `odom -> base_link`, and PGO must not publish TF in this mode.

- [ ] **Step 3: Update known issues status**

Change Travel from "paused" to "experimental restored; requires on-vehicle validation".

- [ ] **Step 4: Add devlog entries**

Use the required File / Change / Reason / Effect format in both CN and EN logs.

## Chunk 6: Verification

### Task 7: Run available local verification

**Files:**
- No new files.

- [ ] **Step 1: Run launch tests**

Run: `python3 -m pytest src/bringup/test/test_system_travel_launch.py -q`

Expected: PASS.

- [ ] **Step 2: Run syntax checks**

Run:

```bash
python3 -m py_compile \
  src/bringup/launch/system_travel.launch.py \
  src/perception/localizer/launch/localizer_launch.py
```

Expected: PASS.

- [ ] **Step 3: Note unavailable local ROS verification**

If ROS 2 Humble is unavailable on the workstation, record that full `colcon build --packages-select bringup localizer --symlink-install --parallel-workers 1` and on-vehicle relocalization still need Jetson verification.
