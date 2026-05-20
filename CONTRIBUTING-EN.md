# Contributing Guide

This document is the single source of truth for development conventions in this repository, intended for all engineering maintainers and team developers.

## Role Division

| Role | Responsibility |
|------|----------------|
| kevinlasnh | Defines requirements, makes final decisions, executes on-vehicle tests, reviews PRs |
| Claude | Architecture, research, solution design, result review |
| Codex | Implementation execution, build verification, commits, PRs, documentation sync |
| Team developers | Collaborate under the same branch and PR rules |

## General Development Rules

1. Understand the chain before modifying code or parameters.
2. Do not modify the primary operating chain without verifying the impact scope.
3. Upstream dependencies are fetched through `dependencies.repos` + `vcs import`; this repository does not maintain a `src/third_party/` directory.
4. Critical modifications must explain what, why, and risk.
5. **Do not modify code directly on the vehicle (Jetson).** The standard flow is: modify code in your local repository on your own computer -> push to your branch on GitHub -> pull that branch on the vehicle -> build and deploy on the vehicle. The Jetson is only for compiling, running, and testing.

## Standard Development Flow

> The following flow is the standard workflow for all team developers. Each step indicates whether AI usage is allowed.

### Step 1: Session Initialization (Manual)

SSH into the Jetson, check repository and hardware status:

```bash
ssh jetson@100.97.227.24
cd ~/XJTLU-autonomous-vehicle
git status
git checkout main
git pull --ff-only
source install/setup.bash
```

First-time deployment or new machine initialization:

```bash
make setup
make build
source install/setup.bash
bash scripts/init_runtime_data.sh
```

Manually check hardware status:

```bash
# Check LiDAR (Livox MID360)
ls /dev/ttyUSB* /dev/ttyACM*
ros2 topic echo /livox/lidar --once

# Check GPS
ros2 topic echo /fix --once

# Check IMU (BMI088, integrated in RM C Board)
ros2 topic echo /imu --once

# Check STM32 serial port
ls /dev/stm32_board
```

Confirm all hardware is functioning before starting work.

### Step 2: Receive Task and Create/Switch Branch (Manual)

After kevinlasnh assigns a task, create a new branch from the latest `main`, or switch to an existing working branch:

```bash
# New task: create new branch
git checkout -b gps-route-collector

# Continue existing task: switch to existing branch and sync
git checkout your-branch
git pull origin your-branch
```

Branch naming uses descriptive names directly, without prefixes:

- Examples: `gps`, `nav-tuning`, `lidar-fix`, `docs-sync`

#### Branch Lifecycle

1. kevinlasnh assigns a task with clear content and deadline.
2. Developer creates a new branch from the latest `main`, named after the task.
3. Develop and test on your own branch (including on-vehicle testing).
4. Submit a PR once the feature is complete and tests pass.
5. kevinlasnh reviews and approves, then merge into `main`.
6. Delete the branch after merging.

One branch per task. Do not mix unrelated work in a single branch.

### Step 3: Research and Understand Current State (Manual, AI Prohibited)

Before writing any code, you must complete the following research:

1. **Read code**: Read all source files related to the task, understand the current data flow and call chain.
2. **Read documentation**: Read relevant docs in `docs-EN/` or `docs-CN/` (architecture, knowledge, devlog, etc.), understand historical decisions and current constraints.
3. **Read parameters**: Check `master_params.yaml` and related YAML configs, confirm current parameter values.
4. **Read launch chain**: Trace the complete launch chain from `Makefile`'s `launch-*` entries, understand node startup order and dependencies.
5. **Design a plan**: Based on the above research, formulate your implementation plan.

**AI is prohibited in this step.** The purpose is to ensure you truly understand the code and system, rather than relying on AI summaries.

### Step 4: Plan Output and Review (AI Allowed)

Organize your plan into a document containing:

1. Which files to modify, what changes to make
2. Why make these changes
3. Expected impact scope (which nodes, which operating modes are affected)
4. Risk points

**Submit your plan to AI for review** (Claude, ChatGPT, or other tools), have AI check:
- Whether the plan has gaps
- Whether there is a better implementation approach
- Whether it would introduce side effects

After review, send the final plan to kevinlasnh for confirmation. **Do not begin coding until kevinlasnh confirms.**

### Step 5: Deployability Review (AI Allowed)

Before coding, evaluate the plan for deployability:

1. Can the changes compile successfully on the Jetson?
2. Will it break the existing launch chain?
3. Are new dependencies needed? If so, are `package.xml` and `CMakeLists.txt` both updated?
4. Are parameter changes correctly propagated in `master_params.yaml`?

**AI can be used in this step** to help check plan deployability.

### Step 6: Coding (Manual, AI Not Recommended)

Modify code in your local repository on your own computer:

1. First verify the actual code state, launch chain, and current parameters.
2. Then make changes -- do not write from historical memory alone.
3. If changes affect system behavior, first confirm which operating modes are impacted.
4. When adding new launch files / modes / parameter profiles, update documentation concurrently rather than leaving it for later.

### Step 7: Push to GitHub and Deploy on Vehicle (Manual)

```bash
# On your local computer
git add path/to/file1 path/to/file2
git commit -m "Explain what changed and why"
git push -u origin your-branch

# SSH into Jetson
ssh jetson@100.97.227.24
cd ~/XJTLU-autonomous-vehicle
git checkout your-branch
git pull origin your-branch
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
source install/setup.bash
```

Commit rules:

1. Only add specific files. Do not use `git add -A` or `git add .`.
2. Do not commit unrelated changes.
3. Commit messages must be in English, stating what changed and why.

Build rules:

1. `--parallel-workers 1` is a hard requirement (Jetson memory constraint).
2. `source install/setup.bash` must be re-executed after every build.
3. Python packages can skip rebuilding under `--symlink-install`, but runtime verification is still required.
4. `build-navigation` currently includes `waypoint_collector`, `waypoint_nav_tool`, `gps_waypoint_dispatcher`.
5. After modifying `CMakeLists.txt` or install rules, confirm that target files actually exist in `install/`.

### Step 8: System Verification (Manual)

Launch the system in the appropriate mode based on the scope of changes:

```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-explore-gps
make launch-nav-gps
make launch-corridor
make launch-travel
```

At minimum, verify the following:

- All relevant nodes are online
- Key topics / actions have data
- `map -> odom -> base_link` TF chain is complete
- Session logs are landing in `~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/`

If system verification fails, go back to Step 6 to fix the code, loop until it passes.

### Step 9: On-Vehicle Testing and Physical Review (Manual)

After system verification passes, conduct on-vehicle testing:

1. **Before testing**: Confirm the PS2 gamepad is available, someone is always ready to press `X` to disable motors.
2. **During testing**: Observe vehicle physical behavior, record anomalies (yaw drift, stalls, collision risks, etc.).
3. **After testing**:
   - Record the physical assessment results
   - Record the test session path (e.g., `runtime-data/logs/2026-04-16-14-30-00/`)
   - Record improvement suggestions

### Step 10: Log Analysis (AI Allowed)

After every on-vehicle test, logs must be analyzed. You can analyze manually or with AI assistance.

```bash
# Find the latest log directory
LATEST_LOG=$(ls -td ~/XJTLU-autonomous-vehicle/runtime-data/logs/*/ 2>/dev/null | head -1)

# View bag summary
ros2 bag info ${LATEST_LOG}bag/

# Extract topic statistics from .db3 using sqlite3
python3 -c "
import sqlite3, glob
db = glob.glob('${LATEST_LOG}bag/*.db3')[0]
conn = sqlite3.connect(db)
for row in conn.execute('SELECT t.name, COUNT(m.id) FROM messages m JOIN topics t ON m.topic_id=t.id GROUP BY t.name'):
    print(f'{row[0]}: {row[1]} msgs')
conn.close()
"
```

**Mandatory data points to check** (after every on-vehicle test):

| Data Point | Source Topic | What to Look For |
|------------|-------------|-----------------|
| Total run duration | bag metadata | Whether it completed normally |
| GPS fix quality | `/fix` | fix type, satellite count, interruptions |
| Corridor status sequence | `/gps_corridor/status` | Whether all states completed as expected |
| Goal positions | `/gps_corridor/goal_map` | Deviation from expected targets |
| Velocity commands | `/cmd_vel` | Abnormal stalls (vel=0 lasting >3s) |
| TF continuity | `/tf` | map->odom->base_link jumps |
| Path planning | `/plan` | Whether path is reasonable, excessive replanning |

### Step 11: Problem Assessment and Escalation (Manual, AI Prohibited)

Based on test results and log analysis, assess the problem type yourself:

- **Architecture issue** (data flow design error, incorrect node responsibility division, etc.): Stop work, report to kevinlasnh, wait for redesign.
- **Minor issue** (parameter adjustment needed, edge case not handled, small bug, etc.): Fix it yourself, loop back to Step 6.
- **PASS** (feature meets expectations, tests passed): Proceed to Step 12.

**AI is prohibited for this assessment.** The purpose is to develop your own understanding and judgment of the system. If unsure about an issue, consult kevinlasnh.

### Step 12: Submit PR

Once the feature is complete and tests pass:

```bash
gh auth status
gh pr create
```

PRs must clearly describe:

1. What was changed
2. Why it was changed
3. How it was tested (include session path and key test conclusions)
4. Which layers are affected

All PRs must be reviewed and approved by kevinlasnh before merging into `main`.

After merging:

```bash
gh pr merge --merge --delete-branch
git checkout main
git pull --ff-only
git fetch --prune
```

If `gh auth status` on the Jetson fails, you can run PR operations on a trusted workstation already logged into GitHub CLI for the same branch, then have the Jetson `git pull` to sync.

### Step 13: Push runtime-data to Hugging Face

Before the end of your work day, push test runtime-data to Hugging Face:

```bash
cd ~/XJTLU-autonomous-vehicle/runtime-data
git add logs/<your-session-directory>/ gnss/current_route.yaml
git commit -m "Add test session YYYY-MM-DD-HH-MM-SS"
git push origin main
```

If `git push` fails, ask kevinlasnh for help.

### Step 14: Documentation Update (AI Allowed)

Update the day's work documentation following the Devlog Documentation Format. One copy in Chinese and one in English, with matching content.

AI can be used to assist with documentation writing, but you must ensure accuracy and format compliance.

### Breakpoints and Progress Saving

If you need to pause work midway (end of day, break, etc.), you must ensure:

1. Current progress is not lost (code is committed and pushed to your branch).
2. You can pick up where you left off next time.

The specific tool or method for recording progress is up to you (notes, TODO files, branch descriptions, etc.), as long as no progress is lost.

## AI Usage Rules

| Step | AI Allowed? |
|------|-------------|
| Session initialization (hardware checks, etc.) | Prohibited |
| Research, reading code, understanding current state | Prohibited |
| Plan review | Allowed |
| Deployability review | Allowed |
| Coding | Not recommended |
| Build and system verification | Prohibited |
| On-vehicle testing | Prohibited |
| Log analysis | Allowed |
| Problem assessment and escalation | Prohibited |
| Documentation writing | Allowed |

## Daily Update Requirement

Every team member must push to their branch at least once before the end of their work day, updating project documentation (progress, notes, etc.). As long as any work was done that day (no matter how small), documentation updates are mandatory. If no work was done at all that day, no update is required.

## Devlog Documentation Format

All devlog entries must follow the format below. One copy in Chinese and one in English (`docs-CN/devlog/YYYY-MM.md` + `docs-EN/devlog/YYYY-MM.md`), with matching content.

### Basic Structure

```markdown
## YYYY.MM.DD

### [Change Topic Title]

- One-line summary of the change
- Specific parameter values, commit hash (e.g., `abc1234`), file names
```

### Complex Change Format

When changes involve multiple files or require design decision explanations, use the four-element format:

```markdown
#### Subtopic Name (`commit hash`)

- **File**: specific file path
- **Change**: what was done
- **Reason**: why it was done
- **Effect**: what impact it produces
```

### Mandatory Requirements

1. Date format is `YYYY.MM.DD`.
2. Each topic uses `### [Square Bracket Title]`.
3. Reference specific commit hashes, parameter values (including old and new values), and file paths.
4. No vague descriptions (e.g., "optimized performance"). Must specify exactly what parameter changed, from what value to what value.
5. When multiple changes are involved, split into subtopics, each with its own four-element block.
6. Test results must include: test session path, key findings, user decisions.

## GPS Navigation-Specific Verification

If changes involve `nav-gps`, at minimum add these checks:

1. Whether `build_scene_runtime.py` successfully generates `current_scene/` compiled artifacts
2. Whether `gps_anchor_localizer` enters `NAV_READY`
3. Whether `gps_waypoint_dispatcher` successfully reads `scene_points.yaml`
4. Whether `/compute_route` / `/follow_path` / `/navigate_to_pose` actions are online
5. Whether `goto_name` / `list_destinations` / `stop` behave correctly
6. Whether mock / replay `/fix` can be used for software smoke testing when there is no live fix indoors

## Fixed-Launch GPS Corridor Workflow

When the task objective narrows to "fixed launch position -> GPS route endpoint" corridor verification, prefer using:

1. Collect corridor multi-waypoint route:
   - `python3 scripts/collect_gps_route.py`
2. Return the vehicle to the fixed Launch Pose, align the heading
3. Launch directly:
   - `make launch-corridor`
   - or `bash scripts/launch_with_logs.sh corridor`
4. Observe:
   - `/gps_corridor/status`
5. `gps_global_aligner_node` + `gps_route_runner_node` automatically:
   - Standalone aligner estimates smoothed `ENU->map` transform
   - Checks whether current `/fix` is close to `start_ref`
   - Bootstrap startup (`yaw0 + launch_yaw_deg`)
   - GPS waypoints -> ENU -> map conversion
   - Freeze alignment within waypoint, split subgoals per segment
   - Sequential `NavigateToPose`

This workflow no longer requires:
- `nav_gps_menu.py`
- `goto_name`
- `route_server`
- `scene_gps_bundle.yaml`

## Parameter and YAML Rules

1. Modifying already-tuned YAML parameters without a documented reason is not allowed.
2. Parameter changes must be simultaneously recorded in the relevant documentation.
3. The current unified parameter entry point is `src/bringup/config/master_params.yaml`.
4. Nav2 primary configuration files are:
   - `nav2_default.yaml`
   - `nav2_explore.yaml`
   - `nav2_travel.yaml`

## Runtime Data and Logging Rules

Runtime data lives inside the workspace at `~/XJTLU-autonomous-vehicle/runtime-data/`:

```text
~/XJTLU-autonomous-vehicle/runtime-data/
├── config/
├── gnss/
├── logs/
│   ├── <timestamp>/
│   │   ├── console/
│   │   ├── data/
│   │   └── system/
│   └── latest -> <timestamp>
├── maps/
├── perf/
└── planning/
```

Additional rules:

1. `make launch-*` automatically creates session log directories.
2. `console/` stores ROS 2 console logs.
3. `data/` stores per-node custom logs, driven by the legacy runtime interface variable `FYP_LOG_SESSION_DIR`.
4. `system/` stores `tegrastats.log` and `session_info.yaml`.
5. When using `ros2 launch` directly, some legacy log fallback paths may still exist, such as `logs/twist_log/`.

## GNSS Current Constraints

1. The current GNSS stack operates at basic GPS accuracy level, not maintained as an RTK workflow.
2. Indoors without satellite reception, `/fix` showing `status=-1` or `NaN` is expected behavior.
3. `gnss_calibration` does not write a valid `gnss_offset.txt` under no-fix conditions.
4. On the current Jetson runtime, `~/XJTLU-autonomous-vehicle/runtime-data/gnss/` may contain `gnss_offset.invalid_*.txt` as invalid sample traces.

## On-Vehicle Safety Rules

1. The PS2 gamepad must always be kept available.
2. During autonomous driving, someone must always be ready to press the `X` button to disable motors.
3. The red emergency stop button on the vehicle body always takes priority over software commands.
4. The `B` button is prohibited as an emergency braking method because it causes severe wheel reversal.

## System Environment Rules

1. The current Jetson wired connection `Wired connection 1` is set to auto-connect with priority `100`; do not disable it casually.
2. Check `gh auth status` on the machine performing PR operations.
3. If the Jetson's `gh` login session expires, PRs and merges can be completed on a local workstation already logged into GitHub CLI, then have the Jetson pull `main` back.

## AI Collaboration Flow

kevinlasnh uses Claude + Codex + Gemini for AI-assisted development, with the control plane maintained in a separate repository. Team developers do not need to know this workflow -- follow the Standard Development Flow in this document.

## Documentation Trigger Rules

When code, launch files, parameters, system environment, or workflows change, corresponding documentation must be updated. Both CN and EN documentation directories must be updated simultaneously.

| Change Type | Minimum Documentation to Sync |
|-------------|-------------------------------|
| Node source code | `devlog`, related `knowledge`, `architecture.md` when necessary |
| Launch files | `devlog`, `commands.md`, `architecture.md` when necessary |
| YAML parameters | `devlog`, corresponding knowledge document |
| Scripts and tools | `devlog`, `commands.md` |
| Bug fixes / new bugs | `devlog`, `known_issues.md` |
| System environment changes | `devlog`, `commands.md` |
| New/removed ROS2 packages | `devlog`, `architecture.md`, `commands.md` |
| Hardware changes | `devlog`, `architecture.md`, `known_issues.md` |
| GPS/GNSS related | `devlog`, `gps_planning.md`, `pgo.md` |
| Physical test conclusions | `devlog`, `nav2_tuning.md`, `known_issues.md` |

Most common sync targets:

- `docs-CN/devlog/YYYY-MM.md` + `docs-EN/devlog/YYYY-MM.md`
- `docs-CN/commands.md` + `docs-EN/commands.md`
- `docs-CN/architecture.md` + `docs-EN/architecture.md`
- `docs-CN/known_issues.md` + `docs-EN/known_issues.md`
- `docs-CN/knowledge/*.md` + `docs-EN/knowledge/*.md`

## Session End Checklist

A session is not considered closed until all of the following are complete:

1. Changes have been verified.
2. Related documentation has been synced.
3. Branch has been pushed.
4. PR has been created and merged, or there is a clear record of why only a feature branch was left.
5. Jetson is back on the latest `main`, or there is a clear record of the current branch and reason for staying.
6. If there are new system facts, issues, or blockers, they have been written to the development log and issue tracker.

## Invariant Rules

1. YAML parameter changes must document the reason.
2. Builds must use `--parallel-workers 1`.
3. `source install/setup.bash` must be re-executed after every build.
4. Upstream dependencies fetched through `dependencies.repos` are not treated as project custom development areas.
5. Documentation descriptions must reflect the current executable state, not historical conventions.
