# Contributing Guide

For any changes to the repository, make sure to follow this guide.

## General Development Rules

1. Understand the chain before modifying code or parameters.
2. Do not modify the primary operating chain without verifying the impact scope.
3. Upstream dependencies are fetched through `dependencies.repos` + `vcs import`; this repository does not maintain a `src/third_party/` directory.
4. Critical modifications must explain what, why, and risk.
5. **Do not modify code directly on the vehicle (Jetson).** The standard flow is: modify code in your local repository on your own computer -> push to your branch on GitHub -> pull that branch on the vehicle -> build and deploy on the vehicle. The Jetson is only for compiling, running, and testing. **Any code changes in the Jetson directly will be deleted.**

## Standard Development Flow

### Step 1: Session Initialization

SSH into the Jetson, check repository and hardware status:

```bash
ssh badger@100.79.128.21
git pull --ff-only
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

### Step 2: Receive Task and Create/Switch Branch

After receiving a task, create a new branch from the latest `main`, or switch to an existing working branch:

```bash
# New task: create new branch
git checkout -b <BRANCH-NAME>

# Continue existing task: switch to existing branch and sync
git checkout <BRANCH-NAME>
git pull origin <BRANCH-NAME>
```

Branch names must be descriptive (changing what, doing what):

- Good Examples: `rtk-signal`, `nav-tuning`, `lidar-fix`, `docs-sync`
- Bad Examples: `gps`, `lio`, `frc`

Use one branch per task. Do not mix unrelated work in a single branch.

### Step 3: Research and Preparation

Before writing any code:

1. **Check relevant files**: Make a list of which files or functions are related to the task.
2. **Read launch chain**: Trace the launch chain from `Makefile`'s `launch-*` entries. Which launch modes call those files, which topics/nodes are used.
3. **Read parameters**: Check `master_params.yaml` and related YAML configs, confirm current parameter values.
4. **Design a plan**: Decide what needs to change and why.
5. **Ask questions**: If you cannot find something in the documentation, feel free to discuss with us.

### Step 4: Code and commit to Git

When you feel comfortable, you can start adding small changes and testing them in the robot. For any bigger changes, make sure to talk with us first.

Test in the Jetson after commiting in Git. To push into the repo, run from your computer:
```bash
git add .
git commit -m "<COMMIT_MSG>"
git push -u origin <BRANCH_NAME>
```

For commit messages, use the following commit convention:
- `feat`: A new feature for the robot
- `fix`: A bug fix
- `docs`: Documentation-only updates
- `style`: Code formatting changes (e.g., white-space, missing semi-colons) that do not alter logic
- `refactor`: Rewriting or restructuring code without changing its behavior or fixing bugs
- `perf`: Code optimizations aimed at improving performance
- `test`: Adding missing tests or correcting existing tests
- `build`: Changes impacting build tools, configurations, or external dependencies
- `chore`: Auxiliary tasks that don't modify code in `src/` or tests (e.g., updating `.gitignore`)

### Step 5: Test in the robot

Deploy in the Jetson:
```bash
ssh badger@100.79.128.21
git fetch
git switch <BRANCH_NAME>
git pull
```

If you changed any code under `src/`, you must build again. Common build commands:
```bash
# Layered build
mbuild build-bringup
mbuild build-fastlio2
mbuild build-sensor
mbuild build-perception
mbuild build-planning
mbuild build-navigation

# Single package build
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
ss
```
Using `mbuild build-*` automatically sources the new workspace. If you use `colcon build` manually, remember to keep `--parallel-workers 1` and source with `ss` after building.

If successful, launch the system in the appropriate mode based on the scope of changes:
```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-travel
make launch-explore-gps
make launch-nav-gps
make launch-rtk-basic
make launch-tightly-coupled
```

At minimum, verify the following:
- All relevant nodes are online
- Key topics / actions have data
- `map -> odom -> base_footprint -> base_link` TF chain is complete (no missing links in Foxglove)
- When moving the robot, see if any unexpected behavior occurs
- Session logs are landing in `~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/`

### Step 6: Verify your changes

After verifying the code changes in the robot, check the new changes you made and check:

1. Do all launch modes compile successfully on the robot?
2. Does it change the existing launch chain?
3. Which launch modes, nodes, or topics are affected by the new changes?
4. Are new dependencies needed? If so, are `package.xml` and `CMakeLists.txt` both updated?
5. Are parameter changes correctly reflected in `master_params.yaml`?
6. Does the changes introduce any risk points?

You may use an agentic AI to help check for these considerations. If there is any code you don't fully understand, ask us.

### Step 7: Documentation

By this point, we assume you have completed your task and it works properly in the robot. Then the last step is to document it. At minimum, you must update the following:
- Any new or affected commands should be updated in [commands.md](docs-EN/commands.md)
- A summary of all changes must be documented in the newest devlog
- You MUST update both the Chinese and English version. You may write it in your native language, then ask AI to translate it to the other.

> If you changed any parameter in [master_params.yaml](src/bringup/config/master_params.yaml), you MUST write a thorough analysis of why, how it impacts the robot, and which edge cases should be considered.

The devlog must follow this format:
```md
## YYYY.MM.DD

### [Change Topic Title]

#### File
- `file1/`
- `file2/`

#### Change
#### Reason
#### Effect
```

After you are done, make sure you have uploaded all your changes with `git push`.

### Step 8: Pull request

After uploading everything, go to [github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk) and create a pull request. Follow the default format and checklist.