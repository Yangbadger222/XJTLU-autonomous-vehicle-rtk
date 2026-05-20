# 贡献指南

本文档是本仓库的唯一开发规范入口，面向所有工程维护者和团队开发者。

## 角色分工

| 角色 | 责任 |
|------|------|
| kevinlasnh | 提需求、做最终决策、执行实车测试、PR 审查 |
| Claude | 架构、调研、方案设计、结果复审 |
| Codex | 执行实现、构建验证、提交、PR、文档同步 |
| 团队开发者 | 按同样分支和 PR 规则协作 |

## 通用开发规则

1. 先理解链路，再改代码或参数。
2. 不要在未验证影响面的情况下改动主运行链。
3. 上游依赖通过 `dependencies.repos` + `vcs import` 拉取，不在本仓库维护 `src/third_party/` 目录。
4. 关键修改必须能说明 what、why、risk。
5. **禁止直接在车上（Jetson）修改代码。** 标准流程是：在自己电脑上的本地仓库修改代码 -> push 到 GitHub 自己的分支 -> 在车上 pull 该分支 -> 在车上构建和部署。Jetson 只做编译、运行和测试。

## 标准开发流程

> 以下流程是所有团队开发者的标准工作流。每个步骤标注了是否允许使用 AI。

### Step 1: 会话初始化（手动）

SSH 到 Jetson，检查仓库和硬件状态：

```bash
ssh jetson@100.97.227.24
cd ~/XJTLU-autonomous-vehicle
git status
git checkout main
git pull --ff-only
source install/setup.bash
```

首次部署或新机器初始化：

```bash
make setup
make build
source install/setup.bash
bash scripts/init_runtime_data.sh
```

手动检查硬件状态：

```bash
# 检查 LiDAR（Livox MID360）
ls /dev/ttyUSB* /dev/ttyACM*
ros2 topic echo /livox/lidar --once

# 检查 GPS
ros2 topic echo /fix --once

# 检查 IMU（BMI088，集成在 RM C Board 中）
ros2 topic echo /imu --once

# 检查下位机（STM32）串口
ls /dev/stm32_board
```

确认所有硬件正常后再开始工作。

### Step 2: 接收任务与创建/切换分支（手动）

kevinlasnh 派发任务后，从最新 `main` 创建新分支，或切换到已有的工作分支继续：

```bash
# 新任务：创建新分支
git checkout -b gps-route-collector

# 继续已有任务：切换到已有分支并同步
git checkout your-branch
git pull origin your-branch
```

分支命名直接用描述名，不加前缀：

- 示例: `gps`、`nav-tuning`、`lidar-fix`、`docs-sync`

#### 分支生命周期

1. kevinlasnh 派发任务，明确任务内容和期限。
2. 开发者从最新 `main` 创建新分支，按任务内容命名。
3. 在自己的分支上开发和测试（包括实车测试）。
4. 功能完成且测试通过后，提交 PR。
5. kevinlasnh 审查通过后合并到 `main`。
6. 合并后删除该分支。

一个分支只做一件任务，不要在一个分支里混做多个不相关的任务。

### Step 3: 调研与理解现状（手动，禁止使用 AI）

动手写代码之前，必须先完成以下调研：

1. **读代码**：阅读与任务相关的所有源文件，理解当前的数据流和调用链。
2. **读文档**：阅读 `docs-EN/` 或 `docs-CN/` 中的相关文档（architecture、knowledge、devlog 等），理解历史决策和当前约束。
3. **读参数**：检查 `master_params.yaml` 和相关 YAML 配置，确认当前参数值。
4. **读 launch 链**：从 `Makefile` 的 `launch-*` 入口追踪完整的 launch 链，理解节点启动顺序和依赖关系。
5. **设计方案**：基于以上调研，形成自己的实现方案。

**这一步禁止使用 AI。** 目的是确保你真正理解代码和系统，而不是依赖 AI 的总结。

### Step 4: 方案输出与审查（可用 AI）

将你的方案整理成文档，包含：

1. 要改什么文件、改什么内容
2. 为什么这样改
3. 预期影响范围（哪些节点、哪些运行模式受影响）
4. 风险点

**提交方案给 AI 进行审查**（可以使用 Claude、ChatGPT 等工具），让 AI 检查：
- 方案是否有遗漏
- 是否有更好的实现方式
- 是否会引入副作用

审查完成后，将最终方案发给 kevinlasnh 确认。**kevinlasnh 确认后方可开始编码。**

### Step 5: 部署性审查（可用 AI）

编码前，对方案做部署性评估：

1. 改动是否能在 Jetson 上成功编译？
2. 是否会破坏现有的 launch 链？
3. 是否需要新增依赖？如果需要，`package.xml` 和 `CMakeLists.txt` 是否都更新了？
4. 参数变更是否在 `master_params.yaml` 中正确传递？

**这一步可以借助 AI** 帮你检查方案的可部署性。

### Step 6: 编码（手动，建议不用 AI）

在自己电脑上的本地仓库中修改代码：

1. 先核对真实代码状态、launch 链和当前参数。
2. 再做改动，不凭历史记忆直接写。
3. 如果改动涉及系统行为，先确认影响的运行模式。
4. 新增 launch / mode / 参数 profile 时，同步更新文档，不留到后面补。

### Step 7: 推送到 GitHub 并在车上部署（手动）

```bash
# 在本地电脑上
git add path/to/file1 path/to/file2
git commit -m "Explain what changed and why"
git push -u origin your-branch

# SSH 到 Jetson
ssh jetson@100.97.227.24
cd ~/XJTLU-autonomous-vehicle
git checkout your-branch
git pull origin your-branch
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
source install/setup.bash
```

提交规则：

1. 只添加具体文件，不要使用 `git add -A` 或 `git add .`。
2. 不提交无关改动。
3. Commit message 使用英文，写明改了什么和为什么。

构建规则：

1. `--parallel-workers 1` 是硬性要求（Jetson 内存限制）。
2. 每次构建后都要重新 `source install/setup.bash`。
3. Python 包虽可借助 `--symlink-install` 免重编，但仍要做运行验证。
4. `build-navigation` 当前包含 `waypoint_collector`、`waypoint_nav_tool`、`gps_waypoint_dispatcher`。
5. 修改 `CMakeLists.txt` 或安装规则后，确认 `install/` 中目标文件实际存在。

### Step 8: 系统验证（手动）

按改动范围选择合适模式启动系统：

```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-explore-gps
make launch-nav-gps
make launch-corridor
make launch-travel
```

验证时至少检查：

- 相关节点是否都在线
- 关键 topic / action 是否有数据
- `map -> odom -> base_link` TF 是否完整
- session 日志是否落到 `~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/`

如果系统验证失败，回到 Step 6 修改代码，循环直到通过。

### Step 9: 实车测试与物理评审（手动）

系统验证通过后，进行实车测试：

1. **测试前**：确认 PS2 手柄可用，有人随时准备按 `X` 键失能电机。
2. **测试中**：观察车辆物理行为，记录异常（偏航、停顿、碰撞风险等）。
3. **测试后**：
   - 记录物理运行状态的人工评审结果
   - 记录测试 session 路径（如 `runtime-data/logs/2026-04-16-14-30-00/`）
   - 记录改进意见

### Step 10: 日志分析（可用 AI）

每次实车测试后，必须分析日志。可以自己手动分析，也可以借助 AI。

```bash
# 找到最新日志目录
LATEST_LOG=$(ls -td ~/XJTLU-autonomous-vehicle/runtime-data/logs/*/ 2>/dev/null | head -1)

# 查看 bag 概要
ros2 bag info ${LATEST_LOG}bag/

# 用 sqlite3 从 .db3 提取 topic 统计
python3 -c "
import sqlite3, glob
db = glob.glob('${LATEST_LOG}bag/*.db3')[0]
conn = sqlite3.connect(db)
for row in conn.execute('SELECT t.name, COUNT(m.id) FROM messages m JOIN topics t ON m.topic_id=t.id GROUP BY t.name'):
    print(f'{row[0]}: {row[1]} msgs')
conn.close()
"
```

**必须检查的数据点**（每次实车测试后）：

| 数据点 | 来源 topic | 关注什么 |
|--------|-----------|---------|
| 运行总时长 | bag metadata | 是否正常完成 |
| GPS fix 质量 | `/fix` | fix type、卫星数、有无中断 |
| corridor status 序列 | `/gps_corridor/status` | 是否按预期走完所有状态 |
| 目标点位置 | `/gps_corridor/goal_map` | 与预期目标的偏差 |
| 速度指令 | `/cmd_vel` | 有无异常停顿（vel=0 持续 >3s） |
| TF 连续性 | `/tf` | map->odom->base_link 是否有跳变 |
| 路径规划 | `/plan` | 路径是否合理、有无反复重规划 |

### Step 11: 问题判断与上报（手动，禁止使用 AI）

根据测试结果和日志分析，自己判断问题性质：

- **架构问题**（数据流设计错误、节点职责划分不合理等）：停止工作，向 kevinlasnh 上报，等待重新设计。
- **小问题**（参数需要调整、边界条件未处理、小 bug 等）：自行修复，回到 Step 6 循环。
- **PASS**（功能符合预期，测试通过）：进入 Step 12。

**这一步禁止使用 AI 判断。** 目的是锻炼你自己对系统的理解和判断能力。遇到拿不准的问题，向 kevinlasnh 咨询。

### Step 12: 提交 PR

功能完成且测试通过后：

```bash
gh auth status
gh pr create
```

PR 必须写清楚：

1. 改了什么
2. 为什么改
3. 怎么测的（包含 session 路径和关键测试结论）
4. 影响哪些层

所有 PR 必须经过 kevinlasnh 审查通过后才能合并到 `main`。

合并后：

```bash
gh pr merge --merge --delete-branch
git checkout main
git pull --ff-only
git fetch --prune
```

如果 Jetson 上 `gh auth status` 失败，可以在已登录 GitHub CLI 的受信任工作站上对同一分支完成 PR 操作，然后再让 Jetson `git pull` 同步结果。

### Step 13: 推送 runtime-data 到 Hugging Face

当天工作结束之前，将测试产生的 runtime-data 推送到 Hugging Face：

```bash
cd ~/XJTLU-autonomous-vehicle/runtime-data
git add logs/<your-session-directory>/ gnss/current_route.yaml
git commit -m "Add test session YYYY-MM-DD-HH-MM-SS"
git push origin main
```

如果 `git push` 失败，向 kevinlasnh 求助。

### Step 14: 文档更新（可用 AI）

更新当天的工作文档，按 Devlog 文档格式规范书写。中英文各一份，内容完全对应。

可以借助 AI 辅助文档写作，但必须确保内容准确、符合格式规范。

### 断点与进度保存

如果需要中途暂停工作（下班、休息等），必须确保：

1. 当前进度不会丢失（代码已 commit 并 push 到自己的分支）。
2. 下次回来能接着当前的工作继续。

具体用什么工具或方法记录进度由你自己决定（笔记、TODO 文件、分支描述等），只要不丢失进度即可。

## AI 使用规则

| 步骤 | 是否允许 AI |
|------|------------|
| 会话初始化（硬件检查等） | 禁止 |
| 调研、读代码、理解现状 | 禁止 |
| 方案审查 | 允许 |
| 部署性审查 | 允许 |
| 编码 | 建议不用 |
| 构建与系统验证 | 禁止 |
| 实车测试 | 禁止 |
| 日志分析 | 允许 |
| 问题判断与上报 | 禁止 |
| 文档写作 | 允许 |

## 每日更新要求

每位团队成员在当天工作结束之前，必须至少 push 一次到自己的分支，更新项目文档（进度、笔记等）。只要当天做了任何工作（无论多小），文档更新就是必须的。当天完全没有工作则不需要更新。

## Devlog 文档格式规范

所有 devlog 条目必须按以下格式书写。中英文各一份（`docs-CN/devlog/YYYY-MM.md` + `docs-EN/devlog/YYYY-MM.md`），内容完全对应。

### 基本结构

```markdown
## YYYY.MM.DD

### [变更主题标题]

- 一句话概述改动
- 具体参数值、commit hash（如 `abc1234`）、文件名
```

### 复杂改动格式

当改动涉及多个文件或需要说明设计决策时，必须使用四要素格式：

```markdown
#### 子主题名称 (`commit hash`)

- **文件**: 具体文件路径
- **改动**: 做了什么
- **原因**: 为什么做
- **效果**: 产生什么影响
```

### 强制要求

1. 日期格式统一为 `YYYY.MM.DD`。
2. 每个主题用 `### [方括号标题]` 包裹。
3. 引用具体的 commit hash、参数值（含旧值和新值）、文件路径。
4. 不写空泛描述（如"优化了性能"），必须写明具体改了什么参数、从什么值改到什么值。
5. 涉及多处改动时，按子主题拆分，每个子主题独立写四要素。
6. 测试结果必须包含：测试 session 路径、关键发现、用户决策。

## GPS 导航专项验证

如果改动涉及 `nav-gps`，至少补下面这些检查：

1. `build_scene_runtime.py` 是否成功生成 `current_scene/` 编译产物
2. `gps_anchor_localizer` 是否进入 `NAV_READY`
3. `gps_waypoint_dispatcher` 是否成功读取 `scene_points.yaml`
4. `/compute_route` / `/follow_path` / `/navigate_to_pose` action 是否在线
5. `goto_name` / `list_destinations` / `stop` 是否行为正确
6. 室内无 live fix 时，是否可用 mock / replay `/fix` 做软件 smoke

## Fixed-Launch GPS Corridor 工作流

当任务目标收缩成"固定启动位 -> GPS 路线终点"的 corridor 验证时，优先使用：

1. 采集 corridor 多点路线：
   - `python3 scripts/collect_gps_route.py`
2. 将车停回固定 Launch Pose，朝向摆正
3. 直接启动：
   - `make launch-corridor`
   - 或 `bash scripts/launch_with_logs.sh corridor`
4. 观察：
   - `/gps_corridor/status`
5. 由 `gps_global_aligner_node` + `gps_route_runner_node` 自动：
   - 独立 aligner 估计平滑 `ENU->map` 变换
   - 检查当前 `/fix` 是否靠近 `start_ref`
   - Bootstrap 启动（`yaw0 + launch_yaw_deg`）
   - GPS waypoints -> ENU -> map 转换
   - Waypoint 内冻结 alignment，按段切 subgoals
   - 串行 `NavigateToPose`

这套工作流不再需要：
- `nav_gps_menu.py`
- `goto_name`
- `route_server`
- `scene_gps_bundle.yaml`

## 参数与 YAML 规则

1. 不允许无原因改动已调好的 YAML 参数。
2. 参数变更必须同时记录到相关文档。
3. 当前统一参数入口是 `src/bringup/config/master_params.yaml`。
4. Nav2 主配置文件是：
   - `nav2_default.yaml`
   - `nav2_explore.yaml`
   - `nav2_travel.yaml`

## 运行时数据与日志规则

运行时数据位于工作区内部的 `~/XJTLU-autonomous-vehicle/runtime-data/`：

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

补充规则：

1. `make launch-*` 会自动创建 session 日志目录。
2. `console/` 存 ROS 2 控制台日志。
3. `data/` 存节点自定义日志，由 legacy 运行接口变量 `FYP_LOG_SESSION_DIR` 驱动。
4. `system/` 存 `tegrastats.log` 和 `session_info.yaml`。
5. 直接 `ros2 launch` 时，部分旧日志 fallback 路径仍会存在，例如 `logs/twist_log/`。

## GNSS 当前约束

1. 当前 GNSS 栈按基础 GPS 精度使用，不按 RTK 工作流维护。
2. 室内无卫星条件下，`/fix` 为 `status=-1` 或 `NaN` 是预期现象。
3. `gnss_calibration` 在 no-fix 情况下不会写有效 `gnss_offset.txt`。
4. 当前 Jetson 运行时 `~/XJTLU-autonomous-vehicle/runtime-data/gnss/` 中可出现 `gnss_offset.invalid_*.txt` 作为无效样本留痕。

## 实车安全规则

1. PS2 手柄始终保持可用。
2. 自动驾驶时必须有人随时准备按 `X` 键失能电机。
3. 车身红色急停按钮始终优先于软件命令。
4. `B` 键禁用为紧急刹车手段，因为会导致车轮严重反转。

## 系统环境规则

1. 当前 Jetson 有线连接 `Wired connection 1` 处于自动连接状态，优先级为 `100`，不要随意关闭。
2. 在执行 PR 操作的机器上先检查 `gh auth status`。
3. 如果 Jetson 的 `gh` 登录态失效，可以在已登录 GitHub CLI 的本地工作站上完成 PR/merge，再让 Jetson 回拉 `main`。

## AI 协作流

kevinlasnh 使用 Claude + Codex + Gemini 进行 AI 辅助开发，其控制面在独立仓库中维护。团队开发者不需要了解该流程，按本文档的标准开发流程执行即可。

## 文档触发规则

代码、launch、参数、系统环境、流程有变动时，必须同步更新对应文档。中英文文档目录必须同时更新。

| 变化类型 | 最少要同步的文档 |
|----------|------------------|
| 节点源码 | `devlog`、相关 `knowledge`、必要时 `architecture.md` |
| Launch 文件 | `devlog`、`commands.md`、`architecture.md` |
| YAML 参数 | `devlog`、对应知识文档 |
| 脚本与工具 | `devlog`、`commands.md` |
| Bug 修复 / 新 Bug | `devlog`、`known_issues.md` |
| 系统环境变化 | `devlog`、`commands.md` |
| 新增/删除 ROS2 包 | `devlog`、`architecture.md`、`commands.md` |
| 硬件变更 | `devlog`、`architecture.md`、`known_issues.md` |
| GPS/GNSS 相关 | `devlog`、`gps_planning.md`、`pgo.md` |
| 物理测试结论 | `devlog`、`nav2_tuning.md`、`known_issues.md` |

最常见的同步目标：

- `docs-CN/devlog/YYYY-MM.md` + `docs-EN/devlog/YYYY-MM.md`
- `docs-CN/commands.md` + `docs-EN/commands.md`
- `docs-CN/architecture.md` + `docs-EN/architecture.md`
- `docs-CN/known_issues.md` + `docs-EN/known_issues.md`
- `docs-CN/knowledge/*.md` + `docs-EN/knowledge/*.md`

## 会话结束检查表

会话在以下项目全部完成前不算收口：

1. 改动已验证。
2. 相关文档已同步。
3. 分支已推送。
4. PR 已创建并合并，或者明确记录为什么本次只停在 feature 分支。
5. Jetson 已回到最新 `main`，或者明确记录当前停留分支和原因。
6. 如果有新的系统事实、问题或阻塞，已写入开发日志和问题追踪。

## 不变规则

1. YAML 参数改动必须记录原因。
2. 构建必须使用 `--parallel-workers 1`。
3. 每次构建后都要重新 `source install/setup.bash`。
4. 通过 `dependencies.repos` 拉取的上游依赖不作为项目自定义开发区。
5. 文档描述必须反映当前真实可执行状态，不能保留过时命令当"标准流程"。
