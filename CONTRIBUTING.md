以下是为您的文档提供的中文翻译版本，保留了原始的 Markdown 格式和技术术语：

---

# 贡献指南 (Contributing Guide)

在对本仓库进行任何更改之前，请务必遵循本指南。

## 通用开发规则

1. 在修改代码或参数之前，请务必先理解相关的调用链 (chain)。
2. 在未确认影响范围的情况下，严禁修改主运行链路。
3. 上游依赖通过 `dependencies.repos` + `vcs import` 获取；本仓库不维护 `src/third_party/` 目录。
4. 关键性修改必须说明修改了什么、为什么修改以及潜在的风险。
5. **严禁直接在车端 (Jetson) 修改代码。** 标准流程是：在你个人电脑的本地仓库中修改代码 -> 推送到 GitHub 上的个人分支 -> 在车端拉取该分支 -> 在车端进行编译和部署。Jetson 仅用于编译、运行和测试。**任何直接在 Jetson 上进行的代码更改都将被删除。**

## 标准开发流程

### 第一步：会话初始化 (Session Initialization)

通过 SSH 登录 Jetson，检查仓库和硬件状态：

```bash
ssh badger@100.79.128.21
git pull --ff-only
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
# 检查激光雷达 (LiDAR, Livox MID360)
ls /dev/ttyUSB* /dev/ttyACM*
ros2 topic echo /livox/lidar --once

# 检查 GPS
ros2 topic echo /fix --once

# 检查 IMU (BMI088，集成在 RM C 板中)
ros2 topic echo /imu --once

# 检查 STM32 串口
ls /dev/stm32_board
```

在开始工作前，请确认所有硬件运行正常。

### 第二步：接收任务并创建/切换分支

接到任务后，从最新的 `main` 分支创建一个新分支，或者切换到已有的工作分支：

```bash
# 新任务：创建新分支
git checkout -b <BRANCH-NAME>

# 继续现有任务：切换到已有分支并同步
git checkout <BRANCH-NAME>
git pull origin <BRANCH-NAME>
```

分支命名必须具有描述性（说明修改了什么，在做什么）：

* **正确示例**：`rtk-signal`, `nav-tuning`, `lidar-fix`, `docs-sync`
* **错误示例**：`gps`, `lio`, `frc`

一个任务使用一个分支。不要在同一个分支中混入无关的工作。

### 第三步：研究与准备

在编写任何代码之前：

1. **检查相关文件**：列出与任务相关的文件或函数清单。
2. **梳理启动链路**：从 `Makefile` 的 `launch-*` 条目开始追踪启动链路 (launch chain)。找出哪些启动模式调用了这些文件，以及使用了哪些话题 (topics)/节点 (nodes)。
3. **查阅参数**：检查 `master_params.yaml` 及相关的 YAML 配置文件，确认当前的参数值。
4. **设计方案**：确定需要修改的内容及原因。
5. **主动提问**：如果在文档中找不到所需信息，请随时与我们讨论。

### 第四步：编码并提交至 Git

当你准备好后，可以开始添加小幅度的更改并在实车上进行测试。对于任何较大的改动，请务必先与我们沟通。

在 Git 提交之后再到 Jetson 上进行测试。在你的个人电脑上运行以下命令推送到仓库：

```bash
git add .
git commit -m "<COMMIT_MSG>"
git push -u origin <BRANCH_NAME>
```

提交信息 (Commit messages) 请使用以下规范：

* `feat`: 为机器人添加的新功能
* `fix`: 修复 Bug
* `docs`: 仅文档更新
* `style`: 不影响代码逻辑的格式修改（例如：空格、缺失的分号等）
* `refactor`: 代码重构（不改变运行行为，也不修复 Bug）
* `perf`: 旨在提升性能的代码优化
* `test`: 添加缺失的测试或修正现有测试
* `build`: 影响构建工具、配置或外部依赖的更改
* `chore`: 不修改 `src/` 目录或测试文件的辅助性任务（例如：更新 `.gitignore`）

### 第五步：实车测试

在 Jetson 上部署：
```bash
ssh badger@100.79.128.21
git fetch
git switch <BRANCH_NAME>
git pull
```

如果你修改了 `src/` 目录下的任何代码，必须重新编译。常用编译命令：
```bash
# 分层编译
mbuild build-bringup
mbuild build-fastlio2
mbuild build-sensor
mbuild build-perception
mbuild build-planning
mbuild build-navigation

# 单包编译
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
ss
```

使用 `mbuild build-*` 会自动 source 新的工作空间。如果手动使用 `colcon build`，请记得保留 `--parallel-workers 1` 参数，并在编译后使用 `ss` 刷新环境变量。

如果编译成功，请根据修改范围，在合适的模式下启动系统：
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

至少需要验证以下几点：
* 所有相关节点均已上线
* 关键话题 / 动作 (actions) 有数据输出
* `map -> odom -> base_footprint -> base_link` 的 TF 树链路完整（在 Foxglove 中没有断链）
* 移动机器人时，观察是否出现异常行为
* 会话日志 (Session logs) 成功保存在 `~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/` 目录下

### 第六步：验证更改

在实车上验证代码更改后，请检查你所做的新修改并确认：

1. 所有启动模式在车端是否都能成功编译？
2. 是否改变了现有的启动链路？
3. 新修改影响了哪些启动模式、节点或话题？
4. 是否需要引入新的依赖？如果是，`package.xml` 和 `CMakeLists.txt` 是否都已同步更新？
5. 参数的修改是否正确反映在了 `master_params.yaml` 中？
6. 此次修改是否引入了任何潜在的风险点？

你可以使用 AI 智能助手来协助检查上述注意事项。如果有任何未完全理解的代码，请向我们提问。

### 第七步：编写文档

到这里，我们假设你已完成任务且代码在实车上运行正常。最后一步是编写文档。你至少需要更新以下内容：

* 任何新增或受影响的命令都需要更新到 [commands.md](https://www.google.com/search?q=docs-EN/commands.md) 中
* 必须在最新的开发日志 (devlog) 中记录所有更改的摘要
* 你**必须**同时更新中文版和英文版。你可以用母语编写，然后使用 AI 翻译成另一种语言。

> 如果你在 [master_params.yaml](https://www.google.com/search?q=src/bringup/config/master_params.yaml) 中修改了任何参数，你**必须**撰写一份详尽的分析报告，说明修改原因、对机器人的影响以及需要考虑的边缘情况 (edge cases)。

开发日志必须遵循以下格式：

```md
## YYYY.MM.DD

### [修改主题/标题]

#### 变动文件
- `file1/`
- `file2/`

#### 变更内容
#### 修改原因
#### 带来影响
```

完成后，请确保使用 `git push` 上传了所有更改。

### 第八步：提交 Pull Request (PR)

在上传完所有内容后，前往 [github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk) 创建一个 Pull Request。请遵循默认的 PR 格式和检查清单 (checklist)。