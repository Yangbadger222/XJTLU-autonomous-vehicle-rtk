# serial

这个目录保存项目当前使用的 C++ 串口库源码，主要被 `serial_twistctl`、`gyro_odometry` 等包依赖。

> XJTLU 项目说明：该目录是仓库内保留的上游串口库依赖，不是车辆项目原创应用层代码。修改时应保留原始作者、许可证和 API 语义。

## 当前维护方式

- 不按独立外部库单独执行旧的 `cmake && make install` 流程。
- 在本项目中统一作为 monorepo 内部依赖一起编译。
- 当前工作区是 ROS 2 Humble，不再按旧 README 里的 Foxy 独立安装方式维护。

## 构建方式

从工作区根目录执行:

```bash
colcon build --packages-select serial --symlink-install --parallel-workers 1
source install/setup.bash
```

如果只是构建依赖它的上层包，通常不需要单独执行这一条。

## 说明

- 该包提供底层串口访问 API，不直接形成整车模式入口。
- 上游来源仍是 `wjwwood/serial`，许可证和作者声明以源码中的原始内容为准。
