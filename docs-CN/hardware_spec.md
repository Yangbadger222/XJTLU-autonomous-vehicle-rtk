# 硬件规格参数

> 最后更新: 2026-06-25
>
> 本文档记录当前车辆上所有硬件组件的型号、参数和安装配置。
> 数据来源: 学长论文 (Jia He 2025, Li Tongfei 2025, Tang Longbin 2024) + 实车实测 + 配置文件。
> 软件配置参数见 src/bringup/config/master_params.yaml。

---

## 1. 车辆整体

| 项目 | 值 | 来源 |
|------|-----|------|
| 整车尺寸 (长x宽x高) | **650 x 500 x 450 mm** | Jia He, Solidworks |
| 底盘裸尺寸 | 600 x 520 x 305 mm | Tang Longbin, Sec 4.1.1 |
| 仿真建模质量 | ~25 kg | Jia He, Sec E.2 |
| Z轴转动惯量 | 4 kg*m^2 | Jia He, Sec E.2 |
| 车架材料 | 欧标3030铝型材 + 压铸连接件 + 3D打印件 | Tang Longbin |
| 外壳 | PLA 3D打印 (Bambu Lab P1S), 分4块 | Li Tongfei |
| robot_radius (Nav2) | 0.38625 m | nav2_explore.yaml |

## 2. 底盘与驱动

| 项目 | 值 | 来源 |
|------|-----|------|
| 驱动方式 | 全向差速驱动 | 三篇论文 |
| 车轮总数 | **6个** (左3+右3) | Jia He, Sec 2.2 |
| 动力轮 | 4个90度全向轮 (左前/左后/右前/右后) | Jia He; Tang |
| 被动轮 | 2个橡胶轮 (左中/右中), 防侧滑 | Jia He |
| 轮径 | **85 mm** (型号 KJ85T33) | Jia He, Solidworks |
| 轮距 (track width) | 0.46 m | STM32 源码 |
| 最小转弯半径 | 0 (可原地旋转) | Tang Longbin |
| 悬挂 | 每轮独立双叉臂, 80-140 N/轮 | Tang Longbin |

### 2.1 电机

| 项目 | 值 | 来源 |
|------|-----|------|
| 型号 | DJI RM 3508 BLDC x4 | 三篇论文 |
| 额定电压 | 24V | Tang, Fig 4.16 |
| 空载转速 | 482 rpm | Tang, Fig 4.16 |
| 最大持续扭矩 | 3 N*m | Tang, Fig 4.16 |
| 减速比 | 19.2:1 | Jia He, Sec 2.3.1.A |
| 编码器 | 内置, 4路轮速反馈 | Jia He |

### 2.2 电调

| 项目 | 值 |
|------|-----|
| 型号 | DJI C620 |
| 额定电压 | 24V |
| 最大持续电流 | 20A |
| 信号 | CAN 或 50-500Hz PWM |
| 尺寸 | 49.4x25.8x11.5 mm, 35g |

### 2.3 下位机

| 项目 | 值 |
|------|-----|
| MCU | STM32F407XX (Cortex-M4), RM C Board |
| 电机通信 | CAN -> 4x RM3508 |
| 上位机通信 | USB ttyACM0, 115200 baud |
| 下行协议 | vcx=X.XXX,wc=X.XXX\n |
| 上行协议 | 16 float @100Hz |
| PID | Kp=10, Ki=100, Kd=0, MaxOutput=15000 |
| 设备文件 | /dev/serial_twistctl |

### 2.4 速度性能

| 项目 | 值 | 来源 |
|------|-----|------|
| 最大测试线速度 | **0.7 m/s** | Jia He |
| 最大测试角速度 | **1 rad/s** | Jia He |
| 轨迹优化器限速 | 0.6 m/s | Jia He |
| 最大加速度 (优化器) | 0.3 m/s^2 | Jia He |
| Nav2 max_vel_x | 0.5 m/s | 当前配置 |
| 直线跟踪精度 (0.7 m/s) | 3 cm | Jia He |
| 90度弯道精度 | 4 cm | Jia He |

### 2.5 遥控器

| 项目 | 值 |
|------|-----|
| 类型 | 2.4GHz无线手柄 (PS2兼容) |
| 左摇杆 | 线速度 |
| 右摇杆 | 角速度 |
| 按键 | Y=使能, X=失能/撤力滑停, B=阻尼主动刹车后零电流释放 |
| 优先级 | **最高软件失能为 X 键**；车身红色物理急停覆盖一切软件命令 |
| 断连保护 | 信号丢失自动失能 |

## 3. 计算平台

| 项目 | 值 |
|------|-----|
| 核心模块 | NVIDIA Jetson Orin NX 16GB |
| 载板 | Seeed reComputer 载板 (4x USB 3.0 Type-A, 1x GbE, 1x HDMI) |
| GPU | Ampere, 1024 CUDA, 32 Tensor |
| AI算力 | 100 TOPS (INT8) |
| CPU | 8核 ARM Cortex-A78AE |
| 内存 | 16GB LPDDR5, 102.4 GB/s |
| OS | Ubuntu 22.04 + ROS2 Humble |
| 以太网 | enP8p1s0, 192.168.1.50/24 |
| 远程访问 | Tailscale VPN 100.79.128.21 |

## 4. LiDAR

| 项目 | 值 | 来源 |
|------|-----|------|
| 型号 | Livox MID-360 | Jia He |
| FOV | 360x59 度 | 官方 |
| 扫描方式 | 非重复扫描 | 官方 |
| 量程 | 40 m @10%, 70 m @80% | 官方 |
| 点频 | ~200k pts/s | 官方 |
| 内置IMU | ~200 Hz | Jia He |
| 连接 | 以太网直连 | Jia He |
| 安装X偏移 | **-0.07 m** | Jia He, Fig18 |
| 安装Y偏移 | **+0.12 m** (学长论文写-0.12有误, 实测修正) | Jia He, Fig18 |
| 安装高度 (相对地面) | **0.447 m** | 2026-03-20 实测 |
| lidar_max_range | 15.0 m | master_params |
| lidar_min_range | 0.5 m | master_params |
| t_il (LiDAR-IMU) | [-0.011, -0.02329, 0.04412] m | master_params |

## 5. IMU

### 5.1 主IMU (C Board内置, 当前未被ROS2使用)

| 项目 | 值 |
|------|-----|
| 加速度计 | BMI088, 0.09 mg 分辨率, 10 MHz SPI |
| 陀螺仪 | BMI088, 0.004 deg/s 分辨率 |
| 磁力计 | IST8310, 0.3 uT/LSB, 400 KHz I2C |
| 温控 | PID, 目标15-20C |
| 融合 | Madgwick AHRS @500 Hz |

### 5.2 外置IMU (WIT, GPS规划用)

| 项目 | 值 |
|------|-----|
| 品牌 | 幻尔科技 |
| 磁力计 | +/-2 Gauss, 0.0667 mGauss/LSB |
| ROS2包 | wit_ros2_imu |
| 设备 | /dev/imu_usb (CH340) |

## 6. GNSS

| 项目 | 值 |
|------|-----|
| 模块 | T-RTK UM982 双天线 Mobile 套装 + 4G 模块 (移动端) |
| 系统 | GPS L1/L2/L5, BDS B1I/B2I/B3I/B1C/B2a, GLONASS, Galileo, QZSS (全系统多频 RTK) |
| 精度 | RTK 厘米级定位 (自带有源 4G 模块，通过移动网络引入 NTRIP / CORS 差分数据) |
| 信号硬件 | GNSS 天线 + 碳纤天线支架、50 cm 馈线 x2、4G 模块及配套天线 |
| 配套线材 | GH1.25 / JST-GH 转 USB 通讯线 |
| 航向策略 | 双天线定向硬件功能短期内暂不启用。静止 yaw 仍由 FAST-LIO2 / IMU / launch_yaw_deg 处理，避免底盘打孔或外贴双天线带来的结构风险。 |
| 设备文件 | /dev/rtk_gps |
| GPS Factor | noise_xy=2.5 m, interval=10, hdop_max=3.0 (未更新) |

## 7. 电源

| 项目 | 值 |
|------|-----|
| 电池 | 7S锂电 (24V nominal, 21-29.4V) |
| 容量 | 20 Ah, 2.2 kg |
| 工作电流 | 15A (typical), 30A (max) |
| 温度 | -20~60C |
| 分配 | DC-DC转换器, 各模块独立供电 |
| 安全 | 四针开关 + 硬件急停按钮 |

## 8. 扩展传感器 (未集成到导航栈)

| 设备 | 型号 | 状态 |
|------|------|------|
| 深度相机 | Intel Realsense D455f (87x58 FOV, 90 Hz, 720p) | 扩展模块 |
| AI检测 | YOLOv8 @10 Hz CUDA | 未集成 |

## 9. 设备接口总览

| 设备 | 设备文件 | 连接 | udev |
|------|---------|------|------|
| Livox MID360 | 以太网 192.168.1.x | 以太网 (RJ45网口) | N/A |
| 底盘STM32 | /dev/serial_twistctl | USB 3.0 (Type-A) | 2e3c:5740 |
| T-RTK UM982 模块 | /dev/rtk_um982 | USB 3.0 (Type-A 串口连线) | 10c4:ea60 |
| WIT IMU | /dev/imu_usb | USB 3.0 (Type-A) | 1a86:7523 |
| PS2手柄 | N/A | 2.4 GHz->STM32 | N/A |

## 10. 历史硬件变更

| 组件 | 2024 (Tang) | 2025 (Jia He/Li) | 2026 (当前) |
|------|-------------|-------------------|-------------|
| 载板 | 通用 OEM 载板 | S350 V1.1 OEM 载板 (仅 USB 2.0) | Seeed reComputer 载板 (4x USB 3.0) |
| LiDAR | Unitree L1 | Livox MID-360 | Livox MID-360 |
| GNSS / GPS | WHEELTEC G60 (单频) | WHEELTEC G60 (单频) | T-RTK UM982 + 4G 模块 (RTK 厘米级) |
| 毫米波雷达 | 2x HLK-LD2461 | 已移除 | 无 |
| 深度相机 | 无 | Realsense D455f | Realsense D455f |
| 导航 | Autoware.Universe | 自研ROS2 | FAST-LIO2+PGO+Nav2 |

## 11. 待实测参数

| 参数 | 重要性 | 获取方式 |
|------|--------|---------|
| LiDAR安装高度 (相对地面) | **0.447 m (447 mm)** | 2026-03-20 实测 |
| RTK 天线位置 (相对 base_link) | 高 | 大概 x: +140 mm, y: 0 mm |
| 整车实际重量 | **~25 kg** (仿真值, 暂用) | Jia He 仿真 |
| NVMe SSD容量 | 低 | lsblk |