# Hardware Specifications

> Last updated: 2026-03-20
>
> This document records the models, parameters, and installation configurations of all hardware components on the current vehicle.
> Data sources: senior theses (Jia He 2025, Li Tongfei 2025, Tang Longbin 2024) + on-vehicle measurements + configuration files.
> Software configuration parameters are in src/bringup/config/master_params.yaml.

---

## 1. Vehicle Overview

| Item | Value | Source |
|------|-------|--------|
| Overall dimensions (LxWxH) | **650 x 500 x 450 mm** | Jia He, Solidworks |
| Bare chassis dimensions | 600 x 520 x 305 mm | Tang Longbin, Sec 4.1.1 |
| Simulation model mass | ~25 kg | Jia He, Sec E.2 |
| Z-axis moment of inertia | 4 kg*m^2 | Jia He, Sec E.2 |
| Frame material | European standard 3030 aluminum extrusion + die-cast connectors + 3D printed parts | Tang Longbin |
| Shell | PLA 3D printed (Bambu Lab P1S), 4 pieces | Li Tongfei |
| robot_radius (Nav2) | 0.38625 m | nav2_explore.yaml |

## 2. Chassis and Drivetrain

| Item | Value | Source |
|------|-------|--------|
| Drive type | Omnidirectional differential drive | All three theses |
| Total wheels | **6** (3 left + 3 right) | Jia He, Sec 2.2 |
| Powered wheels | 4x 90-degree omnidirectional wheels (front-left/rear-left/front-right/rear-right) | Jia He; Tang |
| Passive wheels | 2x rubber wheels (mid-left/mid-right), anti-sideslip | Jia He |
| Wheel diameter | **85 mm** (model KJ85T33) | Jia He, Solidworks |
| Track width | 0.46 m | STM32 source code |
| Minimum turning radius | 0 (can rotate in place) | Tang Longbin |
| Suspension | Independent double wishbone per wheel, 80-140 N/wheel | Tang Longbin |

### 2.1 Motors

| Item | Value | Source |
|------|-------|--------|
| Model | DJI RM 3508 BLDC x4 | All three theses |
| Rated voltage | 24V | Tang, Fig 4.16 |
| No-load speed | 482 rpm | Tang, Fig 4.16 |
| Max continuous torque | 3 N*m | Tang, Fig 4.16 |
| Gear ratio | 19.2:1 | Jia He, Sec 2.3.1.A |
| Encoder | Built-in, 4-channel wheel speed feedback | Jia He |

### 2.2 ESCs

| Item | Value |
|------|-------|
| Model | DJI C620 |
| Rated voltage | 24V |
| Max continuous current | 20A |
| Signal | CAN or 50-500Hz PWM |
| Dimensions | 49.4x25.8x11.5 mm, 35g |

### 2.3 Lower-Level Controller

| Item | Value |
|------|-------|
| MCU | STM32F407XX (Cortex-M4), RM C Board |
| Motor communication | CAN -> 4x RM3508 |
| Upper-level communication | USB ttyACM0, 115200 baud |
| Downlink protocol | vcx=X.XXX,wc=X.XXX\n |
| Uplink protocol | 16 float @100Hz |
| PID | Kp=10, Ki=100, Kd=0, MaxOutput=15000 |
| Device file | /dev/serial_twistctl |

### 2.4 Speed Performance

| Item | Value | Source |
|------|-------|--------|
| Max tested linear velocity | **0.7 m/s** | Jia He |
| Max tested angular velocity | **1 rad/s** | Jia He |
| Trajectory optimizer speed limit | 0.6 m/s | Jia He |
| Max acceleration (optimizer) | 0.3 m/s^2 | Jia He |
| Nav2 max_vel_x | 0.5 m/s | Current config |
| Straight-line tracking accuracy (0.7 m/s) | 3 cm | Jia He |
| 90-degree turn accuracy | 4 cm | Jia He |

### 2.5 Remote Controller

| Item | Value |
|------|-------|
| Type | 2.4 GHz wireless gamepad (PS2 compatible) |
| Left stick | Linear velocity |
| Right stick | Angular velocity |
| Buttons | Y=enable, X=disable/coast stop, B=backup coast stop |
| Priority | **Highest software disable is X**; the red physical e-stop overrides all software commands |
| Disconnect protection | Automatic disable on signal loss |

## 3. Computing Platform

| Item | Value |
|------|-------|
| Model | NVIDIA Jetson Orin NX 16GB |
| GPU | Ampere, 1024 CUDA, 32 Tensor |
| AI performance | 100 TOPS (INT8) |
| CPU | 8-core ARM Cortex-A78AE |
| Memory | 16GB LPDDR5, 102.4 GB/s |
| OS | Ubuntu 22.04 + ROS2 Humble |
| Ethernet | enP8p1s0, 192.168.1.50/24 |
| Remote access | Tailscale VPN 100.97.227.24 |

## 4. LiDAR

| Item | Value | Source |
|------|-------|--------|
| Model | Livox MID-360 | Jia He |
| FOV | 360x59 degrees | Official |
| Scan pattern | Non-repetitive scanning | Official |
| Range | 40 m @10%, 70 m @80% | Official |
| Point rate | ~200k pts/s | Official |
| Built-in IMU | ~200 Hz | Jia He |
| Connection | Ethernet direct | Jia He |
| Mounting X offset | **-0.07 m** | Jia He, Fig18 |
| Mounting Y offset | **+0.12 m** (senior thesis stated -0.12, corrected after measurement) | Jia He, Fig18 |
| Mounting height (above ground) | **0.447 m** | 2026-03-20 measured |
| lidar_max_range | 15.0 m | master_params |
| lidar_min_range | 0.5 m | master_params |
| t_il (LiDAR-IMU) | [-0.011, -0.02329, 0.04412] m | master_params |

## 5. IMU

### 5.1 Primary IMU (C Board built-in, currently not used by ROS2)

| Item | Value |
|------|-------|
| Accelerometer | BMI088, 0.09 mg resolution, 10 MHz SPI |
| Gyroscope | BMI088, 0.004 deg/s resolution |
| Magnetometer | IST8310, 0.3 uT/LSB, 400 KHz I2C |
| Temperature control | PID, target 15-20C |
| Fusion | Madgwick AHRS @500 Hz |

### 5.2 External IMU (WIT, for GPS planning)

| Item | Value |
|------|-------|
| Brand | Hiwonder |
| Magnetometer | +/-2 Gauss, 0.0667 mGauss/LSB |
| ROS2 package | wit_ros2_imu |
| Device | /dev/imu_usb (CH340) |

## 6. GNSS

| Item | Value |
|------|-------|
| Module | WHEELTEC G60 (ATGM336H-5N) |
| Systems | BeiDou 2/3, GPS, GLONASS, QZSS |
| Accuracy | ~2.5 m (mushroom antenna measured 3-5 m) |
| Update rate | 5 Hz (typical), 10 Hz (max) |
| TTFF | 32 s (cold), 1 s (hot) |
| Baud rate | 9600 |
| Device | /dev/wheeltec_gps (CP2102) |
| Antenna | Beitian BT-800D mushroom, TNC |
| Feed cable | MCX->TNC, RG316 (**replaced 2026-03-20, device enumeration normal (RF quality pending outdoor verification)**) |
| GPS Factor | noise_xy=2.5 m, interval=10, hdop_max=3.0 |

## 7. Power

| Item | Value |
|------|-------|
| Battery | 7S lithium (24V nominal, 21-29.4V) |
| Capacity | 20 Ah, 2.2 kg |
| Operating current | 15A (typical), 30A (max) |
| Temperature | -20~60C |
| Distribution | DC-DC converters, independent power for each module |
| Safety | Four-pin switch + hardware emergency stop button |

## 8. Expansion Sensors (Not Integrated into Navigation Stack)

| Device | Model | Status |
|--------|-------|--------|
| Depth camera | Intel Realsense D455f (87x58 FOV, 90 Hz, 720p) | Expansion module |
| AI detection | YOLOv8 @10 Hz CUDA | Not integrated |

## 9. Device Interface Overview

| Device | Device File | Connection | udev |
|--------|-------------|------------|------|
| Livox MID360 | Ethernet 192.168.1.x | Ethernet | N/A |
| Chassis STM32 | /dev/serial_twistctl | USB (ttyACM0) | 2e3c:5740 |
| GPS G60 | /dev/wheeltec_gps | USB (ttyUSB0) | 10c4:ea60 |
| WIT IMU | /dev/imu_usb | USB (CH340) | 1a86:7523 |
| PS2 gamepad | N/A | 2.4 GHz->STM32 | N/A |

## 10. Historical Hardware Changes

| Component | 2024 (Tang) | 2025 (Jia He/Li) | 2026 (Current) |
|-----------|-------------|-------------------|-----------------|
| LiDAR | Unitree L1 | Livox MID-360 | Livox MID-360 |
| mmWave radar | 2x HLK-LD2461 | Removed | None |
| Depth camera | None | Realsense D455f | Not integrated |
| Navigation | Autoware.Universe | Custom ROS2 | FAST-LIO2+PGO+Nav2 |

## 11. Parameters Pending Measurement

| Parameter | Importance | Method |
|-----------|------------|--------|
| LiDAR mounting height (above ground) | **0.447 m (447 mm)** | 2026-03-20 measured |
| GPS antenna position (relative to base_link) | **X=-0.105 m, Y=+0.045 m** | 2026-03-20 measured |
| Actual vehicle weight | **~25 kg** (simulation value, used for now) | Jia He simulation |
| NVMe SSD capacity | Low | lsblk |
