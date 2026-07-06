# RTK 测试命令

首先，确保机器人连接的是你的手机热点。切勿使用校园网（Wi-Fi），它非常不稳定。然后，确保你已经启用了 Tailscale。

你需要打开两个终端。

通过 SSH 登录机器人：
```bash
ssh badger@100.79.128.21
```

## NTRIP 账号

RTK 需要一个 NTRIP 账号。你 SSH 的时候，屏幕上会出现当前有没有账号。如需要买新账号，可以点出这里：https://e.tb.cn/h.Ry4kJCGRkkS8a8n?tk=VpOEgN1OG2z

购买后，运行以下命令：
```bash
make ntrip-setup
```

登录成功后，账号信息会自动保留。除非需要更改账号，否则不需要重新登录（包括切换 terminal 或关掉机器人）。

***

## 运行并收集数据

打开 2 个终端。

1. 在第一个终端中，启动 RTK：
```bash
make launch-rtk-basic
```

2. 在第二个终端中，查看 RTK 信号数据：
```bash
ros2 topic echo /rtk/status
```

你可以通过运行 `Ctrl C` 停止查看数据。

RTK 信号状态应该达到 `q=4`。

- 如果 `q=0`，说明 RTK 没有信号。请前往室外开阔区域。
- 如果你已经完全处于室外但 `q=1`，请确保你已成功登录账号。

Use the custom python script:
```bash
python3 /tmp/rtk_heading_calibrate.py
```

完成后，只需运行以下命令即可停止记录数据：

```bash
make kill
```

你也可以 `Ctrl C` 停止。

数据将保存在机器人的 `/runtime-data/logs/` 目录下。

当你想将数据传输到电脑时，可以使用 U 盘，也可以将机器人中的文件上传到网上。你可以运行以下命令：

```bash
cd XJTLU-autonomous-vehicle/
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
```

然后等待上传完成。你可以在这里查看文件：https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main （联系我获取访问权限）

***

## RTK offset test script

```bash
cat > /tmp/rtk_heading_calibrate.py <<'PY'
#!/usr/bin/env python3
import math
import re
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def wrap360(angle):
    return angle % 360.0


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


class Calibrator(Node):
    def __init__(self):
        super().__init__("rtk_heading_calibrator")
        self.lock = threading.Lock()
        self.quality = 0
        self.collecting = False
        self.fixes = deque(maxlen=2000)
        self.headings = []
        self.create_subscription(NavSatFix, "/fix", self.fix_cb, 50)
        self.create_subscription(String, "/rtk/status", self.status_cb, 20)

    def fix_cb(self, msg):
        with self.lock:
            if self.quality == 4 and math.isfinite(msg.latitude):
                self.fixes.append(
                    (time.monotonic(), msg.latitude, msg.longitude)
                )

    def status_cb(self, msg):
        q_match = re.search(r"\bq=(\d+)\b", msg.data)
        # raw_match = re.search(r"\braw=([-+]?\d+(?:\.\d+)?)", msg.data)
        raw_match = re.search(r"\bheading=([-+]?\d+(?:\.\d+)?)", msg.data)
        with self.lock:
            if q_match:
                self.quality = int(q_match.group(1))
            if self.collecting and self.quality == 4 and raw_match:
                self.headings.append(float(raw_match.group(1)))

    def position_average(self, seconds=5.0):
        cutoff = time.monotonic() - seconds
        with self.lock:
            points = [(lat, lon) for t, lat, lon in self.fixes if t >= cutoff]
        if len(points) < 5:
            raise RuntimeError("Not enough RTK Fixed position samples")
        lat = sum(p[0] for p in points) / len(points)
        lon = sum(p[1] for p in points) / len(points)
        return lat, lon, len(points)


rclpy.init()
node = Calibrator()
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()

try:
    input("Keep still for 5 seconds, then press Enter to mark START: ")
    start_lat, start_lon, start_count = node.position_average()
    with node.lock:
        node.headings.clear()
        node.collecting = True

    input("Drive straight, stop for 5 seconds, then press Enter to mark END: ")
    with node.lock:
        node.collecting = False
        raw_values = list(node.headings)
    end_lat, end_lon, end_count = node.position_average()

    if len(raw_values) < 5:
        raise RuntimeError("Not enough valid raw heading samples")

    radius = 6378137.0
    mean_lat = math.radians((start_lat + end_lat) / 2.0)
    north = radius * math.radians(end_lat - start_lat)
    east = radius * math.cos(mean_lat) * math.radians(end_lon - start_lon)
    distance = math.hypot(east, north)
    vehicle_heading = wrap360(math.degrees(math.atan2(east, north)))

    sin_sum = sum(math.sin(math.radians(h)) for h in raw_values)
    cos_sum = sum(math.cos(math.radians(h)) for h in raw_values)
    raw_heading = wrap360(math.degrees(math.atan2(sin_sum, cos_sum)))
    spread = max(abs(wrap180(h - raw_heading)) for h in raw_values)
    offset = wrap180(vehicle_heading - raw_heading)

    print("\n===== Calibration result =====")
    print(f"Distance:        {distance:.3f} m")
    print(f"Vehicle heading: {vehicle_heading:.3f} deg")
    print(f"Raw heading:     {raw_heading:.3f} deg")
    print(f"Raw max spread:  {spread:.3f} deg")
    print(f"OFFSET:          {offset:.3f} deg")
    print(f"heading_offset_deg: {offset:.3f}")
finally:
    rclpy.shutdown()
    thread.join(timeout=1.0)
PY

chmod +x /tmp/rtk_heading_calibrate.py
```