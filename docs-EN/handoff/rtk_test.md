# RTK test commands

First, make sure the robot uses your phone hotspot or the Pisen USB Wi-Fi. Do NOT use our campus wifi, it is very unstable. Then, make sure you have tailscale activated.

SSH into the robot:
```bash
ssh badger@100.79.128.21
```

## NTRIP Account

The RTK requires an NTRIP account. After you SSH, the welcome message will say whether there is an existing account. If you need a new account, you can buy it here: https://e.tb.cn/h.Ry4kJCGRkkS8a8n?tk=VpOEgN1OG2z

After purchasing it, run this command:
```bash
make ntrip-setup
```

Then, input all the account details in there. If successful, the account is saved in the robot, and you do not have to enter it again unless you want to change the account.

***

## Run and collect data

Open 2 terminals. 

1. In one terminal, start the RTK:
```bash
make launch-rtk-basic
```

2. In another terminal, see the RTK signal data:
```bash
ros2 topic echo /rtk/status
```

You can stop seeing the data by running `Ctrl C`.

The RTK signal status should achieve `q=4`.
- If it is `q=0`, the RTK has no signal. Go outdoors in a clear area.
- If you are completely outside but `q=1`, make sure you are logged in.

Use the custom python script:
```bash
python3 /tmp/rtk_heading_calibrate.py
```

When you are done, simply run this to stop recording data:
```bash
make kill
```

You may also press `Ctrl + C`.

The data will be stored in the robot in `/runtime-data/logs/`. When you want to transfer the data to your computer, you can use a USB driver, or you can also upload the robot files online. You can run this command:

```bash
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
```

Then wait until it uploads. You can see the files here: https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main (contact me for access)

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