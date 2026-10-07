# Research operator cockpit

The unified research launch starts the loopback cockpit at
`http://127.0.0.1:8765`. Use SSH forwarding for the authorized remote:

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 badger@100.88.131.52
```

Startup selects replay/shadow/live; the browser cannot enable physical serial,
change parameters or publish velocities. Replay defaults to no actuator or
mission execution. Use an isolated task domain and `ROS_LOCALHOST_ONLY=1`,
with explicit task FYP/ROS logging roots.

Claim one operator window, inspect measured readiness, confirm standing still
under the original generated thresholds, reset, select registered-prior
endpoints, wait for the task acknowledgement, then explicitly request start.
Only the configured live environment can request motion. RTK, original stop
arbitration, KEY, joystick and physical emergency stop remain independent.

Pause, software stop and takeover waiting revoke consent. Takeover waiting
requires the original on-site hardware; there is no browser joystick.
Heartbeat loss, state-reading failure, backend failure, downstream transport
loss or publisher competition latch consent off. Restoration cannot rearm it;
an explicit READY/reset then start sequence is required. Ordered typed permits
carry session, mode and map version. Normal evidence version advancement still
requires matching current map/trajectory/consent snapshots.

Hidden tabs pause autonomous tasks and stop renewing the lease. Escape invokes
software stop. Measured stillness does not validate physical braking.

An optional `bag_catalog_path` supplies original raw input choices. Cataloging
deduplicates CDR streams; playback also verifies sealed metadata, file state and
full storage SHA, including recording timestamps. Only raw LiDAR/IMU and the
measurement clock are played, at 1x. Recorded commands, TF, authority and old
LIO output are excluded. Player controls target only owned process groups.

The map displays ROS grids, permission, pose, timed trajectory and local road
evidence. Display decimation never changes planning inputs. Unknown ground,
missing transforms/health and unregistered priors remain explicit blockers.
The loopback server requires matching Host/Origin, session cookie, window ID
and CSRF, with bounded threads and read deadlines. It exposes no arbitrary
topic, shell, file-path or parameter editing.

Read `audit/optimization_v2/`, `RESULTS.json` and
`LIVE_ACCEPTANCE_CHECKLIST.md` for executed lab tests and pending vehicle gates.

Raw pause uses SIGSTOP on the owned player group; resume may catch up scheduling time. The pause/resume acceptance is not a strict wall-clock pacing test.

Current qualification uses a separate source-bound14-package workspace with2 Super/6 EGO patches,16 ROS support fixtures and39 final-PTY faults. The preserved preview remains atce46418; it is not evidence that the new default control/real terrain/road pipeline or hardware acceptance is complete. Recorded vehicle data are available and used; firmware identity and physical braking/estop remain on-site requirements. See RESULTS.json.
