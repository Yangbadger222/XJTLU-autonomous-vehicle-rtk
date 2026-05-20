# waypoint_collector

`waypoint_collector` is an interactive ROS 2 helper for collecting RViz waypoints and sending them to Nav2 through the `FollowWaypoints` action. It is useful for indoor click-to-go experiments and route rehearsal, but it is not the main GPS Corridor runner.

## Role In This Repository

- Collect intermediate waypoints from RViz `Publish Point`.
- Use RViz `2D Goal Pose` as the final goal and trigger execution.
- Send the full sequence to Nav2's `/follow_waypoints` action.
- Publish RViz markers so the operator can see waypoint order before execution.

## Interfaces

Subscribed topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | Intermediate points from RViz Publish Point |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Final goal from RViz 2D Goal Pose |

Published topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/waypoint_markers` | `visualization_msgs/msg/MarkerArray` | RViz waypoint markers and ordering |

Action client:

| Action | Type | Purpose |
|--------|------|---------|
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Nav2 waypoint execution |

## Build

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select waypoint_collector --symlink-install --parallel-workers 1
source install/setup.bash
```

## Run

Start a Nav2-capable mode first:

```bash
make launch-indoor-nav
```

Then run the collector in a second terminal:

```bash
source ~/XJTLU-autonomous-vehicle/install/setup.bash
ros2 run waypoint_collector waypoint_node
```

## RViz Workflow

1. Add a `MarkerArray` display for `/waypoint_markers`.
2. Use `Publish Point` to add intermediate waypoints.
3. Use `2D Goal Pose` to set the final goal and start navigation.
4. Watch terminal feedback for accepted goals, active waypoint index, and failures.

## Current Limitations

- Nav2 must already be active, including the `waypoint_follower` action server.
- The `map -> odom -> base_link` TF chain and costmaps must be healthy before execution.
- Waypoints are executed in click order and are cleared after navigation completes.
- This tool does not validate whether a clicked point is reachable before sending the action.

## Troubleshooting

- If `/follow_waypoints` is unavailable, check that Nav2 is active with `ros2 action list`.
- If a goal is rejected, inspect the global costmap and verify the waypoint lies in reachable free space.
- If markers do not appear, add `/waypoint_markers` as a `MarkerArray` display in RViz.
