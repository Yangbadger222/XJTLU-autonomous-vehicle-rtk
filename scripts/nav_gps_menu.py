#!/usr/bin/env python3
"""
Interactive helper for on-vehicle nav-gps testing.

Default behavior:
1. Launch nav-gps stack
2. Show numbered destination menu from current scene_points.yaml
3. Wait for localization authority only when a destination is selected
4. Publish goto_name requests by numeric selection

Testing behavior:
  python3 scripts/nav_gps_menu.py --no-launch
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
import yaml


DEFAULT_SCENE_POINTS = (
    Path.home()
    / "XJTLU-autonomous-vehicle/runtime-data"
    / "gnss"
    / "current_scene"
    / "scene_points.yaml"
)
DEFAULT_LATEST_LOG_DIR = Path.home() / "XJTLU-autonomous-vehicle/runtime-data" / "logs" / "latest"


def load_destinations(scene_points_file: Path) -> list[tuple[int, str]]:
    with open(scene_points_file, "r", encoding="utf-8") as scene_file:
        data = yaml.safe_load(scene_file) or {}

    raw_nodes = data.get("nodes", {})
    if not isinstance(raw_nodes, dict) or not raw_nodes:
        raise RuntimeError(f"scene points file has no nodes: {scene_points_file}")

    destinations: list[tuple[int, str]] = []
    for raw_id, raw_node in raw_nodes.items():
        if bool(raw_node.get("dest", False)):
            destinations.append((int(raw_id), str(raw_node["name"])))

    destinations.sort(key=lambda item: item[0])
    return destinations


class NavGPSMenu(Node):
    def __init__(self, scene_points_file: Path) -> None:
        super().__init__("nav_gps_menu")
        self.scene_points_file = scene_points_file

        self.system_status = "UNKNOWN"
        self.goal_status = "UNKNOWN"
        self.localization_authority_mode = "UNKNOWN"
        self.localization_motion_allowed = False
        self._last_system_status_printed: str | None = None
        self._last_goal_status_printed: str | None = None
        self._last_localization_authority_printed: str | None = None

        self.goto_pub = self.create_publisher(String, "/gps_waypoint_dispatcher/goto_name", 10)
        self.stop_pub = self.create_publisher(Empty, "/gps_waypoint_dispatcher/stop", 10)

        self.create_subscription(String, "/gps_system/status", self._system_status_callback, 10)
        self.create_subscription(
            String, "/gps_goal_manager/status", self._goal_status_callback, 10
        )
        self.create_subscription(
            String,
            "/localization_authority/mode",
            self._localization_authority_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/localization_authority/motion_allowed",
            self._localization_motion_callback,
            10,
        )

        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")

    def _system_status_callback(self, msg: String) -> None:
        self.system_status = msg.data.strip() or "UNKNOWN"

    def _goal_status_callback(self, msg: String) -> None:
        self.goal_status = msg.data.strip() or "UNKNOWN"

    def _localization_authority_callback(self, msg: String) -> None:
        self.localization_authority_mode = msg.data.strip() or "UNKNOWN"

    def _localization_motion_callback(self, msg: Bool) -> None:
        self.localization_motion_allowed = bool(msg.data)

    def print_status_changes(self) -> None:
        if self.system_status != self._last_system_status_printed:
            print(f"[gps_system] {self.system_status}")
            self._last_system_status_printed = self.system_status
        if self.goal_status != self._last_goal_status_printed:
            print(f"[goal_manager] {self.goal_status}")
            self._last_goal_status_printed = self.goal_status
        if self.localization_authority_mode != self._last_localization_authority_printed:
            print(f"[localization_authority] {self.localization_authority_mode}")
            self._last_localization_authority_printed = self.localization_authority_mode

    def action_servers_ready(self) -> bool:
        return self.follow_path_client.wait_for_server(timeout_sec=0.1)

    def publish_goto_name(self, destination_name: str) -> None:
        message = String(data=destination_name)
        for _ in range(3):
            self.goto_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.2)

    def publish_stop(self) -> None:
        message = Empty()
        for _ in range(3):
            self.stop_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.2)

    def wait_for_nav_ready(
        self,
        timeout_s: float,
        launch_proc: subprocess.Popen[str] | None = None,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.print_status_changes()

            if launch_proc is not None and launch_proc.poll() is not None:
                raise RuntimeError(
                    f"nav-gps launch exited early with code {launch_proc.returncode}"
                )

            if self.localization_motion_allowed and self.action_servers_ready():
                print(
                    "[nav_gps_menu] Motion authority is ready "
                    f"({self.localization_authority_mode}) and FollowPath is online."
                )
                return

        detail = (
            f"last gps_system={self.system_status}, "
            f"localization_authority={self.localization_authority_mode}, "
            f"motion_allowed={self.localization_motion_allowed}, "
            f"goal_manager={self.goal_status}"
        )
        if self.system_status == "NO_FIX":
            detail += "; raw GPS still has no valid fix"
        elif self.system_status == "NO_ANCHOR":
            detail += "; current position is not close enough to any anchor"
        elif self.system_status == "AMBIGUOUS_ANCHOR":
            detail += "; anchor match is ambiguous"
        elif self.system_status == "UNSTABLE_FIX":
            detail += "; GNSS samples are still unstable"
        raise TimeoutError(f"Timed out waiting for localization motion authority ({detail})")

    def wait_for_goal_result(
        self,
        timeout_s: float,
        launch_proc: subprocess.Popen[str] | None = None,
    ) -> None:
        terminal_prefixes = ("SUCCEEDED", "FAILED", "CANCELLED", "REJECTED")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.print_status_changes()

            if launch_proc is not None and launch_proc.poll() is not None:
                raise RuntimeError(
                    f"nav-gps launch exited early with code {launch_proc.returncode}"
                )

            if self.goal_status.startswith(terminal_prefixes):
                return

        raise TimeoutError("Timed out waiting for goal result")


def launch_nav_gps(repo_root: Path) -> subprocess.Popen[str]:
    command = (
        f"cd {repo_root} && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "bash scripts/launch_with_logs.sh nav-gps"
    )
    return subprocess.Popen(
        ["bash", "-lc", command],
        preexec_fn=os.setsid,
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )


def stop_launch(launch_proc: subprocess.Popen[str] | None) -> None:
    if launch_proc is None:
        return

    if launch_proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        launch_proc.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGTERM)
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch nav-gps and choose destinations by number."
    )
    parser.add_argument(
        "--scene-points-file",
        default=str(DEFAULT_SCENE_POINTS),
        help="Path to scene_points.yaml. Default: current runtime scene",
    )
    parser.add_argument(
        "--repo-root",
        default=str(Path.home() / "XJTLU-autonomous-vehicle"),
        help="Workspace root used for launching nav-gps",
    )
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for localization motion authority before giving up",
    )
    parser.add_argument(
        "--goal-timeout",
        type=float,
        default=600.0,
        help="Seconds to wait for goal result after selection",
    )
    parser.add_argument(
        "--no-launch",
        action="store_true",
        help="Skip launching nav-gps. Useful for smoke tests against an already-running stack.",
    )
    parser.add_argument(
        "--wait-before-menu",
        action="store_true",
        help="Wait for motion authority before showing destinations. Default shows the menu first.",
    )
    return parser.parse_args()


def print_menu(destinations: list[tuple[int, str]]) -> None:
    print()
    print("Available destinations:")
    for index, (node_id, name) in enumerate(destinations, start=1):
        print(f"  {index}. {name} (node {node_id})")
    print()
    print("Commands:")
    print("  <number>  send goto_name for that destination")
    print("  r         reload destination list")
    print("  s         publish stop")
    print("  q         quit and stop launched nav-gps")


def main() -> int:
    args = parse_args()
    scene_points_file = Path(args.scene_points_file).expanduser()
    repo_root = Path(args.repo_root).expanduser()

    if not scene_points_file.exists():
        print(f"scene_points file not found: {scene_points_file}", file=sys.stderr)
        return 1

    launch_proc: subprocess.Popen[str] | None = None
    rclpy.init()
    node = NavGPSMenu(scene_points_file)

    try:
        if not args.no_launch:
            print("[nav_gps_menu] Launching nav-gps in background ...")
            print(f"[nav_gps_menu] Session logs: {DEFAULT_LATEST_LOG_DIR}")
            launch_proc = launch_nav_gps(repo_root)
            time.sleep(2.0)

        if args.wait_before_menu:
            print("[nav_gps_menu] Waiting for localization motion authority ...")
            node.wait_for_nav_ready(args.ready_timeout, launch_proc=launch_proc)
        else:
            print("[nav_gps_menu] Checking nav-gps startup status ...")
            startup_deadline = time.time() + 5.0
            while time.time() < startup_deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                node.print_status_changes()
                if launch_proc is not None and launch_proc.poll() is not None:
                    raise RuntimeError(
                        f"nav-gps launch exited early with code {launch_proc.returncode}"
                    )
                if node.localization_motion_allowed and node.action_servers_ready():
                    print(
                        "[nav_gps_menu] Motion authority is ready "
                        f"({node.localization_authority_mode}) and FollowPath is online."
                    )
                    break
            else:
                print(
                    "[nav_gps_menu] Menu is available; selecting a destination "
                    "will wait for motion authority before sending the goal."
                )

        while True:
            destinations = load_destinations(scene_points_file)
            if not destinations:
                print("No destinations found in scene_points.yaml")
                return 1

            print_menu(destinations)
            raw = input("Select destination > ").strip().lower()

            if raw == "q":
                print("[nav_gps_menu] Quitting.")
                return 0

            if raw == "r":
                continue

            if raw == "s":
                print("[nav_gps_menu] Publishing stop.")
                node.goal_status = "STOP_REQUESTED"
                node.publish_stop()
                stop_deadline = time.time() + 3.0
                while time.time() < stop_deadline:
                    rclpy.spin_once(node, timeout_sec=0.2)
                    node.print_status_changes()
                continue

            if not raw.isdigit():
                print("Invalid input. Enter a number, r, s, or q.")
                continue

            selection = int(raw)
            if selection < 1 or selection > len(destinations):
                print(f"Invalid choice. Pick 1..{len(destinations)}")
                continue

            _, destination_name = destinations[selection - 1]
            if not (node.localization_motion_allowed and node.action_servers_ready()):
                print("[nav_gps_menu] Waiting for localization motion authority ...")
                node.wait_for_nav_ready(args.ready_timeout, launch_proc=launch_proc)
            print(f"[nav_gps_menu] Sending goal: {destination_name}")
            node.goal_status = "GOAL_REQUESTED"
            node.publish_goto_name(destination_name)
            node.wait_for_goal_result(args.goal_timeout, launch_proc=launch_proc)

    except KeyboardInterrupt:
        print("\n[nav_gps_menu] Interrupted by user.")
        return 130
    except Exception as exc:
        print(f"[nav_gps_menu] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_launch(launch_proc)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
