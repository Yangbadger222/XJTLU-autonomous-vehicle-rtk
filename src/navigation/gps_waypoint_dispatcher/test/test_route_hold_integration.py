import asyncio
import threading
import time

import pytest


def _wait_future(future, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS action future timed out"
    return future.result()


def test_humble_navigate_to_pose_cancel_returns_nonempty_acknowledgement():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("nav2_msgs.action")
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    rclpy.init()
    server_node = Node("fake_nav_to_pose_server")
    client_node = Node("route_hold_cancel_client")

    async def execute_callback(goal_handle):
        while not goal_handle.is_cancel_requested:
            await asyncio.sleep(0.01)
        goal_handle.canceled()
        return NavigateToPose.Result()

    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    server = ActionServer(
        server_node,
        NavigateToPose,
        "navigate_to_pose",
        execute_callback=execute_callback,
        cancel_callback=cancel_callback,
    )
    client = ActionClient(client_node, NavigateToPose, "navigate_to_pose")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=1.0)
        goal_handle = _wait_future(
            client.send_goal_async(NavigateToPose.Goal()), timeout_s=2.0
        )
        assert goal_handle.accepted

        cancel_response = _wait_future(goal_handle.cancel_goal_async(), timeout_s=2.0)

        assert cancel_response.goals_canceling
    finally:
        executor.shutdown(timeout_sec=1.0)
        server.destroy()
        client_node.destroy_node()
        server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
