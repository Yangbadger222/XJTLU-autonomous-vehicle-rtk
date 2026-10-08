"""Regression for a valid HTTP request interleaved with final STOP publication."""
import ast
from pathlib import Path
from types import SimpleNamespace
from test_operator_console import prepared

def test_closing_console_cannot_be_rearmed_before_final_permit():
    _, console = prepared()
    assert console.command("window-a", "start", request_id="before-close")["accepted"]
    assert console.permit().motion_requested
    attempts, emitted = [], []

    def final_tick():
        # The real close method has released its model lock and HTTP can
        # still be processing an authenticated in-flight request here.
        for action, identity in (("claim", ""), ("reset", "race-reset"),
                                 ("start", "race-start"), ("start", "before-close")):
            attempts.append(console.command("window-a", action, request_id=identity))
        emitted.append(console.permit())

    target = SimpleNamespace(console=console, replay_generation=0, _tick=final_tick,
        http=SimpleNamespace(close=lambda: None), _stop_player=lambda: None)
    path = Path(__file__).resolve().parents[1] / "research_runtime/console_node.py"
    tree = ast.parse(path.read_text())
    methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == "ConsoleNode"
               for method in cls.body if isinstance(method, ast.FunctionDef) and method.name == "close"]
    assert len(methods) == 1
    # Execute the production close function without importing ROS. Only its
    # context liveness and owned I/O are replaced; the model is the real one.
    scope = {"rclpy": SimpleNamespace(ok=lambda: True)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), scope)
    scope["close"](target)
    assert attempts and all(not item["accepted"] for item in attempts)
    assert len(emitted) == 1
    assert emitted[0].state == "STOP_LATCHED"
    assert not emitted[0].motion_requested
    assert not emitted[0].lease_active
    assert not console.pending
