import sys

from research_runtime import safety_bridge
from research_runtime.replay_sim import run


def test_replay_closed_loop_smoke_has_stop_evidence():
    result = run()
    assert result["trajectory"]["valid"]
    assert result["tracker"]["valid"]
    assert result["serial_denied"] == "vcx=0,wc=0\n"
    assert result["truth_access"] is False
    assert result["selected_candidate"] == "view-safe"
    assert set(result["policy_comparison"]["policies"]) == {"PASSIVE", "PERIODIC_LOOK", "TASK_AWARE_LOOK"}
    assert result["policy_comparison"]["truth_is_evaluator_only"] is True
    assert result["policy_comparison"]["status"] == "NOT_RUN"
    assert all(item["status"] == "NOT_RUN" for item in result["policy_comparison"]["policies"].values())


def test_safety_bridge_replay_entry_strips_mode_before_harness(monkeypatch, tmp_path):
    output = tmp_path / "replay.json"
    monkeypatch.setattr(safety_bridge, "rclpy", None)
    monkeypatch.setattr(sys, "argv", ["research_safety_bridge", "--mode", "replay",
                                       "--output", str(output)])
    assert safety_bridge.main() == 0
    assert '"serial_denied": "vcx=0,wc=0\\n"' in output.read_text()
