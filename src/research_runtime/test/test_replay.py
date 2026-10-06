from research_runtime.replay_sim import run


def test_replay_closed_loop_smoke_has_stop_evidence():
    result = run()
    assert result["trajectory"]["valid"]
    assert result["serial_denied"] == "vcx=0,wc=0\n"
    assert result["truth_access"] is False
    assert result["selected_candidate"] == "view-safe"
    assert set(result["policy_comparison"]["policies"]) == {"PASSIVE", "PERIODIC_LOOK", "TASK_AWARE_LOOK"}
    assert result["policy_comparison"]["truth_is_evaluator_only"] is True
