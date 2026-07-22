from pathlib import Path


LAUNCH = Path("src/bringup/launch/system_gps_corridor.launch.py")
MAKEFILE = Path("Makefile")
LAUNCH_SCRIPT = Path("scripts/launch_with_logs.sh")


def test_corridor_fgo_is_explicit_and_disables_the_legacy_tf_owner():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "fgo_authority" in text
    assert "UnlessCondition(LaunchConfiguration('fgo_authority'))" in text
    assert "fgo_map_odom_corrector_node" in text
    assert "system_fgo_gil_float.launch.py" in text
    assert "um982_mixed.yaml" in text


def test_corridor_fgo_uses_frozen_alignment_and_advisory_raw_fix_checks():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "'/fgo_gil/enu_to_map'" in text
    assert "'map_gps_consistency_mode'" in text
    assert "'advisory'" in text


def test_corridor_fgo_has_a_make_target_and_runtime_cleanup():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    launch_script = LAUNCH_SCRIPT.read_text(encoding="utf-8")

    assert "launch-corridor-fgo" in makefile
    assert "[f]go_map_odom_corrector" in makefile
    assert 'corridor-fgo) LAUNCH_FILE="system_gps_corridor.launch.py"' in launch_script
    assert 'LAUNCH_ARGS+=("fgo_authority:=true")' in launch_script
    assert "[f]go_gil_float_fgo_node" in launch_script
