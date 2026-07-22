from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_FILE = REPO_ROOT / "src/bringup/launch/system_rtk_raw.launch.py"
MIXED_CONFIG = (
    REPO_ROOT / "src/sensor_drivers/gnss/um982_rtk_driver/config/um982_mixed.yaml"
)
MAKEFILE = REPO_ROOT / "Makefile"
LAUNCH_WRAPPER = REPO_ROOT / "scripts/launch_with_logs.sh"


def test_raw_launch_uses_one_unified_serial_owner():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="um982_rtk_driver"' in text
    assert 'executable="um982_rtk_node"' in text
    assert "um982_raw_node" not in text
    assert text.count("Node(") == 1


def test_raw_launch_records_raw_and_production_outputs():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"/gnss/raw/frame"' in text
    assert '"/gnss/raw/observation_epoch"' in text
    assert '"/gnss/raw/ephemeris"' in text
    assert '"/gnss/raw/diagnostics"' in text
    assert '"/fix"' in text
    assert '"/heading"' in text
    assert '"/rtk/status"' in text
    assert '"/rtk/nmea_sentence"' in text
    assert '"bag",' in text


def test_mixed_config_uses_single_production_port_at_high_baud():
    text = MIXED_CONFIG.read_text(encoding="utf-8")

    assert "port: /dev/rtk_um982" in text
    assert "baud: 921600" in text
    assert "stream_mode: mixed" in text
    assert "max_payload_bytes: 65535" in text
    assert "max_buffer_bytes: 131072" in text
    assert "epoch_dedup_capacity: 256" in text
    assert "ephemeris_topic: /gnss/raw/ephemeris" in text


def test_build_launch_and_cleanup_entry_points_include_unified_driver():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    wrapper = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert "build-rtk-raw:" in makefile
    assert "gnss_raw_msgs um982_raw_driver um982_rtk_driver bringup" in makefile
    assert "[u]m982_rtk_node" in makefile
    assert "/dev/rtk_um982" in makefile
    assert "/dev/rtk_um982_raw" not in makefile
    assert 'rtk-raw)      LAUNCH_FILE="system_rtk_raw.launch.py"' in wrapper
    assert "FYP_UM982_MIXED_PARAMS_FILE" in wrapper
    assert "[u]m982_rtk_node" in wrapper
    assert "/dev/rtk_um982" in wrapper
    assert "/dev/rtk_um982_raw" not in wrapper
