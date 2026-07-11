from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_FILE = REPO_ROOT / "src/bringup/launch/system_rtk_raw.launch.py"
RAW_CONFIG = REPO_ROOT / "src/sensor_drivers/gnss/um982_raw_driver/config/um982_raw.yaml"
MAKEFILE = REPO_ROOT / "Makefile"
LAUNCH_WRAPPER = REPO_ROOT / "scripts/launch_with_logs.sh"


def test_raw_launch_is_isolated_from_production_gnss():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="um982_raw_driver"' in text
    assert 'executable="um982_raw_node"' in text
    assert "um982_rtk_driver" not in text
    assert "/fix" not in text
    assert "/heading" not in text


def test_raw_launch_records_frame_and_diagnostics():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"/gnss/raw/frame"' in text
    assert '"/gnss/raw/diagnostics"' in text
    assert '"bag",' in text


def test_raw_config_uses_dedicated_high_baud_port():
    text = RAW_CONFIG.read_text(encoding="utf-8")

    assert "port: /dev/rtk_um982_raw" in text
    assert "baud: 921600" in text
    assert "port: /dev/rtk_um982\n" not in text
    assert "max_payload_bytes: 65535" in text
    assert "max_buffer_bytes: 131072" in text


def test_build_launch_and_cleanup_entry_points_include_raw_driver():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    wrapper = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert "build-rtk-raw:" in makefile
    assert "gnss_raw_msgs um982_raw_driver bringup" in makefile
    assert "[u]m982_raw_node" in makefile
    assert "/dev/rtk_um982_raw" in makefile
    assert 'rtk-raw)      LAUNCH_FILE="system_rtk_raw.launch.py"' in wrapper
    assert "FYP_UM982_RAW_PARAMS_FILE" in wrapper
    assert "[u]m982_raw_node" in wrapper
    assert "/dev/rtk_um982_raw" in wrapper
