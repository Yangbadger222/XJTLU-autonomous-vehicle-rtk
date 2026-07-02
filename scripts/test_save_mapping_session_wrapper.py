from pathlib import Path


def test_save_mapping_session_wrapper_sources_ros_without_nounset():
    text = Path("scripts/save_mapping_session.sh").read_text(encoding="utf-8")

    assert "set -euo pipefail" in text
    assert "set +u\nsource /opt/ros/humble/setup.bash\nsource install/setup.bash\nset -u" in text
