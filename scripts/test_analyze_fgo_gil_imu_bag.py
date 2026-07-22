import importlib.util
import math
from pathlib import Path
import sqlite3
import struct


SCRIPT = Path(__file__).resolve().parent / "analyze_fgo_gil_imu_bag.py"
spec = importlib.util.spec_from_file_location("analyze_fgo_gil_imu_bag", SCRIPT)
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def _align(buffer: bytearray, boundary: int, origin: int = 4) -> None:
    buffer.extend(b"\x00" * ((-(len(buffer) - origin)) % boundary))


def _pack_double(buffer: bytearray, value: float) -> None:
    _align(buffer, 8)
    buffer.extend(struct.pack("<d", value))


def _imu_cdr(stamp_s: float, acceleration=(0.0, 0.0, 9.8)) -> bytes:
    seconds = math.floor(stamp_s)
    nanoseconds = round((stamp_s - seconds) * 1_000_000_000)
    frame = b"livox_frame\x00"
    buffer = bytearray(b"\x00\x01\x00\x00")
    buffer.extend(struct.pack("<iI", seconds, nanoseconds))
    buffer.extend(struct.pack("<I", len(frame)))
    buffer.extend(frame)
    for value in (0.0, 0.0, 0.0, 1.0):
        _pack_double(buffer, value)
    for value in (0.0,) * 9:
        _pack_double(buffer, value)
    for value in (0.01, 0.02, 0.03):
        _pack_double(buffer, value)
    for value in (0.0,) * 9:
        _pack_double(buffer, value)
    for value in acceleration:
        _pack_double(buffer, value)
    for value in (0.0,) * 9:
        _pack_double(buffer, value)
    return bytes(buffer)


def _create_bag(path: Path, stamps, accelerations=None) -> None:
    path.mkdir()
    connection = sqlite3.connect(path / "bag_0.db3")
    connection.executescript(
        """
        CREATE TABLE topics(
          id INTEGER PRIMARY KEY,
          name TEXT NOT NULL,
          type TEXT NOT NULL,
          serialization_format TEXT NOT NULL,
          offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(
          id INTEGER PRIMARY KEY,
          topic_id INTEGER NOT NULL,
          timestamp INTEGER NOT NULL,
          data BLOB NOT NULL);
        """
    )
    connection.execute(
        "INSERT INTO topics VALUES(1, '/livox/imu', 'sensor_msgs/msg/Imu', 'cdr', '')"
    )
    accelerations = accelerations or [(0.0, 0.0, 9.8)] * len(stamps)
    for index, (stamp, acceleration) in enumerate(zip(stamps, accelerations), start=1):
        connection.execute(
            "INSERT INTO messages VALUES(?, 1, ?, ?)",
            (index, int((1000.0 + index * 0.01) * 1e9), _imu_cdr(stamp, acceleration)),
        )
    connection.commit()
    connection.close()


def test_decode_imu_cdr_reads_stamp_and_measurements():
    decoded = analyzer.decode_imu_cdr(_imu_cdr(123.25, (1.0, 2.0, 3.0)))

    assert decoded["stamp_s"] == 123.25
    assert decoded["frame_id"] == "livox_frame"
    assert decoded["angular_velocity"] == (0.01, 0.02, 0.03)
    assert decoded["linear_acceleration"] == (1.0, 2.0, 3.0)


def test_analyze_bag_reports_rate_gap_duplicate_and_reversal(tmp_path):
    bag = tmp_path / "bag"
    _create_bag(bag, [10.00, 10.01, 10.01, 10.20, 10.19, 10.20])

    metrics = analyzer.analyze_imu_bag(bag, max_gap_s=0.05)

    assert metrics["decoded_count"] == 6
    assert metrics["duplicates"] == 1
    assert metrics["gaps_over_threshold"] == 1
    assert metrics["time_reversals"] == 1
    assert metrics["segment_count"] == 3
    assert not metrics["continuous_preintegration_ready"]


def test_analyze_bag_reports_nonfinite_measurement(tmp_path):
    bag = tmp_path / "bag"
    _create_bag(
        bag,
        [20.00, 20.01],
        [(0.0, 0.0, 9.8), (float("inf"), 0.0, 9.8)],
    )

    metrics = analyzer.analyze_imu_bag(bag)

    assert metrics["nonfinite_measurements"] == 1
    assert not metrics["continuous_preintegration_ready"]


def test_cli_writes_json_output(tmp_path):
    bag = tmp_path / "bag"
    output = tmp_path / "metrics.json"
    _create_bag(bag, [30.00, 30.01, 30.02])

    rc = analyzer.main([str(bag), "--json-output", str(output)])

    assert rc == 0
    assert output.exists()
    assert '"effective_rate_hz"' in output.read_text(encoding="utf-8")
