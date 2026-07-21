from types import SimpleNamespace

import pytest

from scripts.evaluate_fgo_gil_bag import (
    PoseSample,
    RawEpochSample,
    align_receiver_epochs,
    availability_fraction,
    metadata_only_metrics,
    outage_drift_metrics,
    parse_tegrastats,
    summarize_events,
    trajectory_metrics,
)


class FakeDiagnosticArray:
    def __init__(self, name, values):
        items = [SimpleNamespace(key=key, value=str(value)) for key, value in values.items()]
        self.status = [SimpleNamespace(name=name, values=items)]


def fake_odom(timestamp_ns, position):
    stamp = SimpleNamespace(
        sec=timestamp_ns // 1_000_000_000,
        nanosec=timestamp_ns % 1_000_000_000,
    )
    point = SimpleNamespace(x=position[0], y=position[1], z=position[2])
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        pose=SimpleNamespace(pose=SimpleNamespace(position=point)),
    )


def fake_raw_epoch(receiver, milliseconds_of_week):
    return SimpleNamespace(
        receiver=receiver,
        week=2428,
        milliseconds_of_week=milliseconds_of_week,
    )


def test_metadata_only_reports_missing_raw_topic_explicitly():
    metrics = metadata_only_metrics({"topic_names": {"/livox/imu"}})
    assert metrics["raw_gnss_status"] == "RAW_GNSS_UNAVAILABLE"
    assert metrics["trajectory"]["status"] == "NOT_DECODED"


def test_metadata_only_rejects_declared_raw_topic_without_messages():
    metrics = metadata_only_metrics(
        {
            "topic_names": {"/gnss/raw/observation_epoch"},
            "topic_counts": {"/gnss/raw/observation_epoch": 0},
        }
    )

    assert metrics["raw_gnss_status"] == "RAW_GNSS_UNAVAILABLE"


def test_availability_integrates_stale_bounded_output_intervals():
    fraction = availability_fraction(
        [0, 1_000_000_000, 4_000_000_000],
        0,
        5_000_000_000,
        1.0,
    )
    assert fraction == pytest.approx(0.6)


def test_trajectory_metrics_removes_rigid_ecef_to_local_alignment():
    references = [
        PoseSample(index * 1_000_000_000, position)
        for index, position in enumerate(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 1.0, 1.0)]
        )
    ]
    estimates = [
        PoseSample(
            sample.timestamp_ns,
            (100.0 + sample.position[1], 200.0 - sample.position[0], 50.0 + sample.position[2]),
        )
        for sample in references
    ]
    metrics, _ = trajectory_metrics(estimates, references, 0.01)
    assert metrics["status"] == "OK"
    assert metrics["ape_m"]["max"] < 1.0e-9
    assert metrics["rpe_m"]["max"] < 1.0e-9


def test_empty_reference_trajectory_fails_closed():
    metrics, timed_errors = trajectory_metrics(
        [PoseSample(0, (1.0, 2.0, 3.0))], [], 0.1
    )

    assert metrics["status"] == "INSUFFICIENT_MATCHED_POSES"
    assert metrics["matched_poses"] == 0
    assert timed_errors == []


def test_summarize_reports_fixing_rate_rtf_and_raw_availability():
    topics = {
        "/fgo_gil/odom",
        "/fastlio2/lio_odom",
        "/gnss/raw/observation_epoch",
        "/fgo_gil/ambiguity_status",
        "/fgo_gil/performance",
    }
    events = []
    for index in range(4):
        stamp = index * 1_000_000_000
        events.append(("/fgo_gil/odom", fake_odom(stamp, (float(index), 0.0, 0.0)), stamp))
        events.append(("/fastlio2/lio_odom", fake_odom(stamp, (float(index), 0.0, 0.0)), stamp))
        events.append(
            (
                "/gnss/raw/observation_epoch",
                fake_raw_epoch(1, index * 1000),
                stamp,
            )
        )
        events.append(
            (
                "/gnss/raw/observation_epoch",
                fake_raw_epoch(3, index * 1000),
                stamp + 10_000_000,
            )
        )
    events.extend(
        [
            (
                "/fgo_gil/ambiguity_status",
                FakeDiagnosticArray(
                    "fgo_gil/ambiguity",
                    {"solution_status": "FLOAT", "ratio": 2.0},
                ),
                1,
            ),
            (
                "/fgo_gil/ambiguity_status",
                FakeDiagnosticArray(
                    "fgo_gil/ambiguity",
                    {"solution_status": "FIXED", "ratio": 4.0},
                ),
                2,
            ),
            (
                "/fgo_gil/performance",
                FakeDiagnosticArray(
                    "fgo_gil/performance",
                    {"real_time_factor": 0.25, "optimization_latency_ms": 50.0},
                ),
                3,
            ),
        ]
    )
    metrics = summarize_events(events, topics, 0, 4_000_000_000)
    assert metrics["raw_gnss_status"] == "RAW_GNSS_AVAILABLE"
    assert metrics["ambiguity"]["fixing_rate"] == 0.5
    assert metrics["performance"]["real_time_factor"]["max"] == 0.25
    assert metrics["trajectory"]["ape_m"]["max"] < 1.0e-9
    assert metrics["raw_receiver_epochs"]["aligned"] == 4


def test_raw_epoch_alignment_is_one_to_one_and_reports_incomplete_input():
    aligned = align_receiver_epochs(
        [
            RawEpochSample(0, 2428, 1000),
            RawEpochSample(1_000_000_000, 2428, 2000),
            RawEpochSample(2_000_000_000, 2428, 3000),
        ],
        [
            RawEpochSample(10_000_000, 2428, 1010),
            RawEpochSample(1_020_000_000, 2428, 2020),
        ],
    )
    assert aligned == [5_000_000, 1_010_000_000]

    metrics = summarize_events(
        [
            (
                "/gnss/raw/observation_epoch",
                fake_raw_epoch(1, 1000),
                0,
            )
        ],
        {"/gnss/raw/observation_epoch"},
        0,
        1_000_000_000,
    )
    assert metrics["raw_gnss_status"] == "RAW_GNSS_INCOMPLETE"
    assert metrics["outage_drift"]["status"] == "DD_GNSS_UNAVAILABLE"


def test_raw_epoch_alignment_uses_gnss_time_despite_reception_jitter():
    master = [
        RawEpochSample(100_000_000, 2428, 1000),
        RawEpochSample(1_100_000_000, 2428, 2000),
    ]
    base = [
        RawEpochSample(800_000_000, 2428, 1000),
        RawEpochSample(1_800_000_000, 2428, 2000),
    ]

    assert align_receiver_epochs(master, base) == [450_000_000, 1_450_000_000]


def test_outage_drift_uses_change_in_aligned_error_not_vehicle_motion():
    raw = [0, 1_000_000_000, 5_000_000_000]
    errors = [
        (1_000_000_000, (0.1, 0.0, 0.0)),
        (5_000_000_000, (0.6, 0.0, 0.0)),
    ]
    metrics = outage_drift_metrics(raw, errors, 2.0)
    assert metrics["outage_count"] == 1
    assert metrics["drift_m"]["max"] == pytest.approx(0.5)


def test_parse_tegrastats_reports_ram_and_mean_cpu():
    metrics = parse_tegrastats(
        [
            "RAM 2000/15721MB CPU [10%@729,20%@729,off,30%@729]",
            "RAM 2400/15721MB CPU [20%@729,40%@729,60%@729]",
        ]
    )
    assert metrics["status"] == "OK"
    assert metrics["ram_used_mb"]["max"] == 2400.0
    assert metrics["cpu_mean_percent"]["mean"] == pytest.approx(30.0)
