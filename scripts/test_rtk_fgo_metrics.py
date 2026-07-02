from scripts.evaluate_rtk_fgo_bag import (
    parse_correction,
    parse_diagnostic_values,
    parse_status_line,
    summarize_events,
)


def test_parse_status_line_extracts_state_and_gate():
    parsed = parse_status_line(
        "state=RTK_LOCKED gate=STRONG_CANDIDATE reason=RTK fixed quality accepted"
    )

    assert parsed["state"] == "RTK_LOCKED"
    assert parsed["gate"] == "STRONG_CANDIDATE"


def test_parse_status_line_keeps_reason_until_next_key():
    parsed = parse_status_line(
        'state=LOCAL_ONLY gate=REJECTED reason=implied RTK speed too high publish_tf=false'
    )

    assert parsed["reason"] == "implied RTK speed too high"


def test_parse_correction_reads_commit_and_norm():
    parsed = parse_correction([1.0, 0.12, 8.0])

    assert parsed["committed"] is True
    assert parsed["correction_norm_m"] == 0.12
    assert parsed["strong_sample_count"] == 8.0


class FakeKeyValue:
    def __init__(self, key, value):
        self.key = key
        self.value = value


class FakeDiagnosticStatus:
    def __init__(self, values):
        self.values = [FakeKeyValue(key, value) for key, value in values.items()]


class FakeDiagnosticArray:
    def __init__(self, values):
        self.status = [FakeDiagnosticStatus(values)]


def test_parse_diagnostic_values_reads_rtk_quality_fields():
    parsed = parse_diagnostic_values(
        FakeDiagnosticArray(
            {
                "rtk_gga_quality": "4",
                "rtk_satellites": "29",
                "rtk_hdop": "0.6",
                "position_innovation_m": "12.5",
                "heading_innovation_rad": "0.25",
                "implied_fix_speed_mps": "1.2",
            }
        )
    )

    assert parsed["rtk_gga_quality"] == 4
    assert parsed["rtk_satellites"] == 29
    assert parsed["rtk_hdop"] == 0.6
    assert parsed["position_innovation_m"] == 12.5
    assert parsed["heading_innovation_rad"] == 0.25
    assert parsed["implied_fix_speed_mps"] == 1.2


def test_summarize_events_includes_quality_and_innovation_stats():
    metrics = summarize_events(
        [
            (
                "/rtk_fgo/factor_diagnostics",
                FakeDiagnosticArray(
                    {
                        "rtk_gga_quality": "4",
                        "rtk_satellites": "29",
                        "rtk_hdop": "0.6",
                        "position_innovation_m": "12.5",
                    }
                ),
                10,
            ),
            (
                "/rtk_fgo/factor_diagnostics",
                FakeDiagnosticArray(
                    {
                        "rtk_gga_quality": "5",
                        "rtk_satellites": "24",
                        "rtk_hdop": "0.9",
                        "position_innovation_m": "2.5",
                    }
                ),
                20,
            ),
        ]
    )

    assert metrics["rtk_quality_counts"] == {"4": 1, "5": 1}
    assert metrics["rtk_satellites"]["max"] == 29
    assert metrics["rtk_hdop"]["median"] == 0.75
    assert metrics["position_innovation_m"]["max"] == 12.5
