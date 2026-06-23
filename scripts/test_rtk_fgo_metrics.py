from scripts.evaluate_rtk_fgo_bag import parse_correction, parse_status_line


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
