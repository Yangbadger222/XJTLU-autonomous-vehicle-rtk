from gps_waypoint_dispatcher.corridor_quality import parse_gga_quality


def test_corridor_gga_quality_accepts_raw_fixed_and_float_samples():
    assert parse_gga_quality(
        "$GNGGA,123519.00,3116.123456,N,12130.654321,E,4,22,0.6,12.3,M,0.0,M,,*78"
    ) == 4
    assert parse_gga_quality(
        "$GNGGA,123520.00,3116.123456,N,12130.654321,E,5,20,0.8,12.4,M,0.0,M,,*78"
    ) == 5


def test_corridor_gga_quality_rejects_bad_checksum_missing_quality_and_truncation():
    assert parse_gga_quality(
        "$GNGGA,123519.00,3116.123456,N,12130.654321,E,4,22,0.6,12.3,M,0.0,M,,*00"
    ) is None
    assert parse_gga_quality(
        "$GNGGA,123519.00,3116.123456,N,12130.654321,E,,22,0.6,12.3,M,0.0,M,,*4C"
    ) is None
    assert parse_gga_quality("$GNGGA,123519.00,3116.123456,N*25") is None
    assert parse_gga_quality("$GNTHS,341.3344,A*1F") is None
