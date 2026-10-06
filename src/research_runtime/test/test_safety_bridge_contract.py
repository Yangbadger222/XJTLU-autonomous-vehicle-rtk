from research_runtime.safety_bridge import _parameter_bool, _valid_map_version


def test_launch_boolean_strings_are_parsed_by_value():
    assert _parameter_bool("true") is True
    assert _parameter_bool("false") is False
    assert _parameter_bool("0") is False
    assert _parameter_bool("ON") is True


def test_unknown_map_versions_are_not_runtime_valid():
    assert not _valid_map_version("")
    assert not _valid_map_version(" unknown ")
    assert _valid_map_version("road-v4")
