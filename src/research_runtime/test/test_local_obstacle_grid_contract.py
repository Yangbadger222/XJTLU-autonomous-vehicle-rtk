import pytest

from research_runtime.local_obstacle_grid_node import _parameter_bool, _valid_map_version


def test_local_grid_boolean_strings_are_parsed_by_value():
    assert _parameter_bool(True) is True
    assert _parameter_bool("true") is True
    assert _parameter_bool("false") is False
    assert _parameter_bool("0") is False
    assert _parameter_bool("ON") is True


def test_local_grid_rejects_ambiguous_boolean_values():
    with pytest.raises(ValueError):
        _parameter_bool("sometimes")


def test_local_grid_requires_a_named_map_version():
    assert not _valid_map_version("")
    assert not _valid_map_version(" unknown ")
    assert _valid_map_version("road-v4")
