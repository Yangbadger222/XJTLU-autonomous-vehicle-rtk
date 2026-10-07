import math
from research_runtime.safety_bridge import rtk_mode_is_allowed


def test_healthy_lio_and_boolean_authority_never_allow_bridge_motion():
    for mode in ('LIO_BRIDGE','RTK_DEGRADED','UNKNOWN',''):
        assert not rtk_mode_is_allowed(mode,.1)
    assert rtk_mode_is_allowed('RTK_AUTHORITATIVE',.1)
    assert rtk_mode_is_allowed('RTK_REACQUIRING',.1)
    for age in (.5001,-.01,math.nan):
        assert not rtk_mode_is_allowed('RTK_AUTHORITATIVE',age)
