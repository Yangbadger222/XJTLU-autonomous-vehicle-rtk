"""锚库状态机全路径单测（衰减/重置/重挂，纯 Python 跑在工作站与 colcon test）。"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from frc_nodes.anchor_store import AnchorStore


@pytest.fixture
def store(tmp_path):
    s = AnchorStore(tmp_path / "anchors.db", session_id="2026-06-10-10-00-00",
                    scene_id="S2", transitions_jsonl=tmp_path / "log.jsonl")
    yield s
    s.close()


def make_anchor(store, severity=0.6, x=5.0, y=1.0):
    return store.create_anchor("map", severity, keyframe_id=42,
                               dx=0.1, dy=-0.2, dyaw=0.0,
                               map_x=x, map_y=y, map_yaw=0.5,
                               route_id="R2", layout_id="L1")


class TestLifecycle:
    def test_silver_starts_candidate(self, store):
        a = store.get(make_anchor(store, severity=0.6))
        assert a.state == "candidate" and a.p_usable == 1.0 and a.hit_count == 1

    def test_gold_starts_confirmed(self, store):
        a = store.get(make_anchor(store, severity=1.0))
        assert a.state == "confirmed"

    def test_second_event_confirms(self, store):
        aid = make_anchor(store, severity=0.6)
        a = store.on_event(aid, severity=0.6)
        assert a.state == "confirmed" and a.hit_count == 2 and a.p_usable == 1.0

    def test_decay_to_stale_then_retired(self, store):
        aid = make_anchor(store, severity=1.0)
        # beta=0.8: 1.0 -> 0.8 -> ... 0.262(stale at 6 次) ... <0.1 retired(11 次)
        states = []
        for _ in range(11):
            a = store.on_smooth_traversal(aid, beta=0.8)
            states.append(a.state)
        assert "stale" in states
        assert states[-1] == "retired"
        assert a.p_usable < 0.1

    def test_event_resets_decay(self, store):
        aid = make_anchor(store, severity=1.0)
        for _ in range(6):
            store.on_smooth_traversal(aid, beta=0.8)
        assert store.get(aid).state == "stale"
        a = store.on_event(aid, severity=0.6)
        assert a.p_usable == 1.0 and a.state == "confirmed"

    def test_retired_stops_decaying_and_injecting(self, store):
        aid = make_anchor(store, severity=1.0)
        for _ in range(20):
            store.on_smooth_traversal(aid, beta=0.5)
        a = store.get(aid)
        assert a.state == "retired"
        assert a.injection_weight() == 0.0

    def test_stale_injects_half(self, store):
        aid = make_anchor(store, severity=1.0)
        for _ in range(6):
            store.on_smooth_traversal(aid, beta=0.8)
        a = store.get(aid)
        assert a.state == "stale"
        assert a.injection_weight() == pytest.approx(a.severity * a.p_usable * 0.5)


class TestCrossSession:
    def test_remount_downgrades_and_reattaches(self, tmp_path):
        s1 = AnchorStore(tmp_path / "a.db", session_id="sess-1", scene_id="S2")
        aid = s1.create_anchor("map", 1.0, keyframe_id=100, dx=0, dy=0, dyaw=0,
                               map_x=10.0, map_y=2.0, map_yaw=0.0)
        s1.on_event(aid, 1.0)
        s1.close()

        s2 = AnchorStore(tmp_path / "a.db", session_id="sess-2", scene_id="S2")
        # 新 session 的 keyframe：kf 7 在 (9.5, 2.0)，朝向 90 度
        remounted = s2.remount_stale_sessions(
            {3: (0.0, 0.0, 0.0), 7: (9.5, 2.0, math.pi / 2)})
        assert remounted == [aid]
        a = s2.get(aid)
        assert a.state == "candidate"          # 降级注入
        assert a.keyframe_id == 7              # 重挂最近 keyframe
        assert a.p_usable == 1.0               # P_usable 保留
        # 偏移在 keyframe 局部系：world 偏移 (0.5, 0) 旋转 -90° -> (0, -0.5)
        assert a.dx == pytest.approx(0.0, abs=1e-6)
        assert a.dy == pytest.approx(-0.5, abs=1e-6)
        # 再验证后恢复 confirmed（hit_count 已 >= 2）
        assert s2.on_event(aid, 0.6).state == "confirmed"
        s2.close()

    def test_same_session_not_remounted(self, tmp_path):
        s = AnchorStore(tmp_path / "a.db", session_id="sess-1", scene_id="S2")
        make = s.create_anchor("map", 1.0, 5, 0, 0, 0, 1.0, 1.0, 0.0)
        assert s.remount_stale_sessions({1: (0, 0, 0)}) == []
        assert s.get(make).state == "confirmed"
        s.close()


class TestQueries:
    def test_find_nearby(self, store):
        aid = make_anchor(store, x=5.0, y=1.0)
        assert store.find_nearby(5.3, 1.2, radius_m=1.0).id == aid
        assert store.find_nearby(8.0, 1.0, radius_m=1.0) is None

    def test_scene_isolation(self, tmp_path):
        s = AnchorStore(tmp_path / "a.db", session_id="x", scene_id="S1")
        s.create_anchor("map", 1.0, 1, 0, 0, 0, 0, 0, 0)
        s.close()
        other = AnchorStore(tmp_path / "a.db", session_id="x", scene_id="S3")
        assert other.active_anchors() == []
        other.close()

    def test_transitions_logged(self, store, tmp_path):
        aid = make_anchor(store)
        store.on_event(aid, 0.6)
        store.on_smooth_traversal(aid)
        rows = store.transitions(aid)
        triggers = [r[6] for r in rows]
        assert triggers[0] == "create"
        assert "event" in triggers
        assert (tmp_path / "log.jsonl").exists()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
