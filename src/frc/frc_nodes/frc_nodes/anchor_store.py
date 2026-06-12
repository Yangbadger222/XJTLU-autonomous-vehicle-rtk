"""锚库：SQLite 持久化 + P_usable 生命周期状态机 + 跨 session 重挂。

纯 Python 模块（零 rclpy import），状态机全路径可单测。
memory_manager_node 是唯一写者；risk_pipeline 经 /frc/anchor_states 话题消费，
不直接读库（单写者原则，避免 SQLite 双进程争用）。

状态机（设计文档 §4.4，逐字实现）：
- on_smooth_traversal: p_usable *= beta；< retired_thresh -> retired；
  < stale_thresh -> stale（stale 仍注入但减半）
- on_event: p_usable = 1.0，hit_count += 1；
  state = confirmed if (hit_count >= 2 or severity >= 1.0) else candidate
- 跨 session 重挂（F2）：用 map 位姿快照挂到新 session 最近 keyframe，
  降级 candidate；首次再验证（on_event）后按规则恢复。
"""

import json
import math
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS anchors (
  id INTEGER PRIMARY KEY,
  kind TEXT, state TEXT,
  scene_id TEXT, session_id TEXT,
  keyframe_id INTEGER, dx REAL, dy REAL, dyaw REAL,
  map_x REAL, map_y REAL, map_yaw REAL,
  severity REAL, p_usable REAL DEFAULT 1.0, hit_count INTEGER DEFAULT 1,
  route_id TEXT, layout_id TEXT,
  embedding BLOB,
  created_at TEXT, last_event TEXT, last_verified TEXT
);
CREATE TABLE IF NOT EXISTS anchor_transitions (
  anchor_id INTEGER, stamp TEXT, session_id TEXT,
  from_state TEXT, to_state TEXT, p_usable REAL, trigger TEXT
);
"""

TIER_SEVERITY = {"gold": 1.0, "silver": 0.6, "bronze": 0.3}


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


@dataclass
class Anchor:
    id: int
    kind: str
    state: str
    scene_id: str
    session_id: str
    keyframe_id: int
    dx: float
    dy: float
    dyaw: float
    map_x: float
    map_y: float
    map_yaw: float
    severity: float
    p_usable: float
    hit_count: int
    route_id: str
    layout_id: str

    def injection_weight(self) -> float:
        """高斯渲染强度 = severity x P_usable；stale 减半；retired 不注入。"""
        if self.state == "retired":
            return 0.0
        w = self.severity * self.p_usable
        if self.state == "stale":
            w *= 0.5
        return w


class AnchorStore:
    def __init__(self, db_path, session_id: str, scene_id: str = "",
                 transitions_jsonl=None):
        self.db_path = str(db_path)
        self.session_id = session_id
        self.scene_id = scene_id
        self._jsonl_path = Path(transitions_jsonl) if transitions_jsonl else None
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        self.con = sqlite3.connect(self.db_path)
        self.con.execute("PRAGMA journal_mode=WAL")
        self.con.executescript(SCHEMA)
        self.con.commit()

    # ---------- 查询 ----------

    def get(self, anchor_id: int) -> Anchor | None:
        row = self.con.execute(
            "SELECT id,kind,state,scene_id,session_id,keyframe_id,dx,dy,dyaw,"
            "map_x,map_y,map_yaw,severity,p_usable,hit_count,route_id,layout_id "
            "FROM anchors WHERE id=?", (anchor_id,)).fetchone()
        return Anchor(*row) if row else None

    def active_anchors(self) -> list:
        """注入用：retired 之外的全部锚（含本 scene 历史 session）。"""
        rows = self.con.execute(
            "SELECT id,kind,state,scene_id,session_id,keyframe_id,dx,dy,dyaw,"
            "map_x,map_y,map_yaw,severity,p_usable,hit_count,route_id,layout_id "
            "FROM anchors WHERE state != 'retired' AND scene_id = ?",
            (self.scene_id,)).fetchall()
        return [Anchor(*r) for r in rows]

    def all_anchors(self) -> list:
        rows = self.con.execute(
            "SELECT id,kind,state,scene_id,session_id,keyframe_id,dx,dy,dyaw,"
            "map_x,map_y,map_yaw,severity,p_usable,hit_count,route_id,layout_id "
            "FROM anchors WHERE scene_id = ?", (self.scene_id,)).fetchall()
        return [Anchor(*r) for r in rows]

    def find_nearby(self, map_x: float, map_y: float, radius_m: float,
                    kind: str = "map") -> Anchor | None:
        best, best_d = None, radius_m
        for a in self.active_anchors():
            if a.kind != kind:
                continue
            d = math.hypot(a.map_x - map_x, a.map_y - map_y)
            if d <= best_d:
                best, best_d = a, d
        return best

    # ---------- 写路径（事件 / 衰减 / 重挂 / 位姿刷新）----------

    def create_anchor(self, kind: str, severity: float, keyframe_id: int,
                      dx: float, dy: float, dyaw: float,
                      map_x: float, map_y: float, map_yaw: float,
                      route_id: str = "", layout_id: str = "",
                      embedding: bytes | None = None) -> int:
        state = "confirmed" if severity >= 1.0 else "candidate"
        now = _now_iso()
        cur = self.con.execute(
            "INSERT INTO anchors (kind,state,scene_id,session_id,keyframe_id,"
            "dx,dy,dyaw,map_x,map_y,map_yaw,severity,p_usable,hit_count,"
            "route_id,layout_id,embedding,created_at,last_event,last_verified) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,1.0,1,?,?,?,?,?,?)",
            (kind, state, self.scene_id, self.session_id, keyframe_id,
             dx, dy, dyaw, map_x, map_y, map_yaw, severity,
             route_id, layout_id, embedding, now, now, now))
        self.con.commit()
        anchor_id = cur.lastrowid
        self._transition(anchor_id, "", state, 1.0, "create")
        return anchor_id

    def on_event(self, anchor_id: int, severity: float) -> Anchor:
        """锚附近再次发生事件：P_usable 重置 + 状态升级/恢复。"""
        a = self.get(anchor_id)
        hit_count = a.hit_count + 1
        new_severity = max(a.severity, severity)
        new_state = "confirmed" if (hit_count >= 2 or new_severity >= 1.0) \
            else "candidate"
        now = _now_iso()
        self.con.execute(
            "UPDATE anchors SET p_usable=1.0, hit_count=?, severity=?, state=?,"
            " last_event=?, last_verified=? WHERE id=?",
            (hit_count, new_severity, new_state, now, now, anchor_id))
        self.con.commit()
        if new_state != a.state:
            self._transition(anchor_id, a.state, new_state, 1.0, "event")
        else:
            self._transition(anchor_id, a.state, new_state, 1.0, "event_reinforce")
        return self.get(anchor_id)

    def on_smooth_traversal(self, anchor_id: int, beta: float = 0.8,
                            stale_thresh: float = 0.3,
                            retired_thresh: float = 0.1) -> Anchor:
        """平稳成功通过一次：P_usable 衰减，必要时降级 stale/retired。"""
        a = self.get(anchor_id)
        if a is None or a.state == "retired":
            return a
        p = a.p_usable * beta
        if p < retired_thresh:
            new_state = "retired"
        elif p < stale_thresh:
            new_state = "stale"
        else:
            new_state = a.state
        now = _now_iso()
        self.con.execute(
            "UPDATE anchors SET p_usable=?, state=?, last_verified=? WHERE id=?",
            (p, new_state, now, anchor_id))
        self.con.commit()
        trigger = "smooth_pass" if new_state == a.state else "decay"
        self._transition(anchor_id, a.state, new_state, p, trigger)
        return self.get(anchor_id)

    def update_map_pose(self, anchor_id: int, map_x: float, map_y: float,
                        map_yaw: float) -> None:
        """PGO 回环修正后刷新锚的 map 位姿快照（= keyframe 位姿 ⊕ 偏移）。"""
        self.con.execute(
            "UPDATE anchors SET map_x=?, map_y=?, map_yaw=? WHERE id=?",
            (map_x, map_y, map_yaw, anchor_id))
        self.con.commit()

    def remount_stale_sessions(self, keyframe_poses: dict) -> list:
        """跨 session 重挂（F2）。

        keyframe_poses: {kf_id: (x, y, yaw)} 当前 session 的 keyframe map 位姿。
        对创建于其他 session 的非 retired 锚：用 map 位姿快照找最近 keyframe，
        更新挂载（keyframe_id + 偏移），降级 candidate 注入，等待首过再验证。
        返回被重挂的 anchor_id 列表。
        """
        if not keyframe_poses:
            return []
        remounted = []
        for a in self.active_anchors():
            if a.session_id == self.session_id:
                continue
            kf_id, kf_pose = min(
                keyframe_poses.items(),
                key=lambda kv: math.hypot(kv[1][0] - a.map_x, kv[1][1] - a.map_y))
            kx, ky, kyaw = kf_pose
            cos_y, sin_y = math.cos(-kyaw), math.sin(-kyaw)
            rx, ry = a.map_x - kx, a.map_y - ky
            dx = rx * cos_y - ry * sin_y
            dy = rx * sin_y + ry * cos_y
            dyaw = a.map_yaw - kyaw
            self.con.execute(
                "UPDATE anchors SET keyframe_id=?, dx=?, dy=?, dyaw=?, "
                "state='candidate', session_id=session_id WHERE id=?",
                (kf_id, dx, dy, dyaw, a.id))
            self.con.commit()
            self._transition(a.id, a.state, "candidate", a.p_usable, "remount")
            remounted.append(a.id)
        return remounted

    # ---------- transitions 记录（E5 lifelong 出图直接查这张表）----------

    def _transition(self, anchor_id: int, from_state: str, to_state: str,
                    p_usable: float, trigger: str) -> None:
        stamp = _now_iso()
        self.con.execute(
            "INSERT INTO anchor_transitions "
            "(anchor_id,stamp,session_id,from_state,to_state,p_usable,trigger) "
            "VALUES (?,?,?,?,?,?,?)",
            (anchor_id, stamp, self.session_id, from_state, to_state,
             p_usable, trigger))
        self.con.commit()
        if self._jsonl_path is not None:
            self._jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._jsonl_path, "a") as f:
                f.write(json.dumps({
                    "anchor_id": anchor_id, "stamp": stamp,
                    "session_id": self.session_id, "from_state": from_state,
                    "to_state": to_state, "p_usable": p_usable,
                    "trigger": trigger}) + "\n")

    def transitions(self, anchor_id: int | None = None) -> list:
        if anchor_id is None:
            return self.con.execute(
                "SELECT anchor_id,stamp,session_id,from_state,to_state,"
                "p_usable,trigger FROM anchor_transitions").fetchall()
        return self.con.execute(
            "SELECT anchor_id,stamp,session_id,from_state,to_state,p_usable,"
            "trigger FROM anchor_transitions WHERE anchor_id=?",
            (anchor_id,)).fetchall()

    def close(self) -> None:
        self.con.close()
