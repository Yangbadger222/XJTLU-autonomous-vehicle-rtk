"""Operator state machine; HTTP and ROS cross the same small interface.

No command velocities, safety overrides, parameter writes or device paths.
Task and bag actions return work for the ROS adapter; acknowledgement is
explicit, and consent is withheld until current measured readiness holds.
"""
from collections import deque
import math
import json
import secrets
import threading
import time

from .operator_gate import OperatorConsent, OperatorGate


class OperatorConsole:
    def __init__(self, *, mode, actuator_enabled, mission_enabled, stopped_limits, clock=time.monotonic):
        if mode not in ("replay", "shadow", "live"):
            raise ValueError("invalid startup execution mode")
        self.mode, self.actuator_enabled, self.mission_enabled = mode, actuator_enabled, mission_enabled
        self.stopped_limits, self.clock = stopped_limits, clock
        self.lock = threading.RLock()
        self.session = secrets.token_hex(16)
        self.sequence = 0
        self.owner, self.heartbeat_at = None, -math.inf
        self.state, self.reason = "VIEW_ONLY", "尚未接管操作席"
        self.inputs, self.events, self.pending = {}, deque(maxlen=80), deque(maxlen=16)
        self.history, self.still_since = {}, None
        self.task = {"accepted": False, "task_id": "", "start_node": "", "goal_node": "", "node_ids": [], "reason": "等待任务接口"}
        self.replay = {"state": "IDLE", "bags": [], "reason": "仅回放原始 LiDAR/IMU"}

    def _set(self, state, reason):
        if (state, reason) != (self.state, self.reason):
            self.events.append({"time": time.time(), "state": state, "reason": reason})
        self.state, self.reason = state, reason

    def update(self, name, value):
        with self.lock:
            try:json.dumps(value,allow_nan=False)
            except (ValueError,TypeError):
                self.inputs.pop(name,None)
                if name=="odom":self.still_since=None
                self.tick()
                return False
            self.inputs[name] = (value, self.clock())
            if name == "odom":
                v, w = value.get("v"), value.get("w")
                valid = (isinstance(v, (float, int)) and isinstance(w, (float, int)) and
                         math.isfinite(v) and math.isfinite(w) and
                         abs(v) <= self.stopped_limits[0] and abs(w) <= self.stopped_limits[1])
                self.still_since = (self.still_since if self.still_since is not None else self.clock()) if valid else None
            if name == "observer" and value.get("state") in ("TASK_REACHED", "BUDGET_EXHAUSTED", "SYSTEM_FAULT"):
                self.task["accepted"] = False
            self.tick()
            return True

    def _fresh(self, name, age=.5):
        value, stamp = self.inputs.get(name, (None, -math.inf))
        return value if 0 <= self.clock()-stamp <= age else None

    def invalidate_input(self,name):
        with self.lock:
            self.inputs.pop(name,None)
            if name=='odom':self.still_since=None
            self.tick()

    def blocked(self):
        reasons = []
        if self.mode != "live": reasons.append("当前环境不执行运动："+self.mode)
        if not self.actuator_enabled: reasons.append("启动配置未启用执行器")
        if not self.mission_enabled: reasons.append("启动配置未授权研究任务执行")
        if self._fresh("authority") is not True: reasons.append("RTK 运动权限缺失或过期")
        if self._fresh("rtk_mode") not in ("RTK_AUTHORITATIVE", "RTK_REACQUIRING"): reasons.append("RTK 未处于允许模式")
        if not str(self._fresh("health") or "").startswith("OK"): reasons.append("本车 LIO 健康未知或过期")
        if self._fresh("tf", .2) is not True: reasons.append("TF 完整性缺失或过期")
        if not self._fresh("odom", .2): reasons.append("本车测量里程计缺失或过期")
        if self._fresh("grid") is not True: reasons.append("地面/障碍网格缺失或过期")
        if self._fresh("permission") is not True: reasons.append("许可网格缺失或过期")
        if self._fresh("map_version") in (None, "", "UNKNOWN"): reasons.append("地图版本未就绪")
        observer = self._fresh("observer")
        if not observer: reasons.append("任务管理器离线")
        elif observer.get("state") in ("TASK_REACHED", "BUDGET_EXHAUSTED", "SYSTEM_FAULT"): reasons.append("任务已终止，请重新选择任务")
        if not self.task["accepted"]: reasons.append("尚未选择有效的已注册道路任务")
        return reasons

    def tick(self):
        with self.lock:
            now = self.clock()
            if self._fresh("odom", .2) is None: self.still_since = None
            if self.owner and now-self.heartbeat_at > OperatorGate.TIMEOUT_S:
                self.owner = None
                self._set("STOP_LATCHED", "操作席心跳丢失；需要停车确认、复位和重新启动")
            if self.state == "AUTONOMOUS" and self.blocked():
                self._set("STOP_LATCHED", "运行条件失效："+"；".join(self.blocked()))

    def _stopped(self):
        return self.still_since is not None and self.clock()-self.still_since >= self.stopped_limits[2] and self._fresh("odom", .2) is not None

    def command(self, client, action, payload=None, request_id=""):
        payload = payload or {}
        with self.lock:
            self.tick()
            if action == "claim":
                if self.owner and self.owner != client: return {"accepted": False, "reason": "操作席已由其他窗口占用"}
                self.owner, self.heartbeat_at = client, self.clock()
                return {"accepted": True, "reason": "已接管操作席；尚未请求运动"}
            if self.owner != client: return {"accepted": False, "reason": "请先接管操作席"}
            if action == "heartbeat":
                self.heartbeat_at = self.clock()
                return {"accepted": True, "reason": "heartbeat"}
            if not request_id or len(request_id) > 128: return {"accepted": False, "reason": "缺少有效操作编号"}
            if request_id in self.history: return self.history[request_id]
            result = {"accepted": False, "reason": "未知操作"}
            if action == "stop":
                self._set("STOP_LATCHED", "操作员软件停止；物理急停仍由原硬件执行")
                result = {"accepted": True, "reason": self.reason}
            elif action in ("pause", "takeover"):
                self._set("PAUSED" if action == "pause" else "TAKEOVER_WAIT",
                          "已撤销自主许可" if action == "pause" else "自主许可已撤销；等待现场原手柄/KEY接管")
                result = {"accepted": True, "reason": self.reason}
            elif action == "reset":
                if self._stopped():
                    self._set("READY", "测量里程计已连续确认静止；仍须显式启动")
                    result = {"accepted": True, "reason": self.reason}
                else: result["reason"] = "等待原停车阈值下连续静止确认；这不代表物理制动验收"
            elif action == "start":
                blockers = self.blocked()
                if self.state != "READY": blockers.insert(0, "请先停车确认并复位")
                if not self._stopped():blockers.insert(0,"等待原停车阈值下连续静止确认")
                if not blockers:
                    self._set("AUTONOMOUS", "操作员请求研究任务；原 RTK/手柄/安全链继续仲裁")
                    result = {"accepted": True, "reason": self.reason}
                else: result["reason"] = "；".join(blockers)
            elif action in ("task", "replay_start", "replay_pause", "replay_resume", "replay_stop"):
                if self.state == "AUTONOMOUS": result["reason"] = "请先暂停自主任务"
                elif len(self.pending) >= self.pending.maxlen: result["reason"] = "操作队列已满"
                elif action.startswith("replay_") and (self.mode != "replay" or self.actuator_enabled): result["reason"] = "回放仅限执行器禁用的 replay 环境"
                elif action == "task" and (set(payload) != {"start_node", "goal_node"} or
                      any(payload[k] not in self.task["node_ids"] for k in payload) or payload["start_node"] == payload["goal_node"]): result["reason"] = "任务端点必须是两个不同的已注册道路节点"
                elif action == "replay_start" and (set(payload) != {"bag_id"} or
                      payload["bag_id"] not in [b["id"] for b in self.replay["bags"]]): result["reason"] = "仅允许目录清单中的原始 bag"
                elif action.startswith("replay_") and action!="replay_start" and payload: result["reason"] = "该回放操作不接受额外字段"
                else:
                    self.pending.append((request_id, action, dict(payload), self.clock()))
                    if action == "task": self.task["accepted"] = False
                    result = {"accepted": True, "pending": True, "reason": "请求已提交，等待系统确认"}
            self.history[request_id] = result
            if len(self.history) > 128: self.history.pop(next(iter(self.history)))
            return result

    def finish_request(self,identity,accepted,reason):
        with self.lock:
            self.history[identity]={"accepted":bool(accepted),"pending":False,"reason":reason}
            self.events.append({"time":time.time(),"state":"REQUEST_RESULT","reason":reason})

    def snapshot(self, client=None):
        with self.lock:
            self.tick()
            now = self.clock()
            return {"mode": self.mode, "actuator_enabled": self.actuator_enabled, "mission_enabled": self.mission_enabled,
                    "operator_timeout_s":OperatorGate.TIMEOUT_S,
                    "state": self.state, "reason": self.reason, "owns_lease": bool(client and self.owner == client),
                    "lease_active": self.owner is not None, "stop_confirmed_from_odom": self._stopped(),
                    "blocked": self.blocked(), "task": dict(self.task), "replay": dict(self.replay),
                    "inputs": {k: {"value": v, "age_s": max(0., now-t)} for k, (v,t) in self.inputs.items()},
                    "events": list(self.events), "stopped_limits": list(self.stopped_limits)}

    def permit(self):
        with self.lock:
            self.tick()
            self.sequence += 1
            return OperatorConsent(self.session, self.sequence, self.mode,
                self._fresh("map_version") or "UNKNOWN", self.state,
                bool(self.owner and self.state == "AUTONOMOUS" and not self.blocked()), bool(self.owner))
