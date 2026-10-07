import http.client
import json
from pathlib import Path

import pytest

from research_runtime.operator_console import OperatorConsole
from research_runtime.operator_gate import OperatorConsent, OperatorGate
from research_runtime.console_http import ConsoleHTTP
from research_runtime.physical_parameter_lock import STOP_CONFIRMATION


class Time:
    now = 10.
    def __call__(self): return self.now


def measured(clock, console):
    for key, value in {"authority": True, "rtk_mode": "RTK_AUTHORITATIVE", "health": "OK: SIMULATED",
                       "tf": True, "odom": {"v": 0., "w": 0.}, "grid": True, "permission": True,
                       "map_version": "simulation-map", "observer": {"state": "OPERATOR_PAUSED_OR_PERMISSION_LOST"}}.items():
        console.update(key, value)


def prepared(mode="live", actuator=True):
    clock = Time()
    console = OperatorConsole(mode=mode, actuator_enabled=actuator, mission_enabled=True,
                              stopped_limits=STOP_CONFIRMATION, clock=clock)
    console.task.update(accepted=True, node_ids=["start", "goal"])
    console.command("window-a", "claim")
    for _ in range(25):
        measured(clock, console)
        console.command("window-a", "heartbeat")
        clock.now += .05
    measured(clock, console)
    assert console.command("window-a", "reset", request_id="reset")["accepted"]
    return clock, console


@pytest.mark.parametrize("mode,actuator", [("replay", False), ("shadow", False), ("live", False)])
def test_startup_environment_cannot_be_overridden_by_browser(mode, actuator):
    _, console = prepared(mode, actuator)
    assert not console.command("window-a", "start", request_id="start")["accepted"]
    assert not console.command("window-a", "enable_serial", request_id="serial")["accepted"]
    assert not console.permit().motion_requested


@pytest.mark.parametrize("action,state", [("pause", "PAUSED"), ("takeover", "TAKEOVER_WAIT"), ("stop", "STOP_LATCHED")])
def test_human_actions_revoke_consent_and_require_explicit_reset(action, state):
    _, console = prepared()
    assert console.command("window-a", "start", request_id="start")["accepted"]
    assert console.permit().motion_requested
    assert console.command("window-a", action, request_id="action")["accepted"]
    assert console.state == state and not console.permit().motion_requested
    assert not console.command("window-a", "start", request_id="auto-resume")["accepted"]


def test_rtk_loss_latches_even_when_lio_recovers_and_remains_healthy():
    clock, console = prepared()
    console.command("window-a", "start", request_id="start")
    console.update("authority", False)
    assert console.state == "STOP_LATCHED"
    measured(clock, console)
    assert not console.permit().motion_requested
    assert console.state == "STOP_LATCHED"


def test_one_window_lease_heartbeat_expiry_and_duplicate_request_do_not_resume():
    clock, console = prepared()
    assert not console.command("window-b", "claim")["accepted"]
    console.command("window-a", "start", request_id="start")
    clock.now += .51
    measured(clock, console)
    assert console.state == "STOP_LATCHED" and not console.permit().motion_requested
    assert console.command("window-b", "claim")["accepted"]
    assert not console.command("window-a", "start", request_id="start")["accepted"]
    assert not console.permit().motion_requested


def test_new_odom_after_gap_must_restart_full_original_stop_confirmation():
    clock, console = prepared()
    clock.now += .3
    console.tick()
    measured(clock, console)
    assert not console.command("window-a", "reset", request_id="gap-reset")["accepted"]


def test_task_edit_is_bounded_to_registered_ids_and_is_acknowledged_separately():
    _, console = prepared()
    assert not console.command("window-a", "task", {"start_node": "arbitrary", "goal_node": "goal"}, "bad")["accepted"]
    result = console.command("window-a", "task", {"start_node": "start", "goal_node": "goal"}, "task")
    assert result["accepted"] and result["pending"] and not console.task["accepted"]
    assert console.command("window-a", "task", {"start_node": "start", "goal_node": "goal"}, "task") == result
    assert len(console.pending) == 1


def test_operator_gate_requires_order_freshness_version_and_single_owner():
    gate = OperatorGate()
    gate.receive(OperatorConsent("session",1,"live","m1","READY",False,True),9.9)
    assert not gate.allowed(9.9,mode="live",map_version="m1")
    consent = OperatorConsent("session", 2, "live", "m1", "AUTONOMOUS", True, True)
    assert gate.receive(consent, 10.)
    assert gate.allowed(10.1, mode="live", map_version="m1")
    assert not gate.receive(consent, 10.4)
    assert not gate.allowed(10.51, mode="live", map_version="m1")
    assert not gate.allowed(10.1, mode="live", map_version="m2")
    assert not gate.allowed(10.1, mode="live", map_version="m1", sole_publisher=False)
    assert not gate.allowed(10.1, mode="shadow", map_version="m1")
    assert not gate.receive(OperatorConsent("session", 0, "live", "m1", "AUTONOMOUS", True, True), 10.4)


def test_nonfinite_display_never_breaks_snapshot_or_state_reading():
    _,console=prepared()
    console.command("window-a","start",request_id="start")
    assert not console.update("road",{"points":[[float("nan"),0.]]})
    json.dumps(console.snapshot("window-a"),allow_nan=False)
    assert "road" not in console.inputs


@pytest.mark.parametrize("fault",["expired","competing"])
def test_restored_transport_cannot_rearm_without_ready_then_explicit_start(fault):
    gate=OperatorGate()
    gate.receive(OperatorConsent("s",1,"live","m1","READY",False,True),10.)
    gate.allowed(10.,mode="live",map_version="m1")
    gate.receive(OperatorConsent("s",2,"live","m1","AUTONOMOUS",True,True),10.1)
    assert gate.allowed(10.1,mode="live",map_version="m1")
    assert not gate.allowed(10.7 if fault=="expired" else 10.2,mode="live",map_version="m1",sole_publisher=fault!="competing")
    gate.receive(OperatorConsent("s",3,"live","m1","AUTONOMOUS",True,True),10.8)
    assert not gate.allowed(10.8,mode="live",map_version="m1")
    gate.receive(OperatorConsent("s",4,"live","m1","READY",False,True),10.9)
    assert not gate.allowed(10.9,mode="live",map_version="m1")
    gate.receive(OperatorConsent("s",5,"live","m1","AUTONOMOUS",True,True),11.)
    assert gate.allowed(11.,mode="live",map_version="m1")


def test_http_origin_and_same_cookie_different_window_exclusive_lease():
    _, console = prepared("replay", False)
    console.owner = None
    assets = Path(__file__).parents[1]/"web"
    http = ConsoleHTTP(console, assets, 0)
    http.start()
    try:
        client = http_client(http.port)
        client.request("GET", "/api/session")
        response = client.getresponse()
        cookie = response.getheader("Set-Cookie").split(";", 1)[0]
        csrf = json.loads(response.read())["csrf"]
        headers = {"Content-Type": "application/json", "Cookie": cookie, "X-Console-CSRF": csrf,
                   "X-Console-Window": "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa", "Origin": f"http://127.0.0.1:{http.port}"}
        body = json.dumps({"action": "claim"})
        client.request("POST", "/api/command", body, dict(headers, Origin="http://attacker.invalid"))
        response=client.getresponse();assert response.status == 403;response.read()
        client.request("POST", "/api/command", body, headers)
        response=client.getresponse();assert response.status == 200;response.read()
        headers["X-Console-Window"]="bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb"
        client.request("POST", "/api/command", body, headers)
        response=client.getresponse();assert response.status == 409;response.read()
        client.close()
    finally: http.close()


def http_client(port):
    return http.client.HTTPConnection("127.0.0.1", port, timeout=2)
