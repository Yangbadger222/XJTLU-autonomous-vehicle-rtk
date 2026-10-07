#!/usr/bin/env python3
"""Exercise the actual cockpit-owned raw player; no actuator or synthetic sensor."""
import argparse
import http.cookiejar
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
import uuid

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from research_interfaces.msg import OperatorPermit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--catalog', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '100' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('requires isolated domain 100 and localhost-only')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = json.loads(args.catalog.read_text())['bags']
    selected = next(row for row in rows if row.get('selected_original') and '07-06-18' in row['bag_path'])
    base = 'http://127.0.0.1:8766'
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    window, csrf = str(uuid.uuid4()), ''
    rclpy.init()
    node = rclpy.create_node('console_raw_replay_probe')
    counts = {'lidar': 0, 'imu': 0, 'clock': 0, 'motion_permits': 0}
    last_clock = [None]
    def sensor(key):
        def received(message):
            counts[key] += 1
            if key == 'clock':
                last_clock[0] = message.clock.sec + message.clock.nanosec / 1e9
        return received
    qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(CustomMsg, '/livox/lidar', sensor('lidar'), qos)
    node.create_subscription(Imu, '/livox/imu', sensor('imu'), qos)
    node.create_subscription(Clock, '/clock', sensor('clock'), qos)
    def permit(message):
        if message.motion_requested: counts['motion_permits'] += 1
    node.create_subscription(OperatorPermit, '/research/operator_permit', permit, 10)
    def state():
        return json.loads(opener.open(urllib.request.Request(base+'/api/state', headers={'X-Console-Window': window}), timeout=.5).read())
    def command(action, payload=None):
        body = json.dumps({'action': action, 'payload': payload or {}, 'request_id': str(uuid.uuid4())}).encode()
        req = urllib.request.Request(base+'/api/command', data=body, headers={'Content-Type':'application/json', 'Origin':base,
            'X-Console-CSRF':csrf, 'X-Console-Window':window})
        return json.loads(opener.open(req, timeout=.5).read())
    def phase(duration):
        deadline, renewed = time.monotonic()+duration, 0.
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            if csrf and time.monotonic()-renewed>.15:
                assert command('heartbeat')['accepted']
                renewed=time.monotonic()
        return {'counts':dict(counts), 'clock':last_clock[0], 'replay':state()['replay']}
    log = args.output.with_suffix('.log').open('w')
    console = subprocess.Popen([sys.executable, str(args.install/'research_runtime/lib/research_runtime/research_operator_console'),
        '--ros-args', '-p', 'http_port:=8766', '-p', 'execution_mode:=replay', '-p', 'actuator_enabled:=false',
        '-p', 'mission_execution_enabled:=false', '-p', 'bag_catalog_path:='+str(args.catalog.resolve())],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    sentinel = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'], start_new_session=True)
    result = {'status':'FAIL', 'scope':'Actual raw bag prefix, HTTP controls, owned rosbag process and ROS subscriptions; no estimator, actuator or closed-loop policy claim.'}
    try:
        deadline=time.monotonic()+8
        while True:
            try:csrf=json.loads(opener.open(base+'/api/session',timeout=.5).read())['csrf'];break
            except OSError:
                if time.monotonic()>deadline:raise
                rclpy.spin_once(node,timeout_sec=.05)
        assert command('claim')['accepted']
        assert command('replay_start', {'bag_id':selected['raw_input_sha256']})['accepted']
        deadline=time.monotonic()+50
        while state()['replay']['state']=='VALIDATING' or state()['replay']['state']=='IDLE':
            phase(.2)
            if time.monotonic()>deadline:raise RuntimeError('raw identity validation timeout')
        assert state()['replay']['state']=='PLAYING', state()['replay']
        playing=phase(3.)
        pid=playing['replay']['pid']
        argv=Path('/proc/'+str(pid)+'/cmdline').read_bytes().replace(b'\0',b' ').decode()
        assert command('replay_pause')['accepted']
        phase(.5)  # drain messages already queued before SIGSTOP
        paused_start=phase(.2)
        paused_end=phase(1.2)
        assert command('replay_resume')['accepted']
        resumed=phase(2.)
        assert command('replay_stop')['accepted']
        stopped=phase(.8)
        checks={
            'actual_raw_lidar_and_imu':playing['counts']['lidar']>0 and playing['counts']['imu']>0,
            'clock_and_raw_counts_frozen_while_paused':paused_start['counts']==paused_end['counts'] and paused_start['clock']==paused_end['clock'],
            'raw_and_clock_resume':resumed['counts']['imu']>paused_end['counts']['imu'] and resumed['clock']>paused_end['clock'],
            'owned_player_exited':not Path('/proc/'+str(pid)).exists() and stopped['replay']['state']=='STOPPED',
            'unrelated_task_sentinel_survives':sentinel.poll() is None,
            'no_motion_permit':counts['motion_permits']==0,
            'no_recorded_command_topics': '--topics /livox/lidar /livox/imu' in argv,
        }
        result.update(status='PASS' if all(checks.values()) else 'FAIL', checks=checks,
            bag_id=selected['raw_input_sha256'], player_argv=argv, player_pid=pid,
            playing=playing, paused_start=paused_start, paused_end=paused_end, resumed=resumed, stopped=stopped)
    finally:
        for child in (console,sentinel):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
