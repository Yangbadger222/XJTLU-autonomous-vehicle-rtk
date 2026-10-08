"""ROS adapter and owned raw-bag player for the loopback cockpit."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from research_interfaces.msg import OperatorPermit, ResearchStatus, TimedTrajectory2D, RoadGraphUpdate2D, RoadEvent
from research_interfaces.srv import ManageResearchTask

from .console_http import ConsoleHTTP
from .operator_console import OperatorConsole
from .physical_parameter_lock import STOP_CONFIRMATION, CONTROL_ODOM_TOPIC, CONTROL_CHILD_FRAME
from .runtime_paths import research_path
from .safety_bridge import _parameter_bool
from .bag_identity import verify_cataloged_bag


class ConsoleNode(Node):
    def __init__(self):
        super().__init__("research_operator_console")
        mode = str(self.declare_parameter("execution_mode", "replay").value)
        actuator = _parameter_bool(self.declare_parameter("actuator_enabled", False).value)
        mission = _parameter_bool(self.declare_parameter("mission_execution_enabled", False).value)
        self.console = OperatorConsole(mode=mode, actuator_enabled=actuator, mission_enabled=mission,
                                       stopped_limits=STOP_CONFIRMATION)
        port = int(self.declare_parameter("http_port", 8765).value)
        self.http = ConsoleHTTP(self.console, Path(get_package_share_directory("research_runtime"))/"web", port)
        self.pub = self.create_publisher(OperatorPermit, "/research/operator_permit", 10)
        self.create_subscription(String,"/research/map_version",lambda m:self.console.update("map_version",str(m.data)),10)
        self.create_subscription(String,"/localization_authority/mode",lambda m:self.console.update("rtk_mode",str(m.data)),10)
        self.create_subscription(Bool,"/localization_authority/motion_allowed",lambda m:self.console.update("authority",bool(m.data)),10)
        self.create_subscription(String,"/lio/vehicle_health",lambda m:self.console.update("health",str(m.data)),10)
        self.create_subscription(Bool,"/research/tf_integrity",lambda m:self.console.update("tf",bool(m.data)),10)
        self.create_subscription(Odometry,CONTROL_ODOM_TOPIC,self._odom,10)
        self.create_subscription(OccupancyGrid,"/research/local_obstacle_grid",lambda m:self._grid(m,"grid"),10)
        self.create_subscription(OccupancyGrid,"/research/permission_grid",lambda m:self._grid(m,"permission"),10)
        self.create_subscription(ResearchStatus,"/research/observation_status",lambda m:self._status(m,"observer"),10)
        self.create_subscription(ResearchStatus,"/research/status",lambda m:self._status(m,"tracker"),10)
        self.create_subscription(TimedTrajectory2D,"/research/ego_trajectory",self._trajectory,10)
        self.create_subscription(RoadGraphUpdate2D,"/research/road_graph_updates",self._road,100)
        self.create_subscription(RoadEvent,"/research/road_events",self._event,100)
        self.client=self.create_client(ManageResearchTask,"/research/manage_task")
        self.task_pending=None
        self.inspect_at=0.
        self.player,self.player_log=None,None
        self.bags={}
        self.bag_rows={}
        self.replay_generation=0
        self.validation=None
        catalog=str(self.declare_parameter("bag_catalog_path","").value)
        if catalog:
            payload=json.loads(Path(catalog).read_text())
            for row in payload["bags"]:
                if row.get("selected_original"):
                    identity=row["raw_input_sha256"]
                    self.bags[identity]=Path(row["bag_path"])
                    self.bag_rows[identity]=row
                    self.console.replay["bags"].append({"id":identity,"label":row["bag_path"],"duration_s":row["duration_s"]})
        self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self._tick,clock=self.steady_clock)
        self.http.start()
        self.get_logger().info(f"Research cockpit: http://127.0.0.1:{self.http.port}; startup mode={mode}; actuator={actuator}")

    def _age(self,msg):
        return self.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def _odom(self,msg):
        q=msg.pose.pose.orientation
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        values=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if (msg.header.frame_id!="odom" or msg.child_frame_id!=CONTROL_CHILD_FRAME or not 0<=self._age(msg)<=.2 or
            not all(math.isfinite(v) for v in (*values,norm)) or abs(norm-1)>1e-5): return
        self.console.update("odom",dict(zip(("x","y","v","w"),values),yaw=math.atan2(
            2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))

    def _grid(self,msg,name):
        q=msg.info.origin.orientation
        if (msg.header.frame_id!="odom" or not 0<=self._age(msg)<=.5 or
            not math.isfinite(msg.info.resolution) or msg.info.resolution<=0 or
            len(msg.data)!=msg.info.width*msg.info.height or not 0<len(msg.data)<=250000 or
            abs(q.x)+abs(q.y)+abs(q.z)>1e-9 or abs(q.w-1)>1e-9):return
        self.console.update(name,True)
        # Bounded operator display; never becomes a planning map or safety input.
        step=max(1,math.ceil(max(msg.info.width,msg.info.height)/120))
        cells=[int(msg.data[y*msg.info.width+x]) for y in range(0,msg.info.height,step) for x in range(0,msg.info.width,step)]
        self.console.update(name+"_display",{"frame":"odom","resolution":msg.info.resolution*step,
            "x":msg.info.origin.position.x,"y":msg.info.origin.position.y,
            "width":math.ceil(msg.info.width/step),"height":math.ceil(msg.info.height/step),"cells":cells})

    def _status(self,msg,name):
        if not 0<=self._age(msg)<=.5:return
        self.console.update(name,{"state":msg.state,"reason":msg.reason,"map_version":msg.map_version,
            "motion_allowed":msg.motion_allowed,"actuator_enabled":msg.actuator_enabled})

    def _trajectory(self,msg):
        if msg.header.frame_id!="odom" or not 0<=self._age(msg)<=.5:return
        self.console.update("trajectory",{"status":int(msg.status),"reason":msg.failure_reason,"map_version":msg.map_version,
            "points":[[float(p.x),float(p.y)] for p in msg.points[::max(1,len(msg.points)//300)]]})

    def _road(self,msg):
        if msg.header.frame_id!="odom":return
        self.console.update("road",{"id":msg.update_id,"frame":msg.header.frame_id,"source":msg.source,
            "points":[[float(p.x),float(p.y)] for p in msg.geometry[:300]]})

    def _event(self,msg):
        if msg.header.frame_id!="odom":return
        self.console.update("road_event",{"id":msg.event_id,"kind":msg.kind,"state":msg.state,"x":msg.x,"y":msg.y,
            "observed_length_m":msg.observed_length_m,"unknown_length_m":msg.unknown_length_m})

    def _task(self,identity,action,payload):
        if self.task_pending or not self.client.service_is_ready():
            if action!="inspect":self.console.task["reason"]="任务接口忙或离线"
            return False
        request=ManageResearchTask.Request()
        request.action,request.request_id=action,identity
        request.map_version=self.console._fresh("map_version") or "UNKNOWN"
        request.start_node,request.goal_node=payload.get("start_node",""),payload.get("goal_node","")
        future=self.client.call_async(request)
        self.task_pending=(future,time.monotonic(),action,identity)
        def done(f):
            if not self.task_pending or f is not self.task_pending[0]:return
            self.task_pending=None
            with self.console.lock:
                if f.cancelled() or f.exception():
                    self.console.task["reason"]="任务接口失败"
                    if action=="select":self.console.finish_request(identity,False,"任务接口失败")
                    return
                response=f.result()
                self.console.task.update(node_ids=list(response.node_ids),task_id=response.task_id,
                    start_node=response.start_node,goal_node=response.goal_node,reason=response.reason)
                if action=="select":
                    self.console.task["accepted"]=bool(response.accepted)
                    self.console.finish_request(identity,response.accepted,response.reason)
        future.add_done_callback(done)
        return True

    def _replay(self,action,payload,request_id):
        state=self.console.replay
        try:
            if action=="replay_start":
                if self.player and self.player.poll() is None:raise ValueError("当前已有回放，请先停止")
                if self.validation and self.validation.is_alive():raise ValueError("正在核对原始数据身份")
                if os.environ.get("ROS_LOCALHOST_ONLY")!="1":raise ValueError("回放需要 localhost-only 隔离环境")
                identity=payload["bag_id"]
                self.replay_generation+=1
                generation=self.replay_generation
                state.update(state="VALIDATING",reason="只读核对完整原始输入哈希")
                def verified():
                    try:path=verify_cataloged_bag(self.bag_rows[identity]);error=None
                    except (OSError,ValueError,KeyError) as exc:path=None;error=str(exc)
                    with self.console.lock:
                        if generation!=self.replay_generation:return
                        if error:
                            state.update(state="ERROR",reason=error)
                            self.console.finish_request(request_id,False,error)
                            return
                        try:
                            log_path=research_path("runtime-data/research/active_road/operator-replay.log")
                            log_path.parent.mkdir(parents=True,exist_ok=True)
                            self.player_log=log_path.open("a")
                            self.player=subprocess.Popen(["ros2","bag","play",str(path),"--rate","1.0","--clock","50",
                                "--topics","/livox/lidar","/livox/imu"],stdout=self.player_log,stderr=subprocess.STDOUT,start_new_session=True)
                            state.update(state="PLAYING",bag_id=identity,pid=self.player.pid,reason="完整原始输入身份已核实；1× 测量时钟")
                            self.console.finish_request(request_id,True,state["reason"])
                        except OSError as exc:
                            state.update(state="ERROR",reason=str(exc))
                            self.console.finish_request(request_id,False,str(exc))
                self.validation=threading.Thread(target=verified,daemon=True);self.validation.start()
            elif action=="replay_stop":
                self.replay_generation+=1;self._stop_player();state.update(state="STOPPED",reason="已结束本界面回放/校验")
            elif not self.player or self.player.poll() is not None:raise ValueError("没有运行中的本界面回放")
            elif action=="replay_pause":
                os.killpg(self.player.pid,signal.SIGSTOP);state.update(state="PAUSED",reason="本界面回放进程已暂停")
            elif action=="replay_resume":
                os.killpg(self.player.pid,signal.SIGCONT);state.update(state="PLAYING",reason="已恢复本界面回放")
        except (OSError,ValueError,KeyError) as exc:state.update(reason=str(exc),state="ERROR")

    def _stop_player(self):
        if self.player and self.player.poll() is None:
            os.killpg(self.player.pid,signal.SIGCONT)
            os.killpg(self.player.pid,signal.SIGINT)
            try:self.player.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(self.player.pid,signal.SIGTERM)
                self.player.wait(timeout=2)
        if self.player_log:self.player_log.close();self.player_log=None

    def _tick(self):
        self.console.tick()
        permit=self.console.permit()
        msg=OperatorPermit()
        msg.header.stamp,msg.header.frame_id=self.get_clock().now().to_msg(),"odom"
        msg.session_id,msg.sequence,msg.execution_mode=permit.session_id,permit.sequence,permit.mode
        msg.map_version,msg.state,msg.motion_requested=permit.map_version,permit.state,permit.motion_requested
        msg.lease_active=permit.lease_active
        msg.reason=self.console.reason
        self.pub.publish(msg)
        with self.console.lock:
            if self.console.pending:
                identity,action,payload,queued_at=self.console.pending[0]
                if action=="task":
                    if self._task(identity,"select",payload):self.console.pending.popleft()
                    elif time.monotonic()-queued_at>.5:
                        self.console.pending.popleft()
                        self.console.finish_request(identity,False,"任务接口忙或离线，未选择任务")
                else:
                    self.console.pending.popleft();self._replay(action,payload,identity)
                    if self.console.replay["state"]!="VALIDATING":
                        self.console.finish_request(identity,self.console.replay["state"]!="ERROR",self.console.replay["reason"])
            if time.monotonic()-self.inspect_at>2.:
                self.inspect_at=time.monotonic();self._task("inspect","inspect",{})
            if self.task_pending and time.monotonic()-self.task_pending[1]>.5:
                future,_,action,identity=self.task_pending;self.task_pending=None;future.cancel()
                self.console.task.update(accepted=False,reason="任务接口超时")
                if action=="select":self.console.finish_request(identity,False,"任务接口超时，未确认任务选择")
            if self.player and self.player.poll() is not None and self.console.replay["state"] in ("PLAYING","PAUSED"):
                self.console.replay.update(state="FINISHED" if self.player.returncode==0 else "ERROR",reason="回放退出码："+str(self.player.returncode))

    def close(self):
        with self.console.lock:
            self.console._set("STOP_LATCHED","界面后端退出")
            self.console.pending.clear()
            self.replay_generation+=1
        if rclpy.ok(): self._tick()
        self.http.close()
        self._stop_player()


def main(args=None):
    rclpy.init(args=args)
    node=ConsoleNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
