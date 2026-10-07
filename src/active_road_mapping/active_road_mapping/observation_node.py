"""Finite, asynchronous current-evidence observation orchestration.

This process receives current grids/state only. It does not accept hidden
world truth, future geometry or client asserted candidate safety flags.
Every proposed driving pose goes through the actual EGO query service and
the same footprint validator before it can become a reference request.
"""
from __future__ import annotations

import hashlib
import math
import time
import uuid
from pathlib import Path as FilePath

from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Bool, String
from research_interfaces.msg import ObservationGoal, RoadEvent, RoadEvidence2D, ResearchStatus,RoadGraphUpdate2D
from research_interfaces.srv import PlanRoadReference, ManageResearchTask
from research_interfaces.msg import OperatorPermit
from research_runtime.operator_gate import OperatorGate, consent_from_message

from research_runtime.active_observation import (FiniteObservationPolicy, frontier_events,
    planned_views, visible_fraction, prior_gap_events, task_impact, supported_gap_updates, save_snapshot, load_snapshot,
    RoadGraph,GraphEdge,current_session_evidence)
from research_runtime.active_road import RoadEvidence, EvidenceState,EvidenceStore
from research_runtime.physical_parameter_lock import LOCKED_FOOTPRINT
from research_runtime.runtime_paths import research_path
from research_runtime.prior_mission import registered_prior, transform_graph, confirmed_route_prefix
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.safety_bridge import rtk_mode_is_allowed
from research_runtime.trajectory import TimedPoint, TimedTrajectory, VehicleLimits


class ActiveObservationNode(Node):
    def __init__(self):
        super().__init__("active_observation")
        self.execution_mode = str(self.declare_parameter("execution_mode","replay").value)
        self.mission_enabled = self.declare_parameter("mission_execution_enabled",False).value is True
        self.operator_gate = OperatorGate()
        self.last_tick = time.monotonic()
        self.task_id = "startup"
        if self.execution_mode not in ("replay","shadow","live"):raise ValueError("invalid execution mode")
        self.stop_permission = self.create_publisher(Bool,"/gps_corridor/stop_override",10)
        self.mode = str(self.declare_parameter("policy", "TASK_AWARE_LOOK").value)
        self.policy = FiniteObservationPolicy(self.mode,
            budget_s=float(self.declare_parameter("observation_budget_s", 60.0).value),
            periodic_s=float(self.declare_parameter("periodic_look_s", 10.0).value))
        self.range = float(self.declare_parameter("sensor_range_m", 0.0).value)
        self.fov = float(self.declare_parameter("sensor_fov_rad", 0.0).value)
        self.settle = float(self.declare_parameter("settle_s", 1.0).value)
        self.pose_uncertainty_limit = float(self.declare_parameter("pose_uncertainty_limit_m", .10).value)
        if (not all(math.isfinite(v) for v in (self.range,self.fov,self.settle,self.pose_uncertainty_limit))
                or self.range < 0 or not 0 <= self.fov <= 2*math.pi or self.settle <= 0
                or self.pose_uncertainty_limit <= 0):
            raise ValueError("invalid observation sensor/trust/settling configuration")
        self.limits = VehicleLimits(
            max_curvature_1pm=float(self.declare_parameter("max_curvature_1pm", 0.0).value))
        self.footprint = LOCKED_FOOTPRINT
        self.prior = self.map_graph = self.odom_graph = None
        self.mission_events = []
        manifest = str(self.declare_parameter("prior_manifest_path", "").value)
        self.task_start = str(self.declare_parameter("task_start_node", "").value)
        self.task_goal = str(self.declare_parameter("task_goal_node", "").value)
        self.failure_cost = float(self.declare_parameter("task_failure_cost_m", 100.0).value)
        self.snapshot_path = research_path(str(self.declare_parameter("policy_snapshot_path",
            "runtime-data/research/active_road/observation_policy.json").value))
        self.recorded_evidence = []
        self.localization_session_id=""
        self.create_subscription(String,"/research/localization_session_id",
            lambda msg:setattr(self,"localization_session_id",str(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.evidence_store_path=research_path(str(self.declare_parameter("evidence_store_path",
            "runtime-data/research/active_road/evidence.json").value))
        self.published_updates=set()
        self.tf_integrity,self.tf_received=False,0.
        self.permission,self.permission_received=None,0.
        self.measured_questions={}
        if manifest:
            self.prior,self.raster,self.map_graph,self.prior_manifest = registered_prior(manifest)
            if self.task_start not in self.map_graph.nodes or self.task_goal not in self.map_graph.nodes:
                raise ValueError("task endpoints must name existing registered-prior graph nodes")
            if self.snapshot_path.exists():
                self.recorded_evidence,_ = load_snapshot(self.snapshot_path,self.prior,self.policy)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.version, self.grid, self.pose = "UNKNOWN", None, None
        self.pose_uncertainty = None
        self.submap_id = "odom-session:"+str(uuid.uuid4())
        self.grid_received = self.odom_received = self.health_received = 0.0
        self.health, self.allowed = "UNKNOWN", False
        self.authority_mode,self.authority_mode_received="UNKNOWN",0.
        self.pending, self.active, self.views, self.queue = None, None, [], []
        self.dwell_start = None
        self.started = time.monotonic()
        self.ready_started = None
        self.last_reason = "WAITING_FOR_MEASURED_INPUTS"
        self.client = self.create_client(PlanRoadReference, "/research/ego_plan_query")
        self.events = self.create_publisher(RoadEvent, "/research/road_events", 10)
        self.goals = self.create_publisher(ObservationGoal, "/research/observation_goal", 10)
        self.references = self.create_publisher(Path, "/research/road_reference_input", 10)
        self.graph_updates=self.create_publisher(RoadGraphUpdate2D,"/research/road_graph_updates",100)
        self.evidence_ids={e.evidence_id for e in self.recorded_evidence}
        self.create_subscription(RoadEvidence2D,"/research/road_evidence",self._measured_evidence,1000)
        self.create_subscription(OccupancyGrid,"/research/permission_grid",self._permission,10)
        self.create_subscription(Bool,"/research/tf_integrity",self._tf,10)
        self.create_subscription(RoadEvent,"/research/measured_road_questions",self._question,100)
        self.create_timer(.5,self._save_policy)
        self.status = self.create_publisher(ResearchStatus, "/research/observation_status", 10)
        self.create_subscription(String, "/research/map_version", self._version, 10)
        self.create_subscription(OccupancyGrid, "/research/local_obstacle_grid", self._grid, 10)
        self.create_subscription(Odometry, "/lio/odom_vehicle", self._odom, 10)
        self.create_subscription(String, "/lio/vehicle_health", self._health, 10)
        self.create_subscription(Bool, "/localization_authority/motion_allowed", self._authority, 10)
        self.create_subscription(String,"/localization_authority/mode",self._authority_mode,10)
        self.create_subscription(OperatorPermit,"/research/operator_permit",self._operator,10)
        self.create_service(ManageResearchTask,"/research/manage_task",self._manage_task)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self._tick,clock=self.steady_clock)

    def _operator(self,msg):
        age=self.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if msg.header.frame_id=="odom" and 0<=age<=.5:
            self.operator_gate.receive(consent_from_message(msg),time.monotonic())

    def _operator_allowed(self,now):
        return self.operator_gate.allowed(now,mode=self.execution_mode,map_version=self.version,
            sole_publisher=self.count_publishers("/research/operator_permit")==1)

    def _cancel_observation(self,now):
        if self.pending:
            self.pending=None
            self.pending_future.cancel()
        if self.active:
            view,event,start=self.active
            self.policy.record(view,actual_cost_s=max(0.,now-start),elapsed_s=now-self.started,resolved_geometry=False)
            self._save_policy()
        self.active,self.queue,self.views,self.dwell_start=None,[],[],None

    def _manage_task(self,request,response):
        response.node_ids=sorted(self.map_graph.nodes) if self.map_graph else []
        response.accepted=False
        response.reason="No registered prior/task endpoints"
        if request.action=="inspect":
            response.accepted=bool(self.map_graph)
            response.reason="REGISTERED_PRIOR" if self.map_graph else response.reason
        elif request.action=="select":
            editable=self.operator_gate.editing_allowed(time.monotonic(),mode=self.execution_mode,map_version=self.version,
                sole_publisher=self.count_publishers("/research/operator_permit")==1)
            if not editable or request.map_version!=self.version:
                response.reason="OPERATOR_PAUSE_OR_MAP_VERSION_REQUIRED"
            elif (not request.request_id or len(request.request_id)>128 or self.map_graph is None or
                    request.start_node not in self.map_graph.nodes or request.goal_node not in self.map_graph.nodes or
                    request.start_node==request.goal_node):
                response.reason="INVALID_REGISTERED_ENDPOINTS"
            else:
                self._cancel_observation(time.monotonic())
                self.task_start,self.task_goal=request.start_node,request.goal_node
                self.task_id=request.request_id
                self.ready_started=None
                self.started=time.monotonic()
                self.policy.spent_s,self.policy.last_look_s=0.,-math.inf
                response.accepted,response.reason=True,"TASK_SELECTED_REQUIRES_EXPLICIT_START"
        response.task_id,response.start_node,response.goal_node=self.task_id,self.task_start,self.task_goal
        return response

    def _version(self, msg):
        if msg.data != self.version:
            self.grid, self.queue, self.views = None, [], []
        self.version = msg.data

    def _health(self, msg):
        self.health, self.health_received = msg.data, time.monotonic()

    def _authority(self, msg):
        self.allowed, self.authority_received = msg.data, time.monotonic()

    def _authority_mode(self,msg):
        self.authority_mode,self.authority_mode_received=str(msg.data),time.monotonic()

    def _tf(self,msg):
        self.tf_integrity,self.tf_received=msg.data is True,time.monotonic()

    def _permission(self,msg):
        q=msg.info.origin.orientation
        age=self.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        self.permission=None
        if msg.header.frame_id!='odom' or abs(q.x)+abs(q.y)+abs(q.z)>1e-9 or abs(q.w-1)>1e-9 or not 0<=age<=.5:return
        try:
            self.permission=LocalObstacleGrid('odom','permission',msg.info.resolution,msg.info.origin.position.x,
                msg.info.origin.position.y,msg.info.width,msg.info.height,
                tuple(0 if value==0 else 100 for value in msg.data))
            self.permission_received=time.monotonic()
        except (TypeError,ValueError):pass

    def _question(self,msg):
        from research_runtime.active_observation import ObservationEvent
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if (msg.header.frame_id!='odom' or not 0<=age<=.5 or not msg.event_id or not msg.source or
            msg.kind not in ('geometric_entry','clearance_question','step_question','obstacle_question') or
            not all(math.isfinite(v) for v in (msg.x,msg.y,msg.observed_length_m,msg.unknown_length_m)) or
            msg.observed_length_m<0 or msg.unknown_length_m<0):return
        event=ObservationEvent(msg.event_id,msg.kind,((msg.x,msg.y),),state='UNCERTAIN',
            observed_length_m=msg.observed_length_m,question='sensor-reported geometry/clearance question: '+msg.source)
        self.measured_questions[msg.event_id]=(event,time.monotonic())

    def _grid(self, msg):
        q = msg.info.origin.orientation
        age = (self.get_clock().now().nanoseconds-
               (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if (msg.header.frame_id != "odom" or abs(q.x)+abs(q.y)+abs(q.z) > 1e-9 or
                abs(q.w-1) > 1e-9 or not 0 <= age <= .5):
            self.grid = None
            return
        try:
            self.grid = LocalObstacleGrid("odom", self.version, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                msg.info.width, msg.info.height, tuple(msg.data))
            self.grid_received = time.monotonic()
            self.grid_stamp = msg.header.stamp
        except ValueError:
            self.grid = None

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        age = (self.get_clock().now().nanoseconds-
               (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if (msg.header.frame_id != "odom" or msg.child_frame_id != "base_footprint" or
                not math.isfinite(norm) or norm <= 1e-12 or not 0 <= age <= .20):
            self.pose = None
            return
        x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.pose = values if all(math.isfinite(v) for v in values) else None
        covariance = msg.pose.covariance
        xx,xy,yx,yy = (covariance[i] for i in (0,1,6,7))
        self.pose_uncertainty = None
        if (all(math.isfinite(v) for v in (xx,xy,yx,yy)) and xx >= 0 and yy >= 0
                and abs(xy-yx) <= 1e-8):
            discriminant = math.sqrt((xx-yy)**2+4*xy*xy)
            smallest, largest = .5*(xx+yy-discriminant), .5*(xx+yy+discriminant)
            if smallest >= -1e-10 and largest > 1e-12:
                self.pose_uncertainty = math.sqrt(largest)
        self.odom_received = time.monotonic()

    def _ready(self, now):
        return (self.version not in ("", "UNKNOWN") and self.grid is not None and self.pose is not None
                and now-self.grid_received <= .5 and now-self.odom_received <= .20
                and now-self.health_received <= .5 and self.health.startswith("OK")
                and self.range > 0 and 0 < self.fov <= 2*math.pi
                and self.limits.max_curvature_1pm > 0 and self.pose_uncertainty is not None
                and self.pose_uncertainty <= self.pose_uncertainty_limit)

    def _candidate_safety_ready(self,now):
        return (self._ready(now) and self.tf_integrity and 0<=now-self.tf_received<=.20 and
                self.permission is not None and 0<=now-self.permission_received<=.5)

    def _reference(self, pose):
        msg = Path()
        msg.header.frame_id, msg.header.stamp = "odom", self.get_clock().now().to_msg()
        for i in range(8):
            point = PoseStamped()
            point.header = msg.header
            point.pose.position.x = self.pose[0]+(pose[0]-self.pose[0])*i/7
            point.pose.position.y = self.pose[1]+(pose[1]-self.pose[1])*i/7
            point.pose.orientation.w = 1.0
            msg.poses.append(point)
        return msg

    def _build_queries(self):
        events = list(self.mission_events)+frontier_events(self.grid)
        events += [event for event,received in self.measured_questions.values() if time.monotonic()-received<=.5]
        self.event_by_id = {event.event_id:event for event in events}
        impact_of = lambda event: task_impact(event,self.odom_graph,self.task_start,self.task_goal,
            self.failure_cost) if self.odom_graph else (1.0,"coverage_fallback:no_registered_task")
        for event in events[:100]:
            msg = RoadEvent()
            msg.header.frame_id, msg.header.stamp = "odom", self.grid_stamp
            msg.event_id, msg.kind, msg.state = event.event_id, event.kind, event.state
            msg.x, msg.y = event.targets[0]
            msg.impact, msg.source = impact_of(event)
            msg.observed_length_m=event.observed_length_m
            msg.unknown_length_m=sum(math.dist(a,b) for a,b in zip(event.targets,event.targets[1:]))
            self.events.publish(msg)
        for event in sorted(events,key=lambda event: impact_of(event)[0],reverse=True)[:3]:
            if self.mode=="TASK_AWARE_LOOK" and impact_of(event)[0]<=0:continue
            for step in (1,2,3,4,6,8):
                pose = (self.pose[0]+step*self.grid.resolution_m*math.cos(self.pose[2]),
                        self.pose[1]+step*self.grid.resolution_m*math.sin(self.pose[2]), self.pose[2])
                polygon = [(pose[0]+math.cos(pose[2])*x-math.sin(pose[2])*y,
                            pose[1]+math.sin(pose[2])*x+math.cos(pose[2])*y) for x,y in self.footprint]
                if (self._candidate_safety_ready(time.monotonic()) and not self.permission.polygon_occupied(polygon) and
                    not self.grid.polygon_occupied(polygon) and visible_fraction(event, pose, self.grid,
                        sensor_range_m=self.range, fov_rad=self.fov)[0] > 0):
                    self.queue.append((event, pose, False))

    def _query_result(self, future):
        if self.pending is None or future.cancelled() or future is not self.pending_future:
            return
        event, pose, version, commit = self.pending
        self.pending = None
        if not self._candidate_safety_ready(time.monotonic()) or version != self.version or future.exception() is not None:
            return
        msg = future.result().trajectory
        if msg.status != msg.STATUS_OK:
            return
        seconds = lambda t: t.sec+t.nanosec*1e-9
        trajectory = TimedTrajectory.from_points(msg.trajectory_id, msg.map_version, msg.header.frame_id,
            seconds(msg.generated_at), seconds(msg.valid_until),
            [TimedPoint(p.t,p.x,p.y,p.yaw,p.v,p.w,p.a,p.alpha,p.curvature) for p in msg.points])
        from research_runtime.trajectory import validate_trajectory
        if not validate_trajectory(trajectory,self.limits,now=self.get_clock().now().nanoseconds*1e-9,
            expected_map_version=self.version,footprint=self.footprint,occupied=self.permission.occupied,
            occupied_polygon=self.permission.polygon_occupied,resolution=self.permission.resolution_m).valid:return
        impact,impact_reason = (task_impact(event,self.odom_graph,self.task_start,self.task_goal,
            self.failure_cost) if self.odom_graph else (1.0,"coverage_fallback:no_registered_task"))
        views = planned_views(event, [pose], self.grid, self.limits, self.footprint,
            lambda *_: trajectory, now=self.get_clock().now().nanoseconds*1e-9, impact=impact,
            sensor_range_m=self.range, fov_rad=self.fov, pose_trustworthy=True, sensor_valid=True,
            settle_s=self.settle, attempted=self.policy.attempted)
        if commit:
            if views and self._can_request_motion(time.monotonic()):
                view = views[0]
                self.active = view,event,time.monotonic()
                msg = ObservationGoal()
                msg.header.frame_id, msg.header.stamp = "odom", self.get_clock().now().to_msg()
                msg.goal_id, msg.event_id = view.candidate_id,view.event_id
                msg.x,msg.y,msg.yaw = view.pose
                msg.score,msg.cost,msg.observable_fraction = view.score,view.cost_s,view.observable_fraction
                msg.reachable=msg.safe=msg.pose_trustworthy=msg.sensor_valid=True
                msg.reason=view.reason+";"+impact_reason+";fresh_commit_query"
                self.goals.publish(msg)
        else:
            self.views.extend(views)

    def _can_request_motion(self,now):
        return (self.execution_mode=="live" and self.mission_enabled and self.allowed and
                self._operator_allowed(now) and
                rtk_mode_is_allowed(self.authority_mode,now-self.authority_mode_received) and
                now-getattr(self,"authority_received",0)<=.5 and self._candidate_safety_ready(now))

    def _tick(self):
        now = time.monotonic()
        elapsed=max(0.,now-self.last_tick)
        self.last_tick=now
        reason = "WAITING_FOR_MEASURED_INPUTS"
        if self.execution_mode=="live" and not self._operator_allowed(now):
            if self.ready_started: self.ready_started+=elapsed
            self._cancel_observation(now)
            if self._ready(now): self._mission_update()  # Keep perception/evidence while stopped.
            self._publish_status("OPERATOR_PAUSED_OR_PERMISSION_LOST")
            return
        if self.ready_started and now-self.ready_started>=self.policy.budget_s:
            if self.active:
                view,event,start=self.active
                self.policy.record(view,actual_cost_s=now-start,elapsed_s=now-self.started,resolved_geometry=False)
                self._save_policy()
            self.active,self.queue,self.views=None,[],[]
            self._publish_status("BUDGET_EXHAUSTED")
            return
        if self._ready(now):
            self._mission_update()
            self.ready_started = self.ready_started or now
            if self.odom_graph and math.dist(self.pose[:2],self.odom_graph.nodes[self.task_goal]) <= .20:
                self.active,self.queue,self.views = None,[],[]
                self._publish_status("TASK_REACHED")
                return
            if not self.active and self.odom_graph and self._can_request_motion(now):
                points,_ = confirmed_route_prefix(self.odom_graph,self.task_start,self.task_goal,
                    self.pose,self.grid,self.footprint)
                if len(points)>=3:
                    self.references.publish(self._polyline(points))

            if self.active:
                view, event, start = self.active
                if event.kind=="prior_gap" and self.odom_graph and event.event_id in self.odom_graph.edges:
                    # Current sensor evidence may resolve a gap during approach.
                    # No extra view is needed once its whole corridor is observed.
                    self.policy.record(view,actual_cost_s=now-start,elapsed_s=now-self.started,resolved_geometry=True)
                    self.active=None;self.dwell_start=None;self._save_policy()
                    self._publish_status("GEOMETRY_RESOLVED_DURING_APPROACH")
                    return
                if now-start > 2*view.cost_s+10:
                    self.policy.record(view,actual_cost_s=now-start,elapsed_s=now-self.started,resolved_geometry=False)
                    self.active=None;self.dwell_start=None;self._save_policy()
                    self._publish_status("OBSERVATION_EXECUTION_TIMEOUT_UNCERTAIN")
                    return
                arrived = (math.dist(self.pose[:2], view.pose[:2]) <= .1 and abs(self.pose[3]) < .02
                    and abs(self.pose[4]) < .02 and
                    abs(math.atan2(math.sin(self.pose[2]-view.pose[2]),math.cos(self.pose[2]-view.pose[2]))) < .10)
                if arrived:
                    self.dwell_start = self.dwell_start or now
                    if now-self.dwell_start >= self.settle:
                        known = [point for point in event.targets if self.grid.value_at(*point) in (0,100)]
                        resolved = (self.odom_graph is not None and event.event_id in self.odom_graph.edges
                            if event.kind == "prior_gap" else len(known) >= .8*len(event.targets))
                        self.policy.record(view, actual_cost_s=now-start, elapsed_s=now-self.started,
                                           resolved_geometry=resolved)
                        self._save_policy()
                        self.active, self.dwell_start = None, None
                        reason = "OBSERVED_GEOMETRY" if resolved else "OBSERVATION_UNCERTAIN"
                else:
                    reason = "MOVING_TO_OBSERVATION_POSE"
                    if self._can_request_motion(now):
                        self.references.publish(self._reference(view.pose))
            elif self.pending:
                reason = "EGO_CANDIDATE_QUERY"
                if now-self.query_sent > .50:
                    self.pending = None
                    self.pending_future.cancel()
                    reason = "EGO_CANDIDATE_QUERY_TIMEOUT"
            elif self.queue and self.client.service_is_ready():
                event, pose, commit = self.queue.pop(0)
                request = PlanRoadReference.Request()
                request.request_id = f"view:{event.event_id}:{pose}"
                request.map_version, request.road_reference = self.version, self._reference(pose)
                self.pending = event, pose, self.version, commit
                self.query_sent = now
                self.pending_future = self.client.call_async(request)
                self.pending_future.add_done_callback(self._query_result)
                reason = "EGO_CANDIDATE_QUERY"
            elif self.views:
                view, reason = self.policy.choose(self.views, elapsed_s=now-self.started)
                self.views = []
                if view:
                    event = self.event_by_id.get(view.event_id)
                    if event:
                        # Cached costs rank views; a second actual planner call
                        # checks the selected one against the current state/map.
                        self.queue.insert(0,(event,view.pose,True))
            else:
                if self.mode != "PASSIVE":
                    self._build_queries()
                reason = "NO_SAFE_OBSERVATION_POSE" if not self.queue else "CANDIDATES_FROM_CURRENT_EVIDENCE"
        self._publish_status(reason)

    def _measured_evidence(self,msg):
        if msg.evidence_id in self.evidence_ids or msg.header.frame_id!="odom":return
        try:
            evidence=RoadEvidence(msg.evidence_id,[(p.x,p.y) for p in msg.geometry],EvidenceState(msg.state),
                msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,msg.source,msg.local_submap_id,
                msg.pose_uncertainty_m,msg.observed_length_m)
            from research_runtime.active_road import EvidenceStore
            EvidenceStore._validate_evidence(evidence)
            if evidence.state==EvidenceState.TRAVERSED:return
            self.recorded_evidence.append(evidence);self.evidence_ids.add(evidence.evidence_id)
        except (TypeError,ValueError):pass

    def _polyline(self, points):
        msg = Path()
        msg.header.frame_id,msg.header.stamp = "odom",self.get_clock().now().to_msg()
        for x,y in points:
            point=PoseStamped();point.header=msg.header
            point.pose.position.x,point.pose.position.y,point.pose.orientation.w=x,y,1.0
            msg.poses.append(point)
        return msg

    def _mission_update(self):
        self.odom_graph = None
        if self.map_graph is None:
            return
        try:
            # Original RTK TF intentionally has a 0.10 s future horizon.
            # Query the ground acquisition time instead of treating its latest
            # future-dated transform as an acquisition pose.
            transform=self.tf_buffer.lookup_transform("odom","map",self.grid_stamp)
            stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= .20:
                return
            q=transform.transform.rotation
            if abs(q.x)+abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>1e-5:
                return
            p=transform.transform.translation
            graph=transform_graph(self.map_graph,(p.x,p.y,2*math.atan2(q.z,q.w)))
            self.mission_events=prior_gap_events(graph,target_step_m=self.grid.resolution_m*.25)
            self.odom_graph=supported_gap_updates(graph,self.mission_events,self.grid,self.footprint)
            try:
                store=EvidenceStore.load(self.evidence_store_path)
            except (OSError,ValueError,KeyError,TypeError):store=None
            if store:
                # Historical increments require valid global submap anchors.
                # They influence topology; local execution always rechecks the
                # current ground/permission grid through confirmed_route_prefix.
                historical=[]
                registered=list(store.graph_updates_in_map())
                # Historical exports were captured by the sole writer only
                # under verified RTK/TF/uncertainty gates. Keep current local
                # evidence even if it was never globally qualified.
                for path in sorted(self.evidence_store_path.parent.joinpath('verified_history').glob('*.json'))[:100]:
                    try:
                        snapshot=EvidenceStore.load(path)
                        if snapshot.prior_version!=store.prior_version:continue
                        registered.extend(update for update in snapshot.graph_updates_in_map()
                            if self.localization_session_id and not update['local_submap_id'].startswith(self.localization_session_id+'/'))
                    except (OSError,ValueError,KeyError,TypeError):continue
                seen=set()
                for update in registered:
                    if update['update_id'] in seen:continue
                    seen.add(update['update_id'])
                    if update['update_id'] in store.rolled_back_graph_updates:continue
                    a,b=update['start_node_id'],update['end_node_id']
                    if a not in self.map_graph.nodes or b not in self.map_graph.nodes:continue
                    geometry=tuple(tuple(point) for point in update['geometry_xy'])
                    if math.dist(geometry[0],self.map_graph.nodes[a])>1e-6 or math.dist(geometry[-1],self.map_graph.nodes[b])>1e-6:
                        continue # Registration change requires new verified support; do not snap/extend old evidence.
                    historical.append(GraphEdge(update['update_id'],a,b,geometry,'OBSERVED_GEOMETRY','historical_verified_graph'))
                historical=transform_graph(RoadGraph(historical),(p.x,p.y,2*math.atan2(q.z,q.w)))
                self.odom_graph=RoadGraph(list(self.odom_graph.edges.values())+
                    [edge for key,edge in historical.edges.items() if key not in self.odom_graph.edges])
                if self._candidate_safety_ready(time.monotonic()):
                    for edge in self.odom_graph.edges.values():
                        if edge.source!='current_supported_ground' or edge.edge_id in self.published_updates:continue
                        current=current_session_evidence(self.recorded_evidence,self.localization_session_id)
                        local_ids={e.local_submap_id for e in current}
                        if len(local_ids)!=1:continue # Never invent an odom-session identity from mixed submaps.
                        local_id=next(iter(local_ids))
                        identities=[e.evidence_id for e in current if e.local_submap_id==local_id]
                        if not identities:continue
                        msg=RoadGraphUpdate2D();msg.header.frame_id='odom';msg.header.stamp=self.grid_stamp
                        msg.update_id,msg.prior_version=edge.edge_id,store.prior_version
                        msg.start_node_id,msg.end_node_id=edge.start,edge.end
                        msg.geometry=[Point(x=float(x),y=float(y),z=0.) for x,y in edge.geometry]
                        msg.supported_width_m=2*max(abs(y) for _,y in self.footprint)
                        msg.local_submap_id,msg.pose_uncertainty_m=local_id,self.pose_uncertainty
                        msg.source='current_supported_ground';msg.evidence_ids=identities
                        self.graph_updates.publish(msg)
                        # Retry until the sole persistence writer acknowledges it.
                        if edge.edge_id in store.graph_updates:self.published_updates.add(edge.edge_id)
            self.policy.resolved.intersection_update(self.odom_graph.edges)
        except Exception as exc:
            self.get_logger().debug(f"registered prior withheld: {exc}")

    def _save_policy(self):
        if self.prior:
            save_snapshot(self.snapshot_path,self.prior,self.recorded_evidence,self.policy)

    def _publish_status(self, reason):
        stopped = reason in ("TASK_REACHED","BUDGET_EXHAUSTED","SYSTEM_FAULT") or not self._can_request_motion(time.monotonic())
        self.stop_permission.publish(Bool(data=stopped))
        if reason != self.last_reason:
            self.get_logger().info(reason)
            self.last_reason = reason
        msg = ResearchStatus()
        msg.header.frame_id, msg.header.stamp = "odom", self.get_clock().now().to_msg()
        msg.mode,msg.state,msg.reason,msg.map_version = self.execution_mode,reason,reason+";policy="+self.mode,self.version
        msg.motion_allowed, msg.actuator_enabled = self.allowed, False
        self.status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActiveObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
