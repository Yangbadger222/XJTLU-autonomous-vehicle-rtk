import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from interface.srv import GlobalRelocalize
import math
import random
import sys
import os
import numpy as np
from datetime import datetime
from action_msgs.msg import GoalStatus
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class SurveyNode(Node):
    def __init__(self):
        super().__init__('survey_node')
        
        # Declare parameters
        self.declare_parameter('max_radius', 50.0)
        self.declare_parameter('threshold_x', 200.0)
        self.declare_parameter('confidence_guess', 0.6)
        self.declare_parameter('confidence_confirm', 0.8)
        self.declare_parameter('min_confirm_distance', 5.0)
        self.declare_parameter('map_database_dir', '~/XJTLU-autonomous-vehicle/runtime-data/maps/indoor')
        
        # New parameters
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('robot_base_frame_id', 'base_link')
        self.declare_parameter('nav_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('localizer_service', '/localizer/global_relocalize')
        self.declare_parameter('initial_guess_topic', '/survey/initial_guess')
        self.declare_parameter('confirmed_match_topic', '/survey/confirmed_match')
        self.declare_parameter('frontier_min_dist', 1.5)
        self.declare_parameter('frontier_fallback_min_dist', 0.5)
        self.declare_parameter('hypothesis_min_dist', 3.0)
        self.declare_parameter('hypothesis_fallback_min_dist', 0.5)
        self.declare_parameter('max_candidates', 5)
        self.declare_parameter('timer_period', 1.0)
        
        self.max_radius = self.get_parameter('max_radius').value
        self.threshold_x = self.get_parameter('threshold_x').value
        self.confidence_guess = self.get_parameter('confidence_guess').value
        self.confidence_confirm = self.get_parameter('confidence_confirm').value
        self.min_confirm_distance = self.get_parameter('min_confirm_distance').value
        self.map_database_dir = os.path.expanduser(self.get_parameter('map_database_dir').value)
        
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.robot_base_frame_id = self.get_parameter('robot_base_frame_id').value
        self.nav_to_pose_action = self.get_parameter('nav_to_pose_action').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.localizer_service = self.get_parameter('localizer_service').value
        self.initial_guess_topic = self.get_parameter('initial_guess_topic').value
        self.confirmed_match_topic = self.get_parameter('confirmed_match_topic').value
        self.frontier_min_dist = self.get_parameter('frontier_min_dist').value
        self.frontier_fallback_min_dist = self.get_parameter('frontier_fallback_min_dist').value
        self.hypothesis_min_dist = self.get_parameter('hypothesis_min_dist').value
        self.hypothesis_fallback_min_dist = self.get_parameter('hypothesis_fallback_min_dist').value
        self.max_candidates = self.get_parameter('max_candidates').value
        self.timer_period = self.get_parameter('timer_period').value
        
        self.state = 'Autonomous_Exploration'
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.nav_to_pose_action)
        self.localizer_client = self.create_client(GlobalRelocalize, self.localizer_service)
        
        # Map subscriber
        from rclpy.qos import QoSProfile, DurabilityPolicy
        costmap_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.map_sub = self.create_subscription(OccupancyGrid, self.global_costmap_topic, self.map_callback, costmap_qos)
        self.occupancy_grid = None
        
        # Foxglove Publishers
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.initial_guess_pub = self.create_publisher(Marker, self.initial_guess_topic, marker_qos)
        self.confirmed_match_pub = self.create_publisher(Marker, self.confirmed_match_topic, marker_qos)
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Internal state for tracking movement and matching
        self.active_map_area = 0.0
        self.hypothesis_start_x = None
        self.hypothesis_start_y = None
        self.distance_traveled = 0.0
        self.last_x = None
        self.last_y = None
        self.match_found = False
        self.matched_map_name = "None"
        
        # Map database discovery
        self.maps_tested = []
        self.untested_maps = self.discover_maps()
        self.currently_testing_map = None
        self.waiting_for_service = False
        
        # Timer for the state machine
        self.timer = self.create_timer(self.timer_period, self.state_machine_loop)
        
        self.get_logger().info(f"Survey node initialized in state: {self.state}")
        self.goal_active = False

    def discover_maps(self):
        maps = []
        if os.path.exists(self.map_database_dir):
            for map_dir in os.listdir(self.map_database_dir):
                descriptor_path = os.path.join(self.map_database_dir, map_dir, 'localization', 'descriptor_index', 'scan_context.yaml')
                if os.path.isfile(descriptor_path):
                    maps.append(map_dir)
        self.get_logger().info(f"Discovered maps: {maps}")
        return maps

    def map_callback(self, msg: OccupancyGrid):
        self.occupancy_grid = msg
        data = np.array(msg.data)
        free_cells = np.sum(data == 0)
        self.active_map_area = free_cells * (msg.info.resolution ** 2)

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.global_frame_id, self.robot_base_frame_id, rclpy.time.Time())
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception as e:
            self.get_logger().debug(f"TF Lookup failed: {e}")
            return None, None

    def publish_marker(self, publisher, type, r, g, b, x, y):
        marker = Marker()
        marker.header.frame_id = self.global_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'survey'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 0.8
        publisher.publish(marker)

    def extract_frontier_points(self):
        if self.occupancy_grid is None:
            return []
        
        info = self.occupancy_grid.info
        w, h = info.width, info.height
        data = np.array(self.occupancy_grid.data).reshape((h, w))
        
        free_cells = (data >= 0) & (data < 200)
        unknown_cells = (data == -1)
        
        up = np.roll(unknown_cells, 1, axis=0); up[0,:] = False
        down = np.roll(unknown_cells, -1, axis=0); down[-1,:] = False
        left = np.roll(unknown_cells, 1, axis=1); left[:,0] = False
        right = np.roll(unknown_cells, -1, axis=1); right[:,-1] = False
        
        frontier_mask = free_cells & (up | down | left | right)
        y_idx, x_idx = np.where(frontier_mask)
        
        current_x, current_y = self.get_current_pose()
        if current_x is None:
            return []

        points = []
        for y, x in zip(y_idx, x_idx):
            wx = info.origin.position.x + (x + 0.5) * info.resolution
            wy = info.origin.position.y + (y + 0.5) * info.resolution
            
            # Check distance from the robot, NOT from the map origin
            dist_to_robot = math.hypot(wx - current_x, wy - current_y)
            
            if dist_to_robot <= self.max_radius and dist_to_robot >= self.frontier_min_dist:
                points.append((wx, wy))
        
        # Fallback: Find the furthest point we can if everything is close
        if not points:
            for y, x in zip(y_idx, x_idx):
                wx = info.origin.position.x + (x + 0.5) * info.resolution
                wy = info.origin.position.y + (y + 0.5) * info.resolution
                dist_to_robot = math.hypot(wx - current_x, wy - current_y)
                
                # Absolute minimum distance safety net
                if dist_to_robot >= self.frontier_fallback_min_dist and dist_to_robot <= self.max_radius:
                    points.append((wx, wy))

        return points

    def get_real_frontier_goal(self):
        points = self.extract_frontier_points()
        if not points:
            return None
            
        target = random.choice(points)
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        
        current_x, current_y = self.get_current_pose()
        if current_x is not None and current_y is not None:
            dx = goal.pose.position.x - current_x
            dy = goal.pose.position.y - current_y
            if dx == 0.0 and dy == 0.0:
                goal.pose.orientation.w = 1.0
                goal.pose.orientation.z = 0.0
            else:
                yaw = math.atan2(dy, dx)
                goal.pose.orientation.w = math.cos(yaw / 2.0)
                goal.pose.orientation.z = math.sin(yaw / 2.0)
        else:
            goal.pose.orientation.w = 1.0
            goal.pose.orientation.z = 0.0
            
        return goal

    def get_real_hypothesis_goal(self):
        if self.occupancy_grid is None:
            return None
            
        info = self.occupancy_grid.info
        w, h = info.width, info.height
        data = np.array(self.occupancy_grid.data).reshape((h, w))
        
        y_idx, x_idx = np.where((data >= 0) & (data < 200))
        
        current_x, current_y = self.get_current_pose()
        if current_x is None:
            return None
            
        points = []
        for y, x in zip(y_idx, x_idx):
            wx = info.origin.position.x + (x + 0.5) * info.resolution
            wy = info.origin.position.y + (y + 0.5) * info.resolution
            
            # check distance from the robot, not from the map origin
            dist_to_robot = math.hypot(wx - current_x, wy - current_y)
            
            if dist_to_robot <= self.max_radius:
                if dist_to_robot >= self.hypothesis_min_dist:
                    points.append((wx, wy))
        
        # Fallback if no points are far enough
        if not points:
            for y, x in zip(y_idx, x_idx):
                wx = info.origin.position.x + (x + 0.5) * info.resolution
                wy = info.origin.position.y + (y + 0.5) * info.resolution
                
                dist_to_robot = math.hypot(wx - current_x, wy - current_y)
                
                if dist_to_robot <= self.max_radius:
                    # Relaxed distance constraint
                    if dist_to_robot >= self.hypothesis_fallback_min_dist:
                        points.append((wx, wy))
        
        if not points:
            return None
            
        target = random.choice(points)
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        
        dx = goal.pose.position.x - current_x
        dy = goal.pose.position.y - current_y
        if dx == 0.0 and dy == 0.0:
            goal.pose.orientation.w = 1.0
            goal.pose.orientation.z = 0.0
        else:
            yaw = math.atan2(dy, dx)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            
        return goal

    def send_nav_goal(self, goal: PoseStamped):
        self.get_logger().info(f"Sending nav goal to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().error("Nav2 server not available")
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        self.goal_active = True

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by server")
            self.goal_active = False
            return
        
        self.get_logger().info("Goal accepted by server")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        result = future.result()
        self.goal_active = False
        self.get_logger().info(f"Nav goal finished with status: {result.status}")
        
        if self.state == 'Return_To_Home' and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Successfully returned home. Shutting down system.")
            self.generate_summary()
            sys.exit(0)
        elif self.state == 'Return_To_Home':
            self.get_logger().error("Failed to return home. Path might be blocked. Halting.")
            self.state = 'Halted'

    def generate_summary(self):
        log_dir = os.environ.get('FYP_LOG_SESSION_DIR', '/tmp')
        summary_file = os.path.join(log_dir, 'survey_summary.txt')
        with open(summary_file, 'w') as f:
            f.write("=== Survey Mode Summary ===\n")
            f.write(f"Date: {datetime.now().isoformat()}\n")
            f.write(f"Maps Database Dir: {self.map_database_dir}\n")
            f.write(f"Maps Tested: {', '.join(self.maps_tested)}\n")
            f.write(f"Distance Traveled: {self.distance_traveled:.2f} meters\n")
            f.write(f"Match Found: {self.match_found}\n")
            f.write(f"Matched Map Name: {self.matched_map_name}\n")
            f.write("===========================\n")
        self.get_logger().info(f"Summary written to {summary_file}")

    def update_distance(self, current_x, current_y):
        if self.last_x is not None and self.last_y is not None:
            self.distance_traveled += math.hypot(current_x - self.last_x, current_y - self.last_y)
        self.last_x = current_x
        self.last_y = current_y

    def trigger_map_evaluation(self):
        if not self.untested_maps or self.waiting_for_service:
            return

        # Wait for a valid pose to ensure pointclouds are flowing and the localizer has data
        current_x, current_y = self.get_current_pose()
        if current_x is None:
            self.get_logger().info("Waiting for valid pose before evaluating maps...", throttle_duration_sec=2.0)
            return

        self.currently_testing_map = self.untested_maps.pop(0)
        if self.currently_testing_map not in self.maps_tested:
            self.maps_tested.append(self.currently_testing_map)
            
        descriptor_path = os.path.join(self.map_database_dir, self.currently_testing_map, 'localization', 'descriptor_index', 'scan_context.yaml')
        
        if not self.localizer_client.service_is_ready():
            self.get_logger().warn("GlobalRelocalize service not available")
            self.untested_maps.insert(0, self.currently_testing_map) # retry later
            return

        req = GlobalRelocalize.Request()
        req.descriptor_index = descriptor_path
        req.region = ''
        req.max_candidates = self.max_candidates
        
        self.get_logger().info(f"Evaluating map: {self.currently_testing_map}")
        future = self.localizer_client.call_async(req)
        future.add_done_callback(self.service_response_callback)
        self.waiting_for_service = True

    def service_response_callback(self, future):
        self.waiting_for_service = False
        try:
            response = future.result()
            score = response.best_score
            self.get_logger().info(f"Service returned score {score:.2f} for map {self.currently_testing_map}")
            self.handle_map_score(score)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def handle_map_score(self, score):
        confidence = 0.0 if score == float('inf') else max(0.0, 1.0 - score)
        self.get_logger().info(f"Converted distance score {score:.2f} to confidence {confidence:.2f}")

        if self.state == 'Autonomous_Exploration':
            if confidence >= self.confidence_guess:
                self.get_logger().info(f"Match confidence {confidence:.2f} >= guess threshold. Transitioning to Hypothesis_Testing")
                self.state = 'Hypothesis_Testing'
                current_x, current_y = self.get_current_pose()
                self.hypothesis_start_x = current_x
                self.hypothesis_start_y = current_y
                self.matched_map_name = self.currently_testing_map
                self.goal_active = False
                if current_x is not None:
                    self.publish_marker(self.initial_guess_pub, Marker.SPHERE, 1.0, 1.0, 0.0, current_x, current_y)
        elif self.state == 'Hypothesis_Testing':
            if confidence >= self.confidence_confirm:
                current_x, current_y = self.get_current_pose()
                distance_moved = 0.0
                if self.hypothesis_start_x is not None and current_x is not None:
                    distance_moved = math.hypot(current_x - self.hypothesis_start_x, current_y - self.hypothesis_start_y)
                
                if distance_moved >= self.min_confirm_distance:
                    self.get_logger().info(f"Moved {distance_moved:.2f}m. Localization confirmed! Transitioning to Return_To_Home")
                    self.state = 'Return_To_Home'
                    self.goal_active = False
                    self.match_found = True
                    if current_x is not None:
                        self.publish_marker(self.confirmed_match_pub, Marker.SPHERE, 0.0, 1.0, 0.0, current_x, current_y)
                else:
                    self.get_logger().info(f"Match score high but moved only {distance_moved:.2f}m (< {self.min_confirm_distance}m). Waiting to move further.")
                    if not self.goal_active:
                        goal = self.get_real_hypothesis_goal()
                        if goal:
                            self.send_nav_goal(goal)
            elif confidence < self.confidence_guess:
                self.get_logger().info("False positive match. Transitioning back to Autonomous_Exploration")
                self.state = 'Autonomous_Exploration'
                self.matched_map_name = "None"
                self.goal_active = False
            else:
                if not self.goal_active:
                    goal = self.get_real_hypothesis_goal()
                    if goal:
                        self.send_nav_goal(goal)

    def state_machine_loop(self):
        current_x, current_y = self.get_current_pose()
        if current_x is not None and current_y is not None:
            self.update_distance(current_x, current_y)

        if self.state == 'Halted':
            return
            
        if self.state == 'Autonomous_Exploration':
            if not self.maps_tested and not self.untested_maps:
                self.get_logger().info("No maps found in database. Transitioning directly to Pure_Mapping")
                self.state = 'Pure_Mapping'
                self.goal_active = False
                return

            if not self.waiting_for_service and self.untested_maps:
                self.trigger_map_evaluation()
            
            if self.active_map_area >= self.threshold_x:
                self.get_logger().info(f"Map area {self.active_map_area} >= threshold. Transitioning to Pure_Mapping")
                self.state = 'Pure_Mapping'
                self.goal_active = False
            elif not self.goal_active:
                if self.occupancy_grid is None:
                    self.get_logger().info("Waiting for /global_costmap/costmap topic to be published...")
                    return
                goal = self.get_real_frontier_goal()
                if goal:
                    self.send_nav_goal(goal)
                    
        elif self.state == 'Hypothesis_Testing':
            if not self.waiting_for_service:
                self.untested_maps.insert(0, self.matched_map_name) # re-test the same map
                self.trigger_map_evaluation()
                    
        elif self.state == 'Pure_Mapping':
            if not self.goal_active:
                if self.occupancy_grid is None:
                    self.get_logger().info("Waiting for /map topic to be published...")
                    return
                goal = self.get_real_frontier_goal()
                if goal:
                    self.send_nav_goal(goal)
                else:
                    self.get_logger().info("Fully mapped within radius (no frontiers). Transitioning to Return_To_Home")
                    self.state = 'Return_To_Home'
                    
        elif self.state == 'Return_To_Home':
            if not self.goal_active:
                goal = PoseStamped()
                goal.header.frame_id = self.global_frame_id
                goal.pose.position.x = 0.0
                goal.pose.position.y = 0.0
                goal.pose.orientation.w = 1.0
                self.send_nav_goal(goal)

def main(args=None):
    rclpy.init(args=args)
    node = SurveyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
