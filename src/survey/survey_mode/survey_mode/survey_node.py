import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import random
from action_msgs.msg import GoalStatus

class SurveyNode(Node):
    def __init__(self):
        super().__init__('survey_node')
        
        # Declare parameters
        self.declare_parameter('max_radius', 50.0)
        self.declare_parameter('threshold_x', 200.0)
        self.declare_parameter('confidence_guess', 0.6)
        self.declare_parameter('confidence_confirm', 0.8)
        
        self.max_radius = self.get_parameter('max_radius').value
        self.threshold_x = self.get_parameter('threshold_x').value
        self.confidence_guess = self.get_parameter('confidence_guess').value
        self.confidence_confirm = self.get_parameter('confidence_confirm').value
        
        self.state = 'Autonomous_Exploration'
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Stubs for active map area and match score
        self.active_map_area = 0.0
        self.map_match_score = 0.0
        
        # Timer for the state machine
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        
        self.get_logger().info(f"Survey node initialized in state: {self.state}")
        
        # Internal state for navigation
        self.goal_active = False

    def get_stub_frontier_goal(self):
        # Stub: Generate a random goal within max_radius
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(0, self.max_radius * 0.9) # Ensure within radius
        
        goal.pose.position.x = radius * math.cos(angle)
        goal.pose.position.y = radius * math.sin(angle)
        goal.pose.orientation.w = 1.0
        return goal

    def get_stub_hypothesis_goal(self):
        # Stub: Distant feature-rich coordinate from known map within max_radius
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = min(self.max_radius * 0.8, 10.0)
        goal.pose.position.y = min(self.max_radius * 0.8, 10.0)
        goal.pose.orientation.w = 1.0
        return goal
        
    def query_map_matching_stub(self):
        # Stub for map matching query
        self.active_map_area += 10.0
        # Simulating match score
        self.map_match_score = random.uniform(0.0, 1.0)
        return self.map_match_score

    def validate_goal_radius(self, goal: PoseStamped):
        dist = math.hypot(goal.pose.position.x, goal.pose.position.y)
        return dist <= self.max_radius

    def send_nav_goal(self, goal: PoseStamped):
        if not self.validate_goal_radius(goal):
            self.get_logger().warn(f"Goal rejected: exceeds MAX_RADIUS ({self.max_radius})")
            return
            
        self.get_logger().info(f"Sending nav goal to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
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
        
        if self.state == 'Return_To_Home' and result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("Failed to return home. Path might be blocked. Halting.")
            self.state = 'Halted'

    def state_machine_loop(self):
        if self.state == 'Halted':
            return
            
        if self.state == 'Autonomous_Exploration':
            score = self.query_map_matching_stub()
            
            if score >= self.confidence_guess:
                self.get_logger().info(f"Match score {score:.2f} >= guess threshold. Transitioning to Hypothesis_Testing")
                self.state = 'Hypothesis_Testing'
                self.goal_active = False
            elif self.active_map_area >= self.threshold_x:
                self.get_logger().info(f"Map area {self.active_map_area} >= threshold. Transitioning to Pure_Mapping")
                self.state = 'Pure_Mapping'
                self.goal_active = False
            else:
                if not self.goal_active:
                    goal = self.get_stub_frontier_goal()
                    self.send_nav_goal(goal)
                    
        elif self.state == 'Hypothesis_Testing':
            score = self.query_map_matching_stub()
            
            if score >= self.confidence_confirm:
                self.get_logger().info("Localization confirmed! Transitioning to Return_To_Home")
                self.state = 'Return_To_Home'
                self.goal_active = False
            elif score < self.confidence_guess:
                self.get_logger().info("False positive match. Transitioning back to Autonomous_Exploration")
                self.state = 'Autonomous_Exploration'
                self.goal_active = False
            else:
                if not self.goal_active:
                    goal = self.get_stub_hypothesis_goal()
                    self.send_nav_goal(goal)
                    
        elif self.state == 'Pure_Mapping':
            if not self.goal_active:
                if random.uniform(0, 1) > 0.8:
                    self.get_logger().info("Fully mapped within radius. Transitioning to Return_To_Home")
                    self.state = 'Return_To_Home'
                else:
                    goal = self.get_stub_frontier_goal()
                    self.send_nav_goal(goal)
                    
        elif self.state == 'Return_To_Home':
            if not self.goal_active:
                goal = PoseStamped()
                goal.header.frame_id = 'map' # Assuming map coordinates for odometry origin
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
