from enum import Enum
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from interface_package.srv import Module, LocationInfo, NodeNum
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import math
from time import sleep
from collections import namedtuple

MAX_ATTEMPTS = 3
NAVIGATION_TIME_FACTOR = 13
MIN_NAV_TIME = 5.0

ROBOT_NUMBER = "1"
# ROBOT_NUMBER = "2"
# ROBOT_NUMBER = "3"

Position = namedtuple('Position', ['x', 'y'])

class RobotStatus(Enum):
    HOME = 0  # waiting at Home
    TO_STORE = 1  # moving to Store
    AT_STORE = 2  # arrived at Store
    TO_KIOSK = 3  # moving to Kiosk
    AT_KIOSK = 4  # arrived at Kiosk
    RETURNING = 5  # returning to Home
    AT_HOME = 6  # Arrived at Home

class OrderStatus(Enum):
    DELIVERY_YET = "DY"
    DELIVERY_READY = "DR"  # Not used yet
    DELIVERY_START = "DS"
    DELIVERY_FINISH = "DF"


class DrobotMotor(Node):
    def __init__(self):
        super().__init__("drobot_motor")


        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.initialize_parameters()
        self.initialize_subs_and_pubs()
        self.initialize_clients()
        self.initialize_timer()

        self.get_logger().info(f"R-{ROBOT_NUMBER} motor start")
        

    def initialize_parameters(self):
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.status_change = {
            RobotStatus.HOME: RobotStatus.TO_STORE,
            RobotStatus.TO_STORE: RobotStatus.AT_STORE,
            RobotStatus.AT_STORE: RobotStatus.TO_KIOSK,
            RobotStatus.TO_KIOSK: RobotStatus.AT_KIOSK,
            RobotStatus.AT_KIOSK: RobotStatus.RETURNING,
            RobotStatus.RETURNING: RobotStatus.AT_HOME,
            # RobotStatus.AT_HOME: RobotStatus.HOME
        }
        self.declare_parameter("points", "")

        self.points_str = self.get_parameter("points").get_parameter_value().string_value
        self.positions, self.labels = self.parse_points(self.points_str)

        # for row in range(5):
        #     for col in range(9):
        #         if self.positions[row][col] is not None:
        #             self.get_logger().info(f"Point ({row},{col}): Position={self.positions[row][col]}, Label={self.labels[row][col]}")

        self.waypoint_points = self.get_indices_for_label("Waypoint")

        self.invalid_points = self.get_indices_for_label("Invalid point")
        self.robot_points = self.get_indices_for_label(f"Robot{int(ROBOT_NUMBER)}")

        self.initialize_variables()
        # self.log_processed_parameters()

    def initialize_variables(self):
        self.is_active = False
        self.store_id = ""
        self.kiosk_id = ""
        self.status = RobotStatus.HOME
        self.is_moving = 0

        self.store_points = []
        self.kiosk_points = []

        self.current_point = self.robot_points[0] #tuple
        self.next_point = self.robot_points[0] #tuple
        self.current_position = self.get_position(self.robot_points[0]) #namedtuple
        self.next_position = self.get_position(self.robot_points[0]) #namedtuple
        # print(type(self.current_point))
        # print(type(self.current_position))
        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

        self.x_shift = 0.0
        self.y_shift = 0.0


    def initialize_subs_and_pubs(self):
        self.cmd_vel_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)

    def initialize_clients(self):
        self.reset_sub = self.create_service(Trigger, "/reset", self.reset_callback)
        
        self.robot_arrival_client = self.create_client(NodeNum, "robotArrival")
        self.module_client = self.create_client(Module, "module")
        
    def initialize_timer(self):
        self.moving_timer = self.create_timer(1.0, self.moving_timer_callback)

    def get_position(self, point):
        row, col = point
        return self.positions[row][col]

    def get_label(self, point):
        row, col = point
        return self.labels[row][col]

    def parse_points(self, points_str):
        points = points_str.strip().split(";")
        positions = [[None for _ in range(9)] for _ in range(5)]
        labels = [[None for _ in range(9)] for _ in range(5)]

        for point in points:
            if point:
                key, value = point.split(": ")
                row, col = map(int, key.split(","))
                label, x, y = value.split(", ")
                positions[row][col] = Position(float(x), float(y))
                labels[row][col] = label

        return positions, labels
    
    def get_indices_for_label(self, label_prefix):
        indices = []
        for row in range(5):
            for col in range(9):
                if self.labels[row][col].startswith(label_prefix):
                    indices.append((row, col))
        return indices
    
    def update_positions(self, current_point, next_point):
        self.current_point = current_point
        self.next_point = next_point
        self.current_position = self.get_position(current_point)
        self.next_position = self.get_position(next_point)

    def update_status(self):
        self.get_logger().info(f"Updating status from {self.status.name}")
        
        self.status = self.status_change[self.status]
        self.get_logger().info(f"Status updated to {self.status.name}")

        if self.status == RobotStatus.RETURNING:
            self.returning()
            self.get_logger().info("Returning to Home")
        
        if self.status == RobotStatus.AT_HOME:
            self.reset()
            sleep(1)
            self.status = RobotStatus.HOME
            self.get_logger().info(f"Status updated to {self.status.name}")

    def moving_timer_callback(self):
        if self.is_moving == 1:
            self.get_logger().info(f"move to {self.next_position}")
            self.get_logger().info(f"current point : {self.current_point}, next point : {self.next_point}")
            self.is_moving = 0
            self.send_goal(self.next_position)

    def send_goal(self, goal):
        x = goal.x
        y = goal.y
        goal_pose = self.get_goal_pose(x, y)
        
        distance = self.calc_diff(self.next_position, self.current_position)
        nav_time = max(distance * NAVIGATION_TIME_FACTOR, MIN_NAV_TIME)
    
        self.nav.goToPose(goal_pose)
        i = 0
        while not self.nav.isTaskComplete():
            # i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                # print("Distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds= nav_time):
                    self.nav.cancelTask()
        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().warn("Goal failed!")

        if self.check_succeed(self.current_position):
            self.request_robot_arrival(self.next_point)

    def determine_direction(self, current, next):
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        self.get_logger().info(f"Determine direction: current={current}, next={next}, dx={dx}, dy={dy}, status={self.status}")

        if dx > 0 and dy == 0:
            yaw = 0.0
            self.x_shift == 0.0
            self.y_shift == 0.0
        elif dx < 0 and dy == 0:
            yaw = math.pi
            self.x_shift == -0.1
            self.y_shift == 0.0
        elif dx == 0 and dy > 0:
            yaw = math.pi / 2
            self.x_shift == 0.0
            self.y_shift == -0.05
        elif dx == 0 and dy < 0:
            yaw = -math.pi / 2
            self.x_shift == 0.0
            self.y_shift == 0.05
        else:
            yaw = math.atan2(dy, dx)

        if next in self.store_points:
            yaw = math.pi
        elif next in self.kiosk_points:
            if self.store_id[2] == "1":
                yaw = -math.pi / 2
            elif self.store_id == "2":
                yaw = math.pi / 2
        elif next in self.robot_points:
            yaw = 0.0

        return yaw

    def get_goal_pose(self, x, y):
        yaw = self.determine_direction(self.current_point, self.next_point)
        self.get_logger().info(f"goal yaw : {yaw}")

        tmp = [0, 0, yaw]
        orientation_val = quaternion_from_euler(tmp[0], tmp[1], tmp[2])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x + self.x_shift
        goal_pose.pose.position.y = y + self.y_shift
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = orientation_val[2]
        goal_pose.pose.orientation.w = orientation_val[3]

        return goal_pose

    def calc_diff(self, goal_position, current_position):
        self.get_logger().info(f"current_position value: ({self.current_position.x}, {self.current_position.y})")
        self.get_logger().info(f"goal position: ({goal_position.x}, {goal_position.y})")


        diff_x = goal_position.x - current_position.x
        diff_y = goal_position.y - current_position.y
        diff = math.sqrt(diff_x ** 2 + diff_y ** 2)
        self.get_logger().info(f"diff : {diff}")
        return diff
    
    def check_succeed(self, current_position):
        diff_dist = self.calc_diff(self.next_position, current_position)
        if diff_dist <= 0.5:
            self.get_logger().info("Moving succeeded!")
            self.verify_checkpoint()
            self.attempt_count = 0
            return True
        else:
            return self.resend_goal()
        
    def resend_goal(self):
        if self.attempt_count < 3:
            self.attempt_count += 1
            self.send_goal(self.next_position)
            self.get_logger().warn(f"Moving failed! Attempt {self.attempt_count}/3")
            return False
        else:
            self.get_logger().error("Moving failed after 3 attempts.")
            self.attempt_count = 0
            self.verify_checkpoint()
            return True
    
    def verify_checkpoint(self):
        if self.next_point in self.store_points and self.status == RobotStatus.TO_STORE:
            self.update_status()
            self.request_module("ST")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Store. So, status updated to {self.status.value}")
        elif self.next_point in self.kiosk_points and self.status == RobotStatus.TO_KIOSK:
            self.update_status()
            self.request_module("KS")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Kiosk. So, status updated to {self.status.value}")
        elif self.next_point in self.robot_points and self.status == RobotStatus.RETURNING:
            self.update_status()
            self.request_module("HM")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Home. So, status updated to {self.status.value}")
        else:
            self.get_logger().info("On waypoints")


    def request_module(self, location):
        self.get_logger().info(f"Request status : {location}")
        module_request = Module.Request()
        module_request.data = location
        future = self.module_client.call_async(module_request)
        future.add_done_callback(self.response_module)

    def response_module(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"module response : {response.success}")
        except Exception as e:
            self.get_logger().error(f"module call failed {e}")

    def request_robot_arrival(self, current_point):
        robot_arrival_request = NodeNum.Request()
        self.get_logger().info(f"Arrived at {current_point}")
        robot_arrival_request.current_x = current_point[0]
        robot_arrival_request.current_y = current_point[1]
        robot_arrival_request.next_x = 0
        robot_arrival_request.next_y = 0
        future = self.robot_arrival_client.call_async(robot_arrival_request)
        future.add_done_callback(self.response_robot_arrival)

    def response_robot_arrival(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"response {response.success} from TM")
        except Exception as e:
            self.get_logger().error(f"arrival call failed : {e}")

    def reset_callback(self, request, response):
        self.reset()
        response.success = True
        response.message = "reset"

        return response
    
    def reset(self):
        self.get_logger().info("Reset")
        self.initialize_variables()

    def returning(self):
        self.get_logger().info("Robot returning")
        self.store_points = []
        self.kiosk_points = []
        self.store_id = ""
        self.kiosk_id = ""
        self.is_active = False

class DrobotTask(Node):
    def __init__(self, motor_node):
        super().__init__("drobot_task")
        self.motor_node = motor_node
        
        self.short_goal_server = self.create_service(NodeNum, "shortGoal", self.short_goal_callback)

        self.motor_order_service = self.create_service(LocationInfo, "location_info", self.motor_order_callback)

    def motor_order_callback(self, request, response):
        store_id = request.store_id
        kiosk_id = request.kiosk_id

        self.get_logger().info(f"Get motor order request! Store : {store_id}, Kiosk : {kiosk_id}")

        if not self.motor_node.is_active:
            self.motor_node.store_id = store_id
            self.motor_node.kiosk_id = kiosk_id
            self.motor_node.store_points = self.motor_node.get_indices_for_label(f"Store{store_id[2]}")
            self.motor_node.kiosk_points = self.motor_node.get_indices_for_label(f"Kiosk{kiosk_id[2]}")
            self.motor_node.is_active = True

            if self.motor_node.status == RobotStatus.RETURNING:
                self.motor_node.status = RobotStatus.TO_STORE

            # self.get_logger().info(f"Kiosk points: {self.motor_node.kiosk_points}")
            # self.get_logger().info(f"Store points: {self.motor_node.store_points}")
            response.success = True
        else:
            response.success = False

        return response

    def short_goal_callback(self, request, response):
        self.get_logger().info(f"Short goal is ({request.next_x}, {request.next_y})")
        next_point = (request.next_x, request.next_y)

        if 0 <= next_point[0] <= 4 and 0 <= next_point[1] <= 8:
            self.motor_node.next_position = self.motor_node.positions[next_point[0]][next_point[1]]
            self.get_logger().info(f"next_position : {self.motor_node.next_position.x}, {self.motor_node.next_position.y}, status : {self.motor_node.status.value}, active :{self.motor_node.is_active}")
        else:
            self.get_logger().warn(f"Invalid point!!")
            response.success = False
            return response

        self.get_logger().info(f"next point : {next_point}")

        valid_status_points = {
            RobotStatus.HOME: self.motor_node.waypoint_points,
            RobotStatus.AT_HOME: self.motor_node.waypoint_points,
            RobotStatus.AT_STORE: self.motor_node.waypoint_points + self.motor_node.kiosk_points,
            RobotStatus.AT_KIOSK: self.motor_node.waypoint_points + self.motor_node.robot_points,
            RobotStatus.TO_STORE: self.motor_node.waypoint_points + self.motor_node.store_points,
            RobotStatus.TO_KIOSK: self.motor_node.waypoint_points + self.motor_node.kiosk_points,
            RobotStatus.RETURNING: self.motor_node.waypoint_points + self.motor_node.robot_points,
        }

        if next_point in valid_status_points.get(self.motor_node.status, []):
            if self.motor_node.status in {RobotStatus.HOME, RobotStatus.AT_HOME, RobotStatus.AT_STORE, RobotStatus.AT_KIOSK}:
                self.motor_node.update_status()
            self.motor_node.update_positions(self.motor_node.next_point, next_point)
            self.motor_node.is_moving = 1
            response.success = True
        else:
            self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
            response.success = False

        return response
    
class DrobotStatus(Node):
    def __init__(self, motor_node):
        super().__init__("drobot_status")
        self.motor_node = motor_node
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.status_pub = self.create_publisher(Int16, "/status", self.qos_profile)

    def status_publish(self, status):
        msg = Int16()
        msg.data = status.value
        self.status_pub.publish(msg)
        # self.get_logger().info(f"Published status: {msg.data}")

    def timer_callback(self):
        self.status_publish(self.motor_node.status)

class AmclSub(Node):
    def __init__(self, motor_node):
        super().__init__("amcl_sub_node")
        self.motor_node = motor_node
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.listener_callback, 10)

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler

        self.motor_node.current_yaw = yaw
        x = position.x
        y = position.y
        self.motor_node.current_position = Position(x, y)

        # self.motor_node.get_logger().info(f"Updated from amcl_pose: current_position value: {self.motor_node.current_position}")

def main(args=None):
    rp.init(args=args)
    executor = MultiThreadedExecutor()

    drobot_motor = DrobotMotor()
    drobot_status = DrobotStatus(motor_node= drobot_motor)
    amcl_sub = AmclSub(motor_node= drobot_motor)
    drobot_task = DrobotTask(motor_node= drobot_motor)

    executor.add_node(drobot_motor)
    executor.add_node(drobot_status)
    executor.add_node(drobot_task)
    executor.add_node(amcl_sub)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rp.ok():
            executor.shutdown()
            drobot_motor.destroy_node()
            drobot_status.destroy_node()
            amcl_sub.destroy_node()
            drobot_task.destroy_node()
            rp.shutdown()


if __name__ == "__main__":
    main()