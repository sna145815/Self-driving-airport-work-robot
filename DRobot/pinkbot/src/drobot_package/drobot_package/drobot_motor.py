import sys
import os
import json
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

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

# ROBOT_NUMBER = "1"
ROBOT_NUMBER = "2"
# ROBOT_NUMBER = "3"

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

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.is_active = False
        self.store_id = ""
        self.kiosk_id = ""
        self.status = RobotStatus.HOME
        self.is_moving = 0

        
        self.status_change = {
            RobotStatus.HOME: RobotStatus.TO_STORE,
            RobotStatus.TO_STORE: RobotStatus.AT_STORE,
            RobotStatus.AT_STORE: RobotStatus.TO_KIOSK,
            RobotStatus.TO_KIOSK: RobotStatus.AT_KIOSK,
            RobotStatus.AT_KIOSK: RobotStatus.RETURNING,
            RobotStatus.RETURNING: RobotStatus.AT_HOME,
            # RobotStatus.AT_HOME: RobotStatus.HOME
        }
        
        self.module_client = self.create_client(Module, "module")

        self.cmd_vel_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.reset_sub = self.create_service(Trigger, "/reset", self.reset_callback)
        
        self.moivig_timer = self.create_timer(1.0, self.moving_timer_callback)
        self.robot_arrival_client = self.create_client(NodeNum, "robotArrival")

        self.get_logger().info(f"R-{ROBOT_NUMBER} motor start")
        
        self.declare_parameter("points", "")

        self.points_str = self.get_parameter("points").get_parameter_value().string_value
        self.positions, self.labels = self.parse_points(self.points_str)


        self.waypoint_points = self.get_indices_for_label("Waypoint")
        # self.kiosk_points = self.get_indices_for_label("Kiosk")
        # self.store_points = self.get_indices_for_label("Store")
        self.store_points = []
        self.kiosk_points = []
        self.invalid_points = self.get_indices_for_label("Invalid point")
        self.robot_points = self.get_indices_for_label(f"Robot{int(ROBOT_NUMBER)}")

        # self.log_processed_parameters()

        self.current_point = list(self.robot_points[0])
        self.next_point = list(self.robot_points[0])
        self.current_position = [self.positions[self.current_point[0]][self.current_point[1]][0], self.positions[self.current_point[0]][self.current_point[1]][1]]
        self.next_position = [self.positions[self.current_point[0]][self.current_point[1]][0], self.positions[self.current_point[0]][self.current_point[1]][1]]
        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

    def parse_points(self, points_str):
        points = points_str.strip().split(";")
        positions = [[None for _ in range(9)] for _ in range(5)]
        labels = [[None for _ in range(9)] for _ in range(5)]

        for point in points:
            if point:
                key, value = point.split(": ")
                row, col = map(int, key.split(","))
                label, x, y = value.split(", ")
                positions[row][col] = [float(x), float(y)]
                labels[row][col] = label

        return positions, labels
    
    def log_processed_parameters(self):
        for i, (pos_row, label_row) in enumerate(zip(self.positions, self.labels)):
            self.get_logger().info(f"Row {i} Positions: {pos_row}")
            self.get_logger().info(f"Row {i} Labels: {label_row}")

        self.get_logger().info(f"Waypoint points: {self.waypoint_points}")
        self.get_logger().info(f"Invalid points: {self.invalid_points}")
        self.get_logger().info(f"Robot points: {self.robot_points}")

    def get_position(self, row, col):
        return self.positions[row][col]

    def get_label(self, row, col):
        return self.labels[row][col]

    def get_indices_for_label(self, label_prefix):
        indices = []
        for row in range(5):
            for col in range(9):
                if self.labels[row][col].startswith(label_prefix):
                    indices.append((row, col))
        return indices

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
        x = goal[0]
        y = goal[1]
        goal_pose = self.get_goal_pose(x, y)
        
        distance = self.calc_diff(self.next_position, self.current_position)
        if distance * 13 > 5.0:
            nav_time = distance * 13
        else:
            nav_time = 5.0
    
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
            print("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            print("Goal was canceled!")
        elif result == TaskResult.FAILED:
        
            print("Goal failed!")

        if self.check_succeed(self.current_position):
            self.request_robot_arrival(self.next_point)

    def determine_direction(self, current, next):
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        self.get_logger().info(f"Determine direction: current={current}, next={next}, dx={dx}, dy={dy}, status={self.status}")

        if dx > 0 and dy == 0:
            yaw = 0.0 #"NORTH"
        elif dx < 0 and dy == 0:
            yaw = math.pi  #"SOUTH"
        elif dx == 0 and dy > 0:
            yaw = math.pi / 2 #"WEST"
        elif dx == 0 and dy < 0:
            yaw = -math.pi / 2 #"EAST"
        else:
            yaw = math.atan2(dy, dx)
        
        if tuple(next) in self.store_points:
            yaw = math.pi  #"SOUTH"
        elif tuple(next) in self.kiosk_points:
            if self.store_id[2] == "1":
                yaw = -math.pi / 2 #"EAST"
            elif self.store_id == "2":
                yaw = math.pi / 2 #"WEST"
        elif tuple(next) in self.robot_points:
            yaw = 0.0 #"NORTH"

        return yaw

    def get_goal_pose(self, x, y):
        yaw = self.determine_direction(self.current_point, self.next_point)
        self.get_logger().info(f"goal yaw : {yaw}")

        tmp = [0, 0, yaw]
        orientation_val = quaternion_from_euler(tmp[0], tmp[1], tmp[2])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = orientation_val[2]
        goal_pose.pose.orientation.w = orientation_val[3]

        return goal_pose

    def check_succeed(self, current_position):
        diff_dist = self.calc_diff(self.next_position, current_position)
        if diff_dist <= 0.3:
            self.get_logger().info("Moving succeeded!")
            self.verify_checkpoint()
            self.attempt_count = 0
            return True
        else:
            if self.attempt_count < self.max_attempts:
                self.attempt_count += 1
                self.send_goal(self.next_position)
                self.get_logger().warn(f"Moving failed! Attempt {self.attempt_count}/{self.max_attempts}")
                return False
            else:
                self.get_logger().error("Moving failed after 3 attempts.")
                self.attempt_count = 0
                diff_dist = self.calc_diff(self.next_position, current_position)
                self.get_logger().info(f"different_distance : {diff_dist}")
                self.verify_checkpoint()
                return True

    def calc_diff(self, goal_position, current_position):
        goal_position_x = goal_position[0]
        goal_position_y = goal_position[1]
        current_position_x = current_position[0]
        current_position_y = current_position[1]
        self.diff_x = goal_position_x - current_position_x
        self.diff_y = goal_position_y - current_position_y
        diff_dist = math.sqrt((self.diff_x) ** 2 + (self.diff_y) ** 2)

        return diff_dist

    def verify_checkpoint(self):
        next_point_tuple = tuple(self.next_point)
        
        if next_point_tuple in self.store_points and self.status == RobotStatus.TO_STORE:
            self.update_status()
            self.request_module("ST")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Store. So, status updated to {self.status.value}")
        elif next_point_tuple in self.kiosk_points and self.status == RobotStatus.TO_KIOSK:
            self.update_status()
            self.request_module("KS")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Kiosk. So, status updated to {self.status.value}")
        elif next_point_tuple in self.robot_points and self.status == RobotStatus.RETURNING:
            self.update_status()
            self.request_module("HM")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Home. So, status updated to {self.status.value}")
        else:
            self.get_logger().info("On waypoints")


    def request_module(self, location):
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
        self.get_logger().info(f"Arrived to : {current_point}")
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

        self.is_active = False
        self.store_id = ""
        self.kiosk_id = ""
        self.status = RobotStatus.HOME
        self.is_moving = 0

        self.store_points = []
        self.kiosk_points = []
        self.current_point = list(self.robot_points[0])
        self.next_point = list(self.robot_points[0])
        self.current_position = [self.positions[self.current_point[0]][self.current_point[1]][0], self.positions[self.current_point[0]][self.current_point[1]][1]]
        self.next_position = [self.positions[self.current_point[0]][self.current_point[1]][0], self.positions[self.current_point[0]][self.current_point[1]][1]]
        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

    
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

        self.get_logger().info("Get motor order request")
        if self.motor_node.is_active == False:
            self.motor_node.store_id = store_id
            self.motor_node.kiosk_id = kiosk_id
            if store_id[2] == "1":
                self.motor_node.store_points = self.motor_node.get_indices_for_label("Store1")
            elif store_id[2] == "2":
                self.motor_node.store_points = self.motor_node.get_indices_for_label("Store2")
            if kiosk_id[2] == "1":
                self.motor_node.kiosk_points = self.motor_node.get_indices_for_label("Kiosk1")
            elif kiosk_id[2] == "2":
                self.motor_node.kiosk_points = self.motor_node.get_indices_for_label("Kiosk2")
            
            self.motor_node.is_active = True

            if self.motor_node.status == RobotStatus.RETURNING:
                self.motor_node.status == RobotStatus.TO_STORE

            self.get_logger().info(f"Kiosk points: {self.motor_node.kiosk_points}")
            self.get_logger().info(f"Store points: {self.motor_node.store_points}")
            response.success = True
        else:
            response.success = False

        return response

    def short_goal_callback(self, request, response):

        self.get_logger().info(f"Before update: current_point={self.motor_node.current_point}, next_point={self.motor_node.next_point}")
        

        self.motor_node.current_point = self.motor_node.next_point[:]
        self.motor_node.next_point[0] = int(request.next_x)
        self.motor_node.next_point[1] = int(request.next_y)
        self.motor_node.next_position[0] = self.motor_node.positions[int(request.next_x)][int(request.next_y)][0]
        self.motor_node.next_position[1] = self.motor_node.positions[int(request.next_x)][int(request.next_y)][1]

        
        self.get_logger().info(f"After update: current_point={self.motor_node.current_point}, next_point={self.motor_node.next_point}")
        
        next_point = (int(request.next_x), int(request.next_y))


        if 0 <= self.motor_node.next_point[0] <= 4 and 0 <= self.motor_node.next_point[0] <= 8:
            self.motor_node.next_position = self.motor_node.positions[self.motor_node.next_point[0]][self.motor_node.next_point[1]]
            self.get_logger().info(f"next_position : {self.motor_node.next_position[0]}, {self.motor_node.next_position[1]}, status : {self.motor_node.status.value}, active :{self.motor_node.is_active}") 
        else:
            self.get_logger().warn(f"Invalid point!!")
            response.success = False
            return response

        self.get_logger().info(f"next point : {next_point}")
        
        if self.motor_node.is_active:
            if self.motor_node.status in [RobotStatus.HOME, RobotStatus.AT_HOME, RobotStatus.AT_STORE, RobotStatus.AT_KIOSK]:
                if self.motor_node.status == RobotStatus.HOME:
                    if next_point in self.motor_node.waypoint_points:
                        self.motor_node.update_status()
                        self.motor_node.is_moving = 1
                        response.success =  True
                    else:
                        self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                        response.success =  False
                elif self.motor_node.status == RobotStatus.AT_STORE:
                    if next_point in self.motor_node.waypoint_points or next_point in self.motor_node.kiosk_points:
                        self.motor_node.update_status()
                        self.motor_node.is_moving = 1
                        response.success =  True
                    else:
                        self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                        response.success =  False
                elif self.motor_node.status == RobotStatus.AT_KIOSK:
                    if next_point in self.motor_node.waypoint_points or next_point in self.motor_node.robot_points:
                        self.motor_node.update_status()
                        self.motor_node.is_moving = 1
                        response.success =  True
                    else:
                        self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                        response.success =  False
            elif self.motor_node.status in [RobotStatus.TO_STORE]:
                if next_point in self.motor_node.waypoint_points or next_point in self.motor_node.store_points:
                    self.motor_node.is_moving = 1
                    response.success =  True
                else:
                    self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                    response.success =  False
            elif self.motor_node.status == RobotStatus.TO_KIOSK:
                if next_point in self.motor_node.waypoint_points or next_point in self.motor_node.kiosk_points:
                    self.motor_node.is_moving = 1
                    response.success =  True
                else:
                    self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                    response.success =  False
        else:
            self.motor_node.get_logger().info(f"status : {self.motor_node.status}, next_point : {next_point}, active : {self.motor_node.is_active}")
            if self.motor_node.status == RobotStatus.RETURNING:
                if next_point in self.motor_node.waypoint_points or next_point in self.motor_node.robot_points:
                    self.motor_node.is_moving = 1
                    response.success =  True
                else:
                    self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
                    response.success =  False
            else:
                self.get_logger().warn("There is not order!!")
                response.success =  False

        # if response.success == True:

        #     self.motor_node.verify_checkpoint()

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
        # self.get_logger().info(f"Received pose: position=({position.x}, {position.y}, {position.z}), orientation=(roll={roll}, pitch={pitch}, yaw={yaw})")
        self.motor_node.current_position = [x, y, yaw]

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

