from enum import Enum
import rclpy as rp
from rclpy.node import Node
from interface_package.srv import OrderInfo, OrderTracking, RobotCall, GoalArrival, DeliveryBox, LocationInfo
from std_msgs.msg import String, Int16
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


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
    DELIVERY_READY = "DR" # Not used yet
    DELIVERY_START = "DS"
    DELIVERY_FINISH = "DF"


class GoalStatus(Enum):
    AT_STORE = 1
    AT_KIOSK = 2
    AT_HOME = 3


class Robot():
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.is_active = False
        self.current_status = RobotStatus.HOME
        self.previous_status = RobotStatus.HOME
        self.current_order_status = OrderStatus.DELIVERY_YET
        self.store_id = ""
        self.kiosk_id = ""
        self.order_id = ""
        self.uid = ""
        self.endPoint = None
    def __str__(self):
        return f"Robot(id = {self.robot_id}, store_id = {self.store_id}, kiosk_id = {self.kiosk_id}, uid = {self.uid})"

    def change_status(self, status):
        if self.is_active != status:
            self.is_active = status
        else:
            raise StatusError(f"Robot is already in status : {status}")

    def update_robot_info(self, order_id, store_id, kiosk_id, uid):
        self.order_id: order_id
        self.store_id = store_id
        self.kiosk_id = kiosk_id
        self.uid = uid

    def reset(self):
        self.is_active = False
        # self.current_status = RobotStatus.HOME
        # self.previous_status = RobotStatus.HOME
        self.current_order_status = OrderStatus.DELIVERY_YET
        self.order_id = ""
        self.store_id = ""
        self.kiosk_id = ""
        self.uid = ""
        # self.endPoint = None

    def returning(self):
        self.is_active = False
        self.current_order_status = OrderStatus.DELIVERY_YET
        self.store_id = ""
        self.kiosk_id = ""
        self.uid = ""

class StatusError(Exception):
    # Exception raised for errors in the robot status.
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class RobotControl(Node):
    def __init__(self):
        super().__init__("robot_control")

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # self.timer_period = 1.0
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.setup_rm()
        self.setup_robots()
        # self.setup_pubs()
        self.setup_subs()
        # self.wait_for_services()

        self.robot_1 = Robot('R-1')
        self.robot_2 = Robot('R-2')
        self.robot_3 = Robot('R-3')
        self.robots = [self.robot_1, self.robot_2, self.robot_3]

    def setup_rm(self):
        self.setup_rm_services()
        self.setup_rm_clients()

    def setup_robots(self):
        self.setup_robot_services()
        self.setup_robot_clients()

    #connect with Robot Manager
    def setup_rm_services(self):
        self.robot_call_service = self.create_service(RobotCall, "robot_call", self.robot_call_callback)

    def setup_rm_clients(self):
        self.delivery_box_client = self.create_client(DeliveryBox, "delivery_box")
        self.goal_arrival_client = self.create_client(GoalArrival, "goal_arrival")

    #connect with robots
    def setup_robot_services(self):
        self.order_tracking_services = {
            'R-1' : self.create_service(OrderTracking, "order_tracking_1", self.order_tracking_callback_1),
            'R-2' : self.create_service(OrderTracking, "order_tracking_2", self.order_tracking_callback_2),
            'R-3' : self.create_service(OrderTracking, "order_tracking_3", self.order_tracking_callback_3)
        }

    def setup_robot_clients(self):
        self.order_clients = {
            'R-1': self.create_client(OrderInfo, "order_info_1"),
            'R-2': self.create_client(OrderInfo, "order_info_2"),
            'R-3': self.create_client(OrderInfo, "order_info_3")
        }
        
        self.motor_order_clients = {
            'R-1': self.create_client(LocationInfo, "location_info_1"),
            'R-2': self.create_client(LocationInfo, "location_info_2"),
            'R-3': self.create_client(LocationInfo, "location_info_3")
        }

    #publishers and subscriptions
    # def setup_pubs(self):
        # self.goal_publishers = {
        #     'R-1' : self.create_publisher(String, "/move_1",self.qos_profile),
        #     'R-2' : self.create_publisher(String, "/move_2", self.qos_profile),
        #     'R-3' : self.create_publisher(String, "/move_3", self.qos_profile)
        # }

    def setup_subs(self):
        self.robots_status = {
            "R-1" : self.create_subscription(Int16, "/robot_status_1", self.robot_status_callback_1, self.qos_profile),
            "R-2" : self.create_subscription(Int16, "/robot_status_2", self.robot_status_callback_2, self.qos_profile),
            "R-3" : self.create_subscription(Int16, "/robot_status_3", self.robot_status_callback_3, self.qos_profile),
        }

    def wait_for_services(self):
        while not self.delivery_box_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delivery_box service...')

        while not self.goal_arrival_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for goal_arrival service...')

        # for robot_id, client in self.order_clients.items():
        #     while not client.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info(f'Waiting for {robot_id} service...')

        self.get_logger().info('All services are now connected')

    def robot_call_callback(self, request, response):

        robot_id = request.robot_id
        order_id = request.order_id
        store_id = request.store_id
        kiosk_id = request.kiosk_id
        uid = request.uid

        if self.robot_1.is_active and self.robot_2.is_active and self.robot_3.is_active:
            self.get_logger().error("All robot is active")
            response.success = False
        else:
            self.get_logger().info("Received Robot Call")
            self.get_logger().info(f"robot_id : {robot_id}, order_id : {order_id}, uid : {uid}")
            if robot_id == "R-1":
                if self.robot_1.is_active:
                    self.get_logger().debug("robot_1 is active")
                    response.success = False
                    return response
                else:
                    selected_robot = self.robot_1
            elif robot_id == "R-2":
                if self.robot_2.is_active:
                    self.get_logger().debug("robot_2 is active")
                    response.success = False
                    return response
                else:
                    selected_robot = self.robot_2
            elif robot_id == "R-3":
                if self.robot_3.is_active:
                    self.get_logger().debug("robot_3 is active")
                    response.success = False
                    return response
                else:
                    selected_robot = self.robot_3
            else:
                self.get_logger().error(f"Unknown robot_id: {robot_id}")
                response.success = False
                return response

            selected_robot.is_active = True
            selected_robot.order_id = order_id
            selected_robot.kiosk_id = kiosk_id
            selected_robot.store_id = store_id
            selected_robot.uid = uid

            self.request_order(robot_id, store_id, kiosk_id, order_id, uid)

            response.success = True

        return response

    def response_delivery_box(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Delivery box request succeeded.")
            else:
                self.get_logger().debug("Delivery box request failed.")
        except Exception as e:
            self.get_logger().error(f"Delivery box request failed with exception: {e}")

    # def timer_callback(self):
    #     for robot in self.robots:
    #         msg = String()
    #         if robot.is_active:
    #             robot_id = robot.robot_id
    #             if robot.current_status == RobotStatus.HOME or robot.current_status == RobotStatus.AT_HOME:
    #                 self.get_logger().info(f"{robot_id} go to {robot.store_id}")
    #                 msg.data = robot.store_id
    #                 self.goal_publishers[robot_id].publish(msg)
    #             elif robot.current_status == RobotStatus.AT_STORE and robot.current_order_status == OrderStatus.DELIVERY_START:
    #                 self.get_logger().info(f"{robot_id} go to {robot.kiosk_id}")
    #                 msg.data = robot.kiosk_id
    #                 self.goal_publishers[robot_id].publish(msg)
    #             elif robot.current_status == RobotStatus.AT_KIOSK and robot.current_order_status == OrderStatus.DELIVERY_FINISH:
    #                 self.get_logger().info(f"go to robot {robot_id} home")
    #                 msg.data = f"H-{robot_id[2]}" # 추가 처리 필요
    #                 self.goal_publishers[robot_id].publish(msg)
    #                 robot.reset()
    #                 print(robot)
    #         else:
    #             pass
    #             ## 에러 처리 해야할 듯

    def robot_status_callback_1(self, msg):
        return self.robot_status_callback(self.robot_1, msg)

    def robot_status_callback_2(self, msg):
        return self.robot_status_callback(self.robot_2, msg)

    def robot_status_callback_3(self, msg):
        return self.robot_status_callback(self.robot_3, msg)

    def robot_status_callback_3(self, msg):
        return self.robot_status_callback(self.robot_3, msg)

    def robot_status_callback(self, robot, msg):
        status = RobotStatus(msg.data)
        robot.previous_status = robot.current_status
        robot.current_status = status
        # self.get_logger().info(f"robot {robot.robot_id} previous status : {robot.previous_status}, current status : {robot.current_status}")

        if robot.previous_status != robot.current_status:
            if status == RobotStatus.AT_STORE:
                self.request_goal_arrival(robot.robot_id, robot.order_id, GoalStatus.AT_STORE)
            elif status == RobotStatus.AT_KIOSK:
                self.request_goal_arrival(robot.robot_id, robot.order_id, GoalStatus.AT_KIOSK)
            elif status == RobotStatus.AT_HOME:
                self.request_goal_arrival(robot.robot_id, robot.order_id, GoalStatus.AT_HOME)

    def order_tracking_callback_1(self, request, response):
        return self.order_tracking_callback(self.robot_1, request, response)

    def order_tracking_callback_2(self, request, response):
        return self.order_tracking_callback(self.robot_2, request, response)

    def order_tracking_callback_3(self, request, response):
        return self.order_tracking_callback(self.robot_3, request, response)

    def order_tracking_callback(self, robot, request, response): # order_id 정보 안받아도 되는거 같음
        status = OrderStatus(request.status)
        try:
            self.get_logger().info(f"status received: {status.name} for robot_id: {robot.robot_id}")
            if status == OrderStatus.DELIVERY_START:
                robot.current_order_status = status
                self.request_delivery_box(robot.robot_id, robot.order_id, 0)
                response.success = True
            elif status == OrderStatus.DELIVERY_FINISH:
                robot.current_order_status = status
                self.request_delivery_box(robot.robot_id, robot.order_id, 1)

                response.success = True
            else:
                self.get_logger().debug(f"Invalid status received: {status} for robot_id: {robot.robot_id}")
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Exception in order_tracking_callback: {e}")
            response.success = False

        return response

    def request_order(self, robot_id, store_id, kiosk_id, order_id, uid):
        self.get_logger().info(f"request order : {order_id} to {robot_id}")
        
        order_request = OrderInfo.Request()
        order_request.uid = uid
        order_request.order_id = order_id

        location_request = LocationInfo().Request()
        location_request.store_id = store_id
        location_request.kiosk_id = kiosk_id

        order_future = self.order_clients[robot_id].call_async(order_request)
        order_future.add_done_callback(self.response_order_callback(robot_id))

        motor_order_future = self.motor_order_clients[robot_id].call_async(location_request)
        motor_order_future.add_done_callback(self.response_motor_order_callback(robot_id))

    def response_order_callback(self, robot_id):
        def response_order(future):
            try:
                response = future.result()
                self.get_logger().info(f"order response from {robot_id} : {response.success}")

            except Exception as e:
                self.get_logger().error(f"order call failed for {robot_id} : {e}")

        return response_order

    def response_motor_order_callback(self, robot_id):
        def response_order(future):
            try:
                response = future.result()
                self.get_logger().info(f"motor_order response from {robot_id} : {response.success}")

            except Exception as e:
                self.get_logger().error(f"motor_order call failed for {robot_id} : {e}")

        return response_order


    def request_delivery_box(self, robot_id, order_id, status):
        # status
        # 0 : Delivery Start
        # 1 : Delivery Finish
        self.get_logger().info(f"request {robot_id} order status(delivery box) : {status} to RobotManager")
        delivery_box_request = DeliveryBox.Request()
        delivery_box_request.status = status
        delivery_box_request.robot_id = robot_id
        delivery_box_request.order_id = order_id

        future = self.delivery_box_client.call_async(delivery_box_request)
        future.add_done_callback(self.response_delivery_box)

    # (Received from robots and send to RM)'s response
    def response_delivery_box(self, future):
        try:
            response = future.result()
            robot_id = response.robot_id
            self.get_logger().info(f"delivery box response from {robot_id} : {response.success}")
        except Exception as e:
            self.get_logger().error(f"delivery box call failed for {robot_id} : {e}")

    def request_goal_arrival(self, robot_id, order_id, status):
        # status
        # 1 : At Store
        # 2 : At Kiosk
        # 3 : At Home
        self.get_logger().info(f"request {robot_id} status : {status.name} to RobotManager")
        goal_arrival_request = GoalArrival.Request()
        goal_arrival_request.status = status.value
        goal_arrival_request.robot_id = robot_id
        goal_arrival_request.order_id = order_id

        future = self.goal_arrival_client.call_async(goal_arrival_request)
        future.add_done_callback(self.response_goal_arrival)

    def response_goal_arrival(self, future):
        try:
            response = future.result()
            robot_id = response.robot_id
            self.get_logger().info(f"goal arrival response from {robot_id} : {response.success}")
        except Exception as e:
            self.get_logger().error(f"goal arrival call failed for {robot_id} : {e}")


def main(args=None):

    rp.init(args=args)
    robot_control = RobotControl()

    try:
        rp.spin(robot_control)
    except KeyboardInterrupt:
        pass
    finally:
        if rp.ok():
            robot_control.destroy_node()
            rp.shutdown()


if __name__ == '__main__':
    main()
