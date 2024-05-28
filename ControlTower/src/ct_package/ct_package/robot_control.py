import rclpy as rp
from rclpy.node import Node
from interface_package.srv import OrderInfo, OrderTracking, RobotCall, GoalArrival, DeliveryBox
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int16
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class RobotControl(Node):
    def __init__(self):
        super().__init__("robot_control")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.robot_call_service = self.create_service(RobotCall, "robot_call", self.robot_call_callback)
        self.delivery_box_client = self.create_client(DeliveryBox,"delivery_box")
        self.goal_arrival_client = self.create_client(GoalArrival, "goal_arrival")

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.order_clients = {
            'R-1' : self.create_client(OrderInfo, "order_info_1"),
            'R-2' : self.create_client(OrderInfo, "order_info_2")
        }

        self.goal_publishers = {
            'R-1' : self.create_publisher(String, "/move_1",qos_profile),
            'R-2' : self.create_publisher(String, "/move_2", qos_profile)
        }

        self.robots_status = {
            "R-1" : self.create_subscription(Int16, "/robot_status_1", self.robot_status_callback_1, qos_profile),
            "2" : self.create_subscription(Int16, "/robot_status_2", self.robot_status_callback_2, qos_profile)
        }

        self.order_tracking_services = {
            'R-1' : self.create_service(OrderTracking, "order_tracking_1", self.order_tracking_callback_1),
            'R-2' : self.create_service(OrderTracking, "order_tracking_2", self.order_tracking_callback_2)
        }


        self.is_robot_active = {'R-1' : 0, 'R-2' : 0}
        self.current_robot_status = {'R-1' : 0, 'R-2' : 0}

        self.current_order_status = {'R-1' : None, 'R-2' : None}

        self.goals = {}
        self.order_info = {}
        self.robot_info = {}

    def update_robot_info(self, robot_id, order_id, store_id, kiosk_id, uid):
        self.robot_info[robot_id] = {
            "order_id": order_id,
            "store_id": store_id,
            "kiosk_id": kiosk_id,
            "uid": uid
        }
    
    def update_order_info(self, robot_id, order_id, store_id, kiosk_id, uid):
        self.order_info[order_id] = {
            "robot_id": robot_id,
            "store_id": store_id,
            "kiosk_id": kiosk_id,
            "uid": uid
        }


    # from RM
    def robot_call_callback(self, request, response):
        self.get_logger().info("Received Robot Call")

        robot_id = request.robot_id
        order_id = request.order_id
        store_id = request.store_id
        kiosk_id = request.kiosk_id
        uid = request.uid

        self.update_robot_info(robot_id, order_id, store_id, kiosk_id, uid)
        self.update_order_info(robot_id, order_id, store_id, kiosk_id, uid)
        print(self.robot_info)
        print(self.order_info)

        if not robot_id or not order_id or not uid or not store_id or not kiosk_id:
            self.get_logger().error("Missing required fields in the request.")
            response.success = False
        else:
            self.is_robot_active[robot_id] = 1
            self.goals[robot_id] = {"store_id" : store_id, "kiosk_id" : kiosk_id}
            self.get_logger().info(f"is robot active : {self.is_robot_active}")
            self.get_logger().info(f"goals : {self.goals}")

            self.request_order(robot_id, order_id, uid)

            response.success = True

        return response

    def response_delivery_box(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Delivery box request succeeded.")
            else:
                self.get_logger().info("Delivery box request failed.")
        except Exception as e:
            self.get_logger().error(f"Delivery box request failed with exception: {e}")


    def timer_callback(self):
        for robot_id, value  in self.is_robot_active.items():
            msg = String()
            if value == 1:
                if self.current_robot_status[robot_id] == 0:
                    self.get_logger().info(f"go to {self.goals[robot_id]['store_id']}")
                    msg.data = self.goals[robot_id]["store_id"]
                    self.goal_publishers[robot_id].publish(msg)
                elif self.current_robot_status[robot_id] == 2 and self.current_order_status[robot_id] == "DS":
                    self.get_logger().info(f"go to {self.goals[robot_id]['kiosk_id']}")
                    msg.data = self.goals[robot_id]["kiosk_id"]
                    self.goal_publishers[robot_id].publish(msg)
                elif self.current_robot_status[robot_id] == 4 and self.current_order_status[robot_id] == "DF":
                    self.get_logger().info(f"go to robot{robot_id} home")
                    msg.data = f"H-{robot_id}" # 추가 처리 필요
                    self.goal_publishers[robot_id].publish(msg)
                    self.goals.pop(robot_id)
            else:
                pass
                ## 에러 처리 해야할 듯

    def robot_status_callback_1(self, msg):
        return self.robot_status_callback('1', msg)

    def robot_status_callback_2(self, msg):
        return self.robot_status_callback('2', msg)

    # def robot_status_callback_3(self, msg):
    #     return self.robot_status_callback(3, msg)
    
    def robot_status_callback(self, robot_id, msg):
        status = msg.data
        # self.get_logger().info(f"robot {robot_id} status : {status}")
        self.current_robot_status[robot_id] = status


    def order_tracking_callback_1(self, request, response):
        return self.order_tracking_callback('1', request, response)
    
    def order_tracking_callback_2(self, request, response):
        return self.order_tracking_callback('2', request, response)

    def order_tracking_callback(self, robot_id, request, response):
        status = request.status
        order_id = request.order_id
        self.get_logger().info(self.robot_info[robot_id]["order_id"])
        try:
            self.get_logger().info(f"status received: {status} for robot_id: {robot_id}")
            if status == "DS":
                self.current_order_status[robot_id] = status
                self.request_delivery_box(robot_id, self.robot_info[robot_id]["order_id"], 1)
                response.success = True
            elif status == "DF":
                self.current_order_status[robot_id] = status
                self.request_delivery_box(robot_id, self.robot_info[robot_id]["order_id"], 2)
                
                self.is_robot_active[robot_id] = 0

                
                print(self.is_robot_active)
                print(self.current_robot_status)
                print(self.current_order_status)
                print(self.goals)
                print(self.order_info)
                print(self.robot_info)

                response.success = True
            else:
                self.get_logger().info(f"Invalid status received: {status} for robot_id: {robot_id}")
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Exception in order_tracking_callback: {e}")
            response.success = False

        return response


    def request_order(self, robot_id, order_id, uid):
        self.get_logger().info(f"send to {robot_id} module")
        order_request = OrderInfo.Request()
        order_request.uid = uid
        order_request.order_id = order_id
        future = self.order_clients[robot_id].call_async(order_request)
        future.add_done_callback(self.response_order_callback(robot_id))

    def response_order_callback(self, robot_id):
        def response_order(future):
            try:
                response = future.result()
                self.get_logger().info(f"Received order response from {robot_id} : {response.success}")

            except Exception as e:
                self.get_logger().info(f"Service call failed for {robot_id} : {e}")
        
        return response_order

    def request_delivery_box(self, robot_id, order_id, status):
        # 1 : 매장 도착
        # 2 : 키오스크 도착
        # 3 : 충전장소 복귀 완료
        self.get_logger().info(f"send robot{robot_id} robot status : {status} to RobotManager")
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
            self.get_logger().info(f"Received delivery box response from {robot_id} : {response.success}")
        except Exception as e:
            self.get_logger().info(f"delivery box call failed for {robot_id} : {e}")

    def request_goal_arrival(self, robot_id, order_id, status):
        self.get_logger().info(f"send robot{robot_id} order status : {status} to RobotManager")
        goal_arrival_request = GoalArrival.Request()
        goal_arrival_request.status = status
        goal_arrival_request.robot_id = robot_id
        goal_arrival_request.order_id = order_id

        future = self.goal_arrival_client.call_async(goal_arrival_request)
        future.add_done_callback(self.response_goal_arrival)

    def response_goal_arrival(self, future):
        try:
            response = future.result()
            robot_id = response.robot_id
            self.get_logger().info(f"Received goal arrival response from {robot_id} : {response.success}")
        except Exception as e:
            self.get_logger().info(f"goal arrival call failed for {robot_id} : {e}")



def main(args=None):

    rp.init(args=args)
    robot_control = RobotControl()
    
    try:
        rp.spin(robot_control)
    finally:
        robot_control.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
