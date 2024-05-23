import rclpy as rp
from rclpy.node import Node
from interface_package.msg import Navfeedback, Navgoal, Navresult
from interface_package.srv import Tcp
from interface_package.msg import OrderLocation
from std_msgs.msg import String


class RobotControl(Node):

    def __init__(self):
        super().__init__("robot_control")

        self.order_location_sub = self.create_subscription(OrderLocation, "order_location", self.order_location_callback, 10)
        self.arrival_pub = self.create_publisher(String, 'goal_arrival', 10)

        self.goal_pub = self.create_publisher(Navgoal, "nav_goal", 10)
        # self.result_client = self.create_client(Navresult, "/nav_result")
        self.feedback_sub = self.create_subscription(Navfeedback,
                                                     "nav_feedback",
                                                     self.nav_feedback_callback,
                                                     10)
        self.goal_msg = Navgoal()

    def order_location_callback(self, msg):
        self.get_logger().info(f"Received order id : {msg.order_id}, location : {msg.location}")
        order_id = msg.order_id
        location = msg.location

        #location 좌표 변환
        if location == '1':
            x = 1.0
            y = 2.0
        elif location == '2':
            x = 2.0
            y = 3.0
        else:
            x = 0.0
            y = 0.0

        self.send_nav_goal(order_id, x, y)

    def nav_feedback_callback(self, msg):
        self.get_logger().info(f"Distance remaining: {msg.distance_remaining}")
        arrival_msg = String()
        arrival_msg.data = "도착하였습니다."
        self.arrival_pub.publish(arrival_msg)

    def send_nav_goal(self, order_id, x, y):
        self.goal_msg.order_id = order_id
        self.goal_msg.x = x
        self.goal_msg.y = y
        self.get_logger().info(f"Sending goal: x={x}, y={y}, order_id = {order_id}")
        self.goal_pub.publish(self.goal_msg)



class UidNode(Node):
    def __init__(self):
        super().__init__('uid_node')

        self.srv = self.create_service(Tcp, 'tcp_string', self.handle_request)
        self.pub = self.create_publisher(String, '/uid', 10)


    def uid_publish(self, uid):
        msg = String()
        msg.data = uid
        self.pub.publish(msg)
        self.get_logger().info(f"published message : {uid}")

    def handle_request(self, request, response):
        # 요청을 로그에 출력
        self.get_logger().info(f'Received request: {request.uid}')
        response.response = 'Request received successfully.'

        uid = request.uid

        self.uid_publish(uid)

        return response



def main(args=None):
    rp.init(args=args)
    node = RobotControl()

    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()
