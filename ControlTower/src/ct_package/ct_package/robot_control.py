import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from interface_package.msg import Navfeedback, Navgoal, Navresult
from interface_package.srv import Tcp
from interface_package.msg import OrderLocation
from std_msgs.msg import String
from interface_package.srv import MotorControl,LCDControl,RFIDControl 


class Motor(Node):

    def __init__(self):
        super().__init__("motor")

        self.order_location_sub = self.create_subscription(OrderLocation, "robot_goal", self.order_location_callback, 10)
        self.arrival_pub = self.create_publisher(String, 'goal_arrival', 10)
        self.order_pub = self.create_publisher(String, '/order_id', 10)


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
        order_msg = String()
        order_msg.data = order_id
        self.order_pub.publish(order_msg)

        #location 좌표 변환
        if location == 'S-1':
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


class RFID(Node):

    def __init__(self):
        super().__init__('rfid')
        self._client = self.create_client(RFIDControl, 'rfid_control')
        self.sub = self.create_subscription(String,'/uid',self.callback,10)
        self.sub
        self.uid=''
        self._motor_client = Servo()

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RFID service not available, waiting again...')
    
    def callback(self,msg):
        self.uid = msg.data
        print("receive uid from Kiosk")

    def send_request(self):
        request = RFIDControl.Request()
        future = self._client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.rid = hex(response.id)
                self.rid = self.hex_string_to_byte_array(self.rid[2:-2])
                self.get_logger().info(f'Successfully received RFID ID: {self.rid}')
                if self.rid == self.uid:
                    self._motor_client.run()

                self.send_request()
            else:
                self.get_logger().warning('Failed to receive RFID ID')

                self.send_request()
        except Exception as e:
            self.get_logger().warning(f'Service call failed: {e}')
            self.send_request()

    def hex_string_to_byte_array(self,hex_string):
        byte_array = [hex_string[i:i+2].upper() for i in range(0, len(hex_string), 2)]
        byte_array_str = ' '.join(byte_array)
        return byte_array_str


class Servo(Node):

    def __init__(self):
        super().__init__('servo')
        self._client = self.create_client(MotorControl, 'motor_control')
        self.state="close"

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, command):
        self.get_logger().info("Sending motor command...")
        request = MotorControl.Request()
        request.command = command

        self.get_logger().info(f'Sending command: {command}')
        future = self._client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            self.get_logger().info(f'Success: {future.result().success}')
        except Exception as e:
            self.get_logger().warning('Service call failed! %r' % e)

    def run(self):
        if self.state=='close':
            self.send_request('open')
            self.state='open'
        else:
            self.send_request('close')
            self.state='close'


class LCD(Node):

    def __init__(self):
        super().__init__('lcd')
        self._client = self.create_client(LCDControl, 'lcd_control')

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, line1, line2):
        self.get_logger().info("Sending LCD command...")
        request = LCDControl.Request()
        request.line1 = line1
        request.line2 = line2

        self.get_logger().info(f'Sending lines: {line1}, {line2}')
        future = self._client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            self.get_logger().info(f'Success: {future.result().success}')
        except Exception as e:
            self.get_logger().warning('Service call failed! %r' % e)

    def run(self):#input
        while True:
            line1 = input("Enter text for line 1: ")
            line2 = input("Enter text for line 2: ")
            self.send_request(line1, line2)


def main(args=None):
    rp.init(args=args)
    motor_node = Motor()
    rfid_node = RFID()
    servo_node = Servo()
    lcd_node = LCD()

    executor = MultiThreadedExecutor()

    executor.add_node(motor_node)
    executor.add_node(rfid_node)
    executor.add_node(servo_node)
    executor.add_node(lcd_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        motor_node.destroy_node()
        rfid_node.destroy_node()
        servo_node.destroy_node()
        lcd_node.destroy_node()
        rp.shutdown()

if __name__ == "__main__":
    main()
