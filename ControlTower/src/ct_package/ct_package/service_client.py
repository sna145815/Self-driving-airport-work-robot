# service_client.py

import rclpy
from rclpy.node import Node
from mini_bot_msg.srv import MotorControl,LCDControl,RFIDControl 
from time import sleep
from std_msgs.msg import String


class RFIDServiceClient(Node):

    def __init__(self):
        super().__init__('rfid_service_client')
        self._client = self.create_client(RFIDControl, 'rfid_control')
        self.sub = self.create_subscription(String,'/uid',self.callback,10)
        self.sub
        self.uid=''
        self._motor_client = MotorServiceClient()

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


class MotorServiceClient(Node):

    def __init__(self):
        super().__init__('motor_service_client')
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


class LCDServiceClient(Node):

    def __init__(self):
        super().__init__('lcd_service_client')
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
    # #LCD
    # rclpy.init(args=args)
    # lcd_service_client = LCDServiceClient()
    # lcd_service_client.run()
    # rclpy.shutdown()

    # #SERVO
    # rclpy.init(args=args)
    # motor_service_client = MotorServiceClient()
    # motor_service_client.run()
    # rclpy.shutdown()

    #RFID
    rclpy.init(args=args)
    rfid_service_client = RFIDServiceClient()
    rfid_service_client.send_request()
    rclpy.spin(rfid_service_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
