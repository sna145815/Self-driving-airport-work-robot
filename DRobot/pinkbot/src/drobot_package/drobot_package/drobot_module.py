import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from interface_package.srv import OrderTracking,OrderInfo
from servo_controller import ServoController
from lcd_controller import LCDController  
from time import sleep
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
from std_msgs.msg import Int16

ROBOT_NUM = 1

class DRobotModule(Node):
    def __init__(self):
        super().__init__('drobot_module')
        self.order_service = self.create_service(OrderInfo, 'order_info', self.order_callback)
        self.order_tracking_client = self.create_client(OrderTracking,'order_tracking')
        self.robot_flag = self.create_subscription(Int16,'/status',self.callback,10)
        self.servo_controller = ServoController()
        self.servo_controller.move_to_zero()
        self.lcd_controller = LCDController()
        self.order_tracking_req = OrderTracking.Request()
        GPIO.setmode(GPIO.BOARD) 
        self.get_logger().info('service start')
        self.lcd_controller.display_string(f"DRobot_{ROBOT_NUM}", 1)
        self.uid=''
        self.order_id=''
        self.count = 0
        self.robot_status = 0

    def order_callback(self, request, response):
        self.uid = request.uid
        self.order_id = request.order_id
        response.success = True
        self.get_logger().info(f'received! UID : {self.uid}  order_ID : {self.order_id}')
        self.lcd_controller.display_string(f"Order No. {self.order_id} ", 2)
        print(f"order_id : {self.order_id}")
        return response
    
    def callback(self, msg):
        if (self.robot_status != msg.data and (msg.data == 2 or msg.data == 4)):
            print(f"{self.robot_status}------{msg.data}")
            
            self.get_logger().info("scan RFID")
            
            self.lcd_controller.display_string_at_position("Tag me", 9, 1)
            try:
                reader = SimpleMFRC522()
                
                while True:
                    id, text = reader.read()
                    self.rid = hex(id)
                    self.rid = self.hex_string_to_byte_array(self.rid[2:-2])
                    self.get_logger().info(f'Read RFID ID: {self.rid}')
                    if self.rid == self.uid or self.rid == '43 09 0F F8': # tmp master key
                        if self.count == 0:
                            self.servo_controller.move_to_max()
                            self.count += 1
                            
                        else:
                            self.servo_controller.move_to_zero()
                            if msg.data == 2:
                                self.send_request("DS") 
                                self.get_logger().info("Delivery Start")
                            
                            elif msg.data == 4:
                                self.send_request("DF") 
                                self.get_logger().info("Delivery Finish")

                            self.lcd_controller.display_string_at_position(" "*6, 9, 1)
                            self.count = 0 
                            self.robot_status = msg.data
                            break
                        sleep(1)
                        

                    else:
                        self.get_logger().info("permission denied!")
                    
            except Exception as e:
                self.get_logger().error(f'Error reading RFID: {str(e)}')

        self.robot_status = msg.data

    def hex_string_to_byte_array(self,hex_string):
        byte_array = [hex_string[i:i+2].upper() for i in range(0, len(hex_string), 2)]
        byte_array_str = ' '.join(byte_array)
        return byte_array_str
    
    def send_request(self,status):
        self.order_tracking_req.order_id = self.order_id
        self.order_tracking_req.status = status
        future = self.order_tracking_client.call_async(self.order_tracking_req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            self.robot_status = 0
            self.get_logger().info(f'Success: {future.result().success}')  
        except Exception as e:
            self.get_logger().warning('Service call failed! %r' % e)
    
    def cleanup_gpio(self):
        self.servo_controller.cleanup()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    order_service = DRobotModule()
    try:
        rclpy.spin(order_service)
    except KeyboardInterrupt:
        pass
    finally:
        order_service.destroy_node()
        order_service.cleanup_gpio()
        rclpy.shutdown()     
if __name__ == '__main__':
    main()