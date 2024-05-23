# service_server.py

import rclpy
from rclpy.node import Node
from interface_package.srv import MotorControl,LCDControl,RFIDControl
from servo_controller import ServoController
from lcd_controller import LCDController  
from time import sleep
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')
        self._service = self.create_service(MotorControl, 'motor_control', self.execute_callback)
        self._lcd_service = self.create_service(LCDControl, 'lcd_control', self.execute_lcd_callback)
        self._rfid_service = self.create_service(RFIDControl, 'rfid_control', self.execute_rfid_callback)
        self._servo_controller = ServoController()
        self._lcd_controller = LCDController()
        GPIO.setmode(GPIO.BOARD) 
        self.get_logger().info('service start')

    def execute_rfid_callback(self, request, response):
        while True:
            try:
                reader = SimpleMFRC522()
                id, text = reader.read()
                self.get_logger().info(f'Read RFID ID: {id}')
                response.id = id
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Error reading RFID: {str(e)}')
                response.success = False
            return response
  


    def execute_callback(self, request, response):

        command = request.command

        self.get_logger().info(f'Received motor command: {command}')
        if command == 'open' or command == 'close':
            self.motor_command_callback(command)
        
        response.success = True
        return response
    
    def execute_lcd_callback(self, request, response):
        line1 = request.line1
        line2 = request.line2
        self.get_logger().info(f'Displaying on LCD: Line 1 - {line1}, Line 2 - {line2}')
        self._lcd_controller.display_string(line1, 1)
        self._lcd_controller.display_string(line2, 2)
        response.success = True
        return response
    
    def motor_command_callback(self, command):
 
        self.get_logger().info('Received motor command: %s' % command) 
        try:
            if command == "open":
                # open
                self._servo_controller.move_to_max()
                sleep(1)
             
            elif command == "close":
                
                # close
                self._servo_controller.move_to_zero()
                sleep(1)

     
        except Exception as e:
            self.get_logger().error("An error occurred while executing motor command: %s" % str(e))

    def cleanup_gpio(self):
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    motor_service_server = ServiceServer()
    
    try:
        rclpy.spin(motor_service_server)
    except KeyboardInterrupt:
        motor_service_server.destroy_node()
        motor_service_server.cleanup_gpio()
        rclpy.shutdown()
    finally:
        motor_service_server.destroy_node()
        motor_service_server.cleanup_gpio()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
