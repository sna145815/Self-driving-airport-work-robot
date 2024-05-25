import rclpy as rp
from rclpy.node import Node
from interface_package.srv import OrderInfo, OrderTracking, RobotCall
class RobotControl(Node):
    def __init__(self):
        super().__init__("robot_control")


        # self.order_service = self.create_service(Order, 'oreder', self.handle_order)
        self.order_service = self.create_service(RobotCall, 'robot_call', self.handle_order)
        self.order_client_1 = self.create_client(OrderInfo, 'order_info_1')
        self.order_tracking_service_1 = self.create_service(OrderTracking,
                                                          'order_tracking_1',
                                                          self.handle_order_tracking_1)
        self.order_client_2 = self.create_client(OrderInfo, 'order_info_2')
        self.order_tracking_service_2 = self.create_service(OrderTracking,
                                                          'order_tracking_2',
                                                          self.handle_order_tracking_2)

        while not self.order_client_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service_1 not available, waiting again...')
        while not self.order_client_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service_2 not available, waiting again...')

        self.order_req_1 = OrderInfo.Request()
        self.order_req_2 = OrderInfo.Request()
 
    
    def handle_order(self, request, response):
        order_id = request.order_id
        store_id = request.store_id
        kiosk_id = request.kiosk_id
        uid = request.uid
        robot_id = request.robot_id

        if robot_id == 1:
            self.send_order_request_1(uid, order_id)
            #모터 서버한테 스토어아이디, 키오스크 아이디 send
        elif robot_id == 2:
            self.send_order_request_2(uid, order_id)
        
        response.success = True

        return response
            


    def handle_order_tracking_1(self, request, response):
        state = request.state

        if state == "DS":
            self.get_logger().info(f"Received state : {state}")
            #모터한테 action 주기
            response.success = True
        elif state == "DF":
            self.get_logger().info(f"Received state : {state}")
            response.success = True
            #모터한테 action 주기
        else:
            response.success = False
        #state = 스토어로 가는 중, 대기 중
        
        return response
    
    def handle_order_tracking_2(self, request, response):
        state = request.state

        if state == "DS":
            self.get_logger().info(f"Received state : {state}")
            #모터한테 action 주기
            response.success = True
        elif state == "DF":
            self.get_logger().info(f"Received state : {state}")
            response.success = True
            #모터한테 action 주기
        else:
            response.success = False
        #state = 스토어로 가는 중, 대기 중
        
        return response
        

    def send_order_request_1(self, uid, order_id):
        self.order_req_1.uid = uid
        self.order_req_1.order_id = order_id
        self.future = self.order_client_1.call_async(self.order_req_1)
        self.future.add_done_callback(self.handle_order_response_1)

    def send_order_request_2(self, uid, order_id):
        self.order_req_2.uid = uid
        self.order_req_2.order_id = order_id
        self.future = self.order_client_2.call_async(self.order_req_2)
        self.future.add_done_callback(self.handle_order_response_2)

    def handle_order_response_1(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received order response: {response.success}")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")


    def handle_order_response_2(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received order response: {response.success}")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")


def main(args=None):
    rp.init(args=args)
    node = RobotControl()
    node.send_order_request_2('43 09 0F F8', '1')
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
