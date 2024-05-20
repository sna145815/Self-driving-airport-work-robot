import rclpy as rp
from rclpy.node import Node
from test_package_msg.srv import Tcp

class TestNode(Node):
    def __init__(self):
        super().__init__('test_service_node')

        self.srv = self.create_service(Tcp, 'tcp_string', self.handle_request)

    def handle_request(self, request, response):
        # 요청을 로그에 출력
        self.get_logger().info(f'Received request: {request.kiosk_location}')
        response.response = 'Request received successfully.'

        return response
    
def main(args=None):
    rp.init(args=args)

    service_server = TestNode()
    rp.spin(service_server)

    service_server.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
