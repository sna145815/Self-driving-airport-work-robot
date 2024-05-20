import rclpy as rp

from rclpy.node import Node



from test_package_msg.srv import Tcp

class TcpNode(Node):
    def __init__(self):
        super().__init__('tcp_node')

        self.tcp_client = self.create_client(Tcp, 'tcp_string')
        while not self.tcp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Tcp.Request()

        self.data = None

    def send_request(self, string):
        self.req.kiosk_location = string
        self.future = self.tcp_client.call_async(self.req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded: %s' % response.response)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)

def main(args=None):
    rp.init(args=args)

    tcp_node = TcpNode()
    rp.spin(tcp_node)

    tcp_node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()