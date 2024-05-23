import rclpy as rp
from rclpy.node import Node
from interface_package.srv import Tcp
from std_msgs.msg import String
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

    service_server = UidNode()
    rp.spin(service_server)

    service_server.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
