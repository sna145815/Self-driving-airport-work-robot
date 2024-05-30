import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int16
from std_msgs.msg import String
from time import sleep
import threading

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

# robot_status = [0, 1, 2, 3, 4, 5, 6]
# 0 : 대기중
# 1 : 매장으로 이동중
# 2 : 매장 도착
# 3 : 키오스크으로 이동중
# 4 : 키오스크 도착
# 5 : 복귀중 
# 6 : 복귀완료

class DrobotMotor(Node):
    def __init__(self):
        super().__init__("drobot_motor")

        self.nav = BasicNavigator()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.timer_period = 1.0
        self.status = 0
        self.before_msg = ""
        
        self.status_pub = self.create_publisher(Int16, "/status", qos_profile)
        self.move = self.create_subscription(String, "/move", self.move_callback, qos_profile)

        self.timer_event = threading.Event()
        self.timer_event.set()
        self.timer_thread = threading.Thread(target=self.timer_callback_thread)
        self.timer_thread.daemon = True
        self.timer_thread.start()
        self.get_logger().info("motor start")

    def move_callback(self, msg):
        if self.before_msg != msg.data:
            if msg.data == 'S-1':
                self.status = 1
                print('send : 1')
                sleep(4)                # 로봇이 목적지까지 이동
                print("send : move")
                self.status = 2
                print('send : 2')
                sleep(2)
                # self.send_goal(1.4, 2.6)
                self.timer_event.clear()  # 쓰레드를 멈춤

            elif msg.data == 'K-1':
                self.timer_event.set()  # 쓰레드를 다시 실행
                self.status = 3
                print('send : 3')
                sleep(4)                # 로봇이 목적지까지 이동
                print("send : move")
                self.status = 4
                print('send : 4')
                sleep(2)
                # self.send_goal(0.32, 2.82)
                self.timer_event.clear()  # 쓰레드를 멈춤

            elif msg.data == 'H-1':
                self.timer_event.set()  # 쓰레드를 다시 실행
                self.status = 5
                print('send : 5')
                sleep(4)
                self.status = 6
                print('send : 6')
                sleep(2)
                # self.send_goal(0.096, 2.578)
                self.status = 0
            
            self.before_msg = msg.data

    def status_publish(self, status):
        msg = Int16()
        msg.data = status
        self.status_pub.publish(msg)
        print(msg.data)

    def timer_callback(self):
        self.status_publish(self.status)

    def timer_callback_thread(self):
        while rp.ok():
            self.timer_event.wait()  # 이벤트가 설정될 때까지 대기
            self.timer_callback()
            sleep(self.timer_period)

    def send_goal(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)
        self.get_logger().info(f"Goal sent to navigator: x: {x}, y: {y}")

        i = 0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
                
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=50.0):
                    self.nav.cancelTask()
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')



def main(args=None):
    rp.init(args=args)
    drobot_motor = DrobotMotor()
    try:
        rp.spin(drobot_motor)
    except KeyboardInterrupt:
        pass
    finally:
        if rp.ok():
            drobot_motor.destroy_node()
            rp.shutdown()
if __name__ == '__main__':
    main()
