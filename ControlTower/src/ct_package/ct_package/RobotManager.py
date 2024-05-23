import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import queue
from interface_package.msg import OrderLocation
# from db_manager import DBManager

import mysql.connector
from mysql.connector import Error

class DBManager:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.conn = None

    def create_connection(self,name):
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                password=self.password,
                database=self.database
            )
            if self.conn.is_connected():
                print(name+" "+f"Connected to MySQL database {self.database}")
        except Error as e:
            print(f"Error connecting to database: {e}")
        return self.conn

    def close_connection(self):
        """Close the database connection."""
        if self.conn.is_connected():
            self.conn.close()
            print("MySQL connection is closed")

    def execute_query(self, query, params=None):
        try:
            cursor = self.conn.cursor()
            if params:
                cursor.execute(query, params)
            else:
                cursor.execute(query)
            self.conn.commit()
            print("Query executed successfully")
        except Error as e:
            print(f"Error executing query: {e}")

    def fetch_query(self, query, params=None):

        try:
            cursor = self.conn.cursor()
            if params:
                cursor.execute(query, params)
            else:
                cursor.execute(query)
            result = cursor.fetchall()
            return result
        except Error as e:
            print(f"Error fetching query: {e}")
            return None




HOST = '192.168.0.30'

ROBOT_1_STAY = "0,0"
ROBOT_1_ORDER_NUM = 0

class RobotManager(Node):
    def __init__(self,dbmanager):
        super().__init__('robot_manager_node')

        self.db_manager = dbmanager

        self.order_queue = queue.Queue()

        self.robot_1_queue = queue.Queue()

        self.robotcall_sub = self.create_subscription(String, 'robotCall', self.robot_call_callback, 10)
        
        self.goal_pub = self.create_publisher(OrderLocation, 'robot_goal', 10)

        self.arrival_sub = self.create_subscription(String, 'goal_arrival', self.arrival_callback, 10)

        #self.position_sub = self.create_subscription(PoseStamped, 'robot_position', self.position_callback, 10)

        self.get_logger().info('로봇 매니저 노드 시작됨')


    def arrival_callback(self, msg):
        self.get_logger().info('로봇이 목표 지점에 도착했습니다.')
        # if self.robot_1_queue.qsize():
        #     self.robot_1_send_goal()
        # else:
        #     print("집으로가~")
        

    def position_callback(self, msg):
        self.get_logger().info('로봇의 현재 위치: x=%f, y=%f, z=%f', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def robot_1_send_goal(self):
        # 로봇에게 발행
        msg = OrderLocation()
        msg.order_id = "1"
        msg.location = "S-1"
        print("PUB한다@~!~")
        self.goal_pub.publish(msg)
        #
        self.get_logger().info('로봇의 목표 지점을 발행했습니다.')

    def robot_call_callback(self, order_num):
        try:
            print("robotCALL LOGIC@!!!")
            self.order_queue.put(order_num)

            # work_robot = self.priority_robot()
            # print("QUERY 진입 전!")
            # print(order_num)
            # result = self.get_order(order_num)
            # print(result)
            # print("QUERY 진입 후!")
            # print(self.robot_1_queue[0])
            self.robot_1_queue.put("S-1")
            self.robot_1_queue.put("K-1")
            ROBOT_1_ORDER_NUM = order_num
            self.robot_1_send_goal()
            # if result:
            #     store_id, kiosk_id = result[0]
            #     if work_robot == "R-1":
            #         self.robot_1_queue.put(store_id)
            #         self.robot_1_queue.put(kiosk_id)
            #         ROBOT_1_ORDER_NUM = order_num
            #         self.robot_1_send_goal(ROBOT_1_ORDER_NUM,store_id)
            #     else:
            #         print("미구현")

            self.order_queue.get()
        except Exception as e:
            print(f"주문 정보를 검색하는 중 오류 발생: {e}")

    def priority_robot(self):
        return "R-1"

    def get_order(self,order_num):
        query = """
                SELECT Store_ID, Kiosk_ID  
                FROM `Order` 
                WHERE OrderNumber = %s;
            """
        params = (order_num,)
        return self.db_manager.fetch_query(query, params)


    def robot_status(self):
        query = """
               SELECT ID 
               FROM Robot 
               WHERE RobotStatus ='대기중'
            """
        return self.db_manager.fetch_query(query)
        

def main(args=None):
    db_manager = DBManager(HOST, 'potato', '1234', 'prj')
    connection = db_manager.create_connection("StoreServer")

    if not connection:
        print("Failed to connect to the database.")
        return
    
    rclpy.init(args=args)

    #  노드 생성
    robot_manager = RobotManager(db_manager)


    rclpy.spin(robot_manager)

    robot_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()