import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import queue
from interface_package.msg import OrderLocation
from ct_package.db_manager import DBManager


HOST = '192.168.0.44'

class RobotManager(Node):
    def __init__(self,dbmanager):
        super().__init__('robot_manager_node')

        self.db_manager = dbmanager

        self.order_queue = queue.Queue()

        self.robotcall_sub = self.create_subscription(String, 'robotCall', self.robot_call_callback, 10)
        
        self.goal_pub = self.create_publisher(OrderLocation, 'robot_goal', 10)

        self.arrival_sub = self.create_subscription(String, 'goal_arrival', self.arrival_callback, 10)

        #self.position_sub = self.create_subscription(PoseStamped, 'robot_position', self.position_callback, 10)

        self.get_logger().info('로봇 매니저 노드 시작됨')

        ##필요 ROS
        # robotcall.srv
        # arrival.srv
        # 실시간 위치 전송 topic
        # 매장/고객 적재함 통신( 매장측은 넣고 출발 할때만 (열땐 필요 없음) 고객측은 음식 수거하고 닫을 때만 (열때 필요 없음))


    def arrival_callback(self, msg):
        self.get_logger().info('로봇이 목표 지점에 도착했습니다.')
        # 0 : 완료
        # 1 : 매장도착
        # 2 : 키오스크 도착  = DB에 아무 처리도 하지 않음
        # 3 : 충전장소 복귀 완료 = DB에 아무 처리도 하지 않음
        if msg.status == 0:
            self.check_order()
        elif msg.status == 1:
            order_num = msg.order_id
            work_robot = msg.robot_id
            self.status_manage(order_num,"매장도착",work_robot,"매장도착","매장도착")
            # 4. StoreManager로 send (매장에 로봇 도착알림 주기 위해서) 추가
        elif msg.status == 2:
            order_num = msg.order_id
            work_robot = msg.robot_id
            self.status_manage(order_num,"배달지도착",work_robot,"배달지도착","배달지도착")

    
    ####################
    # ROS 통신 2개 추가

    # 매장에서 음식을 넣으면 (실질적 배송이 시작 될 시점)
        # self.status_manage(order_num,"배달중",work_robot,"배달중","배달시작")


    #완료는 로봇에서 RFID찍고 음식을 찾았을때 완료 그때 ROS통신 하나 받으면 
        # self.status_manage(order_num,"완료",work_robot,"대기중","완료")
                
    ######################

    def position_callback(self, msg):
        self.get_logger().info('로봇의 현재 위치: x=%f, y=%f, z=%f', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def robot_send_goal(self,order_num,store_id,kiosk_id,uid,work_robot):
        # 로봇에게 발행
        msg = OrderLocation()
        msg.order_id = order_num
        msg.store_id = store_id
        msg.kiosk_id = kiosk_id
        msg.uid = uid
        msg.robot_id = work_robot
        print("PUB한다@~!~")
        self.goal_pub.publish(msg)
        #
        self.get_logger().info(work_robot+' 로봇 배정')


    def robot_call_callback(self, order_num):
        try:
            print("robotCALL LOGIC@!!!")
            self.order_queue.put(order_num)

            #일할 로봇을 뽑아옴
            work_robot = self.priority_robot()

            if work_robot == None:
                return
            else:
                store_id,kiosk_id,uid = self.get_order(order_num)
                self.robot_send_goal(order_num,store_id,kiosk_id,uid,work_robot)
                self.status_manage(order_num,"매장이동중",work_robot,"매장이동중","매장이동중")
                self.order_queue.get()
        except Exception as e:
            print(f"주문 정보를 검색하는 중 오류 발생: {e}")


    def status_manage(self, order_num, order_status, robot_id, robot_status, log_status):
        try:
            query = """
                UPDATE `Order` 
                SET OrderStatus=%s 
                WHERE OrderNumber=%s
                """
            params = (order_status, order_num)
            self.execute_query(query, params)

            query = """
                UPDATE Robot 
                SET RobotStatus=%s 
                WHERE ID=%s
                """
            params = (robot_status, robot_id)
            self.execute_query(query, params)

            query = """
                INSERT INTO RobotLog
                (Robot_ID, EventTime, RobotStatus, Order_ID)
                VALUES (%s, NOW(), %s, %s)
                """
            params = (robot_id, log_status, order_num)
            self.execute_query(query, params)
        except Exception as e:
            # Log the error or handle it as needed
            print(f"An error occurred: {e}")





    def priority_robot(self):
        result = self.robot_status()

        if result is None or len(result) == 0:
            return None
        elif len(result) == 1:
            return result[0][0]
        else:
            #여러대 일때 로직 처리
            #실시간 위치 토픽을 구독 하여
            #현재 해당 로봇들의 위치 좌표와 목적지 좌표값을 비교하여 그중 가까운 로봇으로 할당
            pass


        # for row in result:
        #     print(row[0])

        return "R-1"

    def get_order(self, order_num):
        query = """
                SELECT Store_ID, Kiosk_ID, UID  
                FROM `Order` 
                WHERE OrderNumber = %s;
            """
        params = (order_num,)
        
        result = self.db_manager.fetch_query(query, params)

        if result:
            if len(result) > 0:
                # 결과가 여러 행인 경우 첫 번째 행의 값을 사용
                store_id = result[0][0]
                kiosk_id = result[0][1]
                uid = result[0][2]
                return store_id, kiosk_id, uid
        return None

    def check_order(self):
        if self.order_queue.qsize():
            work_robot = self.priority_robot()
            order_num = self.order_queue.get()
            store_id,kiosk_id,uid = self.get_order(order_num)
            self.robot_send_goal(order_num,store_id,kiosk_id,uid,work_robot)



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