import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import queue
import math
from ct_package.db_manager import DBManager
from interface_package.srv import GoalArrival
from interface_package.srv import RobotCall
from interface_package.srv import StoreAlarm

HOST = '192.168.1.105'

class RobotManager(Node):
    def __init__(self,dbmanager):
        super().__init__('robot_manager_node')

        self.robots = {
                        1: (2, 3),
                        2: (5, 8),
                        3: (12, 6)
                    }

        self.db_manager = dbmanager

        self.order_queue = queue.Queue()

        self.robotcall_cli = self.create_client(RobotCall, 'robotCall')
        
        while not self.robotcall_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RobotCall Service, waiting again...')
        self.get_logger().info('RobotCall Service available.')

        self.store_alarm_cli = self.create_client(StoreAlarm, 'storeAlarm')
        
        while not self.store_alarm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('storeAlarm Service, waiting again...')
        self.get_logger().info('storeAlarm Service available.')

        self.arrival_srv = self.create_service(GoalArrival,'goal_arrival',self.arrival_callback)

        #self.position_sub = self.create_subscription(Location, 'robot_position', self.position_callback, 10)

        self.get_logger().info('로봇 매니저 노드 시작됨')

    def position_callback(self, msg):
        # 메시지에서 로봇의 ID, X, Y 좌표 추출
        robot_id = msg.robot_id
        x = msg.x
        y = msg.y
        
        # 전역 변수 ROBOTS 업데이트
        if robot_id in self.ROBOTS:
            self.ROBOTS[robot_id] = (x, y)

    def arrival_callback(self, req, res):
        try:
            # 1: 매장 도착
            # 2: 키오스크 도착
            # 3: 충전 장소 복귀 완료 = DB에 아무 처리도 하지 않음
            order_num = req.order_id
            work_robot = req.robot_id

            if req.status == 1:
                self.get_logger().info('로봇이 매장에 도착했습니다.')
                self.status_manage(order_num, "매장도착", work_robot, "매장도착", "매장도착")
                self.notify_store(req.status, order_num)
            elif req.status == 2:
                self.get_logger().info('로봇이 배달지에 도착했습니다.')
                self.status_manage(order_num, "배달지도착", work_robot, "배달지도착", "배달지도착")
            elif req.status == 3:
                self.get_logger().info('로봇이 충전소에 도착했습니다.')
            else:
                raise ValueError(f"Invalid status value: {req.status}")

            res.success = True
            res.robot_id = work_robot
        except Exception as e:
            self.get_logger().error(f'Exception in arrival_callback: {e}')
            res.success = False

        return res

    
    ####################
    # ROS 통신 2개 추가

    # 매장에서 음식을 넣으면 (실질적 배송이 시작 될 시점)
        # self.status_manage(order_num,"배달중",work_robot,"배달중","배달시작")


    #완료는 로봇에서 RFID찍고 음식을 찾았을때 완료 그때 ROS통신 하나 받으면 
        # self.status_manage(order_num,"완료",work_robot,"대기중","완료")
        # self.notify_store() 
        # self.check_order() 
    ######################

    def position_callback(self, msg):
        self.get_logger().info('로봇의 현재 위치: x=%f, y=%f, z=%f', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def robot_send_goal(self,order_num,store_id,kiosk_id,uid,work_robot):
        # 로봇에게 발행
        req = RobotCall.Request()
        req.order_id = order_num
        req.store_id = store_id
        req.kiosk_id = kiosk_id
        req.uid = uid
        req.robot_id = work_robot

        future = self.robotcall_cli.call_async(req)
        
        self.get_logger().info(work_robot+' 로봇 배정')

        if future.result() is not None:
            response = future.result()
            self.robotcall_cli.get_logger().info(f'Result: {response.success}')
            return True
        else:
            self.robotcall_cli.get_logger().error('Exception while calling service: %r' % future.exception())
            return False

    def robot_call_callback(self, order_num):
        try:
            print("robotCALL LOGIC@!!!")
            self.order_queue.put(order_num)

            #일할 로봇을 뽑아옴
            work_robot = self.priority_robot(order_num)

            if work_robot == None:
                return
            else:
                store_id,kiosk_id,uid = self.get_order(order_num)
                result = self.robot_send_goal(order_num,store_id,kiosk_id,uid,work_robot)
                if result:
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

    def notify_store(self,status,order_num):
        req = StoreAlarm.Request()
        req.status = status
        req.order_id = order_num
        
        future = self.client.call_async(req)
        
        self.get_logger().info('매장 알람 SEND')

        if future.result() is not None:
            response = future.result()
            self.robotcall_cli.get_logger().info(f'Store Alarm: {response.success}')
        else:
            self.robotcall_cli.get_logger().error('Exception while calling Store Alarm service: %r' % future.exception())

    def priority_robot(self,order_num):
        result = self.robot_status()

        if result is None or len(result) == 0:
            return None
        elif len(result) == 1:
            return result[0][0]
        else:
            #여러대 일때 로직 처리
            return self.find_closest_robot(order_num)

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
    
    def get_store_location(self,order_num):
        query = """
                SELECT B.Location_x as X, B.Location_y as Y 
                FROM `Order`A JOIN Store B 
                ON A.Store_ID = B.ID 
                WHERE OrderNumber = %s
            """
        params = (order_num,)
        
        result = self.db_manager.fetch_query(query, params)

        if result:
            if len(result) > 0:
                X = result[0][0]  # X는 첫 번째 열의 값
                Y = result[0][1]  # Y는 두 번째 열의 값
                return X, Y
            
        return None, None  # 만약 결과가 없으면 None을 반환

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

    def calculate_distance(r_x, r_y, s_x, s_y):
        return math.sqrt((s_x - r_x)**2 + (s_y - r_y)**2)

    def find_closest_robot(self,order_num):
        min_distance = float('inf')
        work_robot = None
        s_x,s_y = self.get_store_location(order_num)
        
        for robot_id, (r_x, r_y) in self.robots.items():
            distance = self.calculate_distance(r_x, r_y,s_x,s_y)
            #print(f"Robot {robot_id} at ({x}, {y}) has distance {distance} to (10, 10)")
            if distance < min_distance:
                min_distance = distance
                work_robot = robot_id
        
        return work_robot        

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