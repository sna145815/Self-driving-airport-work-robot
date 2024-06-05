import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import queue
import socket
import threading
import math
import json
from ct_package.db_manager import DBManager
from interface_package.srv import GoalArrival
from interface_package.srv import RobotCall
from interface_package.srv import StoreAlarm
from interface_package.srv import DeliveryBox
from interface_package.srv import RobotDispatch

HOST_DB = '192.168.0.15'
HOST = '192.168.0.15'
PORT = 9035

class RobotManager(Node):
    def __init__(self,host,port,dbmanager):
        super().__init__('robot_manager_node')
        self.robots = {
                        "R-1": (0.03, 2.5),
                        "R-2": (0.03, 1.6),
                        "R-3": (0.03, 0.6)
                    }
        
        self.work_state = {
                        "R-1": 0,
                        "R-2": 0,
                        "R-3": 0
                    }
        
        self.host = host
        self.port = port
        self.client_list = []
        
        self.db_manager = dbmanager
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_thread = threading.Thread(target=self.start_server)
        server_thread.start()
        self.order_queue = queue.Queue()
        self.setup_services()
        self.get_logger().info('로봇 매니저 노드 시작됨')

    def setup_services(self):
        self.robot_dispatch_srv = self.create_service(RobotDispatch, 'robot_dispatch', self.robot_dispatch_callback)
        self.arrival_srv = self.create_service(GoalArrival, 'goal_arrival', self.arrival_callback)
        self.delivery_box_srv = self.create_service(DeliveryBox, 'delivery_box', self.delivery_box_callback)
        self.store_alarm_cli = self.create_client(StoreAlarm, 'store_alarm')
        #self.setup_service_clients()
        self.setup_topic()

    def setup_topic(self):
        self.position1 = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose_1',self.position_callback1,10)
        #self.position2 = self.create_subscription(String,'/amcl_pose_2',self.position_callback2,1)
        self.position3 = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose_3',self.position_callback3,10)

    def setup_service_clients(self):
        self.robotcall_cli = self.create_client(RobotCall, 'robot_call')
        self.wait_for_service(self.robotcall_cli, 'robot_call')
        self.store_alarm_cli = self.create_client(StoreAlarm, 'store_alarm')
        self.wait_for_service(self.store_alarm_cli, 'StoreAlarm')
        
    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스, 다시 대기 중...')
        self.get_logger().info(f'{service_name} 서비스 이용 가능.')

    def position_callback1(self, msg):
        position = msg.pose.pose.position
        #self.get_logger().info(f'Position: [{position.x}, {position.y}]')
        self.robots["R-1"]=(position.x,position.y)

        # 로봇의 위치 정보를 JSON 형식으로 구성
        robot_position = {
            "robot_id": "R-1",
            "x": position.x,
            "y": position.y
        }
        self.tcp_send(robot_position)
    
    def position_callback2(self, msg):
        position = msg.pose.pose.position
        #self.get_logger().info(f'Position: [{position.x}, {position.y}]')
        self.robots["R-2"]=(position.x,position.y)
    
    def position_callback3(self, msg):
        position = msg.pose.pose.position
        #self.get_logger().info(f'Position: [{position.x}, {position.y}]')
        self.robots["R-1"]=(position.x,position.y)

        # 로봇의 위치 정보를 JSON 형식으로 구성
        robot_position = {
            "robot_id": "R-3",
            "x": position.x,
            "y": position.y
        }
        self.tcp_send(robot_position)

    def arrival_callback(self, req, res):
        try:
            # 1: 매장 도착
            # 2: 키오스크 도착
            # 3: 충전 장소 복귀 완료 = DB에 아무 처리도 하지 않음
            status = req.status
            order_num = req.order_id
            work_robot = req.robot_id

            if status == 1:
                self.get_logger().info(work_robot+'로봇이 매장에 도착했습니다.')
                self.status_manage(order_num, "매장도착", work_robot, "매장도착", "매장도착")
                self.notify_store(1, order_num)
            elif status == 2:
                self.get_logger().info(work_robot+'로봇이 배달지에 도착했습니다.')
                self.status_manage(order_num, "배달지도착", work_robot, "배달지도착", "배달지도착")
            elif status == 3:
                self.get_logger().info(work_robot+'로봇이 충전소에 도착했습니다.')

            res.success = True
            res.robot_id = work_robot
        except Exception as e:
            self.get_logger().error(f'Exception in arrival_callback: {e}')
            res.success = False

        return res

    def delivery_box_callback(self, req, res):
        try:     
            if req.status == 0:
                self.status_manage(req.order_id,"배달중",req.robot_id,"배달중","배달시작")
            elif req.status == 1:
                self.status_manage(req.order_id,"완료",req.robot_id,"대기중","완료")
                self.notify_store(2,req.order_id)
                self.work_state[req.robot_id] = 0
                self.check_order() 
            res.success = True
            res.robot_id = req.robot_id
        except Exception as e:
            self.get_logger().error(f'Exception in arrival_callback: {e}')
            res.robot_id = req.robot_id
            res.success = False

        return res

    def robot_send_goal(self,order_num,store_id,kiosk_id,uid,work_robot):
        # 로봇에게 발행
        req = RobotCall.Request()
        req.order_id = order_num
        req.store_id = store_id
        req.kiosk_id = kiosk_id
        req.uid = uid
        req.robot_id = work_robot

        self.robotcall_cli.call_async(req)
        self.update_order_robotid(order_num,work_robot)
        self.get_logger().info(work_robot+' 로봇 배정')


    def robot_dispatch_callback(self,req, res):
        try:
            order_num = req.order_id
            self.order_queue.put(order_num)

            #일할 로봇을 뽑아옴
            work_robot = self.priority_robot(order_num)

            if work_robot == None:
                self.status_send_tcp()
                res.success = True
                return res
            else:
                self.work_state[work_robot] = 1
                store_id,kiosk_id,uid = self.get_order(order_num)
                self.robot_send_goal(order_num,store_id,kiosk_id,uid,work_robot)
                self.status_manage(order_num,"매장이동중",work_robot,"매장이동중","매장이동중")
                self.notify_store(0,order_num)
                self.order_queue.get()
                res.success = True
                return res
        except Exception as e:
            self.get_logger().error(f"주문 정보를 검색하는 중 오류 발생: {e}")
            res.success = False
            return res

    def update_order_robotid(self,order_num,robot_id):
        try:
            query = """
                UPDATE `Order`
                SET Robot_ID=%s
                WHERE OrderNumber=%s
                """
            params = (robot_id,order_num)
            self.db_manager.execute_query(query, params)
        except Exception as e:
            # Log the error or handle it as needed
            self.get_logger().error(f"An error occurred: {e}")

    def status_manage(self, order_num, order_status, robot_id, robot_status, log_status):
        try:
            query = """
                UPDATE `Order` 
                SET OrderStatus=%s,Robot_ID=%s
                WHERE OrderNumber=%s
                """
            params = (order_status,robot_id,order_num)
            self.db_manager.execute_query(query, params)

            query = """
                UPDATE Robot 
                SET RobotStatus=%s 
                WHERE ID=%s
                """
            params = (robot_status, robot_id)
            self.db_manager.execute_query(query, params)

            query = """
                INSERT INTO RobotLog
                (Robot_ID, EventTime, RobotStatus, Order_ID)
                VALUES (%s, NOW(), %s, %s)
                """
            params = (robot_id, log_status, order_num)
            self.db_manager.execute_query(query, params)
            self.status_send_tcp()
        except Exception as e:
            # Log the error or handle it as needed
            self.get_logger().error(f"An error occurred: {e}")
    
    def notify_store(self,status,order_num):
        req = StoreAlarm.Request()
        req.status = status
        req.order_id = int(order_num)
        
        self.store_alarm_cli.call_async(req)
        
        self.get_logger().info('매장 알람 SEND')

    def priority_robot(self,order_num):
        result = self.robot_status()

        if result is None or len(result) == 0:
            return None
        elif len(result) == 1:
            return result[0][0]
        else:
             self.get_logger().info('대기중 로봇 여러개 가까운 로봇을 찾습니다.')
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
                X = str(result[0][0])  # X 값을 문자열로 변환
                Y = str(result[0][1])  # Y 값을 문자열로 변환
                return X, Y
            
        return None, None  # 만약 결과가 없으면 None을 반환

    def check_order(self):
        if self.order_queue.qsize():
            order_num = self.order_queue.get()
            work_robot = self.priority_robot(order_num)   
            store_id,kiosk_id,uid = self.get_order(order_num)
            self.robot_send_goal(order_num,store_id,kiosk_id,uid,work_robot)
            self.status_manage(order_num,"매장이동중",work_robot,"매장이동중","매장이동중")
            self.notify_store(0,order_num)

    def robot_status(self):
        query = """
               SELECT ID 
               FROM Robot 
               WHERE RobotStatus ='대기중'
            """
        

        return self.db_manager.fetch_query(query)

    def find_closest_robot(self,order_num):
        min_distance = float('inf')
        work_robot = None
        s_x,s_y = self.get_store_location(order_num)
        for robot_id, (r_x, r_y) in self.robots.items():
            if self.work_state[robot_id] == 1:
                continue

            distance = math.sqrt((float(s_x) - float(r_x))**2 + (float(s_y) - float(r_y))**2)

            if distance < min_distance:
                min_distance = distance
                work_robot = robot_id

        self.get_logger().info("선택된 로봇ID : " + str(work_robot))
        return work_robot        
    
    def get_robot_status(self):
        query = """
                SELECT 
                    A.ID, 
                    A.RobotStatus,
                    (SELECT 
                        CASE 
                            WHEN B.Robot_ID IS NULL THEN NULL 
                            ELSE B.OrderNumber 
                        END AS OrderNumber
                    FROM `Order` B
                    WHERE B.OrderStatus <> '완료' AND (B.Robot_ID = A.ID OR B.Robot_ID IS NULL)
                    LIMIT 1) AS OrderNumber
                FROM Robot A;
                """

        return self.db_manager.fetch_query(query)

    def get_robot_logs(self,order_no):
        query = """
                SELECT EventTime,Robot_ID,RobotStatus,Order_ID FROM RobotLog 
                WHERE Order_ID  =%s
                """
        params = (order_no,)
        result = self.db_manager.fetch_query(query,params)

        json = {
            'robot_logs': result
        }

        return json

    def get_unprocessed_orders(self):
        query = """
                SELECT *
                FROM `Order` 
                WHERE Robot_ID IS NULL AND OrderStatus = '대기중'
                """
        return self.db_manager.fetch_query(query)

    def status_send_tcp(self):
        result1 = self.get_robot_status()
        result2 = self.get_unprocessed_orders()
        result3 = self.finish_order()
        result4 = self.cur_order()
        self.send_json(result1,result2,result3,result4)

    def start_server(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.get_logger().info(f"서버 시작됨, {self.host}:{self.port}에서 대기 중")

            while True:
                conn, addr = self.server_socket.accept()
                self.client_list.append(conn)
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.start()
                self.gui_init_send()
                self.get_logger().info(f"{addr}에 대한 스레드 시작됨")
        except Exception as e:
            self.get_logger().error(f"서버 에러: {e}")
        finally:
            self.close_server()
    
    def handle_client(self, conn, addr):
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024).decode()
                        if not data:
                            break
                        for client in self.client_list:
                            try:
                                client.sendall(json.dumps(self.get_robot_logs(data)).encode('utf-8'))
                            except Exception as e:
                                self.get_logger().error(f"Failed to send data to a client: {e}")
                                self.clients.remove(client)  
                    except ConnectionResetError:
                        break
        except KeyboardInterrupt:
            self.get_logger().error("keyboard exit")
            self.close_server()
        finally:
            self.close_server()
    
    def close_server(self):
        self.get_logger().info("로봇매니저 서버 소켓 닫는 중")
        for conn in self.client_list:
            conn.close()
        self.server_socket.close()
        self.get_logger().info("로봇매니저 서버 소켓 닫힘")

    def finish_order(self):
        query = """
                SELECT OrderNumber,OrderStatus,UID,Kiosk_ID,Robot_ID  
                FROM `Order`
                WHERE OrderStatus = '완료'
                """
        return self.db_manager.fetch_query(query)

    def cur_order(self):
        query = """
                SELECT OrderNumber,OrderStatus,UID,Kiosk_ID,Store_ID,Robot_ID  
                FROM `Order`
                WHERE OrderStatus <> '완료' AND OrderStatus <> '대기중' 
                """
        return self.db_manager.fetch_query(query)

    def gui_init_send(self):
        query = """
                SELECT ID, RobotStatus
                FROM Robot 
                """
        result1 = self.db_manager.fetch_query(query)
        result2 = self.get_unprocessed_orders()
        result3 = self.finish_order()
        result4 = self.cur_order()
        self.send_json(result1,result2,result3,result4)

    def tcp_send(self,data):
        result = json.dumps(data)

        for client in self.client_list:
            try:
                client.sendall(result.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to send data to a client: {e}")
                self.clients.remove(client)

    def send_json(self,result1,result2,result3,result4):
        combined_results = {
            'robot_status': result1,
            'unprocessed_orders': result2,
            'finish_orders': result3,
            'cur_orders': result4
        }

        self.tcp_send(combined_results)
       

def main(args=None):
    db_manager = DBManager(HOST_DB, 'potato', '1234', 'prj')
    connection = db_manager.create_connection()

    if not connection:
        print("Failed to connect to the database.")
        return
    
    rclpy.init(args=args)

    #  노드 생성
    robot_manager = RobotManager(HOST,PORT,db_manager)
    rclpy.spin(robot_manager)

    robot_manager.close_server()
    robot_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()