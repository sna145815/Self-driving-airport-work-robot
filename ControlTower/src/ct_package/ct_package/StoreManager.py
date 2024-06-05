import sys
import os
import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ct_package.db_manager import DBManager

from interface_package.srv import StoreAlarm
from interface_package.srv import OrderCall
from interface_package.srv import RobotDispatch
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


# HOST = '192.168.1.102' # server(yjs) rosteam3 wifi
HOST = '192.168.0.15' # server(yjs) ethernet
# HOST_DB = '192.168.1.105' # DB manager kjh rosteam3 wifi
HOST_DB = '192.168.0.15' # DB manager kjh rosteam3 wifi
STORE_PORT = 9023

class StoreManager(Node):
    def __init__(self, host, port, db_manager):
        super().__init__('store_server_node')

        self.host = host
        self.port = port
        self.client_list = []
        self.db_manager = db_manager
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


        # SM to RM 로봇 콜 PUB발행
        # 서비스로 바꿔
        # self.robotcall_publisher = self.create_publisher(String, 'robotCall', 10)
        self.robotDispatch = self.create_client(RobotDispatch,'/robot_dispatch')
        self.req_robotDispatch = RobotDispatch.Request()

        # KM to SM 주문요청 SUB
        # self.order_call_subscriber = self.create_subscription(String, 'orderCall', self.order_call_callback, 10)

        self.storeAlarmServer = self.create_service(StoreAlarm, 'store_alarm', self.store_callback_service)
        self.oderCallServer1 = self.create_service(OrderCall, 'order_call', self.order_call_callback_service)
        # self.oderCallServer1 = self.create_service(OrderCall, 'Kiosk1/order_call', self.order_call_callback_service)
        # self.oderCallServer2 = self.create_service(OrderCall, 'Kiosk2/order_call', self.order_call_callback_service)

    # def robot_dispatch_callback(self, request, response):
    #     self.req_robotDispatch.order_id = "1"
    #     future = self.robotDispatch.call_async(self.req_robotDispatch)
    #     rclpy.spin_until_future_complete(self, future)   
    #     pass
    
    # 로봇이 매장에 도착했다는 알람을 받는 ros service
    def store_callback_service(self, request, response):    # status, orderid
        self.get_logger().info("store_callback_start!!!")
        self.get_logger().info('status : ', request.status)
        self.get_logger().info('orderId : ', request.order_id)

        try:
            if request.order_id:
                response.success = True
                robotStatus = str(request.status)
                orderNo = str(request.order_id)

                query = """
                        SELECT s.Store_ip , o.Robot_ID 
                        FROM `Order` o 
                        JOIN Store s ON o.Store_ID = s.ID 
                        WHERE o.OrderNumber = %s
                    """
                
                result = self.db_manager.fetch_query(query, [orderNo])

                if result:
                    self.get_logger().info("result : ", result)
                    ip = result[0][0]
                    robotID = result[0][1]

                    msg = f"DS,{orderNo},{robotStatus},{robotID}"

                    store_ip = next((client for client in self.client_list if client.getpeername()[0] == ip), None)
                    self.get_logger().info(msg)

                    if store_ip:
                        store_ip.sendall(msg.encode())
                    else:
                        self.get_logger().info("매장 접속 안됨")
                else:
                    self.get_logger().info("DB결과 없음")
            else:
                response.success = False

            self.get_logger().info("--------------------------")

        except Exception as e:
            self.get_logger().error("store_callback Error : ", e)
        except KeyboardInterrupt:
            pass

        return response


    def order_call_callback_service(self, request, response):
        self.get_logger().info("order_call_callback_start!!!")
        try:
            if request.ip:
                response.success = True
                # 받은 메시지를 '/'를 기준으로 분할하여 파싱
                try:
                    # data_list = msg.data.split('/')
                    # store_ip = data_list[0]
                    # order_number = data_list[1]

                    store_ip = request.ip
                    order_number = request.order_no
                    self.get_logger().info(type(order_number))
                    self.get_logger().info("orderNo : ", order_number)

                    query = """
                        SELECT A.OrderNumber, 
                            (SELECT Name FROM Menu WHERE ID = B.Menu_ID) AS Menu_Name,
                            B.Menu_cnt 
                        FROM `Order` A 
                        JOIN OM_list B 
                        ON A.OrderNumber = B.OrderNumber 
                        WHERE A.OrderNumber = %s;
                    """

                    result = self.db_manager.fetch_query(query, [order_number])
                    # self.get_logger().info("result : ", result)

                    if result:
                        self.get_logger().info("result : ", result)
                        order_number = result[0][0]
                        menus = [f"{row[1]}/{row[2]}" for row in result]
                        cnt = len(menus)
                        msg = f"OS,{order_number},{cnt},{','.join(menus)}"
                        store_ip = next((client for client in self.client_list if client.getpeername()[0] == store_ip), None)
                        self.get_logger().info(msg)

                        if store_ip:
                            store_ip.sendall(msg.encode())
                        else:
                            self.get_logger().info("매장 접속 안됨")
                    else:
                        self.get_logger().info("DB결과 없음")
                except Exception as e:
                        self.get_logger().error(f"Error fetching order details: {e}")
                        return None
            else:
                response.success = False
            
            self.get_logger().info(response)
            self.get_logger().info("--------------------------")
            
            return response
        except Exception as e:
            self.get_logger().error("Error message : ", e)
        except KeyboardInterrupt:
            self.close_server()

    
    def handle_client(self, conn, addr):
        self.get_logger().info(f"{addr} 연결됨")
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        self.get_logger().info(f"{addr}로부터 수신: {data.decode()}")
                        cmd, data = data.decode().split(',', 1)
                        data_list = data.split(',')
                        self.get_logger().info(data_list)
                        if cmd == 'DR':
                            self.get_logger().info(f"{data_list[0]} 매장에서 / {data_list[1]} 주문 배차 요청!")
                            self.robot_call(data_list[1])
                        elif cmd == "CS":
                            self.get_logger().info("CS호출")
                            self.order_status(data,0)
                        elif cmd == 'SS':
                            self.get_logger().info("SS호출")
                            self.store_status(data_list[0],data_list[1])
                        elif cmd == 'MS':
                            self.get_logger().info("MS호출")
                            self.menu_status(data_list[0],data_list[1])
                        data_list.clear()
                        self.get_logger().info("-------------------------")
                    except ConnectionResetError:
                        break
        except KeyboardInterrupt:
            self.get_logger().error("keyboard exit")
            self.close_server()
        finally:
            self.close_server()
            # self.get_logger().info(f"{addr} 연결 해제")
            # if conn in self.client_list:
            #     self.client_list.remove(conn)
            # conn.close()

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
                self.get_logger().info(f"{addr}에 대한 스레드 시작됨")
        except Exception as e:
            self.get_logger().error(f"서버 에러: {e}")
        finally:
            self.close_server()

    def close_server(self):
        self.get_logger().info("스토어 서버 소켓 닫는 중")
        for conn in self.client_list:
            conn.close()
        self.server_socket.close()
        self.get_logger().info("스토어 서버 소켓 닫힘")

    def order_status(self,param,status):
        if status == 0:
            query = """
                        UPDATE Order
                        SET OrderStatus='조리중'
                        WHERE OrderNumber=%s;
                        """
        elif status == 1:
            query = """
                        UPDATE Order
                        SET OrderStatus='매장이동'
                        WHERE OrderNumber=%s;
                        """

        params = (param,)
        self.db_manager.execute_query(query, params)


    def store_status(self, s_id, status):
        try:
            if status == '1':
                query = """
                    UPDATE Store
                    SET Status='open'
                    WHERE ID=%s
                    """
                msg = "SS"+"/"+s_id+"/"+"1"
            else:
                query = """
                    UPDATE Store
                    SET Status='close'
                    WHERE ID=%s
                    """
                msg = "SS"+"/"+s_id+"/"+"0"
                
            params = (s_id,)
            self.db_manager.execute_query(query, params)
                                       
            self.k_send(msg)
            

        except Exception as e:
            self.get_logger().error(f"상점 상태 업데이트 중 오류 발생: {e}")
            return None
        
    def robot_call(self, order_number):          
        # msg = String()
        # msg.data = order_number
        # self.robotcall_publisher.publish(msg)
        self.get_logger().info("robot call")

        self.req_robotDispatch.order_id = order_number
        # future = self.robotDispatch.call_async(self.req_robotDispatch)
        self.robotDispatch.call_async(self.req_robotDispatch)
        # rclpy.spin_until_future_complete(self, future)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)


        # if future.result() is not None:
        #     self.get_logger().info(f'응답 받음: {future.result().success}')
        # else:
        #     self.get_logger().error('서비스 호출 실패')
        
        # if future.done():
        #     future.cancel()
        #     del future
        
        
    def menu_status(self, m_id, status):
        try:
            if status == '1':
                query = """
                    UPDATE Menu
                    SET Status='open'
                    WHERE ID=%s
                    """
                msg = "MS"+"/"+m_id+"/"+"1"
            else:
                query = """
                    UPDATE Menu
                    SET Status='close'
                    WHERE ID=%s
                    """
                msg = "MS"+"/"+m_id+"/"+"0"
                
            params = (m_id,)
            self.db_manager.execute_query(query, params)
                                       
            self.k_send(msg)

        except Exception as e:
            self.get_logger().error(f"상점 상태 업데이트 중 오류 발생: {e}")
            return None

    def k_send(self,msg):
        query = """
                SELECT Kiosk_ip 
                FROM Kiosk;
            """
        result = self.db_manager.fetch_query(query)

        if result:
            for kip in result:
                ksend = kip[0]  
                target_client = next((k_client for k_client in self.k_clients if k_client.getpeername()[0] == ksend), None)

                if target_client:
                    target_client.sendall(msg.encode())
                    


# def get_local_ip():

def main(args=None):
    db_manager = DBManager(HOST_DB, 'potato', '1234', 'prj')
    # dbConnName = "StoreServer"
    # connection = db_manager.create_connection(dbConnName)
    

    # if not connection:
    #     self.get_logger().info("Failed to connect to the database.")
    #     return
    
    rclpy.init(args=args)

    #  노드 생성
    store_manager = StoreManager(HOST, STORE_PORT, db_manager)
    server_thread = threading.Thread(target=store_manager.start_server)
    server_thread.start()

    rclpy.spin(store_manager)

    # 노드 종료
    store_manager.close_server()
    store_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

