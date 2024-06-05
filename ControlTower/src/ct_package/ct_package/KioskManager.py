import socket
import threading
import queue
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ct_package.db_manager import DBManager
from interface_package.srv import OrderCall

HOST = '192.168.0.15'  # server(yjs) rosteam3 wifi
HOST_DB = '192.168.0.15'  # server(kjh)
KIOSK_PORTS = [9054, 9055]  # 포트 리스트

class KioskManager(Node):
    def __init__(self, host, ports, dbmanager):
        super().__init__('kiosk_manager_node')
        self.host = host
        self.ports = ports
        self.db_manager = dbmanager
        self.OrderCallClient = self.create_client(OrderCall, '/order_call')
        self.data_queue = queue.Queue()  # 데이터 처리를 위한 큐
        self.server_sockets = []
        self.client_list = []
        

        # 각 포트에 대한 서버 소켓을 설정하고 수신 대기
        for port in self.ports:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((self.host, port))
            server_socket.listen()
            self.server_sockets.append(server_socket)
            thread = threading.Thread(target=self.accept_clients, args=(server_socket,))
            thread.start()
        
        # 데이터 처리 스레드 시작
        self.data_thread = threading.Thread(target=self.process_data)
        self.data_thread.start()

    def accept_clients(self, server_socket):
        while True:
            conn, addr = server_socket.accept()
            self.client_list.append(conn)
            client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            client_thread.start()
            self.get_logger().info(f"{addr}에 대한 스레드 시작됨")
            
    def handle_client(self, conn, addr):
        self.get_logger().info(f"{addr}에서 연결됨")
        try:
            with conn:
                while True:
                    try:
                        recv = conn.recv(1024)
                        if not recv:
                            break
                        self.get_logger().info(f"{addr}로부터 수신: {recv.decode()}")
                        # 받은 데이터를 큐에 넣음
                        self.data_queue.put((recv.decode(), conn))
                    except ConnectionResetError:
                        break
        except Exception as e:
            self.get_logger().error(e)
        finally:
        # 클라이언트 연결이 끊겼을 때 클라이언트 리스트에서 제거
            if conn in self.client_list:
                self.client_list.remove(conn)

    def process_data(self):
        while True:
            data, conn = self.data_queue.get()
            try:
                cmd, data = data.split(',', 1)
                if cmd == 'OR':
                    self.get_logger().info("OR호출")
                    self.order_request(data, conn)
                elif cmd == 'GD':
                    self.get_logger().info("GD호출")
                    d_time = self.get_departure_time(data)
                    msg = "GD," + d_time
                    self.client_send(msg, conn)
                elif cmd == 'OI':
                    self.get_logger().info("OI호출")
                    orders = self.order_select(data)
                    if orders:
                        msg = '/'.join([f"{order[0]},{order[1]}" for order in orders])
                        msg = "OI," + msg
                        self.client_send(msg, conn)
                    else:
                        self.client_send("No orders found", conn)
            except Exception as e:
                self.get_logger().error(f"데이터 처리 중 오류: {e}")

    def order_request(self, data, conn):
        try:
            data_list = data.split(',') # UID, KID, SID, menu/n, menuCnt
            order_number_query = """
                SELECT ordernumber + 1 AS newOrdernumber
                FROM `Order`
                ORDER BY ordernumber DESC
                LIMIT 1;
            """
            # 새로 입력된 orderNo 추출
            result = self.db_manager.fetch_query(order_number_query)
            new_order_number = result[0][0] if result else 1  # 첫 번째 주문의 경우
            # new_order_number = result if result else 1  # 첫 번째 주문의 경우
            insert_order_query = """
                INSERT INTO `Order` (OrderNumber, OrderStatus, UID, Kiosk_ID, Store_ID)
                VALUES (%s, '대기중', %s, %s,%s);
            """
            self.db_manager.execute_query(insert_order_query, (new_order_number, data_list[0], data_list[1],data_list[2]))
            menu_info = data_list[3:-1]  # 메뉴 정보 리스트
            cnt = int(data_list[-1])
            for i in range(cnt):
                menu_name, menu_cnt = menu_info[i].split('/')
                menu_cnt = int(menu_cnt)
                menu_id = self.get_menu_id(menu_name)
                insert_menu_query = """
                    INSERT INTO `OM_list` (OrderNumber, Menu_ID, Menu_cnt)
                    VALUES (%s, %s, %s);
                """
                params = (new_order_number, menu_id, menu_cnt)
                self.db_manager.execute_query(insert_menu_query, params)
            menu_name = data_list[3].split('/')[0]
            query = """
                SELECT A.Store_ip
                FROM Store A
                JOIN Menu B ON A.ID = B.Store_ID
                WHERE B.Name = %s;
            """
            store_ip = self.db_manager.fetch_query(query, (menu_name,))

            ## Store 쪽으로 토픽 던지기
            # msg = String()
            # # msg.data = str(store_ip[0][0]) + "/" + str(new_order_number)
            # msg = str(store_ip) + "/" + str(new_order_number)  # "192.168.1.20/60"
            # self.robotcall_publisher.publish(msg)

            # 서비스로 바꿔
            request = OrderCall.Request()
            request.ip = str(store_ip[0][0]) # yjs ethernet
            request.order_no = str(new_order_number)
            
            self.OrderCallClient.call_async(request)
            # rclpy.spin_until_future_complete(self, future)
            # if future.result() is not None: 
            #     self.get_logger().info(f'응답 받음: {future.result().success}')
            # else:
            #     self.get_logger().error('서비스 호출 실패')
                
        except Exception as e:
            self.get_logger().error(f"주문 요청 처리 중 오류: {e}")
            conn.sendall(f"주문 요청 처리 중 오류: {e}".encode())
        # finally:
        #     self.client_thread.join()
        #     self.client_list = []

    def get_departure_time(self, uid):
        # 출발 시간 조회 처리 구현
        try:
            query = """
                SELECT B.Departure_time
                FROM Passenger A
                JOIN Schedule B ON A.Schedule_ID = B.ID
                WHERE A.UID = %s;
            """
            params = (uid,)
            result = self.db_manager.fetch_query(query, params)
            if result:
                departure_time = result[0][0]
                return departure_time
            else:
                self.get_logger().info("해당 UID에 대한 출발 시간이 없음")
                return None
        except Exception as e:
            self.get_logger().error(f"UID {uid}에 대한 출발 시간 가져오기 오류: {e}")
            return None

    def order_select(self, uid):
        try:
            query = """
                SELECT OrderNumber, OrderStatus
                FROM `Order`
                WHERE UID = %s AND OrderStatus <> '완료';
            """
            params = (uid,)
            result = self.db_manager.fetch_query(query, params)
            return result
        except Exception as e:
            self.get_logger().error(f"UID {uid}에 대한 주문 선택 중 오류: {e}")
            return None
    
    def get_menu_id(self, menuname):
        query = "SELECT ID FROM Menu WHERE Name = %s;"
        result = self.db_manager.fetch_query(query, (menuname,))
        if result:
            return result[0][0]
        else:
            self.get_logger().info(f"'{menuname}' 이름의 메뉴를 찾을 수 없음.")
            return None

    def client_send(self, msg, conn):
        target_client = None
        for client in self.client_list:
            try:
                if client.getpeername()[0] == conn.getpeername()[0]:
                    target_client = client
                    break
            except OSError:
                continue

        if target_client:
            target_client.sendall(msg.encode())
            self.get_logger().info(f"전송 완료: {target_client.getpeername()[0]}")
        else:
            self.get_logger().info("전송 실패: 클라이언트를 찾을 수 없음")

def main(args=None):
    db_manager = DBManager(HOST_DB, 'potato', '1234', 'prj')
    connection = db_manager.create_connection()
    if not connection:
        print("Failed to connect to the database.")
        return
    rclpy.init(args=args)
    kiosk_manager = KioskManager(HOST, KIOSK_PORTS, db_manager)  
    rclpy.spin(kiosk_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()