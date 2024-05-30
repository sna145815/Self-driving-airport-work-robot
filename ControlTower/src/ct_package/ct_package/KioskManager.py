import socket
import threading
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ct_package.db_manager import DBManager
from interface_package.srv import OrderCall
HOST = '192.168.1.101' # server(yjs) rosteam3 wifi
# HOST = '192.168.0.210' # server(yjs) ethernet
HOST_DB = '192.168.1.105' # server(kjh)
KIOSK_PORT = 9059

class KioskManager(Node):
    def __init__(self, host, port, dbmanager):
        super().__init__('kiosk_manager_node')
        
        self.dbConnName = "KioskServer"
        
        self.host = host
        self.port = port
        self.client_list = []
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.db_manager = dbmanager
        self.OrderCallClient = self.create_client(OrderCall, '/order_call')
        
    
    def handle_client(self, conn, addr):    # client 연결되면 실행되는 함수
        print(f"{addr}에서 연결됨")
        try:
            with conn:
                while True:
                    try:
                        recv = conn.recv(1024)
                        if not recv:
                            break
                        print(f"{addr}로부터 수신: {recv.decode()}")
                        cmd, data = recv.decode().split(',', 1)
                        if cmd == 'OR':
                            self.order_request(data, conn)
                        elif cmd == 'GD':
                            d_time = self.get_departure_time(data)
                            msg = "GD,"+ d_time
                            self.client_send(msg, conn)
                        elif cmd == 'OI':
                            orders = self.order_select(data)
                            if orders:
                                msg = '/'.join([f"{order[0]},{order[1]}" for order in orders])
                                msg = "OI,"+msg
                                self.client_send(msg, conn)
                            else:
                                self.client_send("No orders found", conn)
                    except ConnectionResetError:
                        break
        except Exception as e:
            print(e)
        # finally:
        #     print(f"{addr} 연결 해제")
        #     if conn in self.client_list:
        #         self.clients.remove(conn)
        #     conn.close()

    def start_server(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            print(f"서버 시작됨, {self.host}:{self.port}에서 대기 중")
            while True:
                # 클라이언트 연결 정보
                conn, addr = self.server_socket.accept()
                self.client_list.append(conn)
                self.client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                self.client_thread.start()
                print(f"{addr}에 대한 스레드 시작됨")
        except Exception as e:
            print(f"서버 에러: {e}")
        finally:
            self.close_server()

    def close_server(self):
        print("키오스크 서버 소켓 닫는 중")
        for conn in self.client_list:
            conn.close()
        self.server_socket.close()
        print("키오스크 서버 소켓 닫힘")

    def get_departure_time(self, uid):
        try:
            query = """
                SELECT B.Departure_time
                FROM Passenger A
                JOIN Schedule B ON A.Schedule_ID = B.ID
                WHERE A.UID = %s;
            """
            params = (uid,)
            result = self.db_manager.fetch_query(query, self.dbConnName, params)
            if result:
                departure_time = result[0][0]
                return departure_time
            else:
                print("해당 UID에 대한 출발 시간이 없음")
                return None
        except Exception as e:
            print(f"UID {uid}에 대한 출발 시간 가져오기 오류: {e}")
            return None

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
            result = self.db_manager.fetch_query(order_number_query, self.dbConnName)
            new_order_number = result[0][0] if result else 1  # 첫 번째 주문의 경우
            # new_order_number = result if result else 1  # 첫 번째 주문의 경우
            insert_order_query = """
                INSERT INTO `Order` (OrderNumber, OrderStatus, UID, Kiosk_ID, Store_ID)
                VALUES (%s, '대기중', %s, %s,%s); 
            """
            self.db_manager.execute_query(insert_order_query,  self.dbConnName, (new_order_number, data_list[0], data_list[1],data_list[2]))
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
                self.db_manager.execute_query(insert_menu_query, self.dbConnName, params)
            menu_name = data_list[3].split('/')[0]
            query = """
                SELECT A.Store_ip
                FROM Store A
                JOIN Menu B ON A.ID = B.Store_ID
                WHERE B.Name = %s;
            """
            store_ip = self.db_manager.fetch_query(query, self.dbConnName,(menu_name,))

            ## Store 쪽으로 토픽 던지기
            # msg = String()
            # # msg.data = str(store_ip[0][0]) + "/" + str(new_order_number)
            # msg = str(store_ip) + "/" + str(new_order_number)  # "192.168.1.20/60"
            # self.robotcall_publisher.publish(msg)

            # 서비스로 바꿔
            request = OrderCall.Request()
            request.ip = str(store_ip[0][0]) # yjs ethernet
            request.order_no = str(new_order_number)
            
            future = self.OrderCallClient.call_async(request)
            print(future)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None: 
                self.get_logger().info(f'응답 받음: {future.result().success}')
            else:
                self.get_logger().error('서비스 호출 실패')
                
        except Exception as e:
            print(f"주문 요청 처리 중 오류: {e}")
            conn.sendall(f"주문 요청 처리 중 오류: {e}".encode())
        # finally:
        #     self.client_thread.join()
        #     self.client_list = []
            
    
    def get_menu_id(self, menuname):
        query = "SELECT ID FROM Menu WHERE Name = %s;"
        result = self.db_manager.fetch_query(query, self.dbConnName, (menuname,))
        if result:
            return result[0][0]
        else:
            print(f"'{menuname}' 이름의 메뉴를 찾을 수 없음.")
            return None
    def order_select(self, uid):
        try:
            query = """
                SELECT OrderNumber, OrderStatus
                FROM `Order`
                WHERE UID = %s AND OrderStatus <> '완료';
            """
            params = (uid,)
            result = self.db_manager.fetch_query(query, self.dbConnName, params)
            return result
        except Exception as e:
            print(f"UID {uid}에 대한 주문 선택 중 오류: {e}")
            return None
    def client_send(self, msg, conn):
        target_client = next((client for client in self.client_list if client.getpeername()[0] == conn.getpeername()[0]), None)
        if target_client:
            target_client.sendall(msg.encode())
            print(target_client.getpeername()[0])
            print("전송"*10)
            
def main(args=None):
    db_manager = DBManager(HOST_DB, 'potato', '1234', 'prj')
    connection = db_manager.create_connection("KioskServer")
    if not connection:
        print("Failed to connect to the database.")
        return
    rclpy.init(args=args)
    # KioskManager 노드 생성
    kiosk_manager = KioskManager(HOST, KIOSK_PORT, db_manager)  
    # 서버 시작
    kiosk_manager.start_server()
    rclpy.spin(kiosk_manager)
    # 노드 종료
    kiosk_manager.close_server()
    kiosk_manager.destroy_node()
    rclpy.shutdown()
def test():
    pass
if __name__ == '__main__':
    main()
