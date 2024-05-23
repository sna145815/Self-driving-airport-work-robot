import socket
import threading
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ct_package.db_manager import DBManager


sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#from TcpNode import *

HOST = '192.168.0.30'
KIOSK_PORT = 9021

class KioskManager(Node):
    def __init__(self, host, port,dbmanager):
        super().__init__('kiosk_manager_node')
        self.host = host
        self.port = port
        self.client_list = [] 
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.db_manager = dbmanager
        self.robotcall_publisher = self.create_publisher(String, 'orderCall', 10)

    def handle_client(self, conn, addr):
        print(f"{addr}에서 연결됨")
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        print(f"{addr}로부터 수신: {data.decode()}")
                        cmd, data = data.decode().split(',', 1)
                        if cmd == 'OR':
                            self.order_request(data, conn)
                        elif cmd == 'GD':
                            d_time = self.get_departure_time(data)
                            msg = "GD,"+d_time
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
        finally:
            print(f"{addr} 연결 해제")
            if conn in self.client_list:
                self.clients.remove(conn)
            conn.close()

    def start_server(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            print(f"서버 시작됨, {self.host}:{self.port}에서 대기 중")

            while True:
                conn, addr = self.server_socket.accept()
                self.client_list.append(conn)
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.start()
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
            result = self.db_manager.fetch_query(query, params)

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
            data_list = data.split(',')

            order_number_query = """
                SELECT ordernumber + 1 AS newOrdernumber 
                FROM `Order` 
                ORDER BY ordernumber DESC 
                LIMIT 1;
            """
            result = self.db_manager.fetch_query(order_number_query)

            new_order_number = result[0][0] if result else 1  # 첫 번째 주문의 경우

            insert_order_query = """
                INSERT INTO `Order` (OrderNumber, OrderStatus, UID, Kiosk_ID,Store_ID) 
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
            msg = String()
            msg.data = str(store_ip[0][0]) + "/" + str(new_order_number)
            self.robotcall_publisher.publish(msg)
        except Exception as e:
            print(f"주문 요청 처리 중 오류: {e}")
            conn.sendall(f"주문 요청 처리 중 오류: {e}".encode())

    def get_menu_id(self, menuname):
        query = "SELECT ID FROM Menu WHERE Name = %s;"
        result = self.db_manager.fetch_query(query, (menuname,))
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
            result = self.db_manager.fetch_query(query, params)
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
    db_manager = DBManager(HOST, 'potato', '1234', 'prj')
    connection = db_manager.create_connection("KioskServer")

    if not connection:
        print("Failed to connect to the database.")
        return
    
    rclpy.init(args=args)

    # KioskManager 노드 생성
    kiosk_manager = KioskManager(HOST, KIOSK_PORT,db_manager)
    # 서버 시작
    kiosk_manager.start_server()

    rclpy.spin(kiosk_manager)

    # 노드 종료
    kiosk_manager.close_server()
    kiosk_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()