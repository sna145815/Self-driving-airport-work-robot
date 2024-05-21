import socket
import threading
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


from TcpNode import *

class KioskServer():
    def __init__(self, host, port, s_clients,db_manager):
        self.host = host
        self.port = port
        self.clients = []
        self.s_clients = s_clients  # 전달된 clients 리스트 사용
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.db_manager = db_manager
       # self.node = node


    def handle_client(self, conn, addr):
        print(f"Connected by {addr}")
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        print(f"Received from {addr}: {data.decode()}")
                        cmd, data = data.decode().split(',', 1)
                        if cmd == 'OR':
                            self.order_request(data,conn)
                        elif cmd == 'GD':
                            d_time = self.get_deapturetime(data)
                            self.client_send(d_time,conn)
                        elif cmd == 'OI':
                            msg = self.order_select(data)
                            msg = '/'.join([f"{order[0]},{order[1]}" for order in msg])
                            self.client_send(msg,conn)
                    except ConnectionResetError:
                        break
                    

        finally:
            print(f"Disconnected by {addr}")
            if conn in self.clients:
                self.clients.remove(conn)
            conn.close()


    def start_server(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            print(f"Server started, listening on {self.host}:{self.port}")

            while True:
                conn, addr = self.server_socket.accept()
                self.clients.append(conn)
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.start()
                print(f"Started thread for {addr}")
        except Exception as e:
            print(f"Server error: {e}")
        finally:
            self.close_server()

    def close_server(self):
        print("Closing Kiosk server socket")
        for conn in self.clients:
            conn.close()
        self.server_socket.close()
        print("Kiosk server socket closed")

    def get_deapturetime(self,uid):
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
                departure_time = result[0][0]  # 결과에서 Departure_time 가져오기
                return departure_time
            else:
                print("No departure time found for the given UID")
                return None
        except Exception as e:
                print(f"Error getting departure time for UID {uid}: {e}")
                return None


    def order_request(self, data, conn):
        try:
            data_list = data.split(',')
            #주문번호 생성
            order_number_query = """
                    SELECT ordernumber+1 as newOrdernumber FROM `Order` ORDER BY ordernumber DESC LIMIT 1;
                    """
            result = self.db_manager.fetch_query(order_number_query)

            if result:
                new_order_number=result
            else:
                new_order_number = 1 # 첫 번째 주문의 경우

            print(data_list)
            # 새로운 주문 삽입
            insert_order_query = """
                INSERT INTO `Order` (OrderNumber, OrderStatus, UID, Kiosk_ID) 
                VALUES (%s, '대기중', %s, %s);
            """
            self.db_manager.execute_query(insert_order_query, (new_order_number[0][0], data_list[0], data_list[1]))

            menu_info = data_list[2:-1]  # 메뉴 정보 리스트
            cnt = int(data_list[-1])

            for i in range(cnt):
                menu_name, menu_cnt = menu_info[i].split('/')
                menu_cnt = int(menu_cnt)
                menu_id = self.get_menuId(menu_name)

                insert_menu_query = """
                    INSERT INTO `OM_list` (OrderNumber, Menu_ID, Menu_cnt)
                    VALUES (%s, %s, %s);
                """

                params = (new_order_number[0][0], menu_id, menu_cnt)
                self.db_manager.execute_query(insert_menu_query, params)


            # 해당 매장에 주문 알림
            menu_name = data_list[2].split('/')[0]
            query = """
                SELECT A.Store_ip 
                FROM Store A
                JOIN Menu B ON A.ID = B.Store_ID 
                WHERE B.Name = %s;
            """
            result = self.db_manager.fetch_query(query, (menu_name,))

            if result:
                store_ip = result[0][0]  # 결과에서 Store_ip 가져오기
                target_client = next((client for client in self.s_clients if client.getpeername()[0] == store_ip), None)

                if target_client:
                    cnt = int(data_list[-1])
                    menus = data_list[2:2 + cnt]
                    msg = f"OS,{new_order_number[0][0]},{cnt},{','.join(menus)}"
                    target_client.sendall(msg.encode())
                else:
                    conn.sendall(f"No client found with IP {store_ip}".encode())
            else:
                conn.sendall(f"No store IP found for the given menu name: {menu_name}".encode())

                        
        except Exception as e:
                print(f"Error processing order request: {e}")
                conn.sendall(f"Error processing order request: {e}".encode())

    def get_menuId(self, menuname):
        query = "SELECT ID FROM Menu WHERE Name = %s;"
        result = self.db_manager.fetch_query(query, (menuname,))
        if result:
            return result[0][0]
        else:
            print(f"Menu with name '{menuname}' not found.")
            return None

    def order_select(self,uid):
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
            print(f"Error selecting orders for UID {uid}: {e}")
            return None
        

    def client_send(self,msg,conn):
        target_client = next((client for client in self.clients if client.getpeername()[0] == conn.getpeername()[0]), None)
        if target_client:
            target_client.sendall(msg.encode())