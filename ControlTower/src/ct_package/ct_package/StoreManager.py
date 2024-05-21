import socket
import threading

class StoreServer:
    def __init__(self, host, port, clients,k_clients, db_manager):
        self.host = host
        self.port = port
        self.clients = clients  # 전달된 clients 리스트 사용
        self.k_clients = k_clients
        self.db_manager = db_manager
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def handle_client(self, conn, addr):
        print(f"{addr} 연결됨")
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        print(f"{addr}로부터 수신: {data.decode()}")
                        cmd, data = data.decode().split(',', 1)
                        data_list = data.split(',')
                        print(data_list)
                        if cmd == 'DR':
                            #미구현
                            print(f"{data_list[0]} 매장에서 / {data_list[1]} 주문 배차 요청!")
                            conn.sendall("배차 해드릴게요~!".encode())
                        elif cmd == 'SS':
                            self.store_status(data_list[0],data_list[1])
                        elif cmd == 'MS':
                            self.menu_status(data_list[0],data_list[1])
                        data_list.clear()
                    except ConnectionResetError:
                        break
        finally:
            print(f"{addr} 연결 해제")
            if conn in self.clients:
                self.clients.remove(conn)
            conn.close()

    def start_server(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            print(f"서버 시작됨, {self.host}:{self.port}에서 대기 중")

            while True:
                conn, addr = self.server_socket.accept()
                self.clients.append(conn)
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.start()
                print(f"{addr}에 대한 스레드 시작됨")
        except Exception as e:
            print(f"서버 에러: {e}")
        finally:
            self.close_server()

    def close_server(self):
        print("스토어 서버 소켓 닫는 중")
        for conn in self.clients:
            conn.close()
        self.server_socket.close()
        print("스토어 서버 소켓 닫힘")

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
            print(f"상점 상태 업데이트 중 오류 발생: {e}")
            return None
        

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
            print(f"상점 상태 업데이트 중 오류 발생: {e}")
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

