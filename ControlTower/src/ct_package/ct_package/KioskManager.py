import socket
import threading
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


from TcpNode import *

class KioskServer():
    def __init__(self, host, port, s_clients, node):
        self.host = host
        self.port = port
        self.clients = []
        self.s_clients = s_clients  # 전달된 clients 리스트 사용
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.node = node


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
                        print(data)

                        if cmd == 'OR':
                            target_ip = '192.168.0.10' #나중에 SQL로 가져오기
                            target_client = None
                            for client in self.s_clients:
                                if client.getpeername()[0] == target_ip:
                                    target_client = client
                                    break
                            if target_client:
                                print(f"Sending data to {target_ip}")
                                data_list = data.split(',')
                                cnt = int(data_list[-1])
                                menus = data_list[2:2 + cnt]
                                msg = f"OS,{data_list[0]},{cnt},{','.join(menus)}"
                                target_client.sendall(msg.encode())
                            else:
                                print(f"No client found with IP {target_ip}")
                                conn.sendall(f"No client found with IP {target_ip}".encode())
                            data_list = data.split(',')
                            self.node.send_request(data_list[0]) #-->print 대신 send_request
                        elif cmd == 'GD':
                            print("출발시간요청 : ")
                            print(data)
                            #data=UID 로 추후에 SQL로 탑승시간 보내주기
                            conn.sendall("09:25".encode())
                        elif cmd == 'OI':
                            print("주문 조회 : ")
                            print(data)
                            #data=UID 로 추후에 SQL로 주문조회 보내주기
                            msg = "A1,2"
                            conn.sendall(msg.encode())
                        else:
                            print("else")
                            conn.sendall("else".encode())
                        
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
