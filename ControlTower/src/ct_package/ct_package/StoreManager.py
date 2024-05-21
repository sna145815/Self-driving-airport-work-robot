import socket
import threading


class StoreServer:
    def __init__(self, host, port, clients,db_manager):
        self.host = host
        self.port = port
        self.clients = clients  # 전달된 clients 리스트 사용
        self.db_manager = db_manager
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def handle_client(self, conn, addr):
        print(f"Connected by {addr}")
        try:
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        print(self.clients)
                        if not data:
                            break
                        print(f"Received from {addr}: {data.decode()}")
                        cmd, data = data.decode().split(',', 1)
                        data_list = data.split(',')
                        print(data_list)
                        if cmd == 'DR':
                            print(data_list[0]+"매장에서 / "+data_list[1]+"주문"+"배차요청!!!")
                            conn.sendall("배차 해드릴게요~!".encode())
                        elif cmd == 'MS':
                            if data_list[1] == '0':
                                print(data_list[0]+"Open")
                                msg = data_list[0]+ "open 할게요~!"
                                conn.sendall(msg.encode())
                            else:
                                print(data_list[0]+"close")
                                msg = data_list[0]+ "close 할게요~!"
                                conn.sendall(msg.encode())
                        elif cmd == 'SS':
                            if data_list[1] == '0':
                                print(data_list[0]+"Open")
                                msg = data_list[0]+ "open 할게요~!"
                                conn.sendall(msg.encode())
                            else:
                                print(data_list[0]+"close")
                                msg = data_list[0]+ "close 할게요~!"
                                conn.sendall(msg.encode())

                        data_list.clear()
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
        print("Closing Store server socket")
        for conn in self.clients:
            conn.close()
        self.server_socket.close()
        print("Store server socket closed")