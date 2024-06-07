import socket
import threading

# from kioskGUISrv import *

class TCPClient:
    def __init__(self, host, port):
        self.server_address = host
        self.server_port = port
        self.socket = None
        self.connected = False
        self.connect()

        # 시작 시 receive_thread 실행
        self.receive_thread = threading.Thread(target=self.receive)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.data_callback = None

    def set_response_callback(self, callback):
        print("Setting response callback")
        self.data_callback = callback

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_address, self.server_port))
            self.connected = True
            print("Connected to server:", self.server_address, "port:", self.server_port)
        except Exception as e:
            print("Error:", e)

    def send(self, cmd, data):
        message = f"{cmd},{data}"
        try:
            if self.socket:
                self.socket.send(message.encode())
                print("Data sent:", message)
            else:
                print("Socket is not connected.")
        except Exception as e:
            print("Error:", e)

    def receive(self):
        while self.connected:
            try:
                if self.socket:
                    response = self.socket.recv(1024).decode('utf-8')
                    if response:
                        print("Response received:", response)
                        cmd, data = response.split(',', 1)
                        if cmd == 'SS':
                            self.update_status(cmd, data)
                        elif cmd == 'MS':
                            self.update_status(cmd, data)
                        elif cmd == 'GD' or cmd == 'OI':
                            if self.data_callback:
                                print("Calling data callback with data:", data)
                                self.data_callback(data)
                    else:
                        print("No response received")
                else:
                    print("Socket is not connected.")
            except Exception as e:
                print("Error:", e)
                self.connected = False
                self.close()
            except KeyboardInterrupt:
                self.close()

    def update_status(self, cmd, data):
        parts = data.split(',')
        if cmd == 'SS':  # Store Status
            pass
        elif cmd == 'MS':  # Menu Status
            pass
        else:
            print("error")

    def close(self):
        self.connected = False
        try:
            if self.socket:
                self.socket.close()
                print("Connection closed.")
        except Exception as e:
            print("Error:", e)

