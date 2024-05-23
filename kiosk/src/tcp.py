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
        
        self.update_callback = None
    
    # def set_update_callback(self, callback):
    #     self.update_callback = callback
    
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
                        print(response)
                        cmd, data = response.split(',', 1)
                        if cmd == 'SS':
                            self.update_status(cmd, data)
                            print(data)
                                
                        elif cmd == 'MS':
                            self.update_status(cmd, data)
                            
                            print(data)
                            
                        elif cmd == 'GD':
                            print(data)
                        elif cmd == 'OI':
                            print(data)
                else:
                    print("Socket is not connected.")
            except Exception as e:
                print("Error:", e)
                self.connected = False

    def update_status(self, cmd, data):
        parts = data.split(',')

        if cmd == 'SS':  # Store Status
            # store_id = parts[0]
            # status = parts[1]
            # if store_id in StoreDict:
            #     for menu_item in StoreDict[store_id]:
            #         for menu_id, menu_info in menu_item.items():
            #             menu_info["status"] = status  # Update status for all menus in the store
            pass

        elif cmd == 'MS':  # Menu Status
            # menu_id = parts[0]
            # status = parts[1]
            # for store_id in StoreDict:
            #     for menu_item in StoreDict[store_id]:
            #         if menu_id in menu_item:
            #             menu_item[menu_id]["status"] = status  # Update status for the specific menu
            pass

        else:
            print("error")
        
        # if self.update_callback:
        #     self.update_callback()  # 상태 업데이트 콜백 호출
            
    def close(self):
        self.connected = False
        try:
            if self.socket:
                self.socket.close()
                print("Connection closed.")
        except Exception as e:
            print("Error:", e)
