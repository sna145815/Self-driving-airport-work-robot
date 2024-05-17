import os
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
import serial
import struct
from resources_rc import *  # resources_rc.py를 import
import socket
import select
import time

class SerialReceiver(QThread):
    detected = pyqtSignal(bytes)
    recvTotal = pyqtSignal(int)
    changedTotal = pyqtSignal()

    def __init__(self, conn, parent=None):
        super(SerialReceiver, self).__init__(parent)
        self.is_running = False
        self.conn = conn
        print("recv init")
        
    def run(self):
        print("recv start")
        self.is_running = True
        while self.is_running:
            if self.conn.readable():
                res = self.conn.read_until(b'\n')
                if len(res) > 0:
                    res = res[:-2]
                    cmd = res[:2].decode()
                    if cmd == 'GS' and res[2] == 0:
                        print("recv detected")
                        self.detected.emit(res[3:])
                    elif cmd == 'GT' and res[2] == 0:
                        print("recvTotal")
                        print(len(res))
                        self.recvTotal.emit(int.from_bytes(res[3:7], 'little'))
                    elif cmd == 'ST' and res[2] == 0:
                        print("setTotal22")
                        self.changedTotal.emit()
                    else:
                        print("unknown error")
                        print(cmd)

    def stop(self):
        print("recv stop")
        self.is_running = False
        
        
class TCPClient:
    def __init__(self, host='192.168.0.36', port=9021):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))

    def tcp_send_data(self, cmd, data):
        message = f"{cmd},{data}"
        self.client_socket.send(message.encode('utf-8'))

        # select를 사용하여 소켓에서 읽을 수 있는 데이터가 도착할 때까지 대기
        ready, _, _ = select.select([self.client_socket], [], [], 10)
        if self.client_socket in ready:
            response = self.client_socket.recv(1024).decode('utf-8')
            print(f"Server response: {response}")
        else:
            print("No response from server")

    def close_connection(self):
        self.client_socket.close()


class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__()
        self.main_window = uic.loadUi('../data/main.ui')
        self.main_window.show()
        
        self.main_window.orderBtn.hide()
        self.main_window.checkBtn.hide()
        
        self.send_enabled = True

        self.uid = bytes(4)
        self.conn = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        self.recv = SerialReceiver(self.conn)
        self.recv.start()
        self.recv.detected.connect(self.detected)

        self.main_window.orderBtn.clicked.connect(self.go_store)
        self.main_window.checkBtn.clicked.connect(self.go_check)

        self.timer = QTimer()
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.getStatus)
        self.timer.start()
        
    def send(self, command, data=0):
        print("send")
        req_data = struct.pack('<2s4sic', command, self.uid, data, b'\n')
        self.conn.write(req_data)
        return

    def getStatus(self):
        if self.send_enabled:
            self.send(b'GS')
        return
    
    def detected(self, data):
        self.send_enabled = False
        
        uid = data 
        uid_hex = " ".join("{:02X}".format(b) for b in uid)
        print("Detected UID:", uid_hex)
        self.uid = uid_hex  # 감지된 UID 값을 할당

        self.recv.stop()
        
        self.main_window.textEdit_5.hide()
        self.main_window.textEdit_6.hide()
        self.main_window.orderBtn.show()
        self.main_window.checkBtn.show()
        return  
    
    def go_store(self):
        storePage = StoreWindow(self.main_window)  
        storePage.show()
        self.main_window.hide()
        

    def go_check(self):
        checkPage =  CheckWindow(self.main_window)  
        checkPage.show()
        self.main_window.hide()
        

class StoreWindow(QMainWindow):
    def __init__(self, main_window):  # 메인 창의 인스턴스를 받는 매개변수
        super().__init__(main_window)
        self.main_window = main_window
        self.store_window = uic.loadUi('../data/store.ui',self)
        
        self.koreaBtn.clicked.connect(self.updateStore)
        self.chinaBtn.clicked.connect(self.updateStore)
        self.japanBtn.clicked.connect(self.updateStore)
        self.westernBtn.clicked.connect(self.updateStore)
        self.snackBtn.clicked.connect(self.updateStore)
        self.backBtn.clicked.connect(self.back)
        self.basketBtn.clicked.connect(self.go_basket)
        
    def updateStore(self):
        return

    def back(self):
        self.close()
        self.main_window.show()
        return
    
    def go_basket(self):
        basketPage = BasketWindow(self.store_window)  
        basketPage.show()
        self.store_window.hide()
        return


class CheckWindow(QMainWindow):
    def __init__(self, main_window):
        super().__init__(main_window)
        

class BasketWindow(QMainWindow):
    def __init__(self, store_window):
        super().__init__(store_window)
        self.store_window = store_window
        self.basket_window = uic.loadUi('../data/basket.ui',self)
        
        self.backBtn.clicked.connect(self.back)
        self.payBtn.clicked.connect(self.go_pay)
        self.menulist = ["coffe/3","icetea/2"]
    
    def go_pay(self):
        self.close()
        payPage = PayWindow(self.basket_window, self.menulist)
        payPage.show()
        self.basket_window.hide()
        
    def back(self):
        self.close()
        self.store_window.show()
        return    
    
    

        

class PayWindow(QMainWindow):
    def __init__(self, basket_window, menulist):
        super().__init__(basket_window)
        self.basket_window = basket_window
        self.menulist = menulist
        self.pay_window = uic.loadUi('../data/pay.ui',self)
        self.payBtn.hide()
        
        self.uid = bytes(4)
        self.conn = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        self.recv = SerialReceiver(self.conn)
        self.recv.start()
        self.recv.detected.connect(self.detected)
        
        self.tcp_client = TCPClient()  # TCP 클라이언트 생성
        
        self.send_enabled = True
        self.backBtn.clicked.connect(self.back)
        self.payBtn.clicked.connect(self.send_data)  # Serial 통신 테스트용

        self.timer = QTimer()
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.getStatus)
        self.timer.start()
    
    def send(self, command, data=0):
        print("send")
        req_data = struct.pack('<2s4sic', command, self.uid, data, b'\n')
        self.conn.write(req_data)
        return
    
    def getStatus(self):
        if self.send_enabled:
            self.send(b'GS')
        return
        
    def detected(self, data):
        self.send_enabled = False
        
        uid = data 
        uid_hex = " ".join("{:02X}".format(b) for b in uid)
        print("Detected UID:", uid_hex)
        self.uid = uid_hex  # 감지된 UID 값을 할당

        self.recv.stop()
        self.payBtn.show()
        
    def send_data(self):
        print("send_data called")
        cmd = "OR"
        # menulist를 콤마로 구분된 문자열로 변환
        menulist_str = ",".join(self.menulist)
        # 데이터를 하나의 문자열로 결합
        data = f"{self.uid},K-1,{menulist_str},{len(self.menulist)}"
        print(f"Sending data: {cmd},{data}")
        self.tcp_client.tcp_send_data(cmd, data)
        

    def back(self):
        self.close()
        self.recv.stop()  # Receiver 클래스의 스레드 중지
        self.basket_window.show()
        return



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())
