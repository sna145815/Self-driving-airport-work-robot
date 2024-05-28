import sys
import socket
import json
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtGui import QPixmap
from PyQt5 import uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt

# TCP 클라이언트 클래스
class TCPClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def connect(self):
        self.client_socket.connect((self.host, self.port))
        print(f"Connected to server at {self.host}:{self.port}")
    
    def receive_data(self):
        try:
            while True:
                # 데이터 수신
                data = self.client_socket.recv(4096)
                if not data:
                    break
                
                # 바이트를 문자열로 디코딩
                data_str = data.decode('utf-8')
                
                # JSON 문자열을 Python 사전으로 역직렬화
                data_dict = json.loads(data_str)
                
                # 수신된 데이터 처리
                yield data_dict
        
        except Exception as e:
            print(f"Error receiving data: {e}")
        
        finally:
            self.client_socket.close()

# UI 파일 로드
from_class = uic.loadUiType("ControlGUI.ui")[0]

# 데이터를 받는 스레드
class ReceiveThread(QThread):
    data_received = pyqtSignal(dict)

    def __init__(self, client):
        super().__init__()
        self.client = client
    
    def run(self):
        for data in self.client.receive_data():
            print(data)
            self.data_received.emit(data)

# 메인 윈도우 클래스
class MainWindow(QMainWindow, from_class):
    def __init__(self, host, port):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        self.host = host
        self.port = port
        
        # TCP 클라이언트 초기화 및 연결
        self.client = TCPClient(self.host, self.port)
        self.client.connect()
        
        # 이미지 로드 및 QLabel에 설정
        pixmap = QPixmap('map.png')
        self.Map.setPixmap(pixmap)
        self.Map.setScaledContents(True)

        # 데이터 수신 스레드 시작
        self.receive_thread = ReceiveThread(self.client)
        self.receive_thread.data_received.connect(self.update_labels)
        self.receive_thread.start()
    
    # 테이블 위젯에 데이터 추가
    def update_labels(self, data):
        robot_status = data.get('robot_status')
        unprocessed_orders = data.get('unprocessed_orders')
        robot_logs = data.get('robot_logs')
        
        # 테이블 위젯에 데이터 추가
        self.update_table_data(self.RobotStatus, robot_status)
        self.update_table_data(self.OrderStatus, unprocessed_orders)
        self.update_table_data(self.RobotLog, robot_logs)

    # 테이블 위젯에 데이터 추가하는 메서드
    def update_table_data(self, table_widget, data):
        table_widget.setRowCount(len(data))
        for row, row_data in enumerate(data):
            for col, value in enumerate(row_data):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                table_widget.setItem(row, col, item)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow('192.168.1.105', 9077)  # 호스트 및 포트를 적절히 수정
    window.show()
    sys.exit(app.exec_())
