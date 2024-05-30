import sys
import socket
import json
import rclpy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem,QDialog
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen
from PyQt5 import uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt,QTimer

# TCP 클라이언트 클래스
class TCPClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def connect(self):
        self.client_socket.connect((self.host, self.port))
        print(f"Connected to server at {self.host}:{self.port}")
    
    def send_order_no(self, order_no):
        self.client_socket.sendall(order_no.encode())

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
    log_received = pyqtSignal(dict)

    def __init__(self, client):
        super().__init__()
        self.client = client
    
    def run(self):
        for data in self.client.receive_data():
            if 'robot_logs' in data:
                self.log_received.emit(data)
                print(data)
            else:
                self.data_received.emit(data)

class Logmodal(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("log.ui",self)
        self.show()

    def update_table_data(self, table_widget, data):
        table_widget.setRowCount(0)  # 기존 데이터를 지웁니다.
        table_widget.setRowCount(len(data))
        for row, row_data in enumerate(data):
            for col, value in enumerate(row_data):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                table_widget.setItem(row, col, item)

# 메인 윈도우 클래스
class MainWindow(QMainWindow, from_class):
    def __init__(self, host, port):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        self.host = host
        self.port = port

        self.robots = {
                        "R-1": (190, 610),
                        "R-2": (610, 610),
                        "R-3": (1055, 610)
                    }
        
        # TCP 클라이언트 초기화 및 연결
        self.client = TCPClient(self.host, self.port)
        self.client.connect()

        # 데이터 수신 스레드 시작
        self.receive_thread = ReceiveThread(self.client)
        self.receive_thread.data_received.connect(self.update_status)
        self.receive_thread.log_received.connect(self.get_robot_log)
        self.receive_thread.start()

        self.CurOrder.itemClicked.connect(self.send_order_no)
        self.FinishOrder.itemClicked.connect(self.send_order_no)
        self.UnproOrder.itemClicked.connect(self.send_order_no)

        # self.position1 = self.create_subscription(PoseStamped,'/amcl_pose_1',self.position_callback1,1)
        # self.position2 = self.create_subscription(PoseStamped,'/amcl_pose_2',self.position_callback2,1)
        # self.position3 = self.create_subscription(PoseStamped,'/amcl_pose_3',self.position_callback3,1)

        # 이미지 로드 및 QLabel에 설정
        self.pixmap = QPixmap('map.png')
        self.Map.setPixmap(self.pixmap)
        self.Map.setScaledContents(True)

        # 로봇 이미지 로드 및 크기 조정
        self.robot_pixmap = QPixmap('robot.jpg').scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.robot_positions = [(190, 610), (610, 610), (1055, 610)]  # 초기 위치
        self.robot_paths = [[], [],[]]  # 3로봇의 경로

        # 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_positions)
        self.timer.start(100)  # 0.1초마다 업데이트

        #경로 지우기 타이머
        self.clear_path_timer = QTimer()
        self.clear_path_timer.timeout.connect(self.clear_robot_paths)
        self.clear_path_timer.start(20000)  # 10초마다 경로 지우기


    
    # 테이블 위젯에 데이터 추가
    def update_status(self, data):
        robot_status = data.get('robot_status')
        unprocessed_orders = data.get('unprocessed_orders')
        cur_orders = data.get('cur_orders')
        finish_orders = data.get('finish_orders')
        
        # 테이블 위젯에 데이터 추가
        self.update_table_data(self.RobotStatus, robot_status)
        self.update_table_data(self.UnproOrder, unprocessed_orders)
        self.update_table_data(self.FinishOrder, finish_orders)
        self.update_table_data(self.CurOrder, cur_orders)
        
    
    def get_robot_log(self,data):
        self.logmodal(data.get('robot_logs'))


    # 테이블 위젯에 데이터 추가하는 메서드
    def update_table_data(self, table_widget, data):
        table_widget.setRowCount(0)  # 기존 데이터를 지웁니다.
        table_widget.setRowCount(len(data))
        for row, row_data in enumerate(data):
            for col, value in enumerate(row_data):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                table_widget.setItem(row, col, item)

    def send_order_no(self, item):
        row = item.row()
        column = item.column()

        # 클릭된 셀의 행과 열 인덱스 확인
        if self.CurOrder == item.tableWidget() and self.CurOrder.horizontalHeaderItem(column).text() == "OrderNo":
            order_no = self.CurOrder.item(row, column).text()
        elif self.FinishOrder == item.tableWidget() and self.FinishOrder.horizontalHeaderItem(column).text() == "OrderNo":
            order_no = self.FinishOrder.item(row, column).text()
        elif self.UnproOrder == item.tableWidget() and self.UnproOrder.horizontalHeaderItem(column).text() == "OrderNo":
            order_no = self.UnproOrder.item(row, column).text()

        self.client.send_order_no(order_no)

    def logmodal(self,data):
        window_2 = Logmodal()
        window_2.update_table_data(window_2.RobotLog,data)
        window_2.exec_()

    def position_callback1(self, msg):
        position = msg.pose.pose.position
        self.robots["R-1"]=(position.x,position.y)

    def position_callback2(self, msg):
        position = msg.pose.pose.position
        self.robots["R-2"]=(position.x,position.y)

    def position_callback3(self, msg):
        position = msg.pose.pose.position
        self.robots["R-3"]=(position.x,position.y)   

    def update_robot_positions(self):
        new_positions = []

        for robot_id, (r_x, r_y) in self.robots.items():
            new_x = r_x % self.pixmap.width()
            new_y = r_y % self.pixmap.height()
            new_positions.append((new_x, new_y))

        # 이동 경로 저장
        for i, (old_pos, new_pos) in enumerate(zip(self.robot_positions, new_positions)):
            # 이동 경로를 저장하기 전에 각 리스트의 길이를 비교하여 더 짧은 리스트의 길이에 맞춰서만 저장합니다.
            # if i < len(self.robot_paths):
            self.robot_paths[i].append((old_pos, new_pos))

        # 로봇 위치 갱신
        self.robot_positions = new_positions
        
        # 맵 갱신
        self.update_map()

    def update_map(self):
        # 맵 이미지를 새로 그리기
        pixmap = self.pixmap.copy()
        painter = QPainter(pixmap)
        
        # 로봇 이동 경로를 빨간색으로 그리기
        pen = QPen(QColor(255, 0, 0))  # 빨간색 펜
        pen.setWidth(3)  # 선 두께 설정
        painter.setPen(pen)
        for paths in self.robot_paths:
            for path in paths:
                painter.drawLine(path[0][0], path[0][1], path[1][0], path[1][1])
        
        # 로봇 위치를 그리기
        for pos in self.robot_positions:
            painter.drawPixmap(pos[0] - self.robot_pixmap.width() // 2, pos[1] - self.robot_pixmap.height() // 2, self.robot_pixmap)
        
        painter.end()
        self.Map.setPixmap(pixmap)

    def clear_robot_paths(self):
        # 이동 경로 지우기
        self.robot_paths = [[], []]
        self.update_map()  # 맵 갱신

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow('192.168.0.8', 9034)  # 호스트 및 포트를 적절히 수정
    window.show()
    sys.exit(app.exec_())
