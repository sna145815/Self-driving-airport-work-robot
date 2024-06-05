import sys
import socket
import json
import yaml
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem,QDialog
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen,QTransform
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
    position_received = pyqtSignal(str, dict)

    def __init__(self, client):
        super().__init__()
        self.client = client
    
    def run(self):
        for data in self.client.receive_data():
            if 'robot_logs' in data:
                self.log_received.emit(data)
            elif 'robot_id' in data:  # 'robot_id' 키를 확인하여 로봇 위치 데이터를 받았을 때
                robot_id = data['robot_id']
                self.position_received.emit(robot_id, data)  # 로봇 ID와 위치 데이터를 함께 전송
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
        table_widget.resizeColumnsToContents()

# 메인 윈도우 클래스
class MainWindow(QMainWindow, from_class):
    def __init__(self, host, port):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        self.host = host
        self.port = port

        self.robots = {
                        "R-1": (0.03, 2.5),
                        "R-2": (0.03, 1.6),
                        "R-3": (0.03, 0.6)
                    }
        
        # TCP 클라이언트 초기화 및 연결
        self.client = TCPClient(self.host, self.port)
        self.client.connect()

        # 데이터 수신 스레드 시작
        self.receive_thread = ReceiveThread(self.client)
        self.receive_thread.data_received.connect(self.update_status)
        self.receive_thread.log_received.connect(self.get_robot_log)
        self.receive_thread.position_received.connect(self.position)
        self.receive_thread.start()

        self.CurOrder.itemClicked.connect(self.send_order_no)
        self.FinishOrder.itemClicked.connect(self.send_order_no)
        self.UnproOrder.itemClicked.connect(self.send_order_no)

        yaml_file_path = "map.yaml"

        # YAML 파일 읽기
        with open(yaml_file_path, "r") as file:
            map_yaml_data = yaml.safe_load(file)
        
        self.pixmap = QPixmap(map_yaml_data['image'])
        self.Map.setPixmap(self.pixmap)
        self.Map.setScaledContents(True)

        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 6
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.Map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
    
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]
        self.update_map()


    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            pos = event.pos()
            label_pos = self.Map.mapFromParent(pos)
            print("Mouse clicked at label position:", label_pos.x(), ",", label_pos.y())

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

    def position(self, robot_id, data):
        if robot_id == "R-1":
            self.robots["R-1"] = (data["x"], data["y"])
        elif robot_id == "R-2":
            self.robots["R-2"] = (data["x"], data["y"])
        elif robot_id == "R-3":
            self.robots["R-3"] = (data["x"], data["y"])
        self.update_map()

        

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
            if i < len(self.robot_paths):
                self.robot_paths[i].append((old_pos, new_pos))

        # 로봇 위치 갱신
        self.robot_positions = new_positions
        
        # 맵 갱신
        self.update_map()

    def update_map(self):
        self.Map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))

        painter = QPainter(self.Map.pixmap())

        for robot_id, position in self.robots.items():
            x, y = self.calc_grid_position(position[0], position[1])

            # 로봇 표시
            if robot_id == "R-1":
                painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
                painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
                painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), robot_id)
            elif robot_id == "R-2":
                painter.setPen(QPen(Qt.green, 20, Qt.SolidLine))
                painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
                painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), robot_id)
            elif robot_id == "R-3":
                painter.setPen(QPen(Qt.blue, 20, Qt.SolidLine))
                painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
                painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), robot_id)

        painter.end()

    
    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return pos_x, pos_y
    


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow('192.168.0.15', 9035)  # 호스트 및 포트를 적절히 수정
    window.show()
    sys.exit(app.exec_())
