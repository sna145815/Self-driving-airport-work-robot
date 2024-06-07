import os
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
import serial
import struct
import socket
import threading
import time
import resources_rc 
import select
from collections import Counter
from tcp import TCPClient
from Serial import SerialReceiver
import datetime
HOST = '192.168.1.102'
PORT = 9052

StoreDict = {
    "S-1": [
        {"AM-1": {"name": "cheeseBurger", "price": "1,500", "require_time": "5", "description": "유재상의 맛집", "status" : "1"}},
        {"AM-2": {"name": "bigMac","price": "2,500", "require_time": "1",  "description": "고독한 외식가 유재상", "status" : "1"}},
        {"AM-3": {"name": "beefSnackWrap","price": "1,500", "require_time": "3",  "description": "고독한 외식가 유재상", "status" : "1"}},
        {"AM-4": {"name": "McNuggets","price": "4,500", "require_time": "16",  "description": "고독한 외식가 유재상", "status" : "1"}},
        {"AM-5": {"name": "coffee","price": "7,500", "require_time": "20",  "description": "고독한 외식가 유재상", "status" : "1"}},
        {"AM-6": {"name": "coke","price": "2,500", "require_time": "15",  "description": "고독한 외식가 유재상", "status" : "1"}},
        {"AM-7": {"name": "sprite","price": "3,500",  "require_time": "3", "description": "고독한 외식가 유재상", "status" : "1"}},
    ],
    "S-2": [
        {"BM-1": {"name": "americano","price": "3,500",  "require_time": "5", "description": "유재상의 맛집", "status" : "1"}},
        {"BM-2": {"name": "CaramelMacchiato","price": "3,500",  "require_time": "5", "description": "유재상의 맛집", "status" : "1"}},
        {"BM-3": {"name": "Cappuccino","price": "7,500",  "require_time": "6", "description": "유재상의 맛집", "status" : "1"}},
        {"BM-4": {"name": "coldBrew","price": "2,500",  "require_time": "5", "description": "유재상의 맛집", "status" : "1"}},
        {"BM-5": {"name": "GHBT","price": "3,500",  "require_time": "5", "description": "유재상의 맛집", "status" : "1"}},
        {"BM-6": {"name": "frappuccino","price": "13,500",  "require_time": "7", "description": "유재상의 맛집", "status" : "1"}},
    ],
    "S-3": [
        {"CM-1": {"name": "kimbab","price": "123,500",  "require_time": "15", "description": "유재상의 맛집", "status" : "1"}},
        {"CM-2": {"name": "tunaKimbab","price": "33,500",  "require_time": "25", "description": "고독한 외식가 유재상", "status" : "1"}},
        {"CM-3": {"name": "cheeseKimbab","price": "43,500",  "require_time": "12", "description": "고독한 외식가 유재상", "status" : "1"}},
        {"CM-4": {"name": "beefKimbab","price": "13,500",  "require_time": "3", "description": "고독한 외식가 유재상", "status" : "1"}},
        {"CM-5": {"name": "porkKimbab","price": "33,500",  "require_time": "8", "description": "고독한 외식가 유재상", "status" : "1"}},
        {"CM-6": {"name": "shrimpKimbab","price": "34,500",  "require_time": "10", "description": "고독한 외식가 유재상", "status" : "1"}},
    ],
}


class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        recv = kwargs.pop('recv', None)
        super(MainWindow, self).__init__()
        self.main_window = uic.loadUi('./data/main.ui')
        # self.main_window.showFullScreen()
        self.main_window.show()

        self.main_window.orderBtn.hide()
        self.main_window.checkBtn.hide()

        self.send_enabled = True
        self.uidstatus = False
        self.conn = None
        self.recv = recv
        self.response = None

        # TCPClient 인스턴스 생성 및 콜백 함수 설정
        self.tcp_server = TCPClient(HOST, PORT)
        self.tcp_server.set_response_callback(self.handle_tcp_response)

        # RFID Serial communication
        self.uid = bytes(4)

        try:
            self.conn = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)

        self.start_serial_receiver()

        self.recv.detected.connect(self.detected)
        self.main_window.orderBtn.clicked.connect(self.go_store)
        self.main_window.checkBtn.clicked.connect(self.go_check)

        self.timer = QTimer()
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.get_status)
        self.timer.start()

    def handle_tcp_response(self, response):
        # Handle the received response      
        print("tcp_response received")
        self.response = response
        print(f"Response: {response}")

    def start_serial_receiver(self):
        if self.recv is None:
            self.recv = SerialReceiver(self.conn)
            self.recv.start()
        else:
            self.recv.resume()

    def send_data(self):
        print("send_data called")
        cmd = "GD"
        data = f"{self.uid}"
        print(f"Sending data: {cmd},{data}")
        self.tcp_server.send(cmd, data)

    def send(self, command, data=0):
        print("send")
        try:
            req_data = struct.pack('<2s4sic', command, self.uid, data, b'\n')
            self.conn.write(req_data)
        except Exception as e:
            print(f"Error sending data via serial: {e}")

    def get_status(self):
        if self.send_enabled:
            self.send(b'GS')

    def detected(self, data):
        self.send_enabled = False
        uid = data 
        uid_hex = " ".join("{:02X}".format(b) for b in uid)
        print("Detected UID:", uid_hex)
        self.uid = uid_hex
        self.uidstatus = True
        self.recv.pause()

        self.send_data()
        self.main_window.textEdit_5.hide()
        self.main_window.textEdit_6.hide()

        self.main_window.orderBtn.show()
        self.main_window.checkBtn.show()

    def go_store(self):
        if self.uidstatus:
            storePage = StoreWindow(self.main_window, self.recv, self.tcp_server, self.response)  
            storePage.show()
            self.main_window.hide()

    def go_check(self):
        self.main_window.textEdit_5.show()
        if self.uidstatus:
            checkPage = CheckWindow(self.main_window, self.tcp_server, self.uid)  
            checkPage.show()
            self.main_window.hide()


class StoreWindow(QMainWindow):
    def __init__(self, main_window, recv, tcp_server, department):
        super().__init__(main_window)
        self.main_window = main_window
        self.store_window = uic.loadUi('./data/store.ui', self)
        self.tcp_server = tcp_server
        self.recv = recv
        self.department = department
        
        self.store_window.workTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.current_time = datetime.datetime.now().strftime("%H:%M")

        # 매장 정보를 저장할 변수
        self.selected_region = None
        self.selected_store = None
        self.selected_menu = None  # 선택된 메뉴를 저장할 변수
        self.menu = []  # 장바구니에 담긴 메뉴를 저장할 리스트

        # 버튼 시그널 연결
        self.store_window.koreaBtn.clicked.connect(self.show_korean_stores)
        self.store_window.chinaBtn.clicked.connect(self.show_chinese_stores)
        self.store_window.japanBtn.clicked.connect(self.show_japanese_stores)
        self.store_window.westernBtn.clicked.connect(self.show_western_stores)
        self.store_window.snackBtn.clicked.connect(self.show_snack_stores)
        self.store_window.basketBtn.clicked.connect(self.go_basket)
        self.store_window.backBtn.clicked.connect(self.back)
        self.store_window.loadBtn.clicked.connect(self.add_selected_to_basket)  # 담기 버튼 시그널 변경
        
        self.workTable.cellClicked.connect(self.cell_clicked)
        self.recv.stop()
    #     self.process_response(self.department)

    # def process_response(self, response):
    #     # "HH:MM" 형식의 시간 문자열을 받아 처리
    #     if response:
    #         self.current_time = response.strip()
    #         print(f"Received time: {self.current_time}")
    #     else:
    #         pass

    def go_basket(self):
        basketPage = BasketWindow(self.main_window, self.tcp_server, self.menu, self.selected_store)  
        basketPage.show()
        self.store_window.hide()
    
    def add_selected_to_basket(self):
        if self.selected_menu is not None:
            # Check if the menu is already in the basket
            found = False
            for i, item in enumerate(self.menu):
                if self.selected_menu in item:
                    # Increment the count if the menu is already in the basket
                    count = int(item.split('/')[1]) + 1
                    self.menu[i] = f"{self.selected_menu}/{count}"
                    found = True
                    break
            if not found:
                # Add the menu to the basket with count 1 if it's not already in the basket
                self.menu.append(f"{self.selected_menu}/1")

            print("장바구니에 메뉴가 추가되었습니다:", self.menu)  # 장바구니에 추가된 메뉴 확인용 출력
        else:
            print("추가할 메뉴를 선택해주세요.")

    def cell_clicked(self, item):
        # 테이블 셀 클릭 시 호출되는 함수
        selected_row = item  # 선택된 행 번호 가져오기

        # 선택된 행의 메뉴 이름 가져오기
        selected_menu = self.store_window.workTable.item(selected_row, 1).text()
        print("선택된 메뉴:", selected_menu)  # 선택된 메뉴 확인용 출력

        # 선택된 메뉴를 self.selected_menu에 저장
        self.selected_menu = selected_menu
        if self.selected_menu is not None:
            print("선택된 메뉴:", self.selected_menu)  # 선택된 메뉴 확인용 출력
        else:
            print("메뉴를 선택해주세요.")
            
    def update_store(self):
        # 선택된 지역에 해당하는 매장 정보 가져오기
        store_data = StoreDict.get(self.selected_store, [])

        current_time_minutes = self.time_to_minutes(self.current_time)
        department_time_minutes = self.time_to_minutes(self.department)

        # status가 "1"이고 출발시간 이후인 메뉴들만 필터링
        filtered_data = [
            menu_info for menu_info in store_data
            for menu_id, menu_detail in menu_info.items()
            if menu_detail.get("status") == "1" and 
            department_time_minutes - current_time_minutes > int(menu_detail.get("require_time"))
        ]

        # 테이블 초기화
        self.store_window.workTable.clearContents()
        self.store_window.workTable.setRowCount(len(filtered_data))
        self.store_window.workTable.setColumnCount(5)  # 열 개수 수정

        # 테이블에 데이터 채우기
        for row, menu_info in enumerate(filtered_data):
            for menu_id, menu_detail in menu_info.items():
                # 메뉴 정보에서 필요한 데이터 가져오기
                name = menu_detail.get("name", "")
                price = menu_detail.get("price", "")
                require_time = menu_detail.get("require_time", "")
                description = menu_detail.get("description", "")

                # 테이블에 데이터 삽입
                self.store_window.workTable.setItem(row, 0, QTableWidgetItem(self.selected_store))
                self.store_window.workTable.setItem(row, 1, QTableWidgetItem(name))
                self.store_window.workTable.setItem(row, 2, QTableWidgetItem(price))
                self.store_window.workTable.setItem(row, 3, QTableWidgetItem(str(require_time)))  # int 형식을 문자열로 변환하여 설정
                self.store_window.workTable.setItem(row, 4, QTableWidgetItem(description))


    def time_to_minutes(self, time_str):
        h, m = map(int, time_str.split(":"))
        return h * 60 + m
    
    def back(self):
        self.close()
        self.main_window.show()
        
    def show_korean_stores(self):
        # 한식 매장 목록 표시
        self.selected_region = "한식"
        self.selected_store = "S-1"
        self.update_store()
        
    def show_snack_stores(self):
        # 간식 매장 목록 표시
        self.selected_region = "간식"
        self.selected_store = "S-3"
        self.update_store()

    def show_western_stores(self):
        # 양식 매장 목록 표시
        self.selected_region = "양식"
        self.selected_store = "S-2"
        self.update_store()

    def show_chinese_stores(self):
        # 중식 매장 목록 표시
        self.selected_region = "중식"
        self.selected_store = "S-4"
        self.update_store()

    def show_japanese_stores(self):
        # 일식 매장 목록 표시
        self.selected_region = "일식"
        self.selected_store = "S-5"
        self.update_store()





class CheckWindow(QMainWindow):
    def __init__(self, main_window, tcp_server, uid):
        super().__init__(main_window)
        self.main_window = main_window
        self.uid = uid
        self.tcp_server = tcp_server
        
        self.check_window = uic.loadUi('./data/check.ui', self)
        self.backBtn.clicked.connect(self.back)
        
        self.send_data()
        
    def send_data(self):
        print("send_data called")
        cmd = "OI"
        data = f"{self.uid}"
        print(f"Sending data: {cmd},{data}")
        self.tcp_server.send(cmd, data)
    
    def back(self):
        self.close()
        self.new_main = MainWindow(self)


class BasketWindow(QMainWindow):
    def __init__(self, store_window, tcp_server, selected_menulist, selected_store):
        super().__init__(store_window)
        self.store_window = store_window
        self.tcp_server = tcp_server
        self.selected_menulist = selected_menulist
        self.basket_window = uic.loadUi('./data/basket.ui', self)
        self.selected_store = selected_store
        self.basket_window.workTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.basket_window.backBtn.clicked.connect(self.back)
        self.basket_window.payBtn.clicked.connect(self.go_pay)
        
        self.update_basket_table()
        
    def update_basket_table(self):
        # Basket 테이블 초기화
        self.basket_window.workTable.clearContents()
        self.basket_window.workTable.setRowCount(len(self.selected_menulist))
        self.basket_window.workTable.setColumnCount(4)  # 칼럼 수 설정
        self.basket_window.workTable.setHorizontalHeaderLabels(["Store Name", "Menu", "개수", "가격"])  # 칼럼 이름 설정

        self.total_price = 0  # 총 가격 계산을 위한 변수
        store_name = self.get_store_name(self.selected_store)

        for row, menu_info in enumerate(self.selected_menulist):
            menu_name, count = menu_info.split('/')
            price = self.get_menu_price(self.selected_store, menu_name)
            count = int(count)
            self.total_price += price * count

            self.basket_window.workTable.setItem(row, 0, QTableWidgetItem(store_name))  # 매장 이름
            self.basket_window.workTable.setItem(row, 1, QTableWidgetItem(menu_name))  # 메뉴 이름
            self.basket_window.workTable.setItem(row, 2, QTableWidgetItem(str(count)))  # 개수
            self.basket_window.workTable.setItem(row, 3, QTableWidgetItem(str(price * count)))  # 가격

        self.basket_window.lineEdit.setText(str(self.total_price))  # 총합 표시

    def go_pay(self):
        # 결제 페이지로 이동하는 함수
        self.close()
        payPage = PayWindow(self.store_window, self.selected_menulist, self.tcp_server, self.selected_store, self.total_price)
        payPage.showFullScreen()
        self.store_window.hide()
        
    def back(self):
        # 이전 페이지로 돌아가는 함수
        self.close()
        self.store_window.showFullScreen()

    def get_store_name(self, store_id):
        store_names = {
            "S-1": "Starbucks",
            "S-2": "McDonald's",
            "S-3": "Kimbab Heaven",
        }
        return store_names.get(store_id, "Unknown Store")

    def get_menu_price(self, store_id, menu_name):
        for menu_info in StoreDict.get(store_id, []):
            for menu_id, menu_detail in menu_info.items():
                if menu_detail["name"] == menu_name:
                    return int(menu_detail["price"].replace(",", ""))
        return 0

        

class PayWindow(QMainWindow):
    def __init__(self, basket_window, menulist, tcp_server, selected_store, total_price):
        super().__init__(basket_window)
        self.basket_window = basket_window
        self.menulist = menulist
        self.pay_window = uic.loadUi('./data/pay.ui', self)
                
        self.tcp_server = tcp_server
        self.selected_store = selected_store
        self.total_price = total_price
        
        self.uid = bytes(4)
        self.conn = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        self.recv = SerialReceiver(self.conn)
        self.recv.start()

        self.recv.detected.connect(self.detected)
        self.pay_window.lineEdit.setText(str(self.total_price))
        self.send_enabled = True
        self.backBtn.clicked.connect(self.back)
        # self.payBtn.clicked.connect(self.send_data)

        self.timer = QTimer()
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.getStatus)
        self.timer.start()
        
    def send(self, command, data=0):
        print("send")
        req_data = struct.pack('<2s4sic', command, self.uid, data, b'\n')
        self.conn.write(req_data)
        
    def getStatus(self):
        if self.send_enabled:
            self.send(b'GS')
        return
    
    def detected(self, data):
        self.send_enabled = False
        
        uid = data 
        uid_hex = " ".join("{:02X}".format(b) for b in uid)
        print("Detected UID:", uid_hex)
        self.uid = uid_hex
        
        self.send_data()
        self.go_main()
        
    def send_data(self):
        print("send_data called")
        cmd = "OR"
        menulist_str = ",".join(self.menulist)
        data = f"{self.uid},K-1,{self.selected_store},{menulist_str},{len(self.menulist)}"
        print(f"Sending data: {cmd},{data}")
        # TCP 서버를 통해 데이터를 보냅니다.
        self.tcp_server.send(cmd, data)
        QMessageBox.information(self.pay_window, "결제 완료", "결제가 완료되었습니다.", QMessageBox.Ok)
        self.pay_window.close()

        
    def go_main(self):
        self.recv.pause()
        self.new_main = MainWindow(self, self.recv)
        self.pay_window.close()
        
    def back(self):
        self.pay_window.hide()
        self.recv.pause()
        # self.basket_window.showFullScreen()
        self.basket_window.show()
        
        return


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())