import cv2
from PyQt5.QtCore import QThread, pyqtSignal, Qt,QTimer
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication, QTableWidgetItem,QMessageBox, QHeaderView
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import uic,QtWidgets
import socket
import struct
import pickle
import sys
# import mysql.connector
from datetime import datetime
import time
import threading
# import numpy as np

HOST = "192.168.0.210" # yjs ethernet
# HOST = "192.168.1.102" # yjs rosteam3 wifi
# HOST = "192.168.0.30" # jinhong
PORT = 9023

from_class1 = uic.loadUiType("/home/addinedu/amr_ws/git_ws/ros-repo-3/store/selectStore.ui")[0]
from_class2 = uic.loadUiType("/home/addinedu/amr_ws/git_ws/ros-repo-3/store/store.ui")[0]

menuDic_origin = {
    'S-1': {
        'cheeseBurger': 5,
        'bigMac': 150,
        'beefSnackWrap': 15,
        'McNuggets': 60,
        'coffee': 30,
        'coke': 5,
        'sprite': 10
    },
    'S-2': {
        'americano': 5,
        'CaramelMacchiato': 5,
        'Cappuccino': 5,
        'coldBrew': 5,
        'GHBT': 5,
        'frappuccino': 5
    },
    'S-3': {
        'kimbab': 100,
        'tunaKimbab': 150,
        'cheeseKimbab': 120,
        'beefKimbab': 200,
        'porkKimbab': 200,
        'shrimpKimbab': 170
    },
}

menuDic = {
    'S-1': {
        'cheeseBurger': 5,
        'bigMac': 5,
        'beefSnackWrap': 5,
        'McNuggets': 5,
        'coffee': 5,
        'coke': 5,
        'sprite': 5
    },
    'S-2': {
        'americano': 5,
        'CaramelMacchiato': 5,
        'Cappuccino': 5,
        'coldBrew': 5,
        'GHBT': 5,
        'frappuccino': 5
    },
    'S-3': {
        'kimbab': 5,
        'tunaKimbab': 5,
        'cheeseKimbab': 5,
        'beefKimbab': 5,
        'porkKimbab': 5,
        'shrimpKimbab': 5
    },
}


class MainWindow(QMainWindow, from_class1):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        self.storeBtn1.clicked.connect(self.showStoreScreen1)
        self.storeBtn2.clicked.connect(self.showStoreScreen2)
        self.storeBtn3.clicked.connect(self.showStoreScreen3)
        
        self.storeName = ""

    def showStoreScreen1(self):
        self.storeName = "McDonald"
        self.secondScreen = SecondScreen(self.storeName)
        self.secondScreen.show()
        self.hide()

    def showStoreScreen2(self):
        self.storeName = "starbucks"
        self.secondScreen = SecondScreen(self.storeName)
        self.secondScreen.show()
        self.hide()

    def showStoreScreen3(self):
        self.storeName = "kimbab heaven"
        self.secondScreen = SecondScreen(self.storeName)
        self.secondScreen.show()
        self.hide()

class tcpRecvThread(QThread):
    response = pyqtSignal(str)

    def __init__(self, recvSocket):
        super().__init__()
        self.recvSocket = recvSocket
        self.running = True

    def run(self):
        try:
            while self.running:
                receiveBuf = self.recvSocket.recv(1024)
                self.response.emit(receiveBuf.decode())
        except Exception as e:
            print("tcpRecvThread ERROR!! : ", e)
        except KeyboardInterrupt:
            self.recvSocket.close()
        finally:
            self.recvSocket.close()
    
    def stop(self):
        self.running = False

class SecondScreen(QWidget, from_class2):
    def __init__(self, storeName):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store GUI")

        self.storeName = storeName
        self.storeId = ""

        self.nameLabel.setText(self.storeName)
        self.statusLabel.setText("(open)")
       
        
        self.tcpInit()
        self.checkStore()

        # tcp 수신 스레드 설정 및 시작
        self.tcpRecvThread = tcpRecvThread(self.client_socket)
        self.tcpRecvThread.start()
        self.tcpRecvThread.response.connect(self.receiveOrder)
        
        # 초기화면으로 전환(매장 선택)
        self.homeBtn.clicked.connect(self.selectStore)

        # tableWidget 설정, 테이블 더블클릭 이벤트에 대한 함수
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.cellClicked.connect(self.showMenuList)

        self.menuToggle = 1
        self.storeToggle = 1

        # test btn
        self.DRbtn.clicked.connect(self.reqRobot2)
        # self.MSbtn.clicked.connect(self.ctrlMenuStatus)
        self.SSbtn.clicked.connect(self.ctrlStoreStatus)

        # Store 1
        self.menuCbAM1.clicked.connect(self.menuCbAM1_clicked)
        self.menuCbAM2.clicked.connect(self.menuCbAM2_clicked)
        self.menuCbAM3.clicked.connect(self.menuCbAM3_clicked)
        self.menuCbAM4.clicked.connect(self.menuCbAM4_clicked)
        self.menuCbAM5.clicked.connect(self.menuCbAM5_clicked)
        self.menuCbAM6.clicked.connect(self.menuCbAM6_clicked)
        self.menuCbAM7.clicked.connect(self.menuCbAM7_clicked)

        # Store 2
        self.menuCbBM1.clicked.connect(self.menuCbBM1_clicked)
        self.menuCbBM2.clicked.connect(self.menuCbBM2_clicked)
        self.menuCbBM3.clicked.connect(self.menuCbBM3_clicked)
        self.menuCbBM4.clicked.connect(self.menuCbBM4_clicked)
        self.menuCbBM5.clicked.connect(self.menuCbBM5_clicked)
        self.menuCbBM6.clicked.connect(self.menuCbBM6_clicked)

        # Store 3
        self.menuCbCM1.clicked.connect(self.menuCbCM1_clicked)
        self.menuCbCM2.clicked.connect(self.menuCbCM2_clicked)
        self.menuCbCM3.clicked.connect(self.menuCbCM3_clicked)
        self.menuCbCM4.clicked.connect(self.menuCbCM4_clicked)
        self.menuCbCM5.clicked.connect(self.menuCbCM5_clicked)
        self.menuCbCM6.clicked.connect(self.menuCbCM6_clicked)

        self.menuList = {}
    
    def tcpInit(self):
        while True:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((HOST, PORT))
                break
            except Exception as e:
                print(f"error code : {e}")
                print("Retrying in 1 seconds...")
                time.sleep(1) # 1초후 재시도
            except KeyboardInterrupt:
                print("User interrupted the connection attempt.")
                return

        print("TCP Initialized!!!")

    def tcpStop(self):
        if self.tcpRecvThread.isRunning():
            self.tcpRecvThread.quit()
            self.tcpRecvThread.wait()

        # self.tcpRecvThread.client_socket.close()
        self.client_socket.close()

        print("Terminate server!")
        sys.exit(0)

    def checkStore(self):
        if self.storeName == "McDonald":
            self.storeId = "S-1"
            self.menuCbAM1.setText("cheeseBurger")
            self.menuCbAM2.setText("bigMac")
            self.menuCbAM3.setText("beefSnackWrap")
            self.menuCbAM4.setText("McNuggets")
            self.menuCbAM5.setText("coffee")
            self.menuCbAM6.setText("coke")
            self.menuCbAM7.setText("sprite")

            self.menuListGB1.show()
            self.menuListGB2.hide()
            self.menuListGB3.hide()
        elif self.storeName == "starbucks":
            self.storeId = "S-2"
            self.menuCbBM1.setText("americano")
            self.menuCbBM2.setText("CaramelMacchiato")
            self.menuCbBM3.setText("Cappuccino")
            self.menuCbBM4.setText("coldBrew")
            self.menuCbBM5.setText("GHBT")
            self.menuCbBM6.setText("frappuccino")

            self.menuListGB1.hide()
            self.menuListGB2.show()
            self.menuListGB3.hide()
        elif self.storeName == "kimbab heaven":
            self.storeId = "S-3"
            self.menuCbCM1.setText("kimbab")
            self.menuCbCM2.setText("tunaKimbab")
            self.menuCbCM3.setText("cheeseKimbab")
            self.menuCbCM4.setText("beefKimbab")
            self.menuCbCM5.setText("porkKimbab")
            self.menuCbCM6.setText("shrimpKimbab")

            self.menuListGB1.hide()
            self.menuListGB2.hide()
            self.menuListGB3.show()
        else:
            # print("wrong store")
            pass
        
    def selectStore(self):
        retval = QMessageBox.question(self, 'question', 'Are you sure go to select Store?',
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        try:
            if retval == QMessageBox.Yes:
                # self.tcpStop()
                self.client_socket.shutdown(socket.SHUT_WR)

                # # 모든 데이터를 수신할 때까지 기다립니다.
                # while True:
                #     data = self.client_socket.recv(1024)
                #     if not data:
                #         break
                #     print('남은 데이터:', data.decode('utf-8'))

                self.client_socket.close()
                self.tcpRecvThread.stop()
                print("close socket")
        except KeyboardInterrupt:
            self.client_socket.shutdown(socket.SHUT_WR)
            self.client_socket.close()
            self.tcpRecvThread.stop()
            print("exit program")
            sys.exit()
        except Exception as e:
            print(f"error code : {e}")

        
        self.mainWindow = MainWindow()
        self.mainWindow.show()
        self.hide()
        
    def reqRobot(self, orderNo):
        cmd = "DR"
        message = cmd + "," + self.storeId + "," + str(orderNo)

        self.client_socket.sendall(message.encode())
        print("hello Store")


    def reqRobot2(self):
        cmd = "DR"
        storeId = "S-1"
        orderNo = 10
        message = cmd + "," + storeId + "," + str(orderNo)

        self.client_socket.sendall(message.encode())
        print("hello Store")

    def ctrlMenuStatus(self, menuId, menuStatus):
        cmd = "MS"
        message = cmd + "," + str(menuId) + "," + str(menuStatus)

        self.client_socket.sendall(message.encode())

    def ctrlStoreStatus(self):
        # toggle
        self.storeToggle = not self.storeToggle

        if (self.storeToggle == 1):
            storeStatus = 1
            self.statusLabel.setText("(open)")
            # print("store ON")
        else:
            storeStatus = 0
            self.statusLabel.setText("(close)")
            # print("store OFF")

        cmd = "SS"
        message = cmd + "," + self.storeId + "," + str(storeStatus)
        self.client_socket.sendall(message.encode())

    def addList(self, orderNo, orderStatus, totalMenuCnt, DRobotStatus, DRobotNo):
        row = self.tableWidget.rowCount()
        self.tableWidget.insertRow(row)

        int_QTableWidgetItem = QTableWidgetItem()
        int_QTableWidgetItem.setData(Qt.DisplayRole, int(orderNo))

        # self.tableWidget.setItem(row, 0, QTableWidgetItem(orderNo))
        self.tableWidget.setItem(row, 0, int_QTableWidgetItem)
        self.tableWidget.item(row, 0).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 1, QTableWidgetItem(orderStatus))
        self.tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 2, QTableWidgetItem(str(totalMenuCnt)))
        self.tableWidget.item(row, 2).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 3, QTableWidgetItem(DRobotStatus))
        self.tableWidget.item(row, 3).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 4, QTableWidgetItem(DRobotNo))
        self.tableWidget.item(row, 4).setTextAlignment(Qt.AlignCenter)

        self.tableWidget.sortItems(0, Qt.DescendingOrder)
    
    def findRowCol(self, orderNo):
        rowCnt = self.tableWidget.rowCount()
        for row in range(rowCnt):
            item = self.tableWidget.item(row, 0)
            if item is not None and item.text() == orderNo:
                break
        
        return row

    # def updateList(self, row, orderStatus, DRobotStatus):
    #     self.tableWidget.setItem(row, 1, QTableWidgetItem(orderStatus))
    #     self.tableWidget.setItem(row, 3, QTableWidgetItem(DRobotStatus))
    #     self.tableWidget.item(row, 3).setTextAlignment(Qt.AlignCenter)
    
    # def updateOrderStatus(self, row, orderStatus):
    #     self.tableWidget.setItem(row, 1, QTableWidgetItem(orderStatus))
    #     self.tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter)
    #     pass

    def updateList(self, row, col, data):
        self.tableWidget.setItem(row, col, QTableWidgetItem(data))
        self.tableWidget.item(row, col).setTextAlignment(Qt.AlignCenter)

    def showMenuList(self, row, column):
        orderNo = self.tableWidget.item(row, 0).text()     # orderNo 확인
        orderStatus = self.tableWidget.item(row, 1).text() # orderStatus 확인
        print(orderStatus)
        print(row)

        # 메뉴 리스트
        menuList = self.menuList[orderNo]
        strList = []
        menuTimeList = []

        for i in range(len(menuList)):
            menuName = menuList[i].split('/')[0]
            cnt = menuList[i].split('/')[1]
            tmp =  menuName + " : " + cnt + "ea"
            strList.append(tmp)

            # 메뉴별 조리시간을 리스트에 저장
            menuTime = menuDic.get(self.storeId, {}).get(menuName)
            menuTimeList.append(menuTime)

        # 조리시간 제일 오래걸리는 메뉴의 시간을 찾아(타이머 돌려야 되니까)
        maxMenuTime = max(menuTimeList)

        # pop up 종류
        if orderStatus == '주문접수':
            # 조리할건지 물어봐
            msg = "\n Are you going to start cooking?"
            strList.append(msg)
            StrMenuList = "\n".join(strList)

            retval = QMessageBox.question(self, 'Menu List', StrMenuList,
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
            if retval == QMessageBox.Yes:
                self.orderStatus = "조리중"

                # 조리시작 메시지 송신
                cmd = "CS"
                message = cmd + "," + orderNo
                self.client_socket.sendall(message.encode())

                # Timer 가동!!!!!!!!
                self.cookTimer(maxMenuTime, orderNo)

                if maxMenuTime > 10:
                    callCnt = maxMenuTime - 10
                    self.callTimer(callCnt, orderNo)
                else:
                    self.reqRobot(orderNo)

                

                self.tableWidget.setItem(row, 1, QTableWidgetItem(self.orderStatus))
                self.tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter)
        else:
            StrMenuList = "\n".join(strList)

            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.NoIcon)
            msg_box.setText(StrMenuList)
            msg_box.setWindowTitle("Menu List")
            msg_box.addButton(QMessageBox.Ok)
            msg_box.exec_()     
   
    def receiveOrder_origin(self, result):
        cmd = result.split(',')[0]
        orderNo = result.split(',')[1]

        DRobotStatusText = ""

        # self.orderStatus = result.split(',')[2]
        # self.menuCnt = result.split(',')[2]
        # self.menu = []

        if cmd == 'OS':
            orderStatus = "주문접수"
            DRobotStatusText = "배차 대기중" # 주문접수되고 아직 배차 안된 상태
            menuCnt = result.split(',')[2]    # 메뉴의 종류 수량
            DRobotNo = "-"

            menu = []

            cnt = 0

            # print(range(int(self.menuCnt)))

            for i in range(int(menuCnt)):
                tmp = result.split(',')[i+3]
                menu.append(tmp)

                cnt = cnt + int(tmp.split('/')[1])
                
                # print(menu[i])
                # print(type(menu[i]))
                self.lineEdit.setText(cmd + "," + orderNo + "," + menuCnt + "," + menu[i])
            
            totalMenuCnt = cnt
            # print(totalMenuCnt)
            self.menuList[orderNo] = menu
            # print(self.menuList)
            
            self.addList(orderNo, orderStatus, totalMenuCnt, DRobotStatusText, DRobotNo)
        elif cmd == 'DS': # 매장 알람
            # self.orderNo
            DRobotNo = result.split(',')[3]

            # 요청한 주문번호가 있는 행의 번호 찾기
            rowCnt = self.tableWidget.rowCount()
            for row in range(rowCnt):
                item = self.tableWidget.item(row, 0)
                if item is not None and item.text() == orderNo:
                    break

            # row를 반환함

            # print(type(item))
            # print(item)
            # print(row)

            # 명령어 해석
            DRobotStatus = result.split(',')[2]
            if DRobotStatus == '0':
                DRobotStatusText = "배차완료"
            elif DRobotStatus == '1':
                DRobotStatusText = "매장도착" #로봇 번호로 바꾸고 싶다
            elif DRobotStatus == '2':
                DRobotStatusText = "배달완료"
            else:   # wrong command
                pass

            self.updateList(row, 3, DRobotStatusText)
            self.updateList(row, 4, DRobotNo)

            self.lineEdit.setText(cmd + "," + orderNo + "," + DRobotStatus + "," + DRobotNo)            
        else:
            print("wrong command")

        print("recv complete")

        # self.lineEdit.setText(result)

    def receiveOrder(self, recv):
        result = recv.split(',')

        if len(result)>1:
            cmd = result[0]
            orderNo = result[1]
            DRobotStatusText = ""

            if cmd == 'OS':
                orderStatus = "주문접수"
                DRobotStatusText = "배차 대기중" # 주문접수되고 아직 배차 안된 상태
                menuCnt = result[2]    # 메뉴의 종류 수량
                DRobotNo = "-"

                menu = []
                cnt = 0

                for i in range(int(menuCnt)):
                    tmp = result[i+3]
                    menu.append(tmp)
                    cnt = cnt + int(tmp.split('/')[1])
                    
                    self.lineEdit.setText(cmd + "," + orderNo + "," + menuCnt + "," + menu[i])
                
                totalMenuCnt = cnt
                # print(totalMenuCnt)
                self.menuList[orderNo] = menu
                # print(self.menuList)
                
                self.addList(orderNo, orderStatus, totalMenuCnt, DRobotStatusText, DRobotNo)
            elif cmd == 'DS': # 매장 알람
                # self.orderNo
                DRobotNo = result[3]

                # 요청한 주문번호가 있는 행의 번호 찾기
                rowCnt = self.tableWidget.rowCount()
                for row in range(rowCnt):
                    item = self.tableWidget.item(row, 0)
                    if item is not None and item.text() == orderNo:
                        break

                # 명령어 해석
                DRobotStatus = result[2]
                if DRobotStatus == '0':
                    DRobotStatusText = "배차완료"
                elif DRobotStatus == '1':
                    DRobotStatusText = "매장도착"
                elif DRobotStatus == '2':
                    DRobotStatusText = "배달완료"
                else:   # wrong command
                    pass

                self.updateList(row, 3, DRobotStatusText)
                self.updateList(row, 4, DRobotNo)

                self.lineEdit.setText(cmd + "," + orderNo + "," + DRobotStatus + "," + DRobotNo)            
            else:
                print("wrong command")

            print("TCP receive success")
        else:
            print("Received TCP data is too short")

        # self.lineEdit.setText(result)

    def menuCbAM1_clicked(self):
        menuId = "AM-1"
        if (self.menuCbAM1.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)
    
    def menuCbAM2_clicked(self):
        menuId = "AM-2"
        if (self.menuCbAM2.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbAM3_clicked(self):
        menuId = "AM-3"
        if (self.menuCbAM3.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbAM4_clicked(self):
        menuId = "AM-4"
        if (self.menuCbAM4.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbAM5_clicked(self):
        menuId = "AM-5"
        if (self.menuCbAM5.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbAM6_clicked(self):
        menuId = "AM-6"
        if (self.menuCbAM6.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbAM7_clicked(self):
        menuId = "AM-7"
        if (self.menuCbAM7.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)


    def menuCbBM1_clicked(self):
        menuId = "BM-1"
        if (self.menuCbBM1.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbBM2_clicked(self):
        menuId = "BM-2"
        if (self.menuCbBM2.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbBM3_clicked(self):
        menuId = "BM-3"
        if (self.menuCbBM3.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbBM4_clicked(self):
        menuId = "BM-4"
        if (self.menuCbBM4.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbBM5_clicked(self):
        menuId = "BM-5"
        if (self.menuCbBM5.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbBM6_clicked(self):
        menuId = "BM-6"
        if (self.menuCbBM6.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)


    def menuCbCM1_clicked(self):
        menuId = "CM-1"
        if (self.menuCbCM1.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbCM2_clicked(self):
        menuId = "CM-2"
        if (self.menuCbCM2.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbCM3_clicked(self):
        menuId = "CM-3"
        if (self.menuCbCM3.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbCM4_clicked(self):
        menuId = "CM-4"
        if (self.menuCbCM4.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbCM5_clicked(self):
        menuId = "CM-5"
        if (self.menuCbCM5.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)

    def menuCbCM6_clicked(self):
        menuId = "CM-6"
        if (self.menuCbCM6.isChecked()):
            menuStatus = 1
        else:
            menuStatus = 0
        
        self.ctrlMenuStatus(menuId, menuStatus)


    def cookTimeCallback(self, orderNo):
        # status 및 gui 업데이트
        orderStatus ="조리완료"
        # self.func(orderNo, orderStatus)
        row = self.findRowCol(orderNo)
        self.updateList(row, 1, orderStatus)

    def cookTimer(self, timeSec, orderNo):
        timer = threading.Timer(timeSec, self.cookTimeCallback, args=(orderNo,))
        timer.start()

    def callTimeCallback(self, orderNo):
        self.reqRobot(orderNo)

    def callTimer(self, timeSec, orderNo):
        timer = threading.Timer(timeSec, self.callTimeCallback, args=(orderNo,))
        timer.start()




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
