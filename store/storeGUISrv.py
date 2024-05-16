import cv2
from PyQt5.QtCore import QThread, pyqtSignal, Qt,QTimer
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication,QTableWidgetItem,QMessageBox, QHeaderView
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import uic,QtWidgets
import socket
import struct
import pickle
import sys
# import mysql.connector
from datetime import datetime
import time
# import numpy as np

HOST = "192.168.0.210" # yjs
# HOST = "192.168.0.40" # jinhong
PORT = 9022

from_class1 = uic.loadUiType("selectStore.ui")[0]
from_class2 = uic.loadUiType("store.ui")[0]


class MainWindow(QMainWindow, from_class1):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        self.storeBtn1.clicked.connect(self.showStoreScreen1)
        self.storeBtn2.clicked.connect(self.showStoreScreen2)
        self.storeBtn3.clicked.connect(self.showStoreScreen3)
        
        self.data = 0

    def showStoreScreen1(self):
        self.data = "McDonald"
        self.secondScreen = SecondScreen(self.data)
        self.secondScreen.show()
        self.hide()

    def showStoreScreen2(self):
        self.data = "Kimbab Heaven"
        self.secondScreen = SecondScreen(self.data)
        self.secondScreen.show()
        self.hide()

    def showStoreScreen3(self):
        self.data = "Sushi"
        self.secondScreen = SecondScreen(self.data)
        self.secondScreen.show()
        self.hide()

class tcpRecvThread(QThread):
    response = pyqtSignal(str)

    def __init__(self, recvSocket):
        super().__init__()
        self.recvSocket = recvSocket

    def run(self):
        try:
            while True:
                receiveBuf = self.recvSocket.recv(1024)
                self.response.emit(receiveBuf.decode())
        finally:
            self.recvSocket.close()

class SecondScreen(QWidget, from_class2):
    def __init__(self, data):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store GUI")

        self.nameLabel.setText(data)
       
        self.tcpInit()

        self.tcpRecvThread = tcpRecvThread(self.client_socket)
        self.tcpRecvThread.start()
        self.tcpRecvThread.response.connect(self.receiveOrder)

        
        self.homeBtn.clicked.connect(self.selectStore)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.cellClicked.connect(self.showMenuList)

        self.menuToggle = 1
        self.storeToggle = 1

        self.DRbtn.clicked.connect(self.reqRobot)
        self.MSbtn.clicked.connect(self.ctrlMenuStatus)
        self.SSbtn.clicked.connect(self.ctrlStoreStatus)

        self.menuList = {}
        
    
    def selectStore(self):
        retval = QMessageBox.question(self, 'question', 'Are you sure go to select Store?',
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if retval == QMessageBox.Yes:
            self.mainWindow = MainWindow()
            self.mainWindow.show()
            self.hide()
        

    def tcpInit(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))

        print("tcp Init")


    def reqRobot(self):
        print("hello Store")

        cmd = "DR"
        storeID = "S-1"
        orderNo = "A1"

        message = cmd + "," + storeID + "," + orderNo

        self.client_socket.sendall(message.encode())

        # try:
        #     print("")
        # except Exception as e:
        #     print("Error tcp send to GUI : ", e)
        # finally:
        #     self.tcpStop()

    def ctrlMenuStatus(self):
        # toggle
        self.menuToggle = not self.menuToggle

        if (self.menuToggle == 1):
            menuStatus = "1"
        else:
            menuStatus = "0"

        print("menu ON/OFF")

        cmd = "MS"
        menuID = "M-1"
        

        message = cmd + "," + menuID + "," + menuStatus

        self.client_socket.sendall(message.encode())

    def ctrlStoreStatus(self):
        # toggle
        self.storeToggle = not self.storeToggle

        if (self.storeToggle == 1):
            storeStatus = "1"
        else:
            storeStatus = "0"

        print("store ON/OFF")

        cmd = "SS"
        storeID = "S-1"

        message = cmd + "," + storeID + "," + storeStatus

        self.client_socket.sendall(message.encode())

    def addList(self):
        row = self.tableWidget.rowCount()
        self.tableWidget.insertRow(row)

        self.tableWidget.setItem(row, 0, QTableWidgetItem(self.orderNo))
        self.tableWidget.item(row, 0).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 1, QTableWidgetItem(self.orderStatus))
        self.tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 2, QTableWidgetItem(str(self.totalMenuCnt)))
        self.tableWidget.item(row, 2).setTextAlignment(Qt.AlignCenter)
        self.tableWidget.setItem(row, 3, QTableWidgetItem(self.DRobotStatus))
        self.tableWidget.item(row, 3).setTextAlignment(Qt.AlignCenter)
    
    def updateList(self, row):
        self.tableWidget.setItem(row, 3, QTableWidgetItem(self.DRobotStatus))
        self.tableWidget.item(row, 3).setTextAlignment(Qt.AlignCenter)

    def showMenuList(self, row, column):
        orderNo = self.tableWidget.item(row, 0).text()     # orderNo 확인
        orderStatus = self.tableWidget.item(row, 1).text() # orderStatus 확인
        print(orderStatus)

        # 메뉴 리스트
        menuList = self.menuList[orderNo]
        strList = []

        for i in range(len(menuList)):
            menu = menuList[i].split('/')[0]
            cnt = menuList[i].split('/')[1]
            tmp =  menu + " : " + cnt + "ea"
            strList.append(tmp)

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
                # Timer 가동!!!!!!!!
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

        
    def receiveOrder(self, result):
        self.cmd = result.split(',')[0]
        self.orderNo = result.split(',')[1]

        # self.orderStatus = result.split(',')[2]
        # self.menuCnt = result.split(',')[2]
        self.menu = []

        if self.cmd == 'OS':
            self.orderStatus = "주문접수"
            self.DRobotStatus = "대기중"
            self.menuCnt = result.split(',')[2]    # 메뉴의 종류 수량

            self.menu = []

            cnt = 0

            # print(range(int(self.menuCnt)))

            for i in range(int(self.menuCnt)):
                tmp = result.split(',')[i+3]
                self.menu.append(tmp)

                cnt = cnt + int(tmp.split('/')[1])
                
                # print(menu[i])
                # print(type(menu[i]))
                self.lineEdit.setText(self.cmd + "," + self.orderNo + "," + self.menuCnt + "," + self.menu[i])
            
            self.totalMenuCnt = cnt
            print(self.totalMenuCnt)
            self.menuList[self.orderNo] = self.menu
            # print(self.menuList)
            
            self.addList()
        elif self.cmd == 'DS':
            self.orderNo

            # 요청한 주문번호가 있는 행의 번호 찾기
            rowCnt = self.tableWidget.rowCount()
            for row in range(rowCnt):
                item = self.tableWidget.item(row, 0)
                if item is not None and item.text() == self.orderNo:
                    break

            # 명령어 해석
            DRobotStatus = result.split(',')[2]
            if DRobotStatus == '0':
                self.DRobotStatus = "로봇대기중" #로봇 번호로 바꾸고 싶다
            elif self.DRobotStatus == '1':
                self.DRobotStatus = "배달완료"
            else:   # wrong command
                pass

            self.updateList(row)

            self.lineEdit.setText(self.cmd + "," + self.orderNo + "," + self.DRobotStatus)            
        else:
            print("wrong command")

        print("recv complete")

        # self.lineEdit.setText(result)


    def tcpStop(self):
        self.tcpThread.client_socket.close()

        print("Terminate server!")
        sys.exit(0)




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
