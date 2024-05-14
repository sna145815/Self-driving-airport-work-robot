import cv2
from PyQt5.QtCore import QThread, pyqtSignal, Qt,QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication,QTableWidgetItem,QMessageBox
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

# HOST = "192.168.0.210" # yjs
HOST = "192.168.0.40" # jinhong
PORT = 9022


from_class = uic.loadUiType("storeGUI.ui")[0]

class tcpRecvThread(QThread):
    response = pyqtSignal(str)

    def __init__(self, recvSocket):
        super().__init__()
        self.recvSocket = recvSocket

        # self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.socket.connect((HOST, PORT))

        # self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.client_socket.connect((HOST, PORT))

    def run(self):
        try:
            while True:
                receiveBuf = self.recvSocket.recv(1024)
                self.response.emit(receiveBuf.decode())
        finally:
            self.recvSocket.close()



class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store GUI")
       
        self.tcpInit()

        self.tcpRecvThread = tcpRecvThread(self.client_socket)
        self.tcpRecvThread.start()
        self.tcpRecvThread.response.connect(self.receiveOrder)

        self.menuToggle = 1
        self.storeToggle = 1
        self.DRbtn.clicked.connect(self.reqRobot)
        self.MSbtn.clicked.connect(self.ctrlMenuStatus)
        self.SSbtn.clicked.connect(self.ctrlStoreStatus)


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


    def receiveOrder(self, result):
        self.lineEdit.setText(result)


    def tcpStop(self):
        self.tcpThread.client_socket.close()

        print("Terminate server!")
        sys.exit(0)




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
