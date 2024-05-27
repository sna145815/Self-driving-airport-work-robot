import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5 import uic
from PyQt5.QtCore import Qt

# UI 파일 로드
from_class = uic.loadUiType("ControlGUI.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Store Select GUI")

        # 이미지 로드 및 QLabel에 설정
        pixmap = QPixmap('map.png')
        self.Map.setPixmap(pixmap)
        self.Map.setScaledContents(True)  # 이미지가 QLabel 크기에 맞게 조정됩니다.

       

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mywindow = WindowClass()
    mywindow.show()
    sys.exit(app.exec_())
