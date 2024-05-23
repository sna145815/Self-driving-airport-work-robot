from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
import serial

class SerialReceiver(QThread):
    detected = pyqtSignal(bytes)
    recvTotal = pyqtSignal(int)
    changedTotal = pyqtSignal()

    def __init__(self, conn):
        super(SerialReceiver, self).__init__()
        self.conn = conn
        self.running = True
        print("recv init")
        
    def run(self):
        print("recv start")
        while self.running:
            try:
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
            except Exception as e:
                print(f"Error reading from serial port: {e}")
                
    def resume(self):
        self.running = True

    def pause(self):
        self.running = False
        
    def stop(self):
        print("recv stop")
        try:
            self.conn.close()  # Clean up resources
        except Exception as e:
            print(f"Error closing serial port: {e}")