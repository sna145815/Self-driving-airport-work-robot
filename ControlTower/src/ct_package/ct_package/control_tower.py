import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import KioskManager
import StoreManager
from db_manager import DBManager
import threading

from TcpNode import *


store_clients = []

HOST = '192.168.0.17'
SOTRE_PORT = 9022
KIOSK_PORT = 9021


def main(args=None):
    global store_clients

    db_manager = DBManager(HOST, 'potato', '1234', 'prj')
    connection = db_manager.create_connection()

    #rp.init(args=args)
    #tcpnode = TcpNode()
    store = StoreManager.StoreServer(HOST, SOTRE_PORT, store_clients,db_manager)
    #kiosk = KioskManager.KioskServer(HOST, KIOSK_PORT, store_clients,db_manager,tcpnode)
    kiosk = KioskManager.KioskServer(HOST, KIOSK_PORT, store_clients,db_manager)

    store_thread = threading.Thread(target=store.start_server)
    kiosk_thread = threading.Thread(target=kiosk.start_server)

    kiosk_thread.daemon = True 
    store_thread.daemon = True 

    store_thread.start()
    kiosk_thread.start()

    store_thread.join()
    kiosk_thread.join()

    #rp.spin(tcpnode)
    #rp.shutdown()


if __name__ == '__main__':
    main()