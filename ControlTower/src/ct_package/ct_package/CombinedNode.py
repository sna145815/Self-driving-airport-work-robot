#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from db_manager import DBManager
from KioskManager import KioskServer
from StoreManager import StoreServer

HOST = '192.168.0.30'
STORE_PORT = 9022
KIOSK_PORT = 9021
store_clients = []
kiosk_clients = []

class CombinedServerNode(Node):
    def __init__(self):
        super().__init__('combined_server_node')

        db_manager = DBManager(HOST, 'potato', '1234', 'prj')
        connection = db_manager.create_connection()

        if not connection:
            self.get_logger().error("Failed to connect to the database.")
            return

        self.store = StoreServer(HOST, STORE_PORT, store_clients,kiosk_clients, db_manager)
        self.kiosk = KioskServer(HOST, KIOSK_PORT, store_clients,kiosk_clients, db_manager)

        self.store_thread = threading.Thread(target=self.store.start_server)
        self.kiosk_thread = threading.Thread(target=self.kiosk.start_server)

        self.store_thread.daemon = True
        self.kiosk_thread.daemon = True

        self.store_thread.start()
        self.kiosk_thread.start()

def main(args=None):
    rclpy.init(args=args)
    combined_server_node = CombinedServerNode()
    rclpy.spin(combined_server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
