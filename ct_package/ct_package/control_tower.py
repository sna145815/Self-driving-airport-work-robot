import KioskManager
import StoreManager
import threading

import rclpy as rp
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration
import numpy as np

store_clients = []


def main():
    global store_clients
    store = StoreManager.StoreServer('192.168.0.40', 9022, store_clients)
    kiosk = KioskManager.KioskServer('192.168.0.40', 9021, store_clients)

    store_thread = threading.Thread(target=store.start_server)
    kiosk_thread = threading.Thread(target=kiosk.start_server)

    store_thread.start()
    kiosk_thread.start()


    store_thread.join()
    kiosk_thread.join()

    rp.init(args=args)
    node = TestNode()
    try:
        while rp.ok():
            x = float(input("Enter goal x: "))
            y = float(input("Enter goal y: "))
            node.set_goal(x, y)
            rp.spin_once(node, timeout_sec=1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
