import KioskManager
import StoreManager
import threading

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


if __name__ == '__main__':
    main()
