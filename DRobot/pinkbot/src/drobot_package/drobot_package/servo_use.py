# servo_use.py 파일 내용

from servo_controller import ServoController
from time import sleep

def main():
    # 외부에서 사용을 위한 인스턴스 생성
    servo_controller = ServoController()

    try:
        # 서보 모터를 180도로 이동 open
        servo_controller.move_to_max()
        sleep(1)

        # 서보 모터를 0도로 이동 close
        servo_controller.move_to_zero()
        sleep(1)

    finally:
        # 서보 모터 정리
        servo_controller.cleanup()
        

if __name__ == "__main__":
    main()


