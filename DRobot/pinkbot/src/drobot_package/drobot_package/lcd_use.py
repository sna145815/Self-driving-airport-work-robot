##권한 설정 알아서 하셈 ㅋ
#1. sudo raspi-config  -> Interface Options -> SPI,I2C enable
#2. sudo chmod a+rw /dev/i2c-1

from lcd_controller import LCDController

# LCDController 인스턴스 생성``
lcd_controller = LCDController()

robot_num = 'DROBOT-1'
robot_status = 'On delivery'

# 문자열 표시
lcd_controller.display_string(robot_num, 1)
lcd_controller.display_string(robot_status, 2)
