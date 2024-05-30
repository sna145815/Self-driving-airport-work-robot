#sudo apt install python3-pip
#pip3 install RPi.GPIO
# sudo nano /etc/udev/rules.d/99-spi.rules  ->   SUBSYSTEM=="spidev", GROUP="spiuser", MODE="0660"
# sudo groupadd spiuser  
# sudo usermod -aG spiuser 이름
# sudo reboot  

# servo_controller.py 파일 내용

import RPi.GPIO as GPIO

class ServoController:
    def __init__(self, servo_pin=12):
        self.servoPin = servo_pin
        self.SERVO_MAX_DUTY = 12.5
        self.SERVO_MIN_DUTY = 4
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servoPin, GPIO.OUT)
        GPIO.output(self.servoPin, GPIO.LOW)  # 이전 PWM 객체 정리
        self.servo = GPIO.PWM(self.servoPin, 50) #50HZ
        self.servo.start(0)

    def set_servo_pos(self, degree):
        if degree > 180:
            degree = 180
        duty = self.SERVO_MIN_DUTY + (degree * (self.SERVO_MAX_DUTY - self.SERVO_MIN_DUTY) / 180.0)
        print("Degree: {} to {}(Duty)".format(degree, duty))
        self.servo.ChangeDutyCycle(duty)

    def move_to_zero(self):
        self.set_servo_pos(10)

    def move_to_max(self):
        self.set_servo_pos(170)

    def cleanup(self):
        self.servo.stop()
        GPIO.cleanup()

