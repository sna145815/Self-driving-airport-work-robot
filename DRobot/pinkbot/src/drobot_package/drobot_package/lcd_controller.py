import RPi_I2C_driver

class LCDController:
    def __init__(self):
        self.mylcd = RPi_I2C_driver.lcd()

    def display_string(self, string, line):
        self.mylcd.lcd_display_string(string, line)
