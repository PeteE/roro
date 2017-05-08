import smbus
import time
import signal
import sys
from oled_display import OledDisplay

class RobotDriver:
    I2C_BUS = 1
    SERVO_STOP = 134

    # Address of the atmega/arduino servo driver board
    controller_i2c_address = 0x04

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        self.i2c_bus = smbus.SMBus(self.I2C_BUS)

    def exit_gracefully(self,signum, frame):
        print('Stopping servos.')
        self.moveServo(self.SERVO_STOP)
        sys.exit(0)

    def moveServo(self, value):
        self.i2c_bus.write_byte(self.controller_i2c_address, value)

    def readResponse(self):
        number = self.i2c_bus.read_byte(self.controller_i2c_address)
        return number


if __name__ == '__main__':
    driver = RobotDriver()
    while True:
        oled = OledDisplay()
        for i in range(140,220, 10):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)

        for i in range(140,40, -10):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)
