import smbus
import time
import signal
import sys
#from oled_display import OledDisplay

class RobotDriver:
    # This is the address we setup in the Arduino Program
    i2c_address = 0x04
    i2c_bus = 1

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        self.i2c_bus = smbus.SMBus(self.i2c_bus)

    def exit_gracefully(self,signum, frame):
        print('Stopping servos.')
        self.writeNumber(90)
        sys.exit(1)

    def writeNumber(self, value):
        self.i2c_bus.write_byte(self.i2c_address, value)

    def readNumber(self):
        number = self.i2c_bus.read_byte(self.i2c_address)
        return number


if __name__ == '__main__':
    driver = RobotDriver()
    #oled0 = OledDisplay()
    while True:
        for i in range(90, 180, 10):
            print('Sending value: {}'.format(i))
            driver.writeNumber(i)
            #print(driver.readNumber())
            time.sleep(.5)

        for i in range(180, 0, -10):
            print('Sending value: {}'.format(i))
            driver.writeNumber(i)
            #print(driver.readNumber())
            time.sleep(.5)

        for i in range(0, 90, 10):
            print('Sending value: {}'.format(i))
            driver.writeNumber(i)
            #print(driver.readNumber())
            #time.sleep(.5)


