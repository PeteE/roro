import time
import signal
import sys
import serial
#from oled_display import OledDisplay

class RobotDriver:
    SERVO_STOP = 90

    def __init__(self, device, baudrate=9600):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        self.ser = serial.Serial(device)
        self.ser.baudrate = baudrate

    def exit_gracefully(self,signum, frame):
        print('Stopping servos.')
        self.moveServo(self.SERVO_STOP)
        sys.exit(0)

    def moveServo(self, value):
        self.ser.write(chr(value))


if __name__ == '__main__':
    #oled = OledDisplay()
    device = '/dev/ttyAMA0'
    driver = RobotDriver(device=device)
    print('opening serial {}'.format(device))
    while True:
        for i in range(90, 40, -5):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)

        for i in range(40, 90, 5):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)

        for i in range(90, 140, 5):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)
        for i in range(140, 90, -5):
            print('Sending value: {}'.format(i))
            driver.moveServo(i)
            time.sleep(.5)
