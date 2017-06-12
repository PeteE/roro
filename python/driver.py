import time
import signal
import sys
import smbus
import robot_data_pb2
from oled_display import OledDisplay

class RobotDriver:
    SERVO_STOP = 90

    def __init__(self, i2c_address=0x04, i2c_bus=1):
        self.i2c_address = i2c_address
        self.i2c_bus = smbus.SMBus(i2c_bus)

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        self.current_state = robot_data_pb2.RobotData()
        self.set_state(s0_pos=90, s1_pos=90, led_pattern=robot_data_pb2.RobotData.OFF)


    def exit_gracefully(self,signum, frame):
        print('Stopping servos.')
        self.set_state(s0_pos=90, s1_pos=90, led_pattern=robot_data_pb2.RobotData.OFF)
        sys.exit(0)

    def set_state(self, s0_pos, s1_pos, led_pattern):
        self.current_state.s0_pos=s0_pos
        self.current_state.s1_pos=s1_pos
        self.current_state.led_pattern = led_pattern

        data = self.current_state.SerializeToString()
        data_size = len(data)
        # write header
        self.i2c_bus.write_byte(self.i2c_address, (data_size >> 8) & 0xFF)
        self.i2c_bus.write_byte(self.i2c_address, data_size & 0xFF)
        # write data
        for c in data:
            self.i2c_bus.write_byte(self.i2c_address, ord(c))
    
if __name__ == '__main__':
    oled = OledDisplay()
    driver = RobotDriver()

    while True:
        for i in range(90, 40, -5):
            oled_text = 'RobotState:\ns0: {}\ns1: {}'.format(i, i)
            oled.display_text(oled_text)

            driver.set_state(s0_pos=i, s1_pos=i, led_pattern=robot_data_pb2.RobotData.OFF)
            time.sleep(.5)

        for i in range(40, 90, 5):
            oled_text = 'RobotState:\ns0: {}\ns1: {}'.format(i, i)
            oled.display_text(oled_text)
            driver.set_state(s0_pos=i, s1_pos=i, led_pattern=robot_data_pb2.RobotData.OFF)
            time.sleep(.5)


