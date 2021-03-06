import time
import signal
import sys
import smbus
import robot_data_pb2
from oled_display import OledDisplay

class RobotDriver:
    SERVO_STOP = 90

    def __init__(self, i2c_address=0x04, i2c_bus=1, oled_display=None):
        self.i2c_address = i2c_address
        self.i2c_bus = smbus.SMBus(i2c_bus)
        self.oled_display = oled_display

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        self.current_state = robot_data_pb2.RobotData()
        self.set_state(s0_pos=90, s1_pos=90, led_pattern=robot_data_pb2.RobotData.OFF)


    def exit_gracefully(self, signum, frame):
        print('Exiting.')
        self.set_state(s0_pos=90, s1_pos=90, led_pattern=robot_data_pb2.RobotData.OFF)
        if self.oled_display:
            self.oled_display.clear()

        sys.exit(0)

    def get_state(self):
        try:
            data_length = self.i2c_bus.read_byte(self.i2c_address)
            #print('Length: {}'.format(data_length))
            i = 0;
            data = []
            while i < data_length: 
                data.append(self.i2c_bus.read_byte(self.i2c_address))
                i+=1

            rd = robot_data_pb2.RobotData()
            rd.ParseFromString("".join(map(chr, data)))
            print(rd)

            if self.oled_display:
                oled_text = ['RobotState:',
                             's0: {}, s1: {}'.format(rd.s0_pos, rd.s1_pos),
                             'sF: {}, sB: {}'.format(rd.sonarf, rd.sonarb),
                            ] 
                self.oled_display.display_text('\n'.join(oled_text))
        except Exception as e:
            print('Error getting state from robot.')

    def set_state(self, s0_pos, s1_pos, led_pattern):
        try:
            self.current_state.s0_pos=s0_pos
            self.current_state.s1_pos=s1_pos
            self.current_state.led_pattern=led_pattern
            self.current_state.sonarf=0
            self.current_state.sonarb=0

            data = self.current_state.SerializeToString()
            data_size = len(data)
            # write header
            self.i2c_bus.write_byte(self.i2c_address, (data_size >> 8) & 0xFF)
            self.i2c_bus.write_byte(self.i2c_address, data_size & 0xFF)
            # write data
            for c in data:
                self.i2c_bus.write_byte(self.i2c_address, ord(c))
        except Exception as e:
            print(e)

    
if __name__ == '__main__':
    oled = OledDisplay()
    driver = RobotDriver(oled_display=oled)

    while True:
        for i in range(90, 40, -5):
            driver.set_state(s0_pos=i, s1_pos=i, led_pattern=robot_data_pb2.RobotData.RAINBOW)
            time.sleep(.5)
            driver.get_state()

        for i in range(40, 90, 5):
            driver.set_state(s0_pos=i, s1_pos=i, led_pattern=robot_data_pb2.RobotData.RAINBOW)
            time.sleep(.5)
            driver.get_state()


