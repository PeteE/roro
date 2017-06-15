#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>
#include <pb_arduino.h>
#include "robot_data.pb.h"

// setup i2c communication between 
// rpi (master) and arduino (slave)
#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

// define servos
Servo s0, s1;

#define S0_PIN 9
#define S1_PIN 10 

#define TRIGF_PIN  2 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHOF_PIN  3 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGB_PIN  4 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHOB_PIN  5 // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonarf(TRIGF_PIN, ECHOF_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarb(TRIGB_PIN, ECHOB_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// LED Strip
#define LED_STRIP_PIN 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// protobuf stuff
RobotData recv_robot_data;
RobotData curr_robot_data;

int recv_header_bytes_read = 0;
byte recv_payload[32];
int recv_payload_size = 0;
int recv_payload_bytes_read = 0;
long recv_start_time = 0;
long unsigned recv_threshold = 500;

bool send_header_sent = false;
int unsigned send_payload_bytes_sent = 0;
pb_ostream_t send_stream;
byte send_payload[32];
long send_start_time = 0;
long unsigned send_threshold = 500;

const int servo_min = 1000;
const int servo_mid = 1513;
const int servo_max = 2000;
bool servos_stopped = 0;
const int servo_threshold = 20;

void detachServo(Servo s) {
    if(s.attached()) {
        s.detach();
    }
}

void attachServos() {
    if(!s0.attached()) {
        s0.attach(S0_PIN);
    }
    if(!s1.attached()) {
        s1.attach(S1_PIN);
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void ledsOff() {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
    strip.show();
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

void driveServos() {
    int pos0 = constrain(curr_robot_data.s0_pos, 0, 180);
    int pos1 = constrain(curr_robot_data.s1_pos, 0, 180);

    // map position to pwm microseconds
    int pos0_u = map(pos0, 0, 180, 1000, 2000);
    int pos1_u = map(pos1, 0, 180, 1000, 2000);

    if( pos0_u > servo_mid - servo_threshold && pos0_u< servo_mid + servo_threshold) {
        pos0_u = servo_mid;
    }
    if( pos1_u > servo_mid - servo_threshold && pos1_u< servo_mid + servo_threshold) {
        pos1_u = servo_mid;
    }

    //Serial.println(pos0_u);
    //Serial.println(pos1_u);

    int s0_val, s1_val = servo_mid;
    int diff = abs(pos0_u - servo_mid);
    if(pos0_u == servo_mid) {
        detachServo(s0);
        s0_val = servo_mid;
        return;
    } else { 
        attachServos();
    }
    if(pos1_u == servo_mid) {
        detachServo(s1);
        s1_val = servo_mid;
        return;
    } else { 
        attachServos();
    }

    if(pos0_u >= servo_mid) {
        // going forward
        s0_val = servo_mid + diff;
        s1_val = servo_mid - diff;
    } 
    else {
        // going backwards
        s0_val = servo_mid - diff;
        s1_val = servo_mid + diff; 
    }
    s0.writeMicroseconds(s0_val);
    s1.writeMicroseconds(s1_val);
}

void driveLeds() {
    Serial.print("led_pattern: ");
    Serial.println(curr_robot_data.led_pattern);
    switch(curr_robot_data.led_pattern) {
        //case RobotData_LedPattern.RobotData_LedPattern_COLORWIPE:
        case 1:
            Serial.println("colorWipe");
            colorWipe(strip.Color(255, 0, 0), 50); // Red
            colorWipe(strip.Color(0, 255, 0), 50); // Green
            colorWipe(strip.Color(0, 0, 255), 50); // Blue
            break;
        //case RobotData_LedPattern.RobotData_LedPattern_RAINBOW:
        case 2:
            Serial.println("rainbow");
            rainbow(20);
            break;
        default:
            ledsOff();
            break;
    }
}

void updateRobot() {
    curr_robot_data.s0_pos = recv_robot_data.s0_pos;
    curr_robot_data.s1_pos = recv_robot_data.s1_pos;
    curr_robot_data.led_pattern = recv_robot_data.led_pattern;

    driveServos();
    //driveLeds();
}

void check_i2c_timeout() {
    // just in case we receive partial protobuf data
    // we want to do some cleanup if a certain amount of time has passed.
    unsigned long current_time = millis();
    if(current_time - recv_start_time > recv_threshold && 
        (recv_header_bytes_read > 0 || recv_payload_bytes_read > 0) 
    ) {
        recv_start_time = 0;
        recv_payload_size = 0;
        recv_header_bytes_read = 0;
        recv_payload_bytes_read = 0;
        Serial.println("Reading i2c data timed out");
        Wire.begin(SLAVE_ADDRESS);
    }
    if(current_time - send_start_time > send_threshold &&
        (send_header_sent || send_payload_bytes_sent > 0)
    ) {
        send_start_time = 0;
        send_payload_bytes_sent = 0;
        send_header_sent = false;
        Serial.println("Sending i2c data timed out");
        Wire.begin(SLAVE_ADDRESS);
    }
}

// callback for received data
void receiveData(int byteCount) {
    /* Before receiving any protobuf data, we expect to receive a 2 byte header 
       which indicates the number of bytes to read.
    */
    check_i2c_timeout();
    byte data = Wire.read();
    if(recv_header_bytes_read == 0) { 
        recv_start_time = millis();
        recv_payload_size = data << 8;
        recv_header_bytes_read++;
    } else if(recv_header_bytes_read == 1) { 
        recv_payload_size += data;
        recv_header_bytes_read++;
    } else if (recv_payload_bytes_read <= recv_payload_size) {
        /* ready to read payload_size bytes */
        if(recv_payload_bytes_read < recv_payload_size) {
            recv_payload[recv_payload_bytes_read++] = data;
        }
        if(recv_payload_bytes_read >= recv_payload_size) {
            pb_istream_t stream = pb_istream_from_buffer(recv_payload, recv_payload_size);
            if(!pb_decode(&stream, RobotData_fields, &recv_robot_data)) {
                Serial.print("decode failed!");
                recv_start_time = 0;
                recv_header_bytes_read = 0;
                recv_payload_bytes_read = 0;
                recv_payload_size = 0;
            } else {
                recv_start_time = 0;
                recv_header_bytes_read = 0;
                recv_payload_bytes_read = 0;
                recv_payload_size = 0;
                updateRobot();
            }
        }
    }
}

// callback for sending data
void sendData(){
    send_start_time = millis();
    if(!send_header_sent) {
        send_stream = pb_ostream_from_buffer(send_payload, sizeof(send_payload));
        if(pb_encode(&send_stream, RobotData_fields, &curr_robot_data)) {
            Wire.write(send_stream.bytes_written);
            send_header_sent = true;
        }
    } else {
        byte byte_to_send = send_payload[send_payload_bytes_sent++];
        Wire.write(byte_to_send);

        if(send_payload_bytes_sent >= send_stream.bytes_written) {
            send_payload_bytes_sent = 0;
            send_header_sent = false;
        }
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing");

    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    curr_robot_data.sonarf = 0;
    curr_robot_data.sonarb = 0;
    curr_robot_data.s0_pos = servo_mid;
    curr_robot_data.s1_pos = servo_mid;
    curr_robot_data.led_pattern = RobotData_LedPattern_RAINBOW;
    

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    rainbow(20);
    ledsOff();
}

void loop() {
    curr_robot_data.sonarf = sonarf.ping_cm();
    curr_robot_data.sonarb = sonarb.ping_cm();

    check_i2c_timeout();
    delay(100);
}
