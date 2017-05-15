#include <SoftwareSerial.h>
#include <SoftwareServo.h>
// ***
// *** Define the RX and TX pins. Choose any two
// *** pins that are unused. Try to avoid D0 (pin 5)
// *** and D2 (pin 7) if you plan to use I2C.
// ***

#define RX    3   // *** D3, Pin 2
#define TX    4   // *** D4, Pin 3


// ***
// *** Define the software based serial port. Using the
// *** name Serial so that code can be used on other
// *** platforms that support hardware based serial. On
// *** chips that support the hardware serial, just
// *** comment this line.
// ***
SoftwareSerial Serial(RX, TX);
SoftwareServo s0, s1;

const int servo_min = 40;
const int servo_mid = 93;
const int servo_max = 140;
bool servos_stopped = 0;
const int servo_threshold = 2;

void driveServos(int pos) {
    if( pos > servo_mid - servo_threshold && pos < servo_mid + servo_threshold) {
        pos = servo_mid;
    }

    int s0_pos, s1_pos = servo_mid;
    int diff = abs(pos - servo_mid);

    if(pos == servo_mid) {
        // no motion
        s0_pos = servo_mid;
        s1_pos = servo_mid;
    } else if(pos >= servo_mid) {
        // going forward
        s0_pos = servo_mid + diff;
        s1_pos = servo_mid - diff;
    } 
    else {
        // going backwards
        s0_pos = servo_mid - diff;
        s1_pos = servo_mid + diff;
    }
    Serial.print("s0_pos: ");
    Serial.println(s0_pos);
    s0.write(s0_pos);
    s1.write(s1_pos);
}
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");
  s1.attach(0);
  s0.attach(1);
}

void loop() {
  if(Serial.available() > 0) {
    int pos = Serial.read();
    pos = constrain(pos, servo_min, servo_max);
    driveServos(pos);
  }
  SoftwareServo::refresh();        // must call at least once every 50ms or so to keep your servos updating
}
