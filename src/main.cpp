#include <Arduino.h>

const int servo_min = 40;
const int servo_mid = 93;
const int servo_max = 140;
bool servos_stopped = 0;
const int servo_threshold = 6;

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
    //s0.write(s0_pos);
    //s1.write(s1_pos);
}
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");
}

void loop() {
  Serial.println("ON");
  digitalWrite(8, HIGH);
  delay(1000);
  Serial.println("OFF");
  digitalWrite(8, LOW);
  delay(1000);
}
