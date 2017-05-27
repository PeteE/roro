#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// setup i2c communication between 
// rpi (master) and arduino (slave)
#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

// define servos
Servo s0, s1;

#define S0_PIN 0
#define S1_PIN 1
#define LED_FWD_PIN 2
#define LED_BACK_PIN 3

const int servo_min = 1000;
const int servo_mid = 1513;
const int servo_max = 2000;
bool servos_stopped = 0;
const int servo_threshold = 20;

void detachServos() {
    if(s0.attached()) {
        s0.detach();
    }
    if(s1.attached()) {
        s1.detach();
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
void driveServos(int pos) {
    pos = constrain(pos, 0, 180);
    int y_val = map(pos, 0, 180, 1000, 2000);
    if( y_val > servo_mid - servo_threshold && y_val < servo_mid + servo_threshold) {
        y_val = servo_mid;
    }

    int s0_val, s1_val = servo_mid;
    int diff = abs(y_val - servo_mid);

    if(y_val == servo_mid) {
        digitalWrite(LED_FWD_PIN, LOW);
        digitalWrite(LED_BACK_PIN, LOW);
        detachServos();
        s0_val = servo_mid;
        s1_val = servo_mid;
        return;
    } else { 
        attachServos();
    }

    if(y_val >= servo_mid) {
        // going forward
        digitalWrite(LED_FWD_PIN, HIGH);
        digitalWrite(LED_BACK_PIN, LOW);
        s0_val = servo_mid + diff;
        s1_val = servo_mid - diff;
    } 
    else {
        // going backwards
        digitalWrite(LED_BACK_PIN, HIGH);
        digitalWrite(LED_FWD_PIN, LOW);
        s0_val = servo_mid - diff;
        s1_val = servo_mid + diff; 
    }
    s0.writeMicroseconds(s0_val);
    s1.writeMicroseconds(s1_val);
}

// callback for received data
void receiveData(int byteCount) {
    while(Wire.available()) {
        number = Wire.read();
        driveServos(number);
    }
}
// callback for sending data
void sendData(){
    Wire.write(number);
}

void setup() {
    pinMode(LED_FWD_PIN, OUTPUT);
    pinMode(LED_BACK_PIN, OUTPUT);

    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
}

void loop() {
    delay(100);
}
