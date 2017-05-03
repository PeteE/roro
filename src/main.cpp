#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// setup i2c communication between 
// rpi (master) and arduino (slave)
#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

// define servos
Servo s1, s2;
const int servo_min = 1000;
const int servo_mid = 1530;
const int servo_max = 2000;
bool servos_stopped = 0;
const int servo_threshold = 10;

int ledFwdPin = 2;
int ledBckPin = 3;

void driveServos(int pos) {
    pos = constrain(pos, 40, 220);
    int y_val = map(pos, 40, 220, 1000, 2000);
    if( y_val > servo_mid - servo_threshold && y_val < servo_mid + servo_threshold) {
        y_val = servo_mid;
    }

    int s1_val, s2_val = servo_mid;
    int diff = abs(y_val - servo_mid);

    if(y_val == servo_mid) {
        digitalWrite(ledFwdPin, LOW);
        digitalWrite(ledBckPin, LOW);
        s1_val = servo_mid;
        s2_val = servo_mid;
    } else if(y_val >= servo_mid) {
        // going forward
        digitalWrite(ledFwdPin, HIGH);
        digitalWrite(ledBckPin, LOW);
        s1_val = servo_mid + diff;
        s2_val = servo_mid - diff;
    } 
    else {
        // going backwards
        digitalWrite(ledBckPin, HIGH);
        digitalWrite(ledFwdPin, LOW);
        s1_val = servo_mid - diff;
        s2_val = servo_mid + diff; 
    }
    Serial.print("s1val: ");
    Serial.print(s1_val);
    Serial.print(", pos: ");
    Serial.println(pos);
    s1.writeMicroseconds(s1_val);
    s2.writeMicroseconds(s2_val);
}

// callback for received data
void receiveData(int byteCount) {
    while(Wire.available()) {
        number = Wire.read();
        Serial.print("data received: ");
        Serial.println(number);
        driveServos(number);
    }
}
// callback for sending data
void sendData(){
    Wire.write(number);
}

void setup() {

/*
TCCR2B |= _BV(CS20);//set bit (remove this line for a /8 prescaler)
TCCR2B |= _BV(CS21);//set bit
TCCR2B &= ~_BV(CS22);//clear bit
*/

    Serial.begin(9600); // start serial for output
    pinMode(ledFwdPin, OUTPUT);
    pinMode(ledBckPin, OUTPUT);

    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    // setup services
    s1.attach(9);
    s2.attach(10);
    s1.writeMicroseconds(servo_mid);
    s2.writeMicroseconds(servo_mid);
    Serial.println("Ready!");
}

void loop() {
    delay(100);
}
