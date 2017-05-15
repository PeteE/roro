#include <SoftwareServo.h> 

SoftwareServo myservo;                 // create servo object to control a servo 
                                       // a maximum of eight servo objects can be created 

int pos = 0;                           // variable to store the servo position 
int potpin = 3;                        // analog pin used to connect the potentiometer
int val;                               // variable to read the value from the analog pin

void setup() 
{ 
  myservo.attach(1);                  // attaches the servo on pin 1 to the servo object 
} 

void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  pos = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
                              
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    SoftwareServo::refresh();
    delay(15);                       // waits 15ms for the servo to reach the position  
}

