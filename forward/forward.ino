/*Import required libraries*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>  
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  myOtherMotor->setSpeed(150);
  myOtherMotor->run(FORWARD);
  // turn on motor
  myOtherMotor->run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i;
  
  Serial.print("tick-backwards");

  // gets current uptime on microcontroller in millis
  unsigned long currentMillis = millis();

  myMotor->run(FORWARD);
  myOtherMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("tock-forward");

  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);

}
