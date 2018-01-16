/*Import required libraries*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>  
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_VL53L0X.h"
// Create sensor objects
Adafruit_VL53L0X back = Adafruit_VL53L0X();
Adafruit_VL53L0X left = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();
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

  //Sensor Setup

  Serial.begin(115200);
  
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(10, LOW);
  delay(10);
  digitalWrite(13, LOW);
  delay(10);

  digitalWrite(9, HIGH);
  delay(10);
  digitalWrite(10, HIGH);
  delay(10);
  digitalWrite(13, HIGH);
  delay(10);

  digitalWrite(10, LOW);
  delay(10);
  digitalWrite(13, LOW);
  delay(10);

  left.begin(0x30);
  delay(10);

  digitalWrite(10, HIGH);
  delay(10);
  
  right.begin(0x31);
  delay(10);

  digitalWrite(13, HIGH);
  delay(10);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!back.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  //Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i;
  VL53L0X_RangingMeasurementData_t sensor;
//  
  Serial.print("Forward?");
//
//  // gets current uptime on microcontroller in millis
//  unsigned long currentMillis = millis();
//
  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  for (i=0; i<100; i++) {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  
//  left.rangingTest(&sensor, false);
//  right.rangingTest(&sensor, false);

  // measure = sensor.RangeMilliMeter;

   //boolean checks if an object is getting close to sensor
  back.rangingTest(&sensor, false);
  backMeasure = sensor.RangeMilliMeter;
 

  left.rangingTest(&sensor, false);
  leftMeasure = sensor.RangeMilliMeter;
  
  right.rangingTest(&sensor, false);
  rightMeasure = sensor.RangeMilliMeter;
  
  if (backMeasure < 100 && sensor.RangeStatus != 4) {
    Serial.print("I need to move forward faster!");
    myMotor->run(BACKWARD);
    myOtherMotor->run(FORWARD);
    for (i=100; i<200; i++) {
      myMotor->setSpeed(i);
      myOtherMotor->setSpeed(i);  
      delay(10);
    } 
  }
  else if (leftMeasure < 100 && sensor.RangeStatus != 4) {
    Serial.print("I need to turn right!");
    myMotor->setSpeed(i += 20);
  }
  else if (rightMeasure < 100 && sensor.RangeStatus != 4) {
    Serial.print("I need to turn left!");
    myOtherMotor->setSpeed(i += 20);
  } 
  else if (rightMeasure < 100 && leftMeasure < 100 && sensor.RangeStatus != 4) {
    Serial.print("I need to pivot!");
    while(rightMeasure < 100 && leftMeasure < 100) {
    myMotor->run(RELEASE);
    myOtherMotor->run(RELEASE);  
    myMotor->run(FORWARD);
    myOtherMotor->run(FORWARD); }
  }
  
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i);
//    myOtherMotor->setSpeed(i);  
//    delay(10);
//  }
//  
//  Serial.print("tock-forward");
//
//  myMotor->run(BACKWARD);
//  myOtherMotor->run(FORWARD);
//  for (i=0; i<255; i++) {
//    myMotor->setSpeed(i);
//    myOtherMotor->setSpeed(i);  
//    delay(10);
//  }
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i);
//    myOtherMotor->setSpeed(i);  
//    delay(10);
//  }
//
//  Serial.print("tech");
//  myMotor->run(RELEASE);
//  myOtherMotor->run(RELEASE);
//  delay(1000); 



}
