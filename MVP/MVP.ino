/*Import required libraries*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>  
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_VL53L0X.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

//create sensor object for 9DOF
Adafruit_BNO055 bno = Adafruit_BNO055();

// Create sensor objects
Adafruit_VL53L0X back = Adafruit_VL53L0X();
Adafruit_VL53L0X left = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *vibratingMotor = AFMS.getMotor(4);

//variables for linear acceleration
int16_t x, y, z; 
uint16_t stillCount;
//set variable for speed iteration
uint8_t i = 0;
uint8_t leftSpeed = 0;
uint8_t rightSpeed = 0;
uint8_t targetSpeed = 100;

//set variables for escape behavior
uint8_t stalkCount = 0;
uint16_t previousBackMeasure = 1000;
unsigned long escapeMillis;


//set variable types for sensors

VL53L0X_RangingMeasurementData_t backSensor;
VL53L0X_RangingMeasurementData_t leftSensor;
VL53L0X_RangingMeasurementData_t rightSensor;

uint16_t backMeasure;
uint16_t backMeasureTwo;
uint16_t backMeasureThree;
uint16_t backMeasureFour;
uint16_t leftMeasure;
uint16_t rightMeasure; 

void escape() {

targetSpeed = 254;
escapeMillis = millis();
}

void pivot() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(100);
  leftMotor->setSpeed(targetSpeed);
  rightMotor->setSpeed(targetSpeed);
 while(rightMeasure < 150 || leftMeasure < 150) {
        
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      left.rangingTest(&leftSensor, false);
      leftMeasure = leftSensor.RangeMilliMeter;
  
      right.rangingTest(&rightSensor, false);
      rightMeasure = rightSensor.RangeMilliMeter; 
    }
    delay(100);
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    delay(100);
    leftSpeed = 0;
    rightSpeed = 0;
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);

    backMeasureFour = 35;
    backMeasureThree = 35;
    backMeasureTwo = 35;
   
}

void turns() {
  while(rightMeasure < 250 || leftMeasure < 250) {
  back.rangingTest(&backSensor, false);
  backMeasure = backSensor.RangeMilliMeter;
 

  left.rangingTest(&leftSensor, false);
  leftMeasure = leftSensor.RangeMilliMeter;
  
  right.rangingTest(&rightSensor, false);
  rightMeasure = rightSensor.RangeMilliMeter;

  if (rightMeasure < 80 && leftMeasure < 80 && leftSensor.RangeStatus != 4 && rightSensor.RangeStatus != 4) {
    Serial.print("I'm gonna pivot!");
    pivot();
  } 
  else if (leftMeasure < 300 && leftSensor.RangeStatus != 4 && leftSpeed >= 50 && rightSpeed >= 50) {
    Serial.print("I'm gonna turn left!");
    if(leftSpeed < targetSpeed + 50 && leftSpeed < 252) {
    leftSpeed = leftSpeed + 3; 
    } 
    if(rightSpeed > targetSpeed - 100 && rightSpeed > 3) {
    rightSpeed = rightSpeed - 3;
    }
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);

    backMeasureFour = 35;
    backMeasureThree = 35;
    backMeasureTwo = 35;
  }
  else if (rightMeasure < 280 && rightSensor.RangeStatus != 4 && leftSpeed >= 50 && rightSpeed >= 50) {
    if(rightSpeed < targetSpeed + 50 && rightSpeed < 252) {
    Serial.print("I'm gonna turn right!");
    rightSpeed = rightSpeed + 3;
    }
    if(leftSpeed > targetSpeed - 100 && leftSpeed > 3) {
    leftSpeed = leftSpeed - 3;
    }
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);

    backMeasureFour = 35;
    backMeasureThree = 35;
    backMeasureTwo = 35;
     
  }
  }
  
}

void forward(){
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
while(leftSpeed != targetSpeed || rightSpeed != targetSpeed) {
  if(leftSpeed < targetSpeed) {
    leftSpeed = leftSpeed + 2;
  } else if(leftSpeed > targetSpeed) {
    leftSpeed = leftSpeed -2;
  }
  if(rightSpeed < targetSpeed) {
    rightSpeed = rightSpeed + 2;
    
  } else if(rightSpeed > targetSpeed) {
    rightSpeed = rightSpeed - 2;
  }
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  delay(10);
}
}

void checkSensors() {
  back.rangingTest(&backSensor, false);
   
  if(backSensor.RangeStatus != 4 && backSensor.RangeMilliMeter > 60 && backSensor.RangeMilliMeter < 1000 ) {
    backMeasureFour = backMeasureThree;
    backMeasureThree = backMeasureTwo;
    backMeasureTwo = backMeasure;
    backMeasure = backSensor.RangeMilliMeter;
  } 
  else {
    backMeasureThree = 35;
    backMeasureTwo = 35;
    backMeasure = 35;
  }
  
 

  left.rangingTest(&leftSensor, false);
  leftMeasure = leftSensor.RangeMilliMeter;
  
  right.rangingTest(&rightSensor, false);
  rightMeasure = rightSensor.RangeMilliMeter;

  
}


void setup() {
 
  Serial.begin(9600);           // set up Serial library at 9600 bps
  

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  // turn on motor
  leftMotor->run(RELEASE);

  rightMotor->setSpeed(150);
  rightMotor->run(FORWARD);
  // turn on motor
  rightMotor->run(RELEASE); 

  vibratingMotor->setSpeed(150);
  vibratingMotor->run(FORWARD);
  // turn on motor
  vibratingMotor->run(RELEASE);

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
//9DOF sensor set up
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  
}

void loop() {
  

 if ( millis() > escapeMillis + 3000 && targetSpeed == 254) {
  targetSpeed = 100;
 }
 
 checkSensors();
 imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  x = abs(linaccel.x());
  y = abs(linaccel.y());
  z = abs(linaccel.z());

  if (x + y + z > 5) {
     stillCount = 0;
     escape();
  }
  else if (x + y + z < 2) {
    stillCount += 1;

    if (stillCount == 30) {
      vibratingMotor->run(FORWARD);
      vibratingMotor->setSpeed(255);
      delay(1000);
      vibratingMotor->run(RELEASE);
      stillCount = 0;
    }
  }
  else {
    stillCount = 0;
  }

 if (backMeasure + 30 < ((backMeasureTwo + backMeasureThree + backMeasureFour) / 3)) {
  escape();
 }
 
 
 if (rightMeasure < 80 && leftMeasure < 80 && leftSensor.RangeStatus != 4 && rightSensor.RangeStatus != 4) {
    Serial.print("I need to pivot!");
    pivot();
  }
  else if (leftMeasure < 300 && leftSensor.RangeStatus != 4 && leftSpeed >= 50 && rightSpeed >= 50) {
    Serial.print("I need to turn right!");
    turns();
  }
  else if (rightMeasure < 280 && rightSensor.RangeStatus != 4 && rightSpeed >= 50 && leftSpeed >= 50) {
    Serial.print("I need to turn left!");
    turns();
  }
  else {
    forward();
  }

  
  
}
