#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_MotorShield.h>  
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_VL53L0X.h"

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *vibratingMotor = AFMS.getMotor(4);

int16_t x, y, z; 
uint16_t i;
unsigned long restrainedMillis;
unsigned long currentMillis;


void setup() {
  // put your setup code here, to run once:
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

  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

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
}

void loop() {
  // put your main code here, to run repeatedly:

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//
// Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//Serial.print(euler.z());
// Serial.print("\t\t");  


// uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
//
//  delay(BNO055_SAMPLERATE_DELAY_MS);

x = abs(euler.x());
y = abs(euler.y());
z = abs(euler.z()); 
currentMillis = millis(); 

if( x + y + z > 5) {
  Serial.print("I've been Tapped");
  Serial.print("\n");
  Serial.print(millis());
  i = 0;
}
else if ( x + y + z < 2) {
//  Serial.print("I'm restrained");
  
    i += 1;
  
   
  if (i == 2000) {
    Serial.print("\n");
    Serial.print("I'm VIBRATING");
    Serial.print("\n");
    vibratingMotor->run(FORWARD);
    vibratingMotor->setSpeed(255);
    delay(1000);
    vibratingMotor->run(RELEASE);
    i = 0;
  }
}
else {
  Serial.print("\n");
  Serial.print("I'm fine \n");
  
  i = 0;
}

  

}
