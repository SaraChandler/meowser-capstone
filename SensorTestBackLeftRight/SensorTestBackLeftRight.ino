#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X back = Adafruit_VL53L0X();
Adafruit_VL53L0X left = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();

void setup() {
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
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement from BACK... ");
  back.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range back ");
  }
  Serial.print("Reading a measurement from LEFT... ");
  left.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range left ");
  }
  Serial.print("Reading a measurement from RIGHT... ");
  right.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range right ");
  }
    
  delay(1000);
}
