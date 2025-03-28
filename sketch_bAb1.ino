#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int tStep = 2, tDir = 3;
int zStep = 7, zDir = 8;
int invalid_counts = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1); // Stop execution if sensor fails
  }

  pinMode(tStep, OUTPUT);
  pinMode(tDir, OUTPUT);
  pinMode(zStep, OUTPUT);
  pinMode(zDir, OUTPUT);
}

void loop() {
  int vertDistance = 15;  // Total Z travel in cm
  int noZSteps = 20;  // Steps per Z move
  int zCounts = (200 * vertDistance) / noZSteps;
  int thetaCounts = 200;  // Steps per full rotation

  digitalWrite(zDir, LOW);
  for (int j = 0; j < zCounts; j++) {
    
    for (int i = 0; i < thetaCounts; i++) {
      rotateMotor(tStep, 1);
      delay(5);
      double senseDistance = readToFSensor();
      if(senseDistance == -1){
        invalid_counts++;
      }
      Serial.print(j); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(senseDistance);
    }
    if(invalid_counts == thetaCounts){
      delay(500);
      break;
    }
    rotateMotor(zStep, noZSteps);
    delay(500);
    invalid_counts = 0;
  }

  Serial.println("SCAN COMPLETE"); 

  while (true);  // Stop scanning permanently
}

void rotateMotor(int pinNo, int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(pinNo, LOW);
    delayMicroseconds(500);
    digitalWrite(pinNo, HIGH);
    delayMicroseconds(500);
  }
}

double readToFSensor() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 10.0;  // Convert mm to cm
  } else {
    return -1;  // Invalid measurement
  }
}
