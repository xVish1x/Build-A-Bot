#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

// Create objects for the ToF sensors
Adafruit_VL53L0X tofSensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofSensor2 = Adafruit_VL53L0X();

// Servo setup
Servo scanServo;
const int servoPin = 9;

// Variables
const int XSHUT1 = 2; // XSHUT pin for Sensor 1
const int XSHUT2 = 3; // XSHUT pin for Sensor 2
int scanAngleStart = 0;  // Start angle for the scan
int scanAngleEnd = 180;  // End angle for the scan
int stepSize = 10;       // Step size in degrees

void setup() {
  Serial.begin(115200); // Start serial communication
  Wire.begin();

  // Setup XSHUT pins
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // Initialize Sensor 1
  digitalWrite(XSHUT1, LOW);  // Turn off Sensor 1
  digitalWrite(XSHUT2, HIGH); // Keep Sensor 2 active
  delay(10);
  digitalWrite(XSHUT1, HIGH); // Turn on Sensor 1
  if (!tofSensor1.begin(0x30)) { // Assign unique I²C address
    Serial.println("Failed to initialize Sensor 1");
    while (1);
  }

  // Initialize Sensor 2
  digitalWrite(XSHUT2, LOW);  // Turn off Sensor 2
  delay(10);
  digitalWrite(XSHUT2, HIGH); // Turn on Sensor 2
  if (!tofSensor2.begin(0x31)) { // Assign unique I²C address
    Serial.println("Failed to initialize Sensor 2");
    while (1);
  }

  // Setup servo
  scanServo.attach(servoPin);
  scanServo.write(scanAngleStart); // Move servo to start position
  delay(1000);
}

void loop() {
  for (int angle = scanAngleStart; angle <= scanAngleEnd; angle += stepSize) {
    // Move servo to the current angle
    scanServo.write(angle);
    delay(500); // Wait for the servo to stabilize

    // Read distance from Sensor 1
    VL53L0X_RangingMeasurementData_t measure1;
    tofSensor1.rangingTest(&measure1, false);
    int distance1 = (measure1.RangeStatus == 4) ? -1 : measure1.RangeMilliMeter;

    // Read distance from Sensor 2
    VL53L0X_RangingMeasurementData_t measure2;
    tofSensor2.rangingTest(&measure2, false);
    int distance2 = (measure2.RangeStatus == 4) ? -1 : measure2.RangeMilliMeter;

    // Print results
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(", Sensor 1: ");
    Serial.print(distance1);
    Serial.print(" mm, Sensor 2: ");
    Serial.println(distance2);
  }

  delay(1000); // Pause before the next scan
}
