#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// LED Pins
#define RED_LED    21
#define YELLOW_LED 19
#define GREEN_LED  18
#define BLUE_LED   25

// I2C Pins for ESP32
#define SDA_PIN 22
#define SCL_PIN 23

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }
  
  Serial.println("LSM9DS1 found");

  delay(1000);

  // Initialize LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  Serial.println("LED test");

  // Blink each LED
  testLED(RED_LED, "Red");
  testLED(YELLOW_LED, "Yellow");
  testLED(GREEN_LED, "Green");
  testLED(BLUE_LED, "Blue");

  Serial.println("Gyroscope test");

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (true) {
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED)); // blink red to indicate error
      delay(500);
    }
  }

  Serial.println("Gyroscope detected");

  // Setup accelerometer
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
  Serial.print(" Y: "); Serial.print(accel.acceleration.y);
  Serial.print(" Z: "); Serial.println(accel.acceleration.z);

  delay(1000);
}

void testLED(int pin, const char* name) {
  Serial.print("Testing "); 
  Serial.print(name); 
  Serial.println(" LED");
  digitalWrite(pin, HIGH);
  delay(1000);
}
