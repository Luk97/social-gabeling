#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define UNINITIALIZED -1
#define GREEN 0
#define YELLOW 1
#define RED 2

#define HEIGHT_YELLOW_CM 10
#define HEIGHT_RED_CM 20
#define RESET_AFTER_MS 5000
#define DELAY_MS 50

#define ACCEL_NOISE_THRESHOLD 0.2   // Ignore small accelerations (we will have to adjust this)
#define IDLE_THRESHOLD_MS 1000      // If nearly no motion for this time, reset position

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

unsigned long lastUpdateTime = 0;
unsigned long lastMotionTime = 0;
float velocity = 0;
float position = 0;
unsigned long lastTime = 0;
int currentState = GREEN;

uint8_t ledEspAddress[] = {0xEC, 0x94, 0xCB, 0x6F, 0xE5, 0x1A};

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);
  while (!Serial);
  Serial.println("Started");

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  if (!SerialBT.begin("ESP32Sensor", true)) {
    Serial.println("An error occurred initializing Bluetooth");
    return;
  }

  Serial.println("Trying to connect to LED ESP...");

  if (SerialBT.connect(ledEspAddress)) {
    Serial.println("Connected to LED ESP!");
  } else {
    Serial.println("Failed to connect to LED ESP!");
  }

  lastTime = millis();
  lastMotionTime = millis();
}

void loop() {
  if (SerialBT.connected()) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Remove gravity
    float az = accel.acceleration.z - 9.81;

    // Apply deadzone filter
    if (abs(az) < ACCEL_NOISE_THRESHOLD) {
      az = 0;
    }

    // Update velocity and position only if not zero
    if (az != 0) {
      velocity += az * dt;
      position += velocity * dt;
      lastMotionTime = currentTime;
    }

    // Reset if idle for too long
    if (currentTime - lastMotionTime > IDLE_THRESHOLD_MS) {
      velocity = 0;
      position = 0;
    }

    // Convert to cm
    float height_cm = position * 100;
    Serial.print("Height estimate (cm): ");
    Serial.println(height_cm);

    // Switch LEDs after lift was detected
    if (height_cm >= HEIGHT_RED_CM && currentState != 2) {
      SerialBT.println(RED);
      Serial.print("Sent signal: ");
      Serial.println(RED);
      currentState = RED;
      lastUpdateTime = currentTime;
    } else if (height_cm >= HEIGHT_YELLOW_CM && currentState != 1 && currentState != 2) {
      SerialBT.println(YELLOW);
      Serial.print("Sent signal: ");
      Serial.println(YELLOW);
      currentState = YELLOW;
      lastUpdateTime = currentTime;
    }

    // After 5 seconds, reset to green
    if ((currentState == 1 || currentState == 2) && (currentTime - lastUpdateTime > RESET_AFTER_MS)) {
      SerialBT.println(GREEN);
      Serial.print("Sent signal: ");
      Serial.println(GREEN);
      currentState = GREEN;
      velocity = 0;
      position = 0;
    }

    delay(DELAY_MS);
  } else {
    Serial.println("Not connected. Retrying...");
    SerialBT.connect(ledEspAddress);
    delay(1000);
  }
}

