#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define PIN_SDA 22
#define PIN_SCL 23
#define PIN_GREEN 18
#define PIN_YELLOW 19
#define PIN_RED 23
#define PIN_BLUE 17

#define UNINITIALIZED -1
#define GREEN 0
#define YELLOW 1
#define RED 2
#define BLUE 3

#define HEIGHT_YELLOW_CM 10
#define HEIGHT_RED_CM 20
#define RESET_AFTER_MS 5000
#define DELAY_MS 200

#define BAUD_RATE 9600
#define ACCEL_NOISE_THRESHOLD 0.2   // Ignore small accelerations (we will have to adjust this)
#define IDLE_THRESHOLD_MS 1000      // If nearly no motion for this time, reset position
#define GREEN_YELLOW_THRESHOLD 2
#define YELLOW_RED_THRESHOLD 5

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

unsigned long lastUpdateTime = 0;
unsigned long lastMotionTime = 0;
float velocity = 0;
float position = 0;
unsigned long lastTime = 0;
int currentState = GREEN;

uint8_t slaveEspAddress[] = {0xEC, 0x94, 0xCB, 0x6F, 0xE5, 0x1A};

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin(22, 23);
  while (!Serial);
  Serial.println("Started");

  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  // Initialize LSM
  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  if (!SerialBT.begin("ESP32Master", true)) {
    Serial.println("An error occurred initializing Bluetooth");
    return;
  }

  Serial.println("Trying to connect to Slave ESP...");

  if (SerialBT.connect(slaveEspAddress)) {
    Serial.println("Connected to slave ESP!");
  } else {
    Serial.println("Failed to connect to slave ESP!");
  }

  lastTime = millis();
  lastMotionTime = millis();
}

void loop() {
  if (SerialBT.connected()) {
    // Read sensor data
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Raw acceleration values
    float x = accel.acceleration.x;
    float y = accel.acceleration.y;
    float z = accel.acceleration.z;

    // Speed calculation
    float magnitude = sqrt(x * x + y * y + z * z);
    float masterSpeed = abs(magnitude - 9.81);
    Serial.print("masterSpeed: ");
    Serial.print(masterSpeed);    

    // Retrieve speed from slave
    if (SerialBT.available()) {
      String received = SerialBT.readStringUntil('\n');
      float slaveSpeed = received.toFloat();
      Serial.print("\tReceived slave speed: ");
      Serial.println(slaveSpeed);

      // Apply noise filter
      if (masterSpeed < ACCEL_NOISE_THRESHOLD) {
        masterSpeed = 0.0;
      }
      if (slaveSpeed < ACCEL_NOISE_THRESHOLD) {
        slaveSpeed = 0.0;
      }

      // Set own LED state and send it to slave
      float speedDelta = abs(masterSpeed - slaveSpeed);
      if (speedDelta < GREEN_YELLOW_THRESHOLD) {
        setLED(GREEN);
        SerialBT.println(GREEN);
      } else if (speedDelta < YELLOW_RED_THRESHOLD) {
        setLED(YELLOW);
        SerialBT.println(YELLOW);
      } else {
        setLED(RED);
        SerialBT.println(RED);
      }

      digitalWrite(PIN_BLUE, HIGH);
    }
    delay(DELAY_MS);
  } else {
    // Try to connect to slave
    Serial.println("Not connected. Retrying...");
    SerialBT.connect(slaveEspAddress);
    digitalWrite(PIN_BLUE, LOW);
    delay(1000);
  }
}

void setLED(int state) {
  currentState = state;
  digitalWrite(PIN_GREEN, state == GREEN ? HIGH : LOW);
  digitalWrite(PIN_YELLOW, state == YELLOW ? HIGH : LOW);
  digitalWrite(PIN_RED, state == RED ? HIGH : LOW);
}