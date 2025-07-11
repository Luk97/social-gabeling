#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Pin definitions
#define PIN_SDA 22
#define PIN_SCL 23
#define PIN_GREEN 18
#define PIN_YELLOW 19
#define PIN_RED 21
#define PIN_BLUE 25

// States
#define GREEN 0
#define YELLOW 1
#define RED 2

// Constants
#define BAUD_RATE 115200
#define DELAY_MS 200
#define DELAY_STATE_CHANGE_MS 5000
#define CONNECT_INTERVAL_MS 5000

#define ACCEL_NOISE_THRESHOLD 0.2
#define GREEN_YELLOW_THRESHOLD 1.5
#define YELLOW_RED_THRESHOLD 4

// Global variables
unsigned long lastStateChangeTime = 0;
unsigned long lastTime = 0;
unsigned long lastConnectAttempt = 0;
int currentState = GREEN;

uint8_t slaveEspAddress[] = {0xEC, 0x94, 0xCB, 0x6F, 0xCA, 0x02};

void setup() {
  // Initialize Serial
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("Started");

  // Initialize Pins
  Wire.begin(PIN_SDA, PIN_SCL);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  setLED(GREEN); // Default LED state

  // Initialize LSM9DS1 sensor
  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found, restarting in 5s...");
    delay(5000);
    ESP.restart();
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  // Initialize Bluetooth
  if (!SerialBT.begin("ESP32Master", true)) {
    Serial.println("An error occurred initializing Bluetooth");
    return;
  } else {
    Serial.println("Successfully initialized Bluetooth");
  }

  delay(500); // Allow BT stack to stabilize

  // Try initial connection to slave
  Serial.println("Trying to connect to Slave ESP...");
  if (SerialBT.connect(slaveEspAddress)) {
    Serial.println("Connected to slave ESP!");
  } else {
    Serial.println("Failed to connect to slave ESP!");
  }

  printMacAddress();

  lastTime = millis();
  lastConnectAttempt = millis();
}

void loop() {
  if (SerialBT.connected()) {
    // Read sensor data
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Calculate master speed
    float x = accel.acceleration.x;
    float y = accel.acceleration.y;
    float z = accel.acceleration.z;
    float magnitude = sqrt(x * x + y * y + z * z);
    float masterSpeed = abs(magnitude - 9.81);

    Serial.print("masterSpeed: ");
    Serial.print(masterSpeed);

    // Retrieve speed from slave
    if (SerialBT.available()) {
      String received = SerialBT.readStringUntil('\n');
      received.trim();
      if (received.length() > 0) {
        float slaveSpeed = received.toFloat();
        Serial.print("\tReceived slave speed: ");
        Serial.println(slaveSpeed);

        // Apply noise filter
        if (masterSpeed < ACCEL_NOISE_THRESHOLD) masterSpeed = 0.0;
        if (slaveSpeed < ACCEL_NOISE_THRESHOLD) slaveSpeed = 0.0;

        // Check if enough time has passed since last state change
        if (millis() - lastStateChangeTime >= DELAY_STATE_CHANGE_MS) {
          float speedDelta = abs(masterSpeed - slaveSpeed);
          Serial.print("speedDelta: ");
          Serial.println(speedDelta);
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
          lastStateChangeTime = now;
        }
      } else {
        Serial.println("\tReceived empty speed string");
      }
    } else {
      Serial.println("\tNo slave speed received");
    }

    digitalWrite(PIN_BLUE, HIGH);
  } else {
    unsigned long now = millis();
    if (now - lastConnectAttempt >= CONNECT_INTERVAL_MS) {
      Serial.println("Not connected to slave ESP. Retrying...");
      if (SerialBT.connect(slaveEspAddress)) {
        Serial.println("Reconnected to slave ESP!");
      } else {
        Serial.println("Reconnect attempt failed");
      }
      lastConnectAttempt = now;
    }
    digitalWrite(PIN_BLUE, LOW);
  }

  delay(DELAY_MS);
}

void setLED(int state) {
  currentState = state;
  digitalWrite(PIN_GREEN, state == GREEN ? HIGH : LOW);
  digitalWrite(PIN_YELLOW, state == YELLOW ? HIGH : LOW);
  digitalWrite(PIN_RED, state == RED ? HIGH : LOW);
}

// Print MAC address (only for debug)
// Last address: EC:94:CB:6F:C7:86
void printMacAddress() {
  uint8_t mac[6];
  SerialBT.getBtAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}