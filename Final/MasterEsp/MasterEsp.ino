#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <deque>
#include "BluetoothSerial.h"


// ---------- Constants ----------
#define BAUD_RATE 115200
#define DELAY_LOOP_MS 200
#define DELAY_STATE_CHANGE_MS 5000
#define CONNECT_RETRY_MS 5000

#define PIN_SDA 22
#define PIN_SCL 23
#define PIN_GREEN 18
#define PIN_YELLOW 19
#define PIN_RED 21
#define PIN_BLUE 25

#define STATE_GREEN 0
#define STATE_YELLOW 1
#define STATE_RED 2

#define REST_MAGNITUDE 9.81
#define BITE_THRESHOLD 1.5             // Threshold to detect bite (delta from rest)
#define BITE_DEBOUNCE_MS 1500          // Minimum time between bite detections
#define BITE_WINDOW_MS 60000           // 1 minute window for calculating bites/min
#define BITE_UPDATE_INTERVAL_MS 5000   // Update LEDs every 5s

#define GREEN_YELLOW_THRESHOLD 1.5
#define YELLOW_RED_THRESHOLD 4


// ---------- Globals ----------
BluetoothSerial SerialBT;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

std::deque<unsigned long> biteTimestamps;
unsigned long lastBiteTime = 0;
unsigned long lastRateUpdate = 0;
unsigned long lastStateChangeTime = 0;
unsigned long lastConnectAttempt = 0;
bool inMotion = false;
float masterRate = 0.0;

int currentState = STATE_GREEN;
uint8_t slaveEspAddress[] = {0xEC, 0x94, 0xCB, 0x6F, 0xCA, 0x02};


// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("Master ESP started");

  Wire.begin(PIN_SDA, PIN_SCL);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found, restarting...");
    delay(5000);
    ESP.restart();
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  if (!SerialBT.begin("ESP32Master", true)) {
    Serial.println("Bluetooth init failed");
    return;
  }

  Serial.println("Bluetooth ready (ESP32Master)");
  delay(500);
  tryConnectToSlave();

  printMacAddress();
}


// ---------- Loop ----------
void loop() {
  digitalWrite(PIN_BLUE, SerialBT.connected() ? HIGH : LOW);

  if (SerialBT.connected()) {
    handleSensorAndBiteDetection();
    updateBiteRate();
    receiveSlaveRateAndUpdateLEDs();
  } else {
    handleReconnect();
  }

  delay(DELAY_LOOP_MS);
}


// ---------- Functions ----------

/// Tries to connect to the slave ESP
void tryConnectToSlave() {
  Serial.println("Attempting to connect to slave...");
  if (SerialBT.connect(slaveEspAddress)) {
    Serial.println("Connected to slave!");
  } else {
    Serial.println("Connection failed");
  }
}

/// Attempts to reconnect if not connected
void handleReconnect() {
  unsigned long now = millis();
  if (now - lastConnectAttempt >= CONNECT_RETRY_MS) {
    tryConnectToSlave();
    lastConnectAttempt = now;
  }
}

/// Reads sensor and detects new bite events
void handleSensorAndBiteDetection() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float x = accel.acceleration.x;
  float y = accel.acceleration.y;
  float z = accel.acceleration.z;
  float magnitude = sqrt(x * x + y * y + z * z);
  float accelDelta = abs(magnitude - REST_MAGNITUDE);

  unsigned long now = millis();

  // Bite start
  if (accelDelta > BITE_THRESHOLD && !inMotion && (now - lastBiteTime > BITE_DEBOUNCE_MS)) {
    inMotion = true;
    lastBiteTime = now;
    biteTimestamps.push_back(now);
    Serial.println("Detected bite!");
  }

  // Return to idle
  if (accelDelta < BITE_THRESHOLD * 0.7 && inMotion) {
    inMotion = false;
  }
}

/// Computes rolling bite rate from master fork
void updateBiteRate() {
  unsigned long now = millis();
  if (now - lastRateUpdate < BITE_UPDATE_INTERVAL_MS) return;

  while (!biteTimestamps.empty() && (now - biteTimestamps.front() > BITE_WINDOW_MS)) {
    biteTimestamps.pop_front();
  }

  masterRate = biteTimestamps.size() * (60000.0 / BITE_WINDOW_MS);
  Serial.print("Master bite rate: ");
  Serial.println(masterRate);

  lastRateUpdate = now;
}

/// Receives slave rate and determines LED feedback
void receiveSlaveRateAndUpdateLEDs() {
  if (!SerialBT.available()) return;

  String received = SerialBT.readStringUntil('\n');
  received.trim();
  if (received.length() == 0) {
    Serial.println("Received empty slave rate");
    return;
  }

  float slaveRate = received.toFloat();
  Serial.print("Slave bite rate: ");
  Serial.println(slaveRate);

  if (masterRate < 0.1) masterRate = 0.0;
  if (slaveRate < 0.1) slaveRate = 0.0;

  float delta = abs(masterRate - slaveRate);
  Serial.print("Rate delta: ");
  Serial.println(delta);

  unsigned long now = millis();
  if (now - lastStateChangeTime >= DELAY_STATE_CHANGE_MS) {
    if (delta < GREEN_YELLOW_THRESHOLD) {
      setLED(STATE_GREEN);
      SerialBT.println(STATE_GREEN);
    } else if (delta < YELLOW_RED_THRESHOLD) {
      setLED(STATE_YELLOW);
      SerialBT.println(STATE_YELLOW);
    } else {
      setLED(STATE_RED);
      SerialBT.println(STATE_RED);
    }
    lastStateChangeTime = now;
  }
}

/// Updates onboard LED state
void setLED(int state) {
  currentState = state;
  digitalWrite(PIN_GREEN, state == STATE_GREEN ? HIGH : LOW);
  digitalWrite(PIN_YELLOW, state == STATE_YELLOW ? HIGH : LOW);
  digitalWrite(PIN_RED, state == STATE_RED ? HIGH : LOW);
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