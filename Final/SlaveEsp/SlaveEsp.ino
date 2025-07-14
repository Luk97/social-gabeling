#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <deque>
#include "BluetoothSerial.h"


// ---------- Constants ----------
#define BAUD_RATE 115200
#define DELAY_LOOP_MS 200

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


// ---------- Globals ----------
BluetoothSerial SerialBT;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

std::deque<unsigned long> biteTimestamps;
unsigned long lastBiteTime = 0;
unsigned long lastRateUpdate = 0;
bool inMotion = false;


// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("Started");
  
  Wire.begin(PIN_SDA, PIN_SCL);

  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  SerialBT.begin("ESP32Slave");
  Serial.println("Bluetooth device ready, name is ESP32Slave");
  printMacAddress();
}


// ---------- Loop ----------
void loop() {
  digitalWrite(PIN_BLUE, SerialBT.connected() ? HIGH : LOW);

  if (SerialBT.connected()) {
    handleSensorAndBiteDetection();
    updateBiteRateAndSend();
    receiveAndSetLED();
  }

  delay(DELAY_LOOP_MS);
}


// ---------- Functions ----------

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
    Serial.println("Bite detected!");
  }

  // Return to idle
  if (accelDelta < BITE_THRESHOLD * 0.7 && inMotion) {
    inMotion = false;
  }
}

/// Computes rolling bite rate and sends it via Bluetooth
void updateBiteRateAndSend() {
  unsigned long now = millis();
  if (now - lastRateUpdate < BITE_UPDATE_INTERVAL_MS) return;

  // Clean up timestamps
  while (!biteTimestamps.empty() && (now - biteTimestamps.front() > BITE_WINDOW_MS)) {
    biteTimestamps.pop_front();
  }

  float bitesPerMinute = biteTimestamps.size() * (60000.0 / BITE_WINDOW_MS);
  Serial.print("Bite rate: ");
  Serial.println(bitesPerMinute);

  SerialBT.println(bitesPerMinute);
  lastRateUpdate = now;
}

/// Receives LED state from master and updates pins
void receiveAndSetLED() {
  if (!SerialBT.available()) return;

  String received = SerialBT.readStringUntil('\n');
  int state = received.toInt();
  Serial.print("Received LED state: ");
  Serial.println(state);
  setLED(state);
}

/// Updates onboard LED state
void setLED(int state) {
  digitalWrite(PIN_GREEN, state == STATE_GREEN ? HIGH : LOW);
  digitalWrite(PIN_YELLOW, state == STATE_YELLOW ? HIGH : LOW);
  digitalWrite(PIN_RED, state == STATE_RED ? HIGH : LOW);
}

/// Debug function to print MAC address
void printMacAddress() {
  uint8_t mac[6];
  SerialBT.getBtAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print("0");  // leading zero for values < 0x10
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}