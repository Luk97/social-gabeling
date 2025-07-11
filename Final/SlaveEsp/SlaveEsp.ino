#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define PIN_SDA 22
#define PIN_SCL 23
#define PIN_GREEN 18
#define PIN_YELLOW 19
#define PIN_RED 21
#define PIN_BLUE 25

#define UNINITIALIZED -1
#define GREEN 0
#define YELLOW 1
#define RED 2

#define BAUD_RATE 115200
#define DELAY_MS 200

float velocityX = 0;
float velocityY = 0;
float velocityZ = 0;
unsigned long lastTime = 0;
float idleTime = 0;
int currentState = GREEN;

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

  // Initialize LSM
  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  // Initialize Bluetooth
  SerialBT.begin("ESP32Slave");
  Serial.println("Bluetooth device ready, name is ESP32Slave");
  printMacAddress();

  setLED(currentState);
  lastTime = millis();
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
    float speed = abs(magnitude - 9.81);
    Serial.print("Speed: ");
    Serial.println(speed);

    // Send speed to master
    SerialBT.println(speed);

    // Retrieve LED signal from master
    if (SerialBT.available()) {
      String received = SerialBT.readStringUntil('\n');
      int signal = received.toInt();
      Serial.print("\tReceived signal: ");
      Serial.println(signal);
      setLED(signal);
    }

    digitalWrite(PIN_BLUE, HIGH);
  } else {
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
// Last address: EC:94:CB:6F:CA:02
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