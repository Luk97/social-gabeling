#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define PIN_GREEN 18
#define PIN_YELLOW 19
#define PIN_RED 23

#define UNINITIALIZED -1
#define GREEN 0
#define YELLOW 1
#define RED 2

int currentState = GREEN;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32LED");
  Serial.println("Bluetooth device active, name is ESP32LED");

  // Initialize LEDs
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);

  Serial.print("MAC address: ");
  uint8_t mac[6];
  SerialBT.getBtAddress(mac);

  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print("0");  // leading zero for values < 0x10
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  Serial.println("Waiting for connection...");

  setLED(currentState);
}

void loop() {
  if (SerialBT.available()) {
    String received = SerialBT.readStringUntil('\n');
    int signal = received.toInt();
    Serial.print("Received signal: ");
    Serial.println(signal);

    setLED(signal);
  }
}


void setLED(int state) {
  currentState = state;
  digitalWrite(PIN_GREEN, state == 0 ? HIGH : LOW);
  digitalWrite(PIN_YELLOW, state == 1 ? HIGH : LOW);
  digitalWrite(PIN_RED, state == 2 ? HIGH : LOW);
  Serial.print("LED State changed to: ");
  Serial.println(state);
}

