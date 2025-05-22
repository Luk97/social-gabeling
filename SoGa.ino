#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define UNINITIALIZED -1
#define GREEN 0
#define YELLOW 1
#define RED 2

#define PIN_GREEN 11
#define PIN_YELLOW 12
#define PIN_RED 13

#define ACCEL_LOW_THRESHOLD 2
#define ACCEL_MEDIUM_THRESHOLD 5
#define STATE_CHANGE_COOLDOWN_MS 1000
#define DELAY_MS 200

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

unsigned long lastUpdateTime = 0;
int currentState = UNINITIALIZED;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not found");
    while (1);
  }

  // Initialize LEDs
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_YELLOW, LOW);
  digitalWrite(PIN_RED, LOW);
  

  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float x = accel.acceleration.x;
  float y = accel.acceleration.y;
  float z = accel.acceleration.z;

  Serial.print("Accel X: ");
  Serial.print(x);
  Serial.print("  Y: ");
  Serial.print(y);
  Serial.print("  Z: ");
  Serial.print(z);

  float magnitude = sqrt(x * x + y * y + z * z);
  float delta = abs(magnitude - 9.81);
  Serial.print("  Delta: ");
  Serial.println(delta);

  int newState;
  if (delta < ACCEL_LOW_THRESHOLD) {
    newState = GREEN;
  } else if (delta < ACCEL_MEDIUM_THRESHOLD) {
    newState = YELLOW;
  } else {
    newState = RED;
  }

  unsigned long currentTime = millis();
  if (newState != currentState && currentTime - lastUpdateTime >= STATE_CHANGE_COOLDOWN_MS) {
    currentState = newState;
    lastUpdateTime = currentTime;

    digitalWrite(PIN_GREEN, currentState == GREEN ? HIGH : LOW);
    digitalWrite(PIN_YELLOW, currentState == YELLOW ? HIGH : LOW);
    digitalWrite(PIN_RED, currentState == RED ? HIGH : LOW);

    Serial.print("State changed: ");
    Serial.println(currentState);
  }

  delay(DELAY_MS);
}
