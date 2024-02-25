#include <Arduino.h>

#define CLOCK_PIN 26
#define DATA_PIN 25

#define BUFFER_SIZE 23
#define DATA_PERIOD_MS 80

uint8_t buff[BUFFER_SIZE];

uint8_t bitPos;
unsigned long lastPeriodMs;

void IRAM_ATTR _CLK_ISR() {
  unsigned long ms = millis();

  if (ms < lastPeriodMs) lastPeriodMs = ms;
  if (ms - lastPeriodMs < DATA_PERIOD_MS) return;

  if (bitPos == BUFFER_SIZE) bitPos = 0;

  if (bitPos < BUFFER_SIZE) {
    buff[bitPos++] = (GPIO.in >> DATA_PIN) & 0x1;
  }

  if (bitPos == BUFFER_SIZE) lastPeriodMs = ms;
}

float getMeasurement() {
  int8_t sign;
  int16_t value;

  sign = buff[20] == 0x1 ? -1 : 1;

  for (uint8_t i; i < BUFFER_SIZE; i++) {
    if (buff[i] == 0x1) {
      value |= 1 << i;
    }
  }

  return (value * sign) / 100.00;
}

bool isValueAvailable() {
  return bitPos == BUFFER_SIZE;
}

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  pinMode(CLOCK_PIN, INPUT);
  pinMode(DATA_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), _CLK_ISR, RISING);
}

unsigned long lastMillis;

void loop() {
  if (isValueAvailable() && (millis() - lastMillis) > 100) {
    Serial.println(getMeasurement());
    lastMillis = millis();
  }
}
