#include <Arduino.h>

struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button Monitor = {32, 0, false};
Button Radar = {33, 0, false};
Button Enviro = {34, 0, false};

void IRAM_ATTR isrMonitor() {
  Monitor.numberKeyPresses += 1;
  Monitor.pressed = true;
}
void IRAM_ATTR isrRadar() {
  Radar.numberKeyPresses += 1;
  Radar.pressed = true;
}
void IRAM_ATTR isrEnviro() {
  Enviro.numberKeyPresses += 1;
  Enviro.pressed = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(Monitor.PIN, INPUT_PULLUP);
  pinMode(Radar.PIN, INPUT_PULLUP);
  pinMode(Enviro.PIN, INPUT_PULLUP);
  attachInterrupt(Monitor.PIN, isrMonitor, FALLING);
  attachInterrupt(Radar.PIN, isrRadar, FALLING);
  attachInterrupt(Enviro.PIN, isrEnviro, FALLING);
}

void loop() {
  if (Monitor.pressed) {
      Serial.printf("Monitor has been pressed %u times\n", Monitor.numberKeyPresses);
      Monitor.pressed = false;
  }
  if (Radar.pressed) {
      Serial.printf("Radar has been pressed %u times\n", Radar.numberKeyPresses);
      Radar.pressed = false;
  }
  if (Enviro.pressed) {
      Serial.printf("Enviro has been pressed %u times\n", Enviro.numberKeyPresses);
      Enviro.pressed = false;
  }

  //Detach Interrupt after 1 Minute
  static uint32_t lastMillis = 0;
  if (millis() - lastMillis > 60000) {
    lastMillis = millis();
    detachInterrupt(Monitor.PIN);
    detachInterrupt(Radar.PIN);
    detachInterrupt(Enviro.PIN);
	Serial.println("Interrupt Detached!");
  }
}