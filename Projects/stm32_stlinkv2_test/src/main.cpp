#include <Arduino.h>
// STLink v2 tetsed and works with debugging!!!

void setup() {
  pinMode(PC13, OUTPUT);
}

void loop() {
  digitalWrite(PC13, HIGH);
  delay(250);
  digitalWrite(PC13, LOW);
  delay(2500);
}