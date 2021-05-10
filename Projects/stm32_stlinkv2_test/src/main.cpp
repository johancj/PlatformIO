#include <Arduino.h>
// STLink v2 tetsed and works with debugging!!!

void setup() {
  pinMode(PC13, OUTPUT);
  Serial1.begin(115200);
}

void loop() {
  digitalWrite(PC13, HIGH);
  Serial1.println("HIGH");
  delay(250);
  digitalWrite(PC13, LOW);
  Serial1.println("LOW");
  delay(1000);
}