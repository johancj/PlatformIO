#include <Arduino.h>
//#include "stm32f1xx.h"
#define onboard_led PC13

void setup() {
  // put your setup code here, to run once:
  pinMode(onboard_led, OUTPUT);
  Serial1.begin(115200);
}
int i=1;


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(onboard_led, HIGH);
  delay(500);
  digitalWrite(onboard_led, LOW);
  delay(500);
  Serial1.printf("Runde %d i loopen! \n", i++);
}