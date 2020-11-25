#include <Arduino.h>
#include"SBUS.h"

SBUS rxsr(Serial2);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

void setup() {
  rxsr.begin();
  Serial1.begin(115200);
}

void loop() {
  // look for a good SBUS packet from the receiver
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    for(uint8_t i = 0; i < 16; i++){
      Serial2.printf("Channel %d: %d \t", i, channels[i]);
    }
    Serial2.printf("\n\r Failsafe: %d \t and lostFrame: %d \n\r", failSafe, lostFrame);

  }
}

