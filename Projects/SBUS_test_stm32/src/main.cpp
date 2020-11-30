#include <Arduino.h>
#include"SBUS.h"

HardwareSerial Serial2(USART2);   //activating USART 2

SBUS rxsr(Serial2);

//GPIOA->BSRR = 0x00000000;

GPIOD->BSRR = 0x00000001;

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

void setup() {
  rxsr.begin();
  Serial1.begin(9600);
}

void loop() {
  // look for a good SBUS packet from the receiver
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    for(uint8_t i = 0; i < 9; i++){
      Serial1.printf("Ch[%d]: %d   ", i, channels[i]);
    }
    Serial1.printf(" Failsafe: %d  and lostFrame: %d \n\r", failSafe, lostFrame);
  } 
}


