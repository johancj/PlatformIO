#include <Arduino.h>
#include"SBUS.h"

/*
  Tested and working code
*/


HardwareSerial Serial3(USART3);   //activating USART 3

SBUS rxsr(Serial3);



// channel, fail safe, and lost frames data
uint16_t channels[16];l
bool failSafe;
bool lostFrame;

void setup() {
  rxsr.begin();
  Serial1.begin(115200);
}

void loop() {
  // look for a good SBUS packet from the receiver
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    for(uint8_t i = 0; i < 9; i++){
      Serial1.printf("Ch[%d]: %d \t", i, channels[i]);
    }
    Serial1.printf(" Failsafe: %d  and lostFrame: %d \n\r", failSafe, lostFrame);
  } 
}


