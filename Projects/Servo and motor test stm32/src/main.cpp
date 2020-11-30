#include <Arduino.h>


#include"Servo.h"


Servo myServo;
uint16_t value;

void oneshot_125_ini(void){
  
  // Enable peripheral clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; // Enable  I/O port B pheripheral clock and alternate function I/O clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 pheripheral clock
  
  // Configure PB6 as output as alt. funct. @ 50MHz
  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1; 
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

  //Using TIM3, Channel 1, with One-pulse mode
  TIM3->CR1 |= TIM_CR1_OPM; //Select One-pulse mode. Counter stops counting at the next update event (clearing the bit CEN).
  TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM3 as up counter

  TIM3->PSC = 71; // Provides CK_CNT = f_(CK_PSC) / (PSC + 1) = 72MHz/(PSC+1) = 1MHz counter on TIM3.
  TIM3->CCR3 = 1; // t_delay = 1 (The pulse is sendt as soon as poosible, after the pulse is enabled)
  TIM3->ARR = 127; // Initilaises with a 125us oneshot motor pulse (0% Throttle). t_pulse =  (TIMx_ARR - TIMx_CCR + 1)
  

  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; //TIM3, CH1 with compare mode (output on PB6)
  TIM3->CCMR1 |= (0b111UL << TIM_CCMR1_OC1M_Pos); // PWM-mode 2; " In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active."
  //TIM3->CR1 |= TIM_CR1_CEN; //Enable TIM3

}

void setup() {
  pinMode(PA0, INPUT_PULLUP);
  myServo.attach(PB11);
  Serial1.begin(9600);
}

void loop() {
  value = analogRead(PA0);
  value = (uint16_t)((float)2000 - (float)900)/((float) 1000)*((float)value) + 900;
  //value = map(value, 0, 1000, 900, 2000);
  Serial1.println(value);
  myServo.writeMicroseconds(value);
}  
