#include <Arduino.h>
#include <stdio.h>


#include "Servo.h"
#include "PID.h"
#include "USART.h"


Servo myServo;
float value;

PID_t pid;



void oneshot_125_init(void){
  // Enable peripheral clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // Enable  I/O port A pheripheral clock and alternate function I/O clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 pheripheral clock
  
  // Configure PA6 as output as alt. funct. @ 50MHz
  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1; 
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

  // Using TIM3, Channel 1, with One-pulse mode
  TIM3->CR1 |= TIM_CR1_OPM; //Select One-pulse mode. Counter stops counting at the next update event (clearing the bit CEN).
  TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM3 as up counter

  // Configure OC1 as output
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 2 "In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive."
  TIM3->CCER &= ~TIM_CCER_CC1P; // OC1 active high
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; // TIM3, CH1 with output compare mode (output on PA6)
  
  TIM3->CNT = 0; //Clear counter register
  TIM3->PSC = 8; // Provides CK_CNT = f_(CK_PSC) / PSC = 72MHz/PSC = 72MHz counter on TIM3.
  TIM3->CCR1 = 9001; // 9000/(72MHz) = 125us delay. Range=[1, 9001] In OPM: t_delay = 1 (The pulse is sendt as soon as poosible, after the pulse is enabled)
  TIM3->ARR = 18001; // Initilaises with a 125us oneshot motor pulse (0% Throttle). t_pulse =  (TIMx_ARR - TIMx_CCR1)
}

void oneshot_125_send(float motor1_output){ // motor1_output E [0, 100]
  TIM3->CNT = 0; //Clear counter on TIM3
  uint16_t m1_val = ((1.0f - 9001.0f)/(100.0f - 0.0f)*(motor1_output - 0.0f) + 9001.0f); //m1_val E [1, 9001] where 1 is 100% motor thrust
  if(m1_val > 9001){m1_val = 9001;} //ensure the signal is within the output limits
  else if(m1_val < 1){m1_val = 1;}
  TIM3->CCR3 = m1_val;
  
  TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3 (all motors output)
}

void test_one_shot(void){
  pinMode(PA0, INPUT_PULLUP);
  USART_init();
  oneshot_125_init();
  while(1){
    value = analogRead(PA0);
    value = ((100.0f - 0.0f)/(900.0f)*(value) + 0.0f);
    oneshot_125_send(value);
  }
}

void setup() {
  pinMode(PA0, INPUT_PULLUP);
  //myServo.attach(PA6);
  //Serial1.begin(9600);
  oneshot_125_init();
  __enable_irq(); // Enable interrupts ///// Usikker på om denne er nødvendig ////////////
  USART_init();
  //PID_init(&pid);
}

void loop() {
  value = analogRead(PA0);
  value = ((100.0f - 0.0f)/(900.0f)*(value) + 0.0f);
  //value = (uint16_t) ((2000.0f - 900.0f)/(1000.0f)*((float)value) + 900.0f);
  //value = map(value, 0, 1000, 900, 2000);
  //Serial1.print(value); Serial1.print("\t");
  //uint16_t m1_val = ((1.0f - 9001.0f)/(100.0f - 0.0f)*(value - 0.0f) + 9001.0f);
  //Serial1.println(m1_val);
  //myServo.writeMicroseconds(value);
  //oneshot_125_send(value);
  myprintf("TIM2 CNT %d \n\r", TIM2->CNT);
  //delay(18);
}  
