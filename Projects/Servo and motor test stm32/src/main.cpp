#include <Arduino.h>
#include <stdio.h>


#include "Servo.h"
#include "PID.h"
#include "USART.h"
#include "SBUS.h"


Servo myServo;
float value;


/////////// Global variables for the flight controller /////////////
PID_t pid;

uint8_t armed = 0;
uint8_t failsafe_flag = 1;


// Reciever variables
HardwareSerial Serial2(USART2);   //activating USART 2
SBUS rxsr(Serial2);

uint16_t channels[16];
bool failSafe = 1;
bool lostFrame;



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

void oneshot_125_send(float motor1_output){ // motor1_output E [1000, 2000]
  
  if(motor1_output < 1000.0f){
    motor1_output = 1000;
  }
  else if(motor1_output > 2000.0f){
    motor1_output = 2000;
  }

  TIM3->CNT = 0; //Clear counter on TIM3
  uint16_t m1_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor1_output - 1000.0f) + 9001.0f); //m1_val E [1, 9001] where 1 is 100% motor thrust
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

void flight_controller(void){
  // Max motor update frequency: 4 kHz
  // Interrupt for PIDs (100Hz?), gyro update(100/400Hz?), reciever?

  float m1_value = 0.0f; float m2_value = 0.0f; float m3_value = 0.0f; float m4_value = 0.0f;
  float throttle = 0.0f;
  float roll = 1500.0f;
  float pitch = 1500.0f;
  float yaw = 1500.0f;

  ////////// Initialisattions ///////////////
  oneshot_125_init();
  USART_init();
  PID_init(&pid);
  rxsr.begin();

  while (1){
    
    // look for a good SBUS packet from the receiver
    if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
                              ///////////////////// Change and check for correct channel!!! ////////////////////////
      throttle = channels[0];
      roll = channels[1];
      pitch = channels[2];
      yaw = channels[3];

      if(channels[4] > 1800){ ///////////////////// Change armed threshold!!! ////////////////////////
        armed = 1;
      }
      else{
        armed = 0;
      }
    }

    if(pid.PID_updated_flag){
      /*     roll
              ^       
         4    -    2    
              -    
         -----------> pitch         
              -           
         3    -    1     

         up is yaw. Props in configuration.
      */
      // Roll, pitch and yaw is derived from the PID controller.
      m1_value = throttle - roll - pitch 
      m2_value = throttle
      m3_value = throttle
      m4_value = throttle
    }

    if (!failsafe_flag && pid.PID_updated_flag && armed){
      pid.PID_updated_flag = 0;  
      oneshot_125_send(m1_value);
    }  
    else if(failsafe_flag | !armed){
      oneshot_125_send(0.0f);
    }

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
