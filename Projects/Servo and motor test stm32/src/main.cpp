#include <Arduino.h>
#include <stdio.h>


#include "Servo.h"
#include "PID.h"
#include "USART.h"
#include "SBUS.h"
#include "receiver.h"


Servo myServo;
float value;

/////////// Timers and pins overview /////////////
/*
TIM3: Motor output
  - PA0 to PA3
TIM2: PID controller interrupt on overflow (100Hz)
TIM4: read receiver interrupt on compare 1 register (25Hz)
      read LiPo battery voltage interrupt on overflow (25Hz)    
        - PA4 for ADC4
*/

/////////// Global variables for the flight controller /////////////
PID_t pid; // all PID values

uint8_t armed = 0;

volatile uint8_t count_interrupts_PID = 0; // For testing only
volatile uint8_t count_interrupts_reciever = 0; // For testing only

volatile uint16_t adc_val_raw = 0;

// Reciever variables
HardwareSerial Serial2(USART2);   //activating USART 2
SBUS rxsr(Serial2);

uint16_t channels[16];
bool failSafe = 1;
bool lostFrame;

receiver_t receiver;

volatile uint8_t read_receiver_flag = 1;

// motor variables
float motor_max_clamping = 2000.0f;
float motor_min_clamping = 1000.0f;

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

void set_motor_speed(float motor1_output, float motor2_output, float motor3_output, float motor4_output, uint8_t stop_motors){ // motorx_output E [1000, 2000]
  
  ///////////////// Clamping motor values to be in a leagal range ///////////////////////////
  if(motor1_output < motor_min_clamping){
    motor1_output = motor_min_clamping;
  }
  else if(motor1_output > motor_max_clamping){
    motor1_output = motor_max_clamping;
  }
  
  if(motor2_output < motor_min_clamping){
    motor2_output = motor_min_clamping;
  }
  else if(motor2_output > motor_max_clamping){
    motor2_output = motor_max_clamping;
  }

  if(motor3_output < motor_min_clamping){
    motor3_output = motor_min_clamping;
  }
  else if(motor3_output > motor_max_clamping){
    motor3_output = motor_max_clamping;
  }

  if(motor4_output < motor_min_clamping){
    motor4_output = motor_min_clamping;
  }
  else if(motor4_output > motor_max_clamping){
    motor4_output = motor_max_clamping;
  }

  // Checking if the motors should stop
  if(stop_motors){
    motor1_output = 1000.0f;
    motor2_output = 1000.0f;
    motor3_output = 1000.0f;
    motor4_output = 1000.0f;
  }

  
  uint16_t m1_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor1_output - 1000.0f) + 9001.0f); //m1_val E [1, 9001] where 1 is 100% motor thrust
  if(m1_val > 9001){m1_val = 9001;} //ensure the signal is within the output limits
  else if(m1_val < 1){m1_val = 1;}

  uint16_t m2_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor2_output - 1000.0f) + 9001.0f); //m2_val E [1, 9001] where 1 is 100% motor thrust
  if(m2_val > 9001){m2_val = 9001;} //ensure the signal is within the output limits
  else if(m2_val < 1){m2_val = 1;}

  uint16_t m3_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor3_output - 1000.0f) + 9001.0f); //m3_val E [1, 9001] where 1 is 100% motor thrust
  if(m3_val > 9001){m3_val = 9001;} //ensure the signal is within the output limits
  else if(m3_val < 1){m3_val = 1;}

  uint16_t m4_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor4_output - 1000.0f) + 9001.0f); //m4_val E [1, 9001] where 1 is 100% motor thrust
  if(m4_val > 9001){m4_val = 9001;} //ensure the signal is within the output limits
  else if(m4_val < 1){m4_val = 1;}
  
  
  TIM3->CNT = 0; //Clear counter for TIM3
  // Sets one_shot_125 values in the registers
  TIM3->CCR1 = m1_val;
  TIM3->CCR2 = m2_val;
  TIM3->CCR3 = m3_val;
  TIM3->CCR4 = m4_val;
  
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

void battery_read_init(void){
  
  // Set ADC pheripheral clock to sys_clk / 6 = 12MHz (max 14MHz)
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

  //Enable adc- and GPIOA pheripheral clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN; 

  // Set PA4 as input with pull-down resistor
  GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4_0);
  GPIOA->CRL |= GPIO_CRL_CNF4_1;
  GPIOA->ODR &= ~GPIO_ODR_ODR4;

  // Enable end of convertion interrupt
  ADC1->CR1 |= ADC_CR1_EOCIE;
  NVIC_EnableIRQ(ADC1_IRQn);

  ADC1->SMPR2 |= ADC_SMPR2_SMP4; // Sample time selection
  ADC1->SQR1 &= ~ADC_SQR1_L; // 1 convertion only
  ADC1->SQR3 = (4 << ADC_SQR3_SQ1_Pos); // Use ADC channel 4 (PA4)

  ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON;

  delay(1);

  // Actually turn on adc (needs to be turned on twise)
  ADC1->CR2 |= ADC_CR2_ADON;
  delay(1);

  ADC1->CR2 |= ADC_CR2_CAL;
  while(ADC1->CR2 & ADC_CR2_CAL_Msk);

}

void flight_controller(void){
  // Max motor update frequency: 4 kHz
  // Interrupt for PIDs (100Hz?), gyro update(100/400Hz?), reciever and battery voltage(25Hz)

  float m1_value = 1000.0f; float m2_value = 1000.0f; float m3_value = 1000.0f; float m4_value = 1000.0f;
  
  

  ////////// Initialisattions ///////////////
  oneshot_125_init();
  USART_init();
  PID_init(&pid);
  rxsr.begin();
  receiver_init(&receiver);

  while (1){

    if(read_receiver_flag){
      // look for a good SBUS packet from the receiver
      if(rxsr.read(&channels[0], &failSafe, &lostFrame)){

        read_receiver_flag = 0;
                                ///////////////////// Change and check for correct channel!!! ////////////////////////
        receiver.throttle = channels[0];
        receiver.roll = channels[1];
        receiver.pitch = channels[2];
        receiver.yaw = channels[3];

        if(channels[4] > 1800){ ///////////////////// Change armed threshold!!! ////////////////////////
          armed = 1;
        }
        else{
          armed = 0;
        }

        if(failSafe | !armed){
          //set_motor_speed(0.0f, 0.0f, 0.0f, 0.0f, 1);
          printf("Stop motors! \n\r");

          pid.pid_updated_flag = 0; 
        }
        update_target_vals(&pid, &receiver);
      }
    }
    

    if(pid.pid_updated_flag && armed){
      pid.pid_updated_flag = 0;
      
      /* Definitions of motors and rollpitch and yaw vectors   

                    roll
                     ^       
                4    -    2    
                     -    
                -----------> pitch         
                     -           
                3    -    1     

                yaw is up. 
         The propellers are in props-in configuration.
      */
      
      // Roll, pitch and yaw is derived from the PID controller.       
      m1_value = receiver.throttle - pid.roll.output - pid.pitch.output - pid.yaw.output;
      m2_value = receiver.throttle - pid.roll.output + pid.pitch.output + pid.yaw.output;
      m3_value = receiver.throttle + pid.roll.output - pid.pitch.output + pid.yaw.output;
      m4_value = receiver.throttle + pid.roll.output + pid.pitch.output - pid.yaw.output;
      
      if (!failSafe && armed){ //Send motor commands
        //set_motor_speed(m1_value, m2_value, m3_value, m4_value, 0);
        printf("Run motors! \n\r");
        pid.pid_updated_flag = 0; 
      }  
    }

    if(failSafe | !armed){
      //set_motor_speed(0.0f, 0.0f, 0.0f, 0.0f, 1);
      printf("Stop motors! \n\r");
      pid.pid_updated_flag = 0; 
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

void TIM2_IRQHandler(void){ //PID timer(100Hz), TIM2 global handler
	myprintf("TIM2_IRQHandler #%d\n\r", count_interrupts_PID++);
	//PID_update(&pid);
	TIM2->SR &= ~TIM_SR_UIF; // clear update interrupt flag
} 

void TIM4_IRQHandler(void){ //receiver and battery read (25Hz), TIM4 global handler
	myprintf("TIM4_IRQHandler #%d\n\r", count_interrupts_reciever++);
  read_receiver_flag = 1;
	TIM4->SR &= ~TIM_SR_UIF; // clear update interrupt flag
} 

void ADC1_2_IRQHandler(void){
  // Check if we are here due to the end of convertion flag
  // Clear the flag by reading the data register
  if(ADC1->SR & ADC_SR_EOC){
    adc_val_raw = ADC1->DR;
  }
}


