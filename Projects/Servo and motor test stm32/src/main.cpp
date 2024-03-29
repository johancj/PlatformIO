#include <Arduino.h>
#include <stdio.h>


#include "SBUS.h"

extern "C"{
#include "PID.h"
#include "USART.h"
#include "receiver.h"
}

// Testing only
float value;

/////////// Timers and pins overview /////////////
/*
TIM3: Motor output (T3C1-T3C4)
  - PA6, PA7, PB0, PB1
TIM2: PID controller interrupt on overflow (100Hz) (To be adjusted to 1kHz. Can possibly use the mpu6050 external interrupt.)
TIM4: read receiver interrupt on compare 1 register (25Hz. To be adjusted to 50Hz.)
      read LiPo battery voltage interrupt on overflow (25Hz. To be adjusted to 50Hz.)    
        - PA4 for ADC4

Reciever SBUS input (USART3)
 - PB11 (RX3)

Programming pins(USART1)
  - PA9 (TX1)
  - PA10 (RX1)

Gyro and accelerometer (mpu6050 using I2C1)
  - PB6 (SCL1)
  - PB7 (SDA1)
  - PA3 external interrupt pin

Test Pins
  - PA5: analog input of potentiometer
  - PA2(Not implimented): Digital output for testing of i.e. loop times with ocilloscope
*/

/////////// Global variables for the flight controller /////////////
PID_t pid; // all PID values

uint8_t armed = 0;

volatile uint8_t count_interrupts_PID = 0; // For testing only
volatile uint8_t count_interrupts_reciever = 0; // For testing only

volatile uint16_t adc_val_raw = 0;

// Reciever variables
HardwareSerial Serial3(USART3);   //activating USART 3
SBUS rxsr(Serial3);

uint16_t channels[16];
bool failSafe = 1;
bool lostFrame;

receiver_t receiver;

volatile uint8_t read_receiver_flag = 1;

// motor variables
float motor_max_clamping = 2000.0f;
float motor_min_clamping = 1000.0f;

void oneshot_125_init_servo(void){
  // Enable peripheral clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // Enable  I/O port A pheripheral clock and alternate function I/O clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 pheripheral clock
  
  // Configure PA6 as output as alt. funct. @ 50MHz
  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1; 
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

  // Using TIM3, Channel 1, with One-pulse mode
  TIM3->CR1 |= TIM_CR1_OPM; // Select One-pulse mode. Counter stops counting at the next update event (clearing the bit CEN).
  TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM3 as up counter

  // Configure OC1 as output
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 2 "In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive."
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; // TIM3, CH1 with output compare mode (output on PA6)
  TIM3->CCER &= ~TIM_CCER_CC1P; // OC1 active high
  TIM3->CCER|= TIM_CCER_CC1E; // Capture/Compare 1 output enable
  TIM3->BDTR |= TIM_BDTR_MOE;
  
  TIM3->CNT = 0; //Clear counter register
  TIM3->PSC = 8 - 1; // Provides CK_CNT = f_(CK_PSC) / (PSC + 1) = 72MHz/(PSC + 1) = 72MHz counter on TIM3. (With PSC=8-1: 9MHz)
  TIM3->CCR1 = 9001; // 9000/(72MHz) = 125us delay. Range=[1, 9001] In OPM: t_delay = 1 (The pulse is sendt as soon as poosible, after the pulse is enabled)
  TIM3->ARR = 18001; // Initilaises with a 125us oneshot motor pulse (0% Throttle). t_pulse =  (TIMx_ARR - TIMx_CCR1)

  TIM3->EGR |= TIM_EGR_UG; // Update generation
}

void oneshot_125_init(void){
  // Enable peripheral clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // Enable  I/O port A pheripheral clock and alternate function I/O clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 pheripheral clock
  
  // Configure PA6 as output as alt. funct. @ 50MHz
  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1; 
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

  // Using TIM3, Channel 1, with One-pulse mode
  TIM3->CR1 |= TIM_CR1_OPM; // Select One-pulse mode. Counter stops counting at the next update event (clearing the bit CEN).
  TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Sets TIM3 as up counter

  // Configure OC1 as output
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 2 "In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive."
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; // TIM3, CH1 with output compare mode (output on PA6)
  TIM3->CCER &= ~TIM_CCER_CC1P; // OC1 active high
  TIM3->CCER|= TIM_CCER_CC1E; // Capture/Compare 1 output enable
  TIM3->BDTR |= TIM_BDTR_MOE;
  
  TIM3->CNT = 0; //Clear counter register
  TIM3->PSC = 0; // Provides CK_CNT = f_(CK_PSC) / (PSC + 1) = 72MHz/(PSC + 1) = 72MHz counter on TIM3.
  TIM3->CCR1 = 9001; // 9000/(72MHz) = 125us delay. Range=[1, 9001] In OPM: t_delay = 1 (The pulse is sendt as soon as poosible, after the pulse is enabled)
  TIM3->ARR = 18001; // Initilaises with a 125us oneshot motor pulse (0% Throttle). t_pulse =  (TIMx_ARR - TIMx_CCR1)

  TIM3->EGR |= TIM_EGR_UG; // Update generation
}

void oneshot_125_send(float motor1_output){ // motor1_output E [1000, 2000]
  
  if(motor1_output < 1000.0f){
    motor1_output = 1000;
  }
  else if(motor1_output > 2000.0f){
    motor1_output = 2000;
  }

  uint16_t m1_val = ((1.0f - 9001.0f)/(2000.0f - 1000.0f)*(motor1_output - 1000.0f) + 9001.0f); //m1_val E [1, 9001] where 1 is 100% motor thrust
  if(m1_val > 9001){m1_val = 9001;} // Ensure the signal is within the output limits
  else if(m1_val < 1){m1_val = 1;}
  //Serial.print("CCR1 = "); Serial1.println(m1_val);
  TIM3->CCR1 = m1_val;
  
  TIM3->EGR |= TIM_EGR_UG; // Update generation
  TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3 (all motors outputs)
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

void test_one_shot_using_servo(void){
  pinMode(PA5, INPUT_PULLUP);
  oneshot_125_init_servo();
  while(1){
    value = analogRead(PA5);
    value = ((100.0f - 0.0f)/(1023.0f - 20.0f)*(value - 20.0f) + 0.0f);
    value = value * 10 + 1000;
    
    Serial.print("Value = "); Serial1.println(value);
    
    oneshot_125_send(value);

    delay(25);
  }
}

void test_one_shot(void){
  pinMode(PA5, INPUT_PULLUP);
  oneshot_125_init();
  while(1){
    value = analogRead(PA5);
    value = ((100.0f - 0.0f)/(1023.0f - 20.0f)*(value - 20.0f) + 0.0f);
    value = value * 10 + 1000;
    
    Serial.print("Value = "); Serial1.println(value);
    
    oneshot_125_send(value);

    delay(2);
  }
}

void battery_read_init(void){

  ///// Works without interrupts ////////

  // Set ADC pheripheral clock to sys_clk / 6 = 12MHz (max 14MHz)
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

  //Enable adc- and GPIOA pheripheral clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; 

  // Set PA4 as input push-pull (analoge (ADC requirement))
  GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4_0);
  GPIOA->CRL |= GPIO_CRL_CNF4_1;

  

  ADC1->SMPR2 |= ADC_SMPR2_SMP4; // Sample time selection
  ADC1->SQR1 &= ~ADC_SQR1_L; // 1 convertion only
  ADC1->SQR3 = (4 << ADC_SQR3_SQ1_Pos); // Use ADC channel 4 (PA4)

  ADC1->CR2 |= ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_ADON;

  delay(5);

  // Actually turn on adc (needs to be turned on twise)
  ADC1->CR2 |= ADC_CR2_ADON;
  delay(5);

  ADC1->CR2 |= ADC_CR2_CAL;
  delay(5);
  //while(ADC1->CR2 & ADC_CR2_CAL_Msk);

  // Enable end of convertion interrupt 
  //ADC1->CR1 |= ADC_CR1_EOCIE;
  //NVIC_EnableIRQ(ADC1_IRQn);
}

void external_mpu6050_interrupt_init(void){
  // External interrupt from mpu6050 on pin PA3

  /////// Read from PA3 works, but the external interrupt doesn't work for some reason... ////////

  // Enable GPIOA peripheral clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  // Set PA3 as input with pull-down enabled
  GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
  GPIOA->CRL |= GPIO_CRL_CNF3_1;
  GPIOA->ODR &= ~GPIO_ODR_ODR3;
  
  

  // Enable alternate function peripheral clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  
  AFIO->EXTICR[0] &= ~(0xF << AFIO_EXTICR1_EXTI3_Pos); // GPIOA is used (on pin 3)
  
  EXTI->IMR |= EXTI_IMR_IM3; // unmasking interrupt mask for pin 3
  EXTI->RTSR |= EXTI_RTSR_RT3; // Interrupt trigger on rising edge on pin 3


  // Enable interrupt
  NVIC_EnableIRQ(EXTI3_IRQn); 



}

void test_external_interrupts(void){
  external_mpu6050_interrupt_init();
  while(1){
    myprintf("PA3 read: %d \n", ((GPIOA->IDR & GPIO_IDR_IDR3_Msk) >> GPIO_IDR_IDR3_Pos));
    delay(250);
  } 
}

void test_ADC(void){
  battery_read_init();
  while (1) {
    Serial1.printf("ADC read: %d \n", ADC1->DR);
    delay(250);
  }
}

void flight_controller(void){
  // Max motor update frequency: 4 kHz
  // Interrupt for PIDs (100Hz? To be adjusted to 1kHz.), gyro update(100/400Hz? To be adjusted to 1kHz.), reciever and battery voltage(25Hz. To be adjusted to 50Hz.)


  float m1_value = 1000.0f; float m2_value = 1000.0f; float m3_value = 1000.0f; float m4_value = 1000.0f;
  
  

  ////////// Initialisattions ///////////////
  oneshot_125_init();
  USART_init();
  PID_init(&pid);
  rxsr.begin();
  receiver_init(&receiver);

  while (1){

    if(read_receiver_flag){ // look for a good SBUS packet from the receiver
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
  //USART_init();
  Serial1.begin(115200);
  delay(50);
  myprintf((char*)"setup() \n");
  delay(2);


  __enable_irq(); // Enable interrupts ///// Usikker på om denne er nødvendig ////////////
  
  PID_init(&pid);
  //test_one_shot_using_servo();
  
  
 
  //test_external_interrupts();
  
  //test_ADC();
  
}

void loop() {
  
  //value = analogRead(PA5);
  //value = ((100.0f - 0.0f)/(900.0f)*(value) + 0.0f);
  //value = (uint16_t) ((2000.0f - 900.0f)/(1000.0f)*((float)value) + 900.0f);
  //value = map(value, 0, 1000, 900, 2000);
  //Serial1.print(value); Serial1.print("\t");
  //uint16_t m1_val = ((1.0f - 9001.0f)/(100.0f - 0.0f)*(value - 0.0f) + 9001.0f);
  //Serial1.println(m1_val);

  //oneshot_125_send(value);
  //delayMicroseconds(4000);
  Serial1.printf("TIM2 CNT %d \n\r", TIM2->CNT);
  //delay(250);
}  


// You find interrupt handler names in "stm32f103xb.h". Just add "Handler" after "IRQ"!

void TIM2_IRQHandler(void){ //PID timer(100Hz), TIM2 global handler
	Serial1.printf("TIM2_IRQHandler #%d\n\r", count_interrupts_PID++);
  delay(10);
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
  myprintf("%d\n", ADC1->SR);
  delay(2);
  if(ADC1->SR & ADC_SR_EOC){
    adc_val_raw = ADC1->DR;
  }
}

void EXTI3_IRQHandler(void){
  Serial1.println("EXTI3_IRQHandler");
  EXTI->PR |= EXTI_PR_PIF3; // Clear interrupt flag
}
