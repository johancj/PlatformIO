#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "USART.h"

void SystemClock_Config(void);

//FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);

static void myprintf(char* msg, ...){
    
  char buffer[80];
  va_list args;
  va_start(args, msg);
  vsprintf(buffer, msg, args);
  for (uint16_t i = 0; i<strlen(buffer); i++){
    USART_transmitt(buffer[i]);
  }
}

int main(void){

  HAL_Init();
  SystemClock_Config();
  
  USART_init();

  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  //PC13 as push-pull output
  GPIOC->CRH |= GPIO_CRH_MODE13;
  GPIOC->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1 );
  //uint32_t counter = 0;
  //GPIOC->ODR |= GPIO_ODR_ODR13;
  
/*  
  for(uint32_t i = 0; i<2000; i++){
    GPIOC->ODR |= GPIO_ODR_ODR13;
  } */
  
    
  while (1){
    myprintf("Heisansveisan");
/*       
    GPIOC->ODR &= ~GPIO_ODR_ODR13;
    for(uint32_t i = 0; i<2000; i++){
      //GPIOC->ODR &= ~GPIO_ODR_ODR13;
    }
    GPIOC->ODR |= GPIO_ODR_ODR13;
    for(uint32_t i = 0; i<20000000; i++){
      //GPIOC->ODR |= GPIO_ODR_ODR13;
    } */

  }
    
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
    GPIOC->ODR &= ~GPIO_ODR_ODR13;
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    //Error_Handler();
    GPIOC->ODR &= ~GPIO_ODR_ODR13;

  }
  /** Enables the Clock Security System
  */
  //HAL_RCC_EnableCSS();
}


