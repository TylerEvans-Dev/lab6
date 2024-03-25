/**
  *
  * Brandon Mouser & Tyler Evans
  * U0962682, u1313811
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);
/*
 KEY Commands for terminal
 */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
    232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
 190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
 111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
 254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
//

/*
 useful things
 bit follows for the LED
 RED = 6
 BLUE = 7
 ORANGE = 8
 GREEN = 9
 */
int main(void)
{
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    
    
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // done to find the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enables GPIOA
    
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // enabling the clock for ADC
    
    //sets everything to zero in the pins
    // GPIOC is the GPIO_x where the pin is located.
    GPIOC->MODER &= 0; // sets the mode
    GPIOC->OTYPER &= 0; // sets what type
    GPIOC->OSPEEDR &= 0; // sets the speed
    GPIOC->PUPDR &= 0; // sets the pulldown/pullup resitor
    
    //sets all the values in modder to the correct pin into input mode.
    // the value is 01 for output mode.
    //               PC6        PC7       PC8       PC9
    GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
    /*
     DAC init.
     */
    //enabled clock
    //now choosing the pins.
    //PA4, DAC channel 1
    // this should be bits 9 and 8.
    GPIOA->MODER &= ~ ( (1<<9) | (1<<8));
    /*
     Be careful on mode select the channel 1
     */
    //setting the DAC to trigger enable
    //SWTRIGR enables channel one but does it automatically
    // this is in bit 0.
    //1 turns it on
    // this should be bit 2 and 1 sets it
    DAC1->CR |= (1<<2);
    //then to enable channel 1 is bit 0 with a value of 1
    DAC1->CR |= (1<<0);
    while(1) {
     //now this is where one will run the code repeatedly
        for(uint8_t i =0; i < 32; i++){
            //setting the values to be in the right channel 1 reg. 
            DAC1->DHR8R1 = sine_table[i];
            // one milisecond delay as requested
            HAL_Delay(1);
        }
        
     }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
