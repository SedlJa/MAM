/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ARM03 - Stopwatch
 * @author         : Jan Sedlak
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PRESCALER 16000
#define AR 10 // period 0.01s
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// segments order: hgfedcba
const char NumbersASCII[10] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};

// Adress from right to left
const uint8_t LED_addr[4] = {
    0x08, // LED 1
    0x04, // LED 2
    0x02, // LED 3
    0x01  // LED 4
};

// Leading variable for the stopwatch (00.0i)
int i;
// Leading variable for the stopwatch (00.j0)
int j = 0;
// Leading variable for the stopwatch (0k.00)
int k = 0;
// Leading variable for the stopwatch (l0.00)
int l = 0;
// Variable to indicate if time > 10s
int t10 = 0;
// Start/Stop leading variable
int reset_pressed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void GPIO_Config();
void TIM2_Config();
void EXTI1_Config();
void EXTI15_10_Config();
void send_Number(char data, uint8_t LED_addr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  HAL_Init();           // Initialize the HAL library
  SystemClock_Config(); // Configure the system clock

  GPIO_Config();      // Config GPIOs
  TIM2_Config();      // Config timers - TIM2
  EXTI1_Config();     // Config External interrupts EXTI1
  EXTI15_10_Config(); // Config External interrupts EXTI15_10

  while (1)
  {
    if (t10 == 0)
    {
      send_Number(NumbersASCII[i], LED_addr[0]);
      HAL_Delay(1);
      send_Number(NumbersASCII[j], LED_addr[1]);
      HAL_Delay(1);
      send_Number(NumbersASCII[k] | 0x80, LED_addr[2]);
      HAL_Delay(1);
    }
    else
    {
      send_Number(NumbersASCII[i], LED_addr[0]);
      HAL_Delay(1);
      send_Number(NumbersASCII[j], LED_addr[1]);
      HAL_Delay(1);
      send_Number(NumbersASCII[k] | 0x80, LED_addr[2]);
      HAL_Delay(1);
      send_Number(NumbersASCII[l], LED_addr[3]);
      HAL_Delay(1);
    }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void GPIO_Config(void)
{
  // Enable GPIOA and GPIOC clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

  // Set PA5 as output (TIM2 LED)
  GPIOA->MODER |= GPIO_MODER_MODER5_0;

  // Set PA6 as output (Reset LED) + init value ON
  GPIOA->MODER |= GPIO_MODER_MODER6_0;
  GPIOA->ODR |= GPIO_ODR_OD6;

  // Set PA7 as output (Start/Stop LED) + init value OFF
  GPIOA->MODER |= GPIO_MODER_MODER7_0;
  GPIOA->ODR |= GPIO_ODR_OD7;

  // Set PA1 as input (Button - Reset)
  GPIOA->MODER &= ~GPIO_MODER_MODER1;

  // Set PA4 as input (Button - Start/Stop)
  GPIOA->MODER &= ~GPIO_MODER_MODER4;

  // Set PB5 as output (Latch - start low/end high)
  GPIOB->MODER |= GPIO_MODER_MODER5_0;

  // Set PA8 as output (SHCP - clock signat to LED panel)
  GPIOA->MODER |= GPIO_MODER_MODER8_0;

  // Set PA9 as output (STCP - DATA to LED panel)
  GPIOA->MODER |= GPIO_MODER_MODER9_0;
}

void TIM2_Config()
{
  // Enable Timer 2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Set prescaler and auto-reload value
  TIM2->PSC = PRESCALER - 1; // Prescaler value
  TIM2->ARR = AR - 1;        // Auto-reload value

  // Enable update interrupt
  TIM2->DIER |= TIM_DIER_UIE;

  // Enable Timer 2
  TIM2->CR1 |= TIM_CR1_CEN;

  // Enable Timer 2 interrupt in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    // Clear update interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;

    // Toggle LED on PA5
    GPIOA->ODR ^= GPIO_ODR_OD5;

    // Count Numbers to be sent to LEDs
    i++;
    if (i == 10)
    {
      i = 0;
      j++;
      if (j == 10)
      {
        j = 0;
        k++;
        if (k == 10)
        {
          k = 0;
          l++;
          t10 = 1;
          if (l == 10)
          {
            l = 0;
          }
        }
      }
    }
  }
}

void EXTI1_Config()
{
  // Enable SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Connect EXTI Line1 to PA1
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // set port A as input

  // Configure EXTI Line1
  EXTI->IMR |= EXTI_IMR_IM1;   // Unmask interrupt
  EXTI->FTSR |= EXTI_FTSR_TR1; // Falling edge trigger

  // Enable EXTI1 interrupt in NVIC
  NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void)
{
  if (EXTI->PR & EXTI_PR_PR1)
  {
    // Clear interrupt flag
    EXTI->PR |= EXTI_PR_PR1;

    // Reset stopwatch variables
    i = 0;
    j = 0;
    k = 0;
    l = 0;

    // Reset time > 10s variable
    t10 = 0;

    // Toggle Reset LED on
    GPIOA->ODR ^= GPIO_ODR_OD6;
  }
}

void EXTI15_10_Config(void)
{
  // Enable SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Connect EXTI Line4 to PA4
  SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;   // Clear port selection
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA; // set port A as input

  // Configure EXTI Line4
  EXTI->IMR |= EXTI_IMR_IM4;   // Unmask interrupt
  EXTI->FTSR |= EXTI_FTSR_TR4; // Falling edge trigger

  // Enable EXTI4 interrupt in NVIC
  NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void)
{
  if (EXTI->PR & EXTI_PR_PR4)
  {
    // Clear interrupt flag
    EXTI->PR |= EXTI_PR_PR4;

    // Toggle the start/stop state and Start/Stop LED
    reset_pressed = 1;
    GPIOA->ODR ^= GPIO_ODR_OD7;

    // Start/Stop Timer
    if (reset_pressed == 1)
    {
      if (TIM2->CR1 & TIM_CR1_CEN)
      {
        TIM2->CR1 &= ~TIM_CR1_CEN;   // Stop timer
        GPIOA->ODR &= ~GPIO_ODR_OD5; // Turn OFF TIM2 LED
      }
      else
      {
        TIM2->CR1 |= TIM_CR1_CEN; 	 // Enable Timer
      }
    }
  }
}

/**
 * @brief Send number to 7-segment display
 *
 * @param  data Number to be displayed
 * @param  LED_addr address of the LED display
 */
void send_Number(char data, uint8_t LED_addr)
{
  // Set Latch to low - Start condition
  GPIOB->ODR &= ~GPIO_ODR_OD5;

  // Send number to LED display
  for (int i = 0; i < 8; i++)
  {
    // Set clock to low
    GPIOA->ODR &= ~GPIO_ODR_OD8;

    // Set data to be sent
    if (!(data & 0x80))
    {
      GPIOA->ODR |= GPIO_ODR_OD9;
    }
    else
    {
      GPIOA->ODR &= ~GPIO_ODR_OD9;
    }

    // Set clock to high
    GPIOA->ODR |= GPIO_ODR_OD8;

    // Shift data to the left
    data <<= 1;
  }

  // Send address of the LED display
  for (int i = 0; i < 8; i++)
  {
    // Set clock to low
    GPIOA->ODR &= ~GPIO_ODR_OD8;

    // Set data to be sent
    if (LED_addr & 0x80)
    {
      GPIOA->ODR |= GPIO_ODR_OD9;
    }
    else
    {
      GPIOA->ODR &= ~GPIO_ODR_OD9;
    }

    // Set clock to high
    GPIOA->ODR |= GPIO_ODR_OD8;

    // Shift data to the left
    LED_addr <<= 1;
  }

  // Set Latch to high - End condition
  GPIOB->ODR |= GPIO_ODR_OD5;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
