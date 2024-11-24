/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ARMO04 - Terminal with USART
 * @author			: Jan Sedlak
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
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESCALER 16000
#define AR 10 // period 0.01s
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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

char InputBuffer[5]; // Buffer for received data
bool isNumber;
static uint8_t index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void GPIO_Config();
void TIM2_Config();
void USART2_Config();
void USART2_sendTxt(char *txt);
void USART2_sendChar(char c);
void USART2_receiveBuffer(char *buffer);
void send_Number(char data, uint8_t LED_addr);
void send_Error();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  GPIO_Config();
  TIM2_Config();
  USART2_Config();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    index = 0; // Reset index for the next input

    // Check if the input contains any letter
    if (isNumber == true)
    {
      // Convert input to integer
      int number = atoi(InputBuffer);

      // Determine the number of digits
      int numDigits = 0;
      int temp = number;
      do
      {
        numDigits++;
        temp /= 10;
      } while (temp > 0);

      // Display each digit on the corresponding LED display
      for (int i = 0; i < numDigits; i++)
      {
        int digit = number % 10;
        number /= 10;
        send_Number(NumbersASCII[digit], LED_addr[i]);
        HAL_Delay(1); // Small delay to ensure proper display
      }
    }
    else
    {
      send_Error();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
void GPIO_Config()
{
  // TO DO - GPIO configuration

  // Enable clock to GPIOA; GPIOB
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

  // Set TIM2 LED to PA5 pin
  GPIOA->MODER |= GPIO_MODER_MODER5;

  // Set PB5 as output (Latch - start low/end high)
  GPIOB->MODER |= GPIO_MODER_MODER5_0;

  // Set PA8 as output (SHCP - clock signat to LED panel)
  GPIOA->MODER |= GPIO_MODER_MODER8_0;

  // Set PA9 as output (STCP - DATA to LED panel)
  GPIOA->MODER |= GPIO_MODER_MODER9_0;
}

/*	TIM2 configuration function	*/
void TIM2_Config()
{
  // Enable TIM2 clock
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

    // Toggle TIM2 LED
    GPIOA->ODR ^= GPIO_ODR_OD5;
  }
}

/* USART configuration function */
void USART2_Config()
{
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // povoleni hodin pro usart2
  USART2->BRR = (8 << 4) | 11;          // 8.6875
  USART2->CR1 |= USART_CR1_UE;          // povoleni USART rozhrani
  USART2->CR1 |= USART_CR1_RE;          // povoleni prijimace
  USART2->CR1 |= USART_CR1_TE;          // povoleni odesilaciho bloku

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;          // povoleni hodin pro GPIOA
  GPIOA->MODER |= 0x02 << GPIO_MODER_MODE2_Pos; // nastaveni alt fce pro PA2
  GPIOA->MODER |= 0x02 << GPIO_MODER_MODE3_Pos; // nastaveni alt fce pro PA3

  GPIOA->AFR[0] |= 0x07 << GPIO_AFRL_AFSEL2_Pos; // vyber alt fce 7 (USART2) pro PA2
  GPIOA->AFR[0] |= 0x07 << GPIO_AFRL_AFSEL3_Pos; // vyber alt fce 7 (USART2) pro PA3

  USART2->CR1 |= USART_CR1_RXNEIE; // povoleni preruseni pri prijeti znaku
  NVIC_EnableIRQ(USART2_IRQn);     // povoleni preruseni od USART2
}

void USART2_IRQHandler(void)
{
  if (USART2->SR & USART_SR_RXNE)
  {
    // Read Data Register to clear RXNE flag
    char received_char = USART2->DR;

    // Check if received character is a number
    if (received_char >= '0' && received_char <= '9')
    {
      isNumber = true;
      USART2_sendChar(received_char);
    }
    else
    {
      isNumber = false;
    }

    // Store received character in buffer
    if (received_char != '\n' && index < sizeof(InputBuffer) - 1)
    {
      InputBuffer[index++] = received_char;
    }
    else
    {
      InputBuffer[index - 1] = '\0'; // Null-terminate the string
    }
  }
}

// USART char sending function
void USART2_sendChar(char c)
{
  while (!(USART2->SR & USART_SR_TXE))
    ; // Clear space for transmit
  USART2->DR = c;
}

// USART text sending function
void USART2_sendTxt(char *txt)
{
  while (*txt)
  {
    USART2_sendChar(*txt++); // Char shift
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

// Send error to LED display
void send_Error()
{
  // Set Latch to low - Start condition
  GPIOB->ODR &= ~GPIO_ODR_OD5;

  // Send 'E' to LED display
  send_Number(0x79, LED_addr[2]);
  HAL_Delay(1);

  // Send 'r' to LED display
  send_Number(0x50, LED_addr[1]);
  HAL_Delay(1);

  // Send 'r' to LED display
  send_Number(0x50 | 0x80, LED_addr[0]);
  HAL_Delay(1);

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
    send_Error();
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
