/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS1302_REG_SEC              0x80
#define DS1302_REG_MIN              0x82
#define DS1302_REG_HOUR             0x84
#define DS1302_REG_DATE             0x86
#define DS1302_REG_MONTH            0x88
#define DS1302_REG_DAY              0x8A
#define DS1302_REG_YEAR             0x8C

// GPIO Pins
#define DS1302_GPIO	GPIOA
#define DS1302_SCLK	GPIO_PIN_1
#define DS1302_DATA	GPIO_PIN_2
#define DS1302_RST	GPIO_PIN_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t seconds = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void delayUS_DWT(uint32_t);
static void DS1302_SendCmd(uint8_t);
void DATA_Readmode();
void DS1302_ReadData(uint8_t*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void delayUS_DWT(uint32_t us)
{
    /*volatile uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    volatile uint32_t start = DWT->CYCCNT;
    do
    {
    } while (DWT->CYCCNT - start < cycles);*/
	HAL_Delay(us);
}

/* Sends an address or command */
static void DS1302_SendCmd(uint8_t cmd)
{
    uint8_t i;
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, GPIO_PIN_SET);
    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_DATA, (cmd & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, GPIO_PIN_SET);
        delayUS_DWT(1);

        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, GPIO_PIN_RESET);
        delayUS_DWT(1);
        cmd >>= 1;
    }

    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_DATA, GPIO_PIN_RESET);
}

void DATA_Readmode()
{
    // Configure PA2 as an input
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = DS1302_DATA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStruct);
}

void DATA_Writemode()
{
    // Configure PA2 as an output
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = DS1302_DATA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStruct);
}

void DS1302_ReadData(uint8_t* data)
{
    DATA_Readmode();
    uint8_t i;
    uint8_t temp = 0;
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, GPIO_PIN_SET);
    for (i = 0; i < 8; i++)
    {
        temp >>= 1;
        if (HAL_GPIO_ReadPin(DS1302_GPIO, DS1302_DATA))
            temp |= 0x80;
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, GPIO_PIN_SET);
        delayUS_DWT(1);
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, GPIO_PIN_RESET);
        delayUS_DWT(1);
    }
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, GPIO_PIN_RESET);
    *data = temp;
    DATA_Writemode();
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  DS1302_SendCmd(DS1302_REG_SEC | 0x01); // Send the command to read seconds
  DS1302_ReadData(&seconds);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
