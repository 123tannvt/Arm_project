/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#define B   3950.0
#define ANALOG_MAX 2800.0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint8_t code[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
uint16_t count = 1000;
uint16_t adc_value;
float alpha = 0;
uint32_t T = 0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/**
 * Hàm phụ trách hiển thị từng số
 */
void Number_Display(uint8_t number);
/**
 * hàm hiển thị cả số
 */
void Display(uint16_t number);
/**
 * Hàm để delay có quét led
 */
void delay_500ms(uint16_t number);
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

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  while (1)
  {
	  /*bat ADC*/
	  HAL_ADC_Start(&hadc1);

	  /*đ�?c adc*/
	  adc_value = HAL_ADC_GetValue(&hadc1);

	  /*tinh toán */
	  float ts = (float)((ANALOG_MAX/adc_value) - 1);
	  float b = (float)log(ts)/log(2.71828);
	  T = 1.0/((float)(b/3950) + (float)1.0/298.15) - 273.15;

	  if(T > 37)
	  {
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	  }
	  else if( (T > 21) && (T <= 37))
	  {
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	  }
	  else if(T <= 21)
	  {
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	  }
	  /*hiển thị nhiệt độ*/
	  Display(T);
	  /*dừng adc*/
	  HAL_ADC_Stop(&hadc1);
	  /*delay 500 ms*/
	  delay_500ms(T);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED_A_Pin|LED_3_Pin|LED_4_Pin
                          |LED_B_Pin|LED_C_Pin|LED_D_Pin|LED_E_Pin
                          |LED_F_Pin|LED_G_Pin|LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin PA8 */
  GPIO_InitStruct.Pin = LED2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED_A_Pin LED_3_Pin LED_4_Pin
                           LED_B_Pin LED_C_Pin LED_D_Pin LED_E_Pin
                           LED_F_Pin LED_G_Pin LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED_A_Pin|LED_3_Pin|LED_4_Pin
                          |LED_B_Pin|LED_C_Pin|LED_D_Pin|LED_E_Pin
                          |LED_F_Pin|LED_G_Pin|LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Number_Display(uint8_t number)
{
	HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, 1 & (code[number] >> 0));
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1 & (code[number] >> 1));
	HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, 1 & (code[number] >> 2));
	HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, 1 & (code[number] >> 3));
	HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, 1 & (code[number] >> 4));
	HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, 1 & (code[number] >> 5));
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1 & (code[number] >> 6));
}
/// display number

void Display(uint16_t number)
{
	Number_Display(number/1000);
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 1);
	HAL_Delay(2);
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);

	Number_Display((number%1000)/100);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 1);
	HAL_Delay(2);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 0);

	Number_Display((number%100)/10);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 1);
	HAL_Delay(2);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);

	Number_Display(number%10);
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, 1);
	HAL_Delay(2);
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, 0);
}

void delay_500ms(uint16_t number)
{
		for(int i = 0; i < 50; i++)
		{
			Display(number);
		}
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
