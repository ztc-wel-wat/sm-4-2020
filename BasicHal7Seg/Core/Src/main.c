/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#define S7_DIG_MSK 			S7_DIG1_Pin | S7_DIG2_Pin | S7_DIG3_Pin | S7_DIG4_Pin
#define S7_SEG_MSK			S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_DP_Pin

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t codeSegments[] = {
S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin, // 0
		S7_SEG_B_Pin | S7_SEG_C_Pin,							// 1
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_G_Pin | S7_SEG_E_Pin | S7_SEG_D_Pin,				 // 2
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_G_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin, // 3
		S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin, // 4
		S7_SEG_A_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin, // 5
		S7_SEG_A_Pin | S7_SEG_F_Pin | S7_SEG_E_Pin | S7_SEG_D_Pin | S7_SEG_C_Pin | S7_SEG_G_Pin, // 6
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin, // 7
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin, // 8
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin // 9
		// TODO: implementacja znakow ASCII
};

const uint8_t digitModule[] = { S7_DIG4_Pin, S7_DIG3_Pin, S7_DIG2_Pin, S7_DIG1_Pin };

uint16_t display[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t S7_Display(int16_t value) {
	if (value >= 0 && value < 10000) {
		display[0] = codeSegments[value % 10];
		value /= 10;
		display[1] = value ? codeSegments[value % 10] : 0;
		value /= 10;
		display[2] = value ? codeSegments[value % 10] : 0;
		value /= 10;
		display[3] = value ? codeSegments[value % 10] : 0;
		return 0;
	} else if (value < 0 && value > -1000) {
		// TODO: Implementacja dla liczb ujemnych - zadanie dodatkowe 2
		return 0;
	} else {
		return 1;
	}
}
// float pi = 3.14, example: S7_DisplayFractional((uint16_t)(pi * 100), 2);
uint32_t S7_DisplayFractional(uint16_t value, uint8_t fractionalDigit) {
	if (fractionalDigit < 4 && value < 10000) {
		// TODO: Implementacja liczb rzeczywistych - zadanie dodatkowe 3
		return 0;
	} else {
		return 1;
	}
}

void S7_Multiplex(void) {
	static uint8_t dig = 0;

	HAL_GPIO_WritePin(GPIOB, S7_DIG_MSK, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, S7_SEG_MSK, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, display[dig], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, digitModule[dig++], GPIO_PIN_SET);
	dig %= 4;
}

void HAL_SYSTICK_Callback(void) {
	static uint8_t delay = 0;
	if ((delay++ % 5) == 0) {
		S7_Multiplex();
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t muxDelay = 1;
	uint32_t cnt = 0;
//	HAL_GPIO_WritePin(S7_DIG4_GPIO_Port, S7_DIG4_Pin, GPIO_PIN_SET);
	while (1) {
		S7_Display(cnt++);
		HAL_Delay(100);
//		for (int i = 0; i < 4; ++i)
//		{
//			HAL_GPIO_WritePin(GPIOG, display[i], GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOB, digitModule[i], GPIO_PIN_SET);
//			HAL_Delay(muxDelay);
//			HAL_GPIO_WritePin(GPIOB, digitModule[i], GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOG, display[i], GPIO_PIN_RESET);
//
//		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	S7_DIG1_Pin | S7_DIG2_Pin | S7_DIG3_Pin | S7_DIG4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
	S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_DP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : S7_DIG1_Pin S7_DIG2_Pin S7_DIG3_Pin S7_DIG4_Pin */
	GPIO_InitStruct.Pin = S7_DIG1_Pin | S7_DIG2_Pin | S7_DIG3_Pin | S7_DIG4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : S7_SEG_A_Pin S7_SEG_B_Pin S7_SEG_C_Pin S7_SEG_D_Pin
	 S7_SEG_E_Pin S7_SEG_F_Pin S7_SEG_G_Pin S7_SEG_DP_Pin */
	GPIO_InitStruct.Pin = S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
