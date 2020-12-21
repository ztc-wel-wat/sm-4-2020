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
#include "stdio.h"
#include "string.h"
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
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const uint8_t codeSegments[] =
		{
		S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin | S7_SEG_E_Pin
				| S7_SEG_F_Pin, // 0
				S7_SEG_B_Pin | S7_SEG_C_Pin,							// 1
				S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_G_Pin | S7_SEG_E_Pin
						| S7_SEG_D_Pin,				 // 2
				S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_G_Pin | S7_SEG_C_Pin
						| S7_SEG_D_Pin, // 3
				S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin, // 4
				S7_SEG_A_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_C_Pin
						| S7_SEG_D_Pin, // 5
				S7_SEG_A_Pin | S7_SEG_F_Pin | S7_SEG_E_Pin | S7_SEG_D_Pin
						| S7_SEG_C_Pin | S7_SEG_G_Pin, // 6
				S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin, // 7
				S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin
						| S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin, // 8
				S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin
						| S7_SEG_F_Pin | S7_SEG_G_Pin // 9
		// TODO: implementacja znakow ASCII
		};

const uint8_t digitModule[] = { S7_DIG4_Pin, S7_DIG3_Pin, S7_DIG2_Pin,
S7_DIG1_Pin };

uint16_t display[4];

char buffer[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_LPUART1_UART_Init(void);
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

void LED_RGB_SetIntensity(uint16_t r, uint16_t g, uint16_t b) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, b);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, r);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, g);
}

int __io_putchar(int ch) {
//	HAL_UART_Transmit(&hlpuart1, (uint8_t* )&ch, 1, 100);

//	UART_Wai
	while ((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE)
		;

	LPUART1->TDR = (uint8_t) ch;

	return ch;
}

int __io_getchar(void) {

	while ((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE)
		;

	return (LPUART1->RDR & USART_RDR_RDR) & 0xFF;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

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
	MX_TIM4_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint16_t r = 0, g = 0, b = 0;
		HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)buffer, 4);
	while (1) {
		r = rand() & 0xFF;
		g = rand() & 0xFF;
		b = rand() & 0xFF;
//		while(__io_getchar() != '1');

		LED_RGB_SetIntensity(r, g, b);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//		sprintf(buffer, "R = %3d; G = %3d; B = %3d\n", r, g, b);
		printf("R = %3d; G = %3d; B = %3d\n", r, g, b);
		HAL_Delay(100);
//		HAL_UART_Transmit(&hlpuart1, (uint8_t *)buffer, strlen(buffer), 100);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0x00FF;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	S7_DIG1_Pin | S7_DIG2_Pin | S7_DIG3_Pin | S7_DIG4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
			S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin | S7_SEG_D_Pin
					| S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin | S7_SEG_DP_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : S7_DIG1_Pin S7_DIG2_Pin S7_DIG3_Pin S7_DIG4_Pin */
	GPIO_InitStruct.Pin = S7_DIG1_Pin | S7_DIG2_Pin | S7_DIG3_Pin | S7_DIG4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : S7_SEG_A_Pin S7_SEG_B_Pin S7_SEG_C_Pin S7_SEG_D_Pin
	 S7_SEG_E_Pin S7_SEG_F_Pin S7_SEG_G_Pin S7_SEG_DP_Pin */
	GPIO_InitStruct.Pin = S7_SEG_A_Pin | S7_SEG_B_Pin | S7_SEG_C_Pin
			| S7_SEG_D_Pin | S7_SEG_E_Pin | S7_SEG_F_Pin | S7_SEG_G_Pin
			| S7_SEG_DP_Pin;
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
