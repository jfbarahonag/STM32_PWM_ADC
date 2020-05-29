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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUFF_SIZE		(	   10		)
#define PWM_PERIOD		(     1999		)

typedef enum _bool_ {
	OFF,
	ON,
}bool_t;

typedef enum _events_ {
	EVT_NO_EVT,
	EVT_0,
	EVT_30,
	EVT_70,
	EVT_100,
	EVT_ADC,
	EVT_ADC_RECEIVED,
}events_t;

typedef enum _states_ {
	STATE_0,
	STATE_30,
	STATE_70,
	STATE_100,
	STATE_ADC,
}states_t;

typedef struct _button_ {
	uint16_t debounc_count;
	bool_t flag_pressed:1;
	bool_t flag_maybe_pressed:1;
}button_t;

button_t user_button = {
		.debounc_count = 0,
		.flag_maybe_pressed = OFF,
		.flag_pressed = OFF,
};

typedef struct _uart_ {
	uint8_t rx_buff[BUFF_SIZE];
	uint8_t tx_buff[BUFF_SIZE];
}uart_t;

uart_t uart2 = {
		.rx_buff = {0},
		.tx_buff = {0},
};

typedef struct _adc_ {
	volatile uint32_t value;
}adc_t;

adc_t adc = {
		.value = 0,
};

typedef struct _led_ {

	volatile uint16_t counter;
	bool_t status:1;
	bool_t start:1;
	bool_t start_blink:1;

}led_t;

led_t user_led = {
		.start = OFF,
		.status = OFF,
		.counter = 0,
		.start_blink = OFF,
};

typedef struct _fsm_ {
	events_t event;
	states_t state;
	bool_t new_event:1;

}fsm_t;

fsm_t fsm;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
void fsm_init (fsm_t *fsm);
void fsm_run (fsm_t *fsm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (user_button.flag_maybe_pressed == OFF) {
		user_button.flag_maybe_pressed = ON;
	}
}

/**
 * @brief  Conversion complete callback in non-blocking mode.
 * @param hadc ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc.value = HAL_ADC_GetValue(&hadc3);
	fsm.new_event = ON;
	fsm.event = EVT_ADC_RECEIVED;
}

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		/* after EXTI interrupt is triggered */
		if ( user_button.flag_maybe_pressed == ON ) {
			user_button.debounc_count++;
		}
		/* 10mSec elapsed */
		if ( user_button.debounc_count >= 10 ) {

			/* button still pressed */
			if ( !HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) ) {
				user_button.flag_maybe_pressed = OFF;
				user_button.flag_pressed = ON;
				user_button.debounc_count = 0;
			}

		}
		/* Debouncing ignored */
		if ( user_button.flag_pressed == ON ) {
			user_button.flag_pressed =  OFF;
			user_led.status ^= ON; /* Start heartbeat */

			if ( fsm.state == STATE_0 ) {
				fsm.new_event = ON;
				fsm.event = EVT_30;
			}
			else if ( fsm.state == STATE_30 ) {
				fsm.new_event = ON;
				fsm.event = EVT_70;
			}
			else if ( fsm.state == STATE_70 ) {
				fsm.new_event = ON;
				fsm.event = EVT_100;
			}
			else if ( fsm.state == STATE_100 ) {
				fsm.new_event = ON;
				fsm.event = EVT_ADC;
			}
			else if ( fsm.state == STATE_ADC ) {
				fsm.new_event = ON;
				fsm.event = EVT_0;
			}
		}

		if (user_led.status == ON) {
			user_led.counter++;
		} else {
			user_led.counter = 0;
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, OFF);
		}

		if ( user_led.counter >= 1000 ) {
			user_led.counter = 0;
			user_led.start_blink = ON;
		}

	}
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
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */
	uart2.tx_buff[BUFF_SIZE - 1] = '\0';

	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

	fsm_init(&fsm);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		fsm_run(&fsm);
		if ( user_led.start_blink == ON ) {
			user_led.start_blink = OFF;
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			sprintf((char *)(uart2.tx_buff), "%d\r\n",(int)(adc.value));
			HAL_UART_Transmit_IT(&huart2, uart2.tx_buff, (uint16_t)strlen((const char *)uart2.tx_buff));
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.NbrOfDiscConversion = 1;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc3.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 159;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 99;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1000;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 16;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void fsm_init (fsm_t *fsm) {
	fsm->state = STATE_0;
	fsm->event = EVT_NO_EVT;
	fsm->new_event = OFF;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

void fsm_run (fsm_t *fsm) {
	if ( fsm->new_event == ON ) {
		fsm->new_event = OFF;

		switch (fsm->state) {

		case STATE_0:

			if ( fsm->event == EVT_30 ) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 600);
				fsm->state = STATE_30;
			}

			break;

		case STATE_30:

			if ( fsm->event == EVT_70 ) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1399);
				fsm->state = STATE_70;
			}

			break;

		case STATE_70:

			if ( fsm->event == EVT_100 ) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1999);
				fsm->state = STATE_100;
			}

			break;

		case STATE_100:

			if ( fsm->event == EVT_ADC ) {
				HAL_ADC_Start_IT(&hadc3);
				fsm->state = STATE_ADC;
			}

			break;

		case STATE_ADC:

			if ( fsm->event == EVT_0 ) {

				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				fsm->state = STATE_0;

			}
			else if ( fsm->event == EVT_ADC_RECEIVED ) {

				volatile uint32_t pwm_value = (adc.value*1999)/(4050);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
				HAL_ADC_Start_IT(&hadc3);
				fsm->state = STATE_ADC;

			}

			break;

		}

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
