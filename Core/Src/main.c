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

#define BITSET(var,bitno) ((var) |= 1UL<<(bitno))
#define BITCLEAR(var,bitno) ((var) &= ~(1UL<<(bitno)))
#define USE_OF_REGISTERS

#ifndef USE_OF_REGISTERS

#include "main.h"
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

#else

#include "stm32l476xx.h"

#endif
/**
 * @brief  delay in clock cycles.
 * Must be calculated with 0.24uS (1/4MHz) equal to 1 by a rule of three
 * @retval None
 */
void delay ( uint8_t cycles ) {
	uint8_t count = 0;
	while (count < cycles) {
		count++;
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
#ifdef USE_OF_REGISTERS
	/*
	 * AHB2 peripheral clock enable register (RCC->AHB2ENR): Reset value: 0x0000 0000
	 * GPIOA = GPIOC = ADCEN = 0 clock for those peripherals disabled (Must be set each one)
	 */
	BITSET(RCC->AHB2ENR, 0); /*GPIOA*/
	BITSET(RCC->AHB2ENR, 2); /*GPIOC*/
	BITSET(RCC->AHB2ENR, 13); /*ADC*/
	/*
	 * CLOCK CONTROL REGISTER (RCC->CR): Reset value: 0x0000 0063
	 * MSION = 1
	 * MSIRDY = 1
	 * MSIRANGE = 6 = 4MHz
	 * PLLSAI1ON = 0 = SAI1 PLL disabled (Must be set AFTER all configurations)
	 * MSIRGSEL = 0 = MSI Range is provided by MSISRANGE[3:0] in RCC_CSR register (Must be set)
	 */
	BITSET(RCC->CR, 3); /* MSIRGSEL */
	/*
	 * Clock configuration register: Reset value (RCC->CFGR):  0x0000 0000
	 * SW = 0x00 = MSI selected as system clock
	 * HPRE = 0x00 = SYSCLK not divided (AHB Prescaler)
	 */

	/* PLL configuration register (RCC->PLLCFGR): Reset value: 0x0000 1000
	 * PLLSRC = 00 = No Main PLL, PLLSAI1 and PLLSAI2 entry clock source selected (MSI clock must be selected setting 01)
	 * PLLM = 000 = PLLM value = 1 (/M = /1)
	 * PLLN = 0010000 = PLLN value = 16 (xN = x16)
	 * ----------------------------------------------------------
	 * f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
	 * f(VCO clock) = 4MHz x (16 / 1)
	 * f(VCO clock) = 64MHz
	 */
	BITSET(RCC->PLLCFGR, 0); /* PLLSRC */
	/*
	 * [Chosen for ADC] PLLSAI1 configuration register (RCC->PLLSAI1CFGR): Reset value: 0x0000 1000
	 * PLLSAI1N = 0010000 = PLLSAI1N value = 16 (xN = x16)
	 * PLLSAI1R = 00 = 2 = PLLSAI1 division factor for PLLADC1CLK
	 * PLLSAIR1EN = 0 = PLLSAI1 PLLADC1CLK output disabled (Must be set)
	 * ----------------------------------------------------------
	 * f(VCOSAI1 clock) = f(PLL clock input) × (PLLSAI1N / PLLM)
	 * f(VCOSAI1 clock) = 4MHz × (16 / 1)
	 * f(VCOSAI1 clock) = 64MHz
	 * ----------------------------------------------------------
	 * f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
	 * f(PLLSAI1_R) = 64MHz / 2
	 * f(PLLSAI1_R) = 32MHz to ADC
	 */
	BITSET(RCC->PLLSAI1CFGR, 24); /* PLLSAIR1EN */
	BITSET(RCC->CR, 26); /* PLLSAI1ON */
	/*
	 * Peripherals independent clock configuration register (RCC->CCIPR): Reset value: 0x0000 0000
	 * ADCSEL[1:0] = 00 = No clock selected (Must be set as 01 for selection of PLLSAI1"R" as ADCs clk)
	 */
	BITSET(RCC->CCIPR, 28);
	/*
	 * GPIO port mode register (GPIOx->MODER): Reset value: 0xABFF FFFF (for port A)
	 * 										   Reset value: 0xFFFF FFFF (for port C)
	 * MODE0 = 11 = Analog mode
	 * MODE5 = 11 = Analog mode (For LD2 this must be set 01 as General Purpose Output)
	 */
	BITCLEAR(GPIOA->MODER, 11);
	BITSET(GPIOA->BRR, 5); /* Turn off LD2 */

	/*
	 * GPIO port analog switch control register (GPIOx->ASCR): Reset value: 0x0000 0000
	 * ASC0 = 0 = Disconnect analog switch to the ADC input (Must be set to enable ADC input)
	 */
	BITSET(GPIOC->ASCR, 0); /* Connect analog switch to the ADC input */
	/*
	 * ADC control register (ADC->CR): Reset value: 0x2000 0000
	 *
	 * DEEPPWD = 1 = in Deep-power-down (Must be cleared)
	 * ADVREGEN = 0 = Voltage regulator disabled (Must be set after DEEPPWD)
	 * ADCALDIF = 0 = Single-ended inputs mode
	 * ADCAL = 0 = Calibration complete (Must be set to start it, then is cleared by HW)
	 * ADSTART = 0 = No ADC regular conversion is ongoing (Must be set to start a conversion)
	 * ADDIS = 0 = Set for disable ADC
	 * ADEN = 0 = ADC is disabled (OFF state) (Must be set after all configurations)
	 */
	BITCLEAR(ADC1->CR, 29); /* DEEPPWD */
	BITSET(ADC1->CR, 28); /* ADVREGEN */
	delay(80); /* 80 uSeg [datasheet: Page 178] */
	//BITSET(ADC1->CR, 31); /* ADCAL */
	/*
	 * ADC configuration register (ADC1->CFGR): Reset value: 0x8000 0000
	 *
	 * CONT = 0 = Single conversion mode
	 * ALIGN = 0 = Right alignment
	 * RES = 00 = 12-bit resolution
	 */
	/*
	 * ADC regular sequence register 1 (ADC1->SQR1): Reset value: 0x0000 0000
	 * L = 0000 = 1 conversion
	 * SQ1 = 0000 = 1st conversion in regular sequence (Must be written with #channel (CN1 = 0001))
	 */
	BITSET(ADC1->SQR1, 6); /* Channel 1 */

	BITSET(ADC1->CR, 0); /* ADEN */
	delay(10); /* wait stabilization of the ADC */
	BITSET(ADC1->CR, 2); /* ADSTART */
	/* USER CODE END 1 */
#else
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
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	//	uinta8_t MSG[] = "HELLO_WORLD\n";

	//	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG)-1, 100);
	HAL_ADC_Start(&hadc1);
	//	uint16_t ADC_val = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
#endif
	/*
	 * ADC regular data register (ADC1->DR): Reset value: 0x0000 0000
	 * They contain the conversion result from the last converted regular channel.
	 */
	uint16_t ADC_val = 0;
	while (1)
	{
#ifdef USE_OF_REGISTERS
		if ( ADC1->ISR && (1U<<2) ) { /* Check EOC flag */
			ADC_val = ADC1->DR;
		}
		if (ADC_val >= 1600) {
			BITSET(GPIOA->BSRR, 5);  /* LD2 ON */
		} else {
			BITSET(GPIOA->BRR, 5);  /* LD2 OFF */
		}
		BITSET(ADC1->CR, 2); /* ADSTART */;

#else
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_val = HAL_ADC_GetValue(&hadc1);

		if (ADC_val >= 1600) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		HAL_ADC_Start(&hadc1);
#endif
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
#ifndef USE_OF_REGISTERS
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
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
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
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
