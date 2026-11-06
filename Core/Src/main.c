/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_buffer[2];

float v_baterai = 0.0f;
float pres_baterai = 0.0f;
uint16_t mq2_raw = 0;

float suhu = 0.0f;
float kelembapan = 0.0f;
uint8_t dht_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DIVIDER_RATIO 5.54545f
#define DHT_PORT GPIOA
#define DHT_PIN  GPIO_PIN_4

float hitung_v_baterai(uint16_t raw_adc) {
	float v_pin = ((float) raw_adc / 4095.0f) * 3.3f;
	return v_pin * DIVIDER_RATIO;
}

float hitung_pres_baterai(float volt) {
	if (volt >= 14.4f)
		return 100.0f;
	if (volt <= 10.0f)
		return 0.0f;
	return ((volt - 10.0f) / (14.4f - 10.0f)) * 100.0f;
}

void TIM2_Init(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();
	TIM2->PSC = 72 - 1; // 72 MHz / 72 = 1 MHz -> 1 tick = 1 microsecond
	TIM2->ARR = 0xFFFF;
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint16_t us) {
	uint16_t start = TIM2->CNT;
	while ((uint16_t)(TIM2->CNT - start) < us)
		;
}

// Konfigurasi pin dinamis untuk jalur Data DHT22 (PA4)
static void DHT_SetPin_Output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

static void DHT_SetPin_Input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

/**
 * @brief Membaca data DHT22 (atau DHT11) dengan error code diagnostik lengkap
 * @return 1: SUKSES
 *         2: ERROR - Sensor tidak menarik LOW (tidak merespons sinyal START)
 *         3: ERROR - Sensor tertahan di LOW setelah respon awal
 *         4: ERROR - Sensor tertahan di HIGH setelah respon awal
 *         5: ERROR - Timeout pulsa bit LOW
 *         6: ERROR - Timeout pulsa bit HIGH
 *         7: ERROR - Seluruh data bernilai 0 (sinyal kosong)
 *         8: ERROR - Checksum tidak valid (data korup)
 */
uint8_t DHT22_Read_Data(float *suhu, float *kelembaban) {
	uint8_t bits[5] = { 0, 0, 0, 0, 0 };
	uint16_t t0;

	// 1. Kirim START signal: Tarik LOW selama 18 ms (standar universal untuk DHT22/AM2302 & DHT11)
	DHT_SetPin_Output();
	HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
	HAL_Delay(18);

	// Tarik HIGH sebentar (30 us) lalu pindah ke Input Pull-up
	HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
	delay_us(30);
	DHT_SetPin_Input();

	// Matikan interupsi sesaat agar pulsa mikrodetik tidak terdistorsi
	__disable_irq();

	// 2. Handshake Step 1: Menunggu sensor menarik garis ke LOW (maks 100 us)
	t0 = TIM2->CNT;
	while (DHT_PORT->IDR & DHT_PIN) {
		if ((uint16_t)(TIM2->CNT - t0) > 100) {
			__enable_irq();
			return 2; // Sensor tidak merespons
		}
	}

	// 3. Handshake Step 2: Menunggu sensor melepas kembali ke HIGH (~80 us LOW)
	t0 = TIM2->CNT;
	while (!(DHT_PORT->IDR & DHT_PIN)) {
		if ((uint16_t)(TIM2->CNT - t0) > 100) {
			__enable_irq();
			return 3; // Sensor tertahan LOW
		}
	}

	// 4. Handshake Step 3: Menunggu sensor menarik ke LOW lagi (~80 us HIGH)
	t0 = TIM2->CNT;
	while (DHT_PORT->IDR & DHT_PIN) {
		if ((uint16_t)(TIM2->CNT - t0) > 100) {
			__enable_irq();
			return 4; // Sensor tertahan HIGH
		}
	}

	// 5. Baca 40 bit data (5 byte)
	for (int i = 0; i < 40; i++) {
		// Menunggu pulsa LOW 50 us tiap bit selesai
		t0 = TIM2->CNT;
		while (!(DHT_PORT->IDR & DHT_PIN)) {
			if ((uint16_t)(TIM2->CNT - t0) > 100) {
				__enable_irq();
				return 5; // Timeout bit LOW
			}
		}

		// Ukur durasi pulsa HIGH: 26-28 us = bit '0', 70 us = bit '1'
		t0 = TIM2->CNT;
		while (DHT_PORT->IDR & DHT_PIN) {
			if ((uint16_t)(TIM2->CNT - t0) > 120) {
				__enable_irq();
				return 6; // Timeout bit HIGH
			}
		}

		uint16_t duration = (uint16_t)(TIM2->CNT - t0);
		if (duration > 45) {
			bits[i / 8] |= (1 << (7 - (i % 8)));
		}
	}

	// Hidupkan kembali interupsi segera setelah 40 bit selesai dibaca
	__enable_irq();

	// Cek jika seluruh byte 0 (tidak valid)
	if (bits[0] == 0 && bits[1] == 0 && bits[2] == 0 && bits[3] == 0 && bits[4] == 0) {
		return 7;
	}

	// Verifikasi Checksum (Byte 4 = Byte 0 + Byte 1 + Byte 2 + Byte 3)
	if (bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
		return 8; // Checksum mismatch
	}

	// Decode data (Otomatis membedakan format DHT22 dan DHT11)
	if (bits[0] > 4) {
		// Format DHT11 (jika ternyata modul adalah DHT11)
		*kelembaban = (float) bits[0] + ((float) bits[1] / 10.0f);
		*suhu = (float) bits[2] + ((float) bits[3] / 10.0f);
	} else {
		// Format DHT22 / AM2302
		uint16_t raw_rh = ((uint16_t) bits[0] << 8) | bits[1];
		*kelembaban = (float) raw_rh / 10.0f;

		uint16_t raw_temp = (((uint16_t) bits[2] & 0x7F) << 8) | bits[3];
		*suhu = (float) raw_temp / 10.0f;
		if (bits[2] & 0x80) {
			*suhu = -(*suhu);
		}
	}

	return 1; // Pembacaan SUKSES
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
	SystemCoreClockUpdate();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	TIM2_Init();
	DHT_SetPin_Input();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 2);
	HAL_Delay(2000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		v_baterai = hitung_v_baterai(adc_buffer[0]);
		pres_baterai = hitung_pres_baterai(v_baterai);
		mq2_raw = adc_buffer[1];
		dht_status = DHT22_Read_Data(&suhu, &kelembapan);

		if (mq2_raw > 2000) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //BUZZER ON
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //BUZZER OFF
		}

		char uart_tx_buff[100];
		int len = sprintf(uart_tx_buff, "$DATA,%.2f,%.1f,%u,%.1f,%.1f,%u\n",
				v_baterai, pres_baterai, mq2_raw, suhu, kelembapan, dht_status);

		HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buff, len, 100);

		HAL_Delay(2000);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : buzzer_Pin */
	GPIO_InitStruct.Pin = buzzer_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
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
