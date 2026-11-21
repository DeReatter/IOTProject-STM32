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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Konfigurasi Buzzer Active-LOW (Sesuai implementasi cek-buzz: ON = RESET / 0V, OFF = SET / 3.3V)
#define BUZZER_STATE_ON       GPIO_PIN_RESET
#define BUZZER_STATE_OFF      GPIO_PIN_SET

// Konfigurasi Relay Active-LOW (ON = RESET / 0V, OFF = SET / 3.3V)
#define RELAY_STATE_ON        GPIO_PIN_RESET
#define RELAY_STATE_OFF       GPIO_PIN_SET

// Parameter Ambang Batas Sensor (Thresholds)
#define MQ2_THRESHOLD_ROKOK   700      // Nilai batas asap rokok (1000 - 2000 ADC)
#define MQ2_THRESHOLD_FIRE    1500      // Nilai batas kebakaran gas/asap pekat (> 2000 ADC)
#define SUHU_THRESHOLD_FIRE   50.0f     // Nilai batas suhu kebakaran (> 50.0 °C)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_buffer[2];

float v_bat = 0.0f;
float pres_bat = 0.0f;
float v_pv = 0.0f;
float i_pv = 0.0f;
float p_pv = 0.0f;
uint8_t ina_status = 0;

uint16_t mq2_raw = 0;
float suhu = 0.0f;
float kelembapan = 0.0f;
uint8_t dht_status = 0;

// Variabel status & kontrol (volatile agar dapat dimonitor/diedit langsung via Live Expressions)
volatile uint8_t pump_status = 0;
volatile uint8_t buzzer_manual = 0;
volatile uint8_t deteksi_kebakaran = 0;
volatile uint8_t deteksi_rokok = 0;

uint8_t uart_rx_byte = 0;
char uart_rx_line[32];
uint8_t uart_rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Beep(uint8_t jumlah, uint16_t on_ms, uint16_t off_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define DIVIDER_RATIO 5.54545f
#define INA219_ADDR				(0x40 << 1)
#define INA219_REG_CONFIG   	0x00
#define INA219_REG_SHUNTVOLT 	0x01
#define INA219_REG_BUSVOLT  	0x02

void Buzzer_On(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, BUZZER_STATE_ON);
}

void Buzzer_Off(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, BUZZER_STATE_OFF);
}

void Buzzer_Beep(uint8_t jumlah, uint16_t on_ms, uint16_t off_ms) {
	for (uint8_t i = 0; i < jumlah; i++) {
		Buzzer_On();
		HAL_Delay(on_ms);
		Buzzer_Off();
		HAL_Delay(off_ms);
	}
}

float hitung_v_bat(uint16_t raw_adc) {
	float v_pin = ((float) raw_adc / 4095.0f) * 3.3f;
	return v_pin;
//	return v_pin * DIVIDER_RATIO;
}
float hitung_pres_bat(float volt) {
	if (volt >= 12.6f)
		return 100.0f;
	if (volt <= 11.1f)
		return 0.0f;
	return ((volt - 11.1f) / (12.6f - 11.1f)) * 100.0f;
}
void delay_us(uint16_t us) {
	uint16_t start = (uint16_t) htim2.Instance->CNT;
	while ((uint16_t) (htim2.Instance->CNT - start) < us)
		;
}
static void DHT_SetPin_Output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT22_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}
static void DHT_SetPin_Input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT22_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}
//static void DHT_Init_OpenDrain(void) {
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//	GPIO_InitStruct.Pin = DHT_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
//}
uint8_t DHT22_Read_Data(float *suhu_out, float *kelembapan_out) {
	uint8_t bits[5] = { 0, 0, 0, 0, 0 };
	uint16_t t0;

	DHT_SetPin_Output();
	HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
	HAL_Delay(18);

	HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
	delay_us(15);
	DHT_SetPin_Input();

	__disable_irq();

	t0 = (uint16_t) htim2.Instance->CNT;
	while (DHT22_GPIO_Port->IDR & DHT22_Pin) {
		if ((uint16_t) (htim2.Instance->CNT - t0) > 150) {
			__enable_irq();
			return 2;
		}
	}
	t0 = (uint16_t) htim2.Instance->CNT;
	while (!(DHT22_GPIO_Port->IDR & DHT22_Pin)) {
		if ((uint16_t) (htim2.Instance->CNT - t0) > 150) {
			__enable_irq();
			return 3;
		}
	}
	t0 = (uint16_t) htim2.Instance->CNT;
	while (DHT22_GPIO_Port->IDR & DHT22_Pin) {
		if ((uint16_t) (htim2.Instance->CNT - t0) > 150) {
			__enable_irq();
			return 4;
		}
	}
	for (int i = 0; i < 40; i++) {
		t0 = (uint16_t) htim2.Instance->CNT;
		while (!(DHT22_GPIO_Port->IDR & DHT22_Pin)) {
			if ((uint16_t) (htim2.Instance->CNT - t0) > 120) {
				__enable_irq();
				return 5;
			}
		}
		t0 = (uint16_t) htim2.Instance->CNT;
		while (DHT22_GPIO_Port->IDR & DHT22_Pin) {
			if ((uint16_t) (htim2.Instance->CNT - t0) > 120) {
				__enable_irq();
				return 6;
			}
		}
		uint16_t duration = (uint16_t) (htim2.Instance->CNT - t0);
		if (duration > 45) {
			bits[i / 8] |= (1 << (7 - (i % 8)));
		}
	}
	__enable_irq();

	if (bits[0] == 0 && bits[1] == 0 && bits[2] == 0 && bits[3] == 0
			&& bits[4] == 0) {
		return 7;
	}
	if (bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
		return 8;
	}
	uint16_t raw_rh = ((uint16_t) bits[0] << 8) | bits[1];
	*kelembapan_out = (float) raw_rh / 10.0f;
	uint16_t raw_temp = (((uint16_t) bits[2] & 0x7F) << 8) | bits[3];
	*suhu_out = (float) raw_temp / 10.0f;
	if (bits[2] & 0x80) {
		*suhu_out = -(*suhu_out);
	}
	return 1;
}
uint8_t INA219_Init(void) {
	if (HAL_I2C_IsDeviceReady(&hi2c1, INA219_ADDR, 2, 50) != HAL_OK) {
		return 0;
	}
	uint8_t config[3] = { INA219_REG_CONFIG, 0x39, 0x9F };
	if (HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, config, 3, 50) == HAL_OK) {
		return 1;
	}
	return 0;
}
// Ubah return type menjadi uint8_t agar status koneksi I2C terpantau real-time
uint8_t INA219_Read_data(float *v_bus, float *i_ma, float *p_watt) {
	uint8_t buf[2];
	int16_t raw_shunt = 0;
	uint16_t raw_bus = 0;

	if (HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR, INA219_REG_SHUNTVOLT,
	I2C_MEMADD_SIZE_8BIT, buf, 2, 50) != HAL_OK) {
		*i_ma = 0.0f;
		*v_bus = 0.0f;
		*p_watt = 0.0f;
		return 0;
	}
	raw_shunt = (int16_t) ((buf[0] << 8) | buf[1]);
	*i_ma = (float) raw_shunt * 0.1f;
	if (*i_ma < 0.0f) {
		*i_ma = 0.0f;
	}

	if (HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR, INA219_REG_BUSVOLT,
	I2C_MEMADD_SIZE_8BIT, buf, 2, 50) != HAL_OK) {
		*v_bus = 0.0f;
		*p_watt = 0.0f;
		return 0;
	}
	raw_bus = (uint16_t) ((buf[0] << 8) | buf[1]);
	*v_bus = (float) ((raw_bus >> 3) * 4) / 1000.0f;
	*p_watt = (*v_bus) * (*i_ma / 1000.0f);

	return 1;
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
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(IN_RELAY_GPIO_Port, IN_RELAY_Pin, RELAY_STATE_OFF);
	// Inisialisasi buzzer dalam keadaan MATI (Active-LOW: SET = OFF)
	Buzzer_Off();
	// Indikasi sistem startup siap (beep 1x pendek 100ms)
	Buzzer_Beep(1, 100, 100);

	__HAL_TIM_SET_PRESCALER(&htim2, 71);
	htim2.Instance->EGR = TIM_EGR_UG;
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 2);
	ina_status = INA219_Init();
	HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	HAL_Delay(2000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// 0. Sinkronisasi status Relay Pompa ke pin hardware (Active-LOW: RESET = ON, SET = OFF)
		// Memungkinkan pengujian relay langsung lewat Live Expressions di STM32CubeIDE (variabel: pump_status)
		if (pump_status == 1) {
			HAL_GPIO_WritePin(IN_RELAY_GPIO_Port, IN_RELAY_Pin, RELAY_STATE_ON);
		} else {
			HAL_GPIO_WritePin(IN_RELAY_GPIO_Port, IN_RELAY_Pin,
			RELAY_STATE_OFF);
		}

		v_bat = hitung_v_bat(adc_buffer[0]);
		pres_bat = hitung_pres_bat(v_bat);
		mq2_raw = adc_buffer[1];

		dht_status = DHT22_Read_Data(&suhu, &kelembapan);

		// Proteksi: jika data DHT22 gagal dibaca, reset suhu & kelembapan ke 0 agar tidak memicu alarm palsu
		if (dht_status != 1) {
			suhu = 0.0f;
			kelembapan = 0.0f;
		}

		if (ina_status == 1) {
			ina_status = INA219_Read_data(&v_pv, &i_pv, &p_pv);
			v_bat = v_pv;
			pres_bat = hitung_pres_bat(v_bat);
		} else {
			ina_status = INA219_Init();
			v_bat = 0.0f;
			pres_bat = 0.0f;
		}

		// Evaluasi kondisi deteksi:
		// 1. Kebakaran : Gas MQ-2 pekat (> 2000) ATAU suhu tinggi (> 50°C saat DHT22 valid)
		// 2. Asap Rokok: Gas MQ-2 dalam rentang asap rokok (1000 - 2000 ADC) dan suhu normal
		if (mq2_raw > MQ2_THRESHOLD_FIRE
				|| (dht_status == 1 && suhu > SUHU_THRESHOLD_FIRE)) {
			deteksi_kebakaran = 1;
			deteksi_rokok = 0;
		} else if (mq2_raw >= MQ2_THRESHOLD_ROKOK) {
			deteksi_kebakaran = 0;
			deteksi_rokok = 1;
		} else {
			deteksi_kebakaran = 0;
			deteksi_rokok = 0;
		}

		// Pola kendali Buzzer (Active-LOW: ON = RESET, OFF = SET):
		// - Kebakaran / Manual: Buzzer MENYALA TERUS hingga aman
		// - Asap Rokok        : Buzzer BIP BERULANG SELAMA 1 DETIK (5x beep @ 100ms ON / 100ms OFF)
		// - Aman              : Buzzer MATI
		uint16_t delay_sisa_ms = 2000;
		if (buzzer_manual == 1 || deteksi_kebakaran == 1) {
			Buzzer_On();
			delay_sisa_ms = 2000; // Buzzer tetap menyala terus selama delay dan iterasi berikutnya
		} else if (deteksi_rokok == 1) {
			Buzzer_Beep(5, 100, 100); // 5 x 200ms = 1000 ms (1 detik bip berulang)
			Buzzer_Off();
			delay_sisa_ms = 1000; // Kompensasi delay 1 detik agar total siklus tetap 2 detik
		} else {
			Buzzer_Off();
			delay_sisa_ms = 2000;
		}

		// Format pesan serial UART ke ESP32 (TIDAK BERUBAH / TETAP PERSIS)
		char uart_tx_buff[140];
		int len = sprintf(uart_tx_buff,
				"$DATA,%.2f,%.1f,%u,%.1f,%.1f,%u,%.2f,%.1f,%.2f,%u,%u\n", v_bat,
				pres_bat, mq2_raw, suhu, kelembapan, dht_status, v_pv, i_pv,
				p_pv, ina_status, pump_status);

		HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buff, len, 100);

		HAL_Delay(delay_sisa_ms);
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DHT22_Pin | IN_RELAY_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : BUZZER_Pin IN_RELAY_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin | IN_RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : DHT22_Pin */
	GPIO_InitStruct.Pin = DHT22_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	// Memastikan pin buzzer dalam kondisi MATI (SET / 3.3V) segera setelah GPIO diinisialisasi
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, BUZZER_STATE_OFF);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (uart_rx_byte == '\n' || uart_rx_byte == '\r') {
			if (uart_rx_idx > 0) {
				uart_rx_line[uart_rx_idx] = '\0';

				if (strcmp(uart_rx_line, "$CMD,PUMP,1") == 0) {
					pump_status = 1;
					HAL_GPIO_WritePin(IN_RELAY_GPIO_Port, IN_RELAY_Pin,
					RELAY_STATE_ON);
				} else if (strcmp(uart_rx_line, "$CMD,PUMP,0") == 0) {
					pump_status = 0;
					HAL_GPIO_WritePin(IN_RELAY_GPIO_Port, IN_RELAY_Pin,
					RELAY_STATE_OFF);
				} else if (strcmp(uart_rx_line, "$CMD,BUZZER,1") == 0) {
					buzzer_manual = 1;
					Buzzer_On();
				} else if (strcmp(uart_rx_line, "$CMD,BUZZER,0") == 0) {
					buzzer_manual = 0;
					Buzzer_Off();
				}
				uart_rx_idx = 0;
			}
		} else if (uart_rx_idx < sizeof(uart_rx_line) - 1) {
			uart_rx_line[uart_rx_idx++] = (char) uart_rx_byte;
		}
		HAL_UART_Receive_IT(huart, &uart_rx_byte, 1);
	}
}
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
