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
#include "frames.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SPI_HandleTypeDef hspi1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SC
#define SC_Pin        GPIO_PIN_13
#define SC_GPIO_Port  GPIOB

// SI
#define SI_Pin        GPIO_PIN_15
#define SI_GPIO_Port  GPIOB

// RES
#define RES_Pin       GPIO_PIN_11
#define RES_GPIO_Port GPIOB

// CS1B
#define CS_Pin        GPIO_PIN_12
#define CS_GPIO_Port  GPIOB

// A0 (RS / D-C)
#define RS_Pin        GPIO_PIN_14
#define RS_GPIO_Port  GPIOB

// SW1: SINE
#define SW1_Pin       GPIO_PIN_8
#define SW1_GPIO_Port GPIOA

// SW2: SQUARE
#define SW2_Pin       GPIO_PIN_9
#define SW2_GPIO_Port GPIOA

// SW3: TRIANGLE
#define SW3_Pin       GPIO_PIN_10
#define SW3_GPIO_Port GPIOA

// SW4: SAWTOOTH
#define SW4_Pin       GPIO_PIN_11
#define SW4_GPIO_Port GPIOA

#define FSYNC_Pin       GPIO_PIN_4
#define FSYNC_GPIO_Port GPIOA

#define SINE     1
#define SQUARE 	 2
#define TRIANGLE 3
#define SAWTOOTH 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t wave_type = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void data_write(uint8_t d);
void comm_write(uint8_t d);
void init_LCD(void);
void ClearLCD(void);
void DispPic(const uint8_t *lcd_string);
void PicLoop(void);
void GetWave(void);
void SetWave(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void data_write(uint8_t d) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);

    for (uint8_t n = 0; n < 8; n++) {
        HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, (d & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        d <<= 1;
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void comm_write(uint8_t d) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);

    for (uint8_t n = 0; n < 8; n++) {
        HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, (d & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        d <<= 1;
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void init_LCD(void) {
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
    comm_write(0xA0); // ADC select
    comm_write(0xAE); // Display OFF
    comm_write(0xC8); // COM direction scan
    comm_write(0xA2); // LCD bias set
    comm_write(0x2F); // Power Control set
    comm_write(0x21); // Resistor Ratio Set
    comm_write(0x81); // Electronic Volume Command (contrast)
    comm_write(0x20); // Volume value
    comm_write(0xAF); // Display ON
}

void ClearLCD(void) {
    for (uint8_t page = 0xB0; page < 0xB4; page++) {
        comm_write(page);
        comm_write(0x10);
        comm_write(0x00);
        for (uint8_t col = 0; col < 128; col++)
            data_write(0x00);
    }
    comm_write(0xAF);
}

void DispPic(const uint8_t *lcd_string) {
    for (uint8_t page = 0xB0; page < 0xB4; page++) {
        comm_write(page);
        comm_write(0x10);
        comm_write(0x00);
        for (uint8_t col = 0; col < 128; col++)
            data_write(*lcd_string++);
    }
    comm_write(0xAF);
    // HAL_Delay(100);
}

void PicLoop(void) {
	for (int i = 0; i < 106; i++) {
        DispPic(frames[i]);
        HAL_Delay(25);
    }
}

void GetWave(void) {
	// sine
	if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) {
		wave_type = SINE;
	}
	// square
	else if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {
		wave_type = SQUARE;
	}
	// triangle
	else if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {
		wave_type = TRIANGLE;
	}
	// sawtooth
	else if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {
		wave_type = SAWTOOTH;
	}
}

void SetWave(void) {
	switch(wave_type) {
		case (SINE):
			break;
		case (SQUARE):
			break;
		case (TRIANGLE):
			break;
		case (SAWTOOTH):
			break;
	}
}

void ProgramWave() {
	uint16_t txBuf[5] = {
	    0x2100,   // RESET=1
		0x50C7,   // FREQ0 LSB (1 Hz)
	    0x4000,   // FREQ0 MSB
	    0xC000,   // PHASE0
	    0x2020    // Exit reset, triangle
	};

	HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)txBuf, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_SET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  hspi1.Instance = SPI1;                       // SPI peripheral
  hspi1.Init.Mode = SPI_MODE_MASTER;           // MASTER or SLAVE
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;  // Full-duplex
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;     // 8-bit data
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   // Clock idle state
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;       // Clock capture edge
  hspi1.Init.NSS = SPI_NSS_SOFT;               // Software-controlled CS
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 8 MHz
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

  // Initialize SPI
  HAL_SPI_Init(&hspi1);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  init_LCD();
  ClearLCD();
  ProgramWave();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// PicLoop();


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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
