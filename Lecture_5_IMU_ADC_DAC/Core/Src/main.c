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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR          (0x68 << 1)   // HAL uses an 8-bit address
#define MPU6050_REG_WHO_AM_I  0x75			//Register with address storage
#define MPU6050_REG_PWR_MGMT1 0x6B			//Register with MPU configuration
#define MPU6050_REG_ACCEL_XOUT_H 0x3B		//First register with measurements
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Device ID read from the WHO_AM_I register (0x75)
// should be 0x68 for MPU6050
volatile uint8_t who_am_i = 0;

// helper variable used to send data to the MPU (e.g. register configuration)
volatile uint8_t tx_data = 0;

// buffer for raw data read from MPU6050 (14 bytes total):
// [0-5]  → accelerometer (X, Y, Z)
// [6-7]  → temperature
// [8-13] → gyroscope (X, Y, Z)
uint8_t raw_data[14];

// raw accelerometer data (combined from two bytes: High and Low)
// range depends on configuration (e.g. ±2g)
volatile int16_t accel_x, accel_y, accel_z;

// raw gyroscope data (combined from two bytes: High and Low)
// range depends on configuration (e.g. ±250 dps)
volatile int16_t gyro_x, gyro_y, gyro_z;

// raw temperature value from the sensor (needs conversion to °C)
volatile int16_t temp_raw;

// status of MPU initialization / communication:
// 0 → OK
// !=0 → error (e.g. no response, wrong WHO_AM_I)
volatile uint8_t mpu_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);

	uint8_t regs[] = { 0x75, 0x6B, 0x19, 0x1A, 0x1B, 0x1C };
	for (int i = 0; i < 6; i++) {
		HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, regs[i], I2C_MEMADD_SIZE_8BIT,
				&tx_data, 1, 100);
	}
	mpu_status = MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Timer interrupt callback (called on TIM6 update event)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		// check if MPU is initialized correctly
		if (mpu_status == 0) {
			// read sensor data
			if (MPU6050_ReadRaw() == HAL_OK) {
				// toggle LED to indicate successful read
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
		}
	}
}

// Write single register to MPU6050 via I2C
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// Read single register from MPU6050 via I2C
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

// Read multiple consecutive registers from MPU6050
HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, buffer, len, 100);
}

// Initialize MPU6050
uint8_t MPU6050_Init(void) {
	// check if device is present on I2C bus
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) != HAL_OK) {
		return 1;
	}

	// read WHO_AM_I register
	if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
		return 2;
	}

	// verify device ID (expected value depends on device)
	if (who_am_i != 0x72) {
		return 3;
	}

	// wake up the device (clear sleep bit)
	if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT1, 0x00) != HAL_OK) {
		return 4;
	}

	HAL_Delay(100); // wait for stabilization

	return 0;
}

// Read raw accelerometer, gyroscope, and temperature data
HAL_StatusTypeDef MPU6050_ReadRaw(void) {
	HAL_StatusTypeDef status;

	// read 14 bytes starting from ACCEL_XOUT_H
	status = MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
	if (status != HAL_OK) {
		return status;
	}

	// combine high and low bytes into 16-bit values
	accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
	accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
	accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

	temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);

	gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
	gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
	gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

	return HAL_OK;
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
