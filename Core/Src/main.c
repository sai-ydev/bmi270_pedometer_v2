/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>
#include "stdio.h"
#include "bmi270.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUS_TIMEOUT             1000
#define READ_WRITE_LEN     UINT8_C(32)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t interrupt_button = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int8_t SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
		void *intf_ptr);
int8_t SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void bmi2_error_codes_print_result(int8_t rslt) {
	switch (rslt) {
	case BMI2_OK:

		/* Do nothing */
		break;

	case BMI2_W_FIFO_EMPTY:
		printf("Warning [%d] : FIFO empty\r\n", rslt);
		break;
	case BMI2_W_PARTIAL_READ:
		printf("Warning [%d] : FIFO partial read\r\n", rslt);
		break;
	case BMI2_E_NULL_PTR:
		printf(
				"Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
				rslt);
		break;

	case BMI2_E_COM_FAIL:
		printf(
				"Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
				rslt);
		break;

	case BMI2_E_DEV_NOT_FOUND:
		printf(
				"Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_SENSOR:
		printf(
				"Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
				rslt);
		break;

	case BMI2_E_SELF_TEST_FAIL:
		printf(
				"Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_INT_PIN:
		printf(
				"Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
				rslt);
		break;

	case BMI2_E_OUT_OF_RANGE:
		printf(
				"Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
				rslt);
		break;

	case BMI2_E_ACC_INVALID_CFG:
		printf(
				"Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
				rslt);
		break;

	case BMI2_E_GYRO_INVALID_CFG:
		printf(
				"Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
				rslt);
		break;

	case BMI2_E_ACC_GYR_INVALID_CFG:
		printf(
				"Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
				rslt);
		break;

	case BMI2_E_CONFIG_LOAD:
		printf(
				"Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_PAGE:
		printf(
				"Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
				rslt);
		break;

	case BMI2_E_SET_APS_FAIL:
		printf(
				"Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
				rslt);
		break;

	case BMI2_E_AUX_INVALID_CFG:
		printf(
				"Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
				rslt);
		break;

	case BMI2_E_AUX_BUSY:
		printf(
				"Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
				rslt);
		break;

	case BMI2_E_REMAP_ERROR:
		printf(
				"Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
				rslt);
		break;

	case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
		printf(
				"Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
				rslt);
		break;

	case BMI2_E_SELF_TEST_NOT_DONE:
		printf(
				"Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_INPUT:
		printf(
				"Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_STATUS:
		printf(
				"Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n",
				rslt);
		break;

	case BMI2_E_CRT_ERROR:
		printf(
				"Error [%d] : CRT error. It occurs when the CRT test has failed\r\n",
				rslt);
		break;

	case BMI2_E_ST_ALREADY_RUNNING:
		printf(
				"Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
				rslt);
		break;

	case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
		printf(
				"Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
				rslt);
		break;

	case BMI2_E_DL_ERROR:
		printf(
				"Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
				rslt);
		break;

	case BMI2_E_PRECON_ERROR:
		printf(
				"Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
				rslt);
		break;

	case BMI2_E_ABORT_ERROR:
		printf(
				"Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n",
				rslt);
		break;

	case BMI2_E_WRITE_CYCLE_ONGOING:
		printf(
				"Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
				rslt);
		break;

	case BMI2_E_ST_NOT_RUNING:
		printf(
				"Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
				rslt);
		break;

	case BMI2_E_DATA_RDY_INT_FAILED:
		printf(
				"Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
				rslt);
		break;

	case BMI2_E_INVALID_FOC_POSITION:
		printf(
				"Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
				rslt);
		break;

	default:
		printf("Error [%d] : Unknown error code\r\n", rslt);
		break;
	}
}

int8_t bmi2_step_counter_set_config(struct bmi2_dev *bmi2_dev) {
	/* Variable to define result */
	int8_t rslt;

	/* Initialize interrupts for gyroscope */
	struct bmi2_sens_int_config sens_int = { .type = BMI2_STEP_COUNTER,
			.hw_int_pin = BMI2_INT2 };

	/* List the sensors which are required to enable */
	uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_STEP_COUNTER };

	/* Structure to define the type of the sensor and its configurations */
	struct bmi2_sens_config config;

	/* Configure type of feature */
	config.type = BMI2_STEP_COUNTER;

	/* Enable the selected sensors */
	rslt = bmi270_sensor_enable(sens_list, 2, bmi2_dev);

	if (rslt == BMI2_OK) {
		/* Get default configurations for the type of feature selected */
		rslt = bmi270_get_sensor_config(&config, 1, bmi2_dev);

		if (rslt == BMI2_OK) {
			config.cfg.step_counter.watermark_level = 1;

			rslt = bmi270_set_sensor_config(&config, 1, bmi2_dev);
			if (rslt == BMI2_OK) {
				/* Map interrupt to pins */
				rslt = bmi270_map_feat_int(&sens_int, 1, bmi2_dev);
			} else {
				printf("Set Sensor Config failed");
			}

		} else {
			printf("Get sensor config failed");
		}
	} else {
		printf("Sensor Enable Failed");
	}

	return rslt;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint8_t rslt;

	uint8_t chip_id;
	uint8_t bmi270_dev_addr;
	uint8_t ver_major, ver_minor;

	struct bmi2_dev dev;
	struct bmi2_feat_sensor_data sensor_data = { .type = BMI2_STEP_COUNTER };

	char oled_buf[16];

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
	MX_DFSDM1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	printf("Pedometer example\r\n");

	bmi270_dev_addr = BMI2_I2C_PRIM_ADDR;
	dev.intf = BMI2_I2C_INTF;
	dev.read = (bmi2_read_fptr_t) SensorAPI_I2Cx_Read;
	dev.write = (bmi2_write_fptr_t) SensorAPI_I2Cx_Write;

	/* Assign device address to interface pointer */
	dev.intf_ptr = &bmi270_dev_addr;

	/* Configure delay in microseconds */
	dev.delay_us = bmi2_delay_us;

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	dev.read_write_len = 32;

	/* Assign to NULL to load the default config file. */
	dev.config_file_ptr = NULL;

	/* Initialize bmi270. */
	rslt = bmi270_init(&dev);
	bmi2_error_codes_print_result(rslt);

	if (rslt != BMI2_OK) {
		printf("bmi270_wh_init() failed, error code: %d\r\n", rslt);
	} else {
		rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, &dev);
		if (rslt != BMI2_OK) {
			printf("read chip ID failed, error code: %d\r\n", rslt);
			return rslt;
		}

		printf("Chip ID: %02x\r\n", chip_id);
	}

	printf("BMI270 initialized successfully\r\n");

	rslt = bmi2_get_config_file_version(&ver_major, &ver_minor, &dev);
	printf("The firmware version: v%d.%d\r\n", ver_major, ver_minor);

	rslt = bmi2_step_counter_set_config(&dev);
	if (rslt != BMI2_OK) {
		bmi2_error_codes_print_result(rslt);
		return rslt;
	}
	printf("Step counter configured successfully\r\n");
	ssd1306_Init();
	ssd1306_Fill(White);
	ssd1306_SetCursor(4, 18);
	ssd1306_WriteString("Sai's Steps", Font_11x18, Black);
	HAL_Delay(3000);
	ssd1306_UpdateScreen();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		int8_t rslt;
		uint16_t int_status = 0;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		rslt = bmi2_get_int_status(&int_status, &dev);
		if (rslt == BMI2_OK) {
			if (int_status & BMI270_STEP_CNT_STATUS_MASK) {
				rslt = bmi270_get_feature_data(&sensor_data, 1, &dev);
				if (rslt == BMI2_OK) {
					uint32_t steps = sensor_data.sens_data.step_counter_output;
					printf("Step Count:%ld\r\n", steps);
					ssd1306_Fill(White);
					ssd1306_SetCursor(4, 18);
					snprintf(oled_buf, sizeof(oled_buf), "%ld Steps", steps);
					ssd1306_WriteString(oled_buf, Font_11x18, Black);
					ssd1306_UpdateScreen();
				} else {
					printf("Get feature data failed \r\n");
				}
			}
		} else {
			printf("Int Status retrieval failed\r\n");
		}
		if (interrupt_button) {
			ssd1306_Fill(Black);
			ssd1306_SetCursor(4, 18);
			ssd1306_WriteString("Interrupted!", Font_11x18, White);
			ssd1306_UpdateScreen();
			interrupt_button = 0;
		}
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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void) {

	/* USER CODE BEGIN DFSDM1_Init 0 */

	/* USER CODE END DFSDM1_Init 0 */

	/* USER CODE BEGIN DFSDM1_Init 1 */

	/* USER CODE END DFSDM1_Init 1 */
	hdfsdm1_channel1.Instance = DFSDM1_Channel1;
	hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm1_channel1.Init.OutputClock.Selection =
			DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm1_channel1.Init.OutputClock.Divider = 2;
	hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
	hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
	hdfsdm1_channel1.Init.SerialInterface.SpiClock =
			DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
	hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm1_channel1.Init.Awd.Oversampling = 1;
	hdfsdm1_channel1.Init.Offset = 0;
	hdfsdm1_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM1_Init 2 */

	/* USER CODE END DFSDM1_Init 2 */

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
	hi2c1.Init.Timing = 0x10909CEC;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			M24SR64_Y_RF_DISABLE_Pin | M24SR64_Y_GPO_Pin | ISM43362_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin | SPBTLE_RF_RST_Pin | ARD_D9_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin | LED2_Pin
					| SPSGRF_915_SDN_Pin | ARD_D5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			USB_OTG_FS_PWR_EN_Pin | PMOD_RESET_Pin | STSAFE_A100_RESET_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin | M24SR64_Y_GPO_Pin
			| ISM43362_RST_Pin | ISM43362_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin
			| SPSGRF_915_GPIO3_EXTI5_Pin | SPBTLE_RF_IRQ_EXTI6_Pin
			| ISM43362_DRDY_EXTI1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : Button_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
	GPIO_InitStruct.Pin = Button_Pin | VL53L0X_GPIO1_EXTI7_Pin
			| LSM3MDL_DRDY_EXTI8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
	 ARD_A1_Pin ARD_A0_Pin */
	GPIO_InitStruct.Pin = ARD_A5_Pin | ARD_A4_Pin | ARD_A3_Pin | ARD_A2_Pin
			| ARD_A1_Pin | ARD_A0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
	GPIO_InitStruct.Pin = ARD_D1_Pin | ARD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
	GPIO_InitStruct.Pin = ARD_D10_Pin | SPBTLE_RF_RST_Pin | ARD_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D4_Pin */
	GPIO_InitStruct.Pin = ARD_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D7_Pin */
	GPIO_InitStruct.Pin = ARD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
	GPIO_InitStruct.Pin = ARD_D13_Pin | ARD_D12_Pin | ARD_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D3_Pin */
	GPIO_InitStruct.Pin = ARD_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D6_Pin */
	GPIO_InitStruct.Pin = ARD_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
	 SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin
			| LED2_Pin | SPSGRF_915_SDN_Pin | ARD_D5_Pin
			| SPSGRF_915_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
	 QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
	GPIO_InitStruct.Pin = QUADSPI_CLK_Pin | QUADSPI_NCS_Pin
			| OQUADSPI_BK1_IO0_Pin | QUADSPI_BK1_IO1_Pin | QUAD_SPI_BK1_IO2_Pin
			| QUAD_SPI_BK1_IO3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : INTERNAL_I2C2_SCL_Pin INTERNAL_I2C2_SDA_Pin */
	GPIO_InitStruct.Pin = INTERNAL_I2C2_SCL_Pin | INTERNAL_I2C2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
	GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin | INTERNAL_UART3_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
	 PMOD_IRQ_EXTI12_Pin */
	GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin | LSM6DSL_INT1_EXTI11_Pin
			| ARD_D2_Pin | HTS221_DRDY_EXTI15_Pin | PMOD_IRQ_EXTI12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin | SPBTLE_RF_SPI3_CSN_Pin
			| PMOD_RESET_Pin | STSAFE_A100_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
	GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin | USB_OTG_FS_DM_Pin
			| USB_OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
	GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin | INTERNAL_SPI3_MISO_Pin
			| INTERNAL_SPI3_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
	GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
	GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin | PMOD_UART2_RTS_Pin
			| PMOD_UART2_TX_Pin | PMOD_UART2_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int8_t SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
		void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	// send register address
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &reg_addr, 1, BUS_TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1, DevAddress, reg_data, len, BUS_TIMEOUT);
	return 0;
}

int8_t SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	uint8_t buf[len + 1];
	buf[0] = reg_addr;
	memcpy(&buf[1], reg_data, len);

	// send register address
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, buf, len + 1, BUS_TIMEOUT);
	return 0;
}

void bmi2_delay_us(uint32_t period, void *intf_ptr) {
	uint32_t i;

	while (period--) {
		for (i = 0; i < 84; i++) {
			;
		}
	}
}

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);

	if(GPIO_Pin == GPIO_PIN_13){
		interrupt_button = 1;
	}


	/* NOTE: This function should not be modified, when the callback is needed,
	 the HAL_GPIO_EXTI_Callback could be implemented in the user file
	 */
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

