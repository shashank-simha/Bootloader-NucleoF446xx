/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Uncomment the below line to get debug messages over debug uart
#define BL_DEBUG_MSG_EN

#define D_UART &huart3
#define C_UART &huart2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];
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
	MX_USART2_UART_Init();
	MX_CRC_Init();
	MX_USART3_UART_Init();

	/* USER CODE BEGIN 2 */

	printmsg("BL_DEBUG_MSG:Hello from Bootloader\n\r");

	/* Check whether the on board button is pressed or not */
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	{
		printmsg("BL_DEBUG_MSG:Button is pressed .... Entering BL mode\n\r");
		Bootloader_UART_Read_Data();
	}
	else
	{
		printmsg("BL_DEBUG_MSG:Button is not pressed .... Executing user application\n\r");
		Bootloader_Jump_To_User_App();
	}
	/* USER CODE END 2 */
}

/**
 * @brief Print formatted string to console over UART
 * @retval None
 */
void printmsg(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 (Bootloader functions implementation) */

void Bootloader_UART_Read_Data(void)
{
	uint16_t rec_len = 0;

	while (1)
	{
		memset(bl_rx_buffer, 0, 200);
		/*
		 * Read and decode commands coming from host
		 * Read the first two bytes, which gives the length of the command packet
		 */
		HAL_UART_Receive(C_UART, bl_rx_buffer, 2, HAL_MAX_DELAY);
		rec_len = bl_rx_buffer[0];
		rec_len = rec_len << 8;
		rec_len += bl_rx_buffer[1];

		HAL_UART_Receive(C_UART, &bl_rx_buffer[2], rec_len, HAL_MAX_DELAY);

		switch (bl_rx_buffer[2])
		{
		case BL_GET_VER:
			Bootloader_Handle_GetVer_cmd(bl_rx_buffer);
			break;
		case BL_GET_HELP:
			Bootloader_Handle_GetHelp_cmd(bl_rx_buffer);
			break;
		case BL_GET_CID:
			Bootloader_Handle_GetCid_cmd(bl_rx_buffer);
			break;
		case BL_GET_RDP_STATUS:
			Bootloader_Handle_GetRdp_cmd(bl_rx_buffer);
			break;
		case BL_GO_TO_ADDR:
			Bootloader_Handle_Go_cmd(bl_rx_buffer);
			break;
		case BL_FLASH_ERASE:
			Bootloader_Handle_Flash_Erase_cmd(bl_rx_buffer);
			break;
		case BL_MEM_WRITE:
			Bootloader_Handle_Mem_Write_cmd(bl_rx_buffer);
			break;
		case BL_EN_RW_PROTECT:
			Bootloader_Handle_Enable_RW_Protect(bl_rx_buffer);
			break;
		case BL_MEM_READ:
			Bootloader_Handle_Mem_Read(bl_rx_buffer);
			break;
		case BL_READ_SECTOR_P_STATUS:
			Bootloader_Handle_Read_Sector_Protection_Status(bl_rx_buffer);
			break;
		case BL_OTP_READ:
			Bootloader_Handle_Read_OTP(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			Bootloader_Handle_Disable_RW_Protect(bl_rx_buffer);
			break;
		default:
			printmsg("BL_DEBUG_MSG:Invalid command code received from host\n\r");
			break;
		}

	}
}

void Bootloader_Jump_To_User_App(void)
{
	// a function pointer to hold the address of the reset handler of the user app
	void (*app_reset_handler)(void);

	printmsg("BL_DEBUG_MSG:Handing control over to user application\n\r");

	// configure the MSP by reading the value from the base address of the user app
	uint32_t msp_value = *(volatile uint32_t *) USER_APPLICATION_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG:MSP value : %#x\n\r", msp_value);

	// This function comes from CMSIS.

	__set_MSP(msp_value);

	// SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS; // Vector Table Offset Register

	/* fetch the reset handler address of the user application
	 * from the location USER_APPLICATION_BASE_ADDRESS+4
	 */
	uint32_t resethandler_address = *(volatile uint32_t *) (USER_APPLICATION_BASE_ADDRESS + 4);

	app_reset_handler = (void*) resethandler_address;

	printmsg("BL_DEBUG_MSG:App reset handler addr : %#x\n\r", app_reset_handler);

	// jump to reset handler of the user application
	printmsg("BL_DEBUG_MSG:Starting user application\n\r");
	app_reset_handler();
}

void Bootloader_Handle_GetVer_cmd(uint8_t *bl_rx_buffer)
{
	uint8_t bl_version;

	printmsg("BL_DEBUG_MSG:Bootloader_Handle_GetVer_cmd\n\r");

	// calculate command packet length (first 2 bytes)
	uint32_t command_packet_len = ((bl_rx_buffer[0] << 8) | bl_rx_buffer[1]) + 1;

	// extract crc sent by host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer + command_packet_len - 4));

	if(! Bootloader_Verify_CRC(bl_rx_buffer, command_packet_len - 4, host_crc))
	{
		printmsg("BL_DEBUG_MSG:Checksum Success!!\n\r");
		Bootloader_Send_ack(bl_rx_buffer[0], 1);
		bl_version = Get_Bootloader_Version();
		printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n\r",bl_version,bl_version);
		Bootloader_UART_Write_Data(&bl_rx_buffer, 1);
	}
}

void Bootloader_Handle_GetHelp_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_GetCid_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_GetRdp_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Go_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Flash_Erase_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Mem_Write_cmd(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Enable_RW_Protect(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Mem_Read (uint8_t *pBuffer)
{

}

void Bootloader_Handle_Read_Sector_Protection_Status(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Read_OTP(uint8_t *pBuffer)
{

}

void Bootloader_Handle_Disable_RW_Protect(uint8_t *pBuffer)
{

}

void Bootloader_Send_ack(uint8_t command_code, uint16_t follow_len)
{
	// Ack packet contains 3 bytes, first byte is the ack code, next two bytes gives the length of response to follow
	uint8_t ack_buf[3];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = *((uint8_t *) &(follow_len) + 1);
	ack_buf[2] = *((uint8_t *) &(follow_len) + 0);
	HAL_UART_Transmit(C_UART,ack_buf,3,HAL_MAX_DELAY);
}

void Bootloader_Send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}

uint8_t Bootloader_Verify_CRC(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue = 0xff;

    for (uint32_t i = 0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}

void Bootloader_UART_Write_Data(uint8_t *pBuffer, uint32_t len)
{
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

/*
 *  @return macro value of bootloader version
 */
uint8_t Get_Bootloader_Version(void)
{
  return (uint8_t)BL_VERSION;
}

/* USER CODE END 4 (Bootloader functions implementation) */

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

