/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Bootloader functions prototypes */

/* Function to execute bootloader functionalities
 * Reads data from USRT and executes corresponding operations
 */
void Bootloader_UART_Read_Data(void);

/* Funtion to Execute User application
 * Assuming User App is stored at USER_APPLICATION_BASE_ADDRESS
 * i.e, at  FLASH_SECTOR2_BASE_ADDRESS (0x08008000U)
 */
void Bootloader_Jump_To_User_App(void);

void Bootloader_Handle_GetVer_cmd(uint8_t *bl_rx_buffer);
void Bootloader_Handle_GetHelp_cmd(uint8_t *pBuffer);
void Bootloader_Handle_GetCid_cmd(uint8_t *pBuffer);
void Bootloader_Handle_GetRdp_cmd(uint8_t *pBuffer);
void Bootloader_Handle_Go_cmd(uint8_t *pBuffer);
void Bootloader_Handle_Flash_Erase_cmd(uint8_t *pBuffer);
void Bootloader_Handle_Mem_Write_cmd(uint8_t *pBuffer);
void Bootloader_Handle_Enable_RW_Protect(uint8_t *pBuffer);
void Bootloader_Handle_Mem_Read (uint8_t *pBuffer);
void Bootloader_Handle_Read_Sector_Protection_Status(uint8_t *pBuffer);
void Bootloader_Handle_Read_OTP(uint8_t *pBuffer);
void Bootloader_Handle_Disable_RW_Protect(uint8_t *pBuffer);

/*
 * Function to send acknowledgment upon successful CRC verification
 * @args 	commandcode = hex code of the verified/supported command
 * 			follow_len = length of response to follow
 */
void Bootloader_Send_ack(uint8_t command_code, uint16_t follow_len);

/*
 * Function to send no-acknowledgment when CRC verification fails
 */
void Bootloader_Send_nack(void);

/*
 * Function to verify CRC (integrity) of the received command packet
 */
uint8_t Bootloader_Verify_CRC(uint8_t *pData, uint32_t len, uint32_t crc_host);

/*
 * Function to send data to host over uart
 */
void Bootloader_UART_Write_Data(uint8_t *pBuffer, uint32_t len);

/*
 * Function to get the bootloader version
 */
uint8_t Get_Bootloader_Version(void);

/* END BFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR0_BASE_ADDRESS 0x08000000U 	// 16KB
#define FLASH_SECTOR1_BASE_ADDRESS 0x08004000U	// 16KB
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U	// 16KB
#define FLASH_SECTOR3_BASE_ADDRESS 0x0800C000U	// 16KB
#define FLASH_SECTOR4_BASE_ADDRESS 0x08010000U	// 64KB
#define FLASH_SECTOR5_BASE_ADDRESS 0x08020000U	// 128KB
#define FLASH_SECTOR6_BASE_ADDRESS 0x08040000U	// 128KB
#define FLASH_SECTOR7_BASE_ADDRESS 0x08060000U	// 128KB

#define BOOTLOADER_BASE_ADDRESS FLASH_SECTOR0_BASE_ADDRESS
#define USER_APPLICATION_BASE_ADDRESS FLASH_SECTOR2_BASE_ADDRESS
/* USER CODE END Private defines */

/* USER CODE BEGIN Bootloader Command Codes */

// Version 1.0
#define BL_VERSION 0x10

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER				0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP				0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID				0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS		0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR			0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE			0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT		0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ				0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS	0x5A

//This command is used to read the OTP contents.
#define BL_OTP_READ				0x5B

//This command is used disable all sector read/write protection
#define BL_DIS_R_W_PROTECT				0x5C

/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/* CRC */
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

/* USER CODE END Bootloader Command Codes */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
