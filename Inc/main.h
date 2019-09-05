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
void Bootloader_UART_Read_Data(void);

/* Funtion to Execute User application
 * Assuming User App is stored at USER_APPLICATION_BASE_ADDRESS
 * i.e, at  FLASH_SECTOR2_BASE_ADDRESS (0x08008000U)
 */
void Bootloader_Jump_To_User_App(void);

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

#define USER_APPLICATION_BASE_ADDRESS FLASH_SECTOR2_BASE_ADDRESS
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
