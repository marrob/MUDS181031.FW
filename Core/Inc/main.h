/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* Debug ---------------------------------------------------------------------*/
  #define DEVICE_DEBUG_LEVEL    3

/**BMS1**/
#define ISO15765_SERVER_BLOCK_SIZE 0x01
#define ISO15765_SERVER_STMIN      0x00
#define UDS_TX_ADDR 0x18DAF1DB  /*server -> client*/
#define UDS_RX_ADDR 0x18DADBF1

/**BMS2**/
//#define UDS_TX_ADDR 0x18DAF1DC  /*server -> client*/
//#define UDS_RX_ADDR 0x18DADCF1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


#if (DEVICE_DEBUG_LEVEL > 0)
#define  DeviceUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceUsrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 1)

#define  DeviceErrLog(...)  {printf("ERROR.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceErrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 2)
#define  DeviceDbgLog(...)  {printf("DEBUG.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceDbgLog(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t SamplesStoreDtcRead0001[859];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TERM_Pin GPIO_PIN_12
#define TERM_GPIO_Port GPIOB
#define LIVE_LED_Pin GPIO_PIN_15
#define LIVE_LED_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOC
#define LED_Y_Pin GPIO_PIN_7
#define LED_Y_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
/* Generic  -------------------------------------------------------------------*/
  #define DEVICE_NAME             "MCAN190529"    /*!< Eszk�z neve pl.:MDIO130204*/
  #define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
  #define DEVICE_FW               "200224_1647"   /*!< Eszk�z Firmware verzi�sz�ma pl.:"1.0.0.0"*/
  #define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
  #define DEVICE_PCB              "00"            /*!< Eszk�z PCB verzi�sz�ma pl.:"A0"*/
  #define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
  #define DEVICE_MNF              "KONVOLUCIO"
  #define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
