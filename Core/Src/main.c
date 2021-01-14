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
  *
  * Processor:                      STM32F405RGT6W
  * System Clock source             PLL (HSE)
  * SYSCLK(Hz)                      168000000
  * HCLK(Hz)                        168000000
  * AHB Prescaler                   1
  * APB1 Prescaler                  4
  * APB2 Prescaler                  2
  * HSE Frequency(Hz)               8000000
  * PLL_M                           8
  * PLL_N                           336 // 336->168MHz, 360->180MHz, 288 ->144MHz
  * PLL_P                           2
  * PLL_Q                           7   // 7->168MHz, 7->180MHz, 6->144MHz
  * VDD(V)                          3.3
  *
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiveLed.h"
#include <stdio.h>
#include "iso15765_server.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _DebugStateTypeDef
{
  DBG_NONE,
  DBG_RxIT,
}DebugStateTypeDef;

typedef enum _CtrlStatesTypeDef
{
  SDEV_START,                   //0
  SDEV_SYS_INIT,
  SDEV_STDBY,                   //2
  SDDEV_CHARGE_OFF,
  SDEV_CHANGE,
  SDEV_RUN,                    //3
}CtrlStateTypeDef;

typedef struct _State
{
  CtrlStateTypeDef Next;
  CtrlStateTypeDef Curr;
  CtrlStateTypeDef Pre;
}StateTypeDef;

typedef struct _AppTypeDef
{
  int Counter;
  StateTypeDef States;
  Iso15765Handle_Type   Transport;
  Iso15765Handle_Type   Bms2Transport;
}DeviceTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef LiveLed;
DebugStateTypeDef DebugState;
DeviceTypeDef Device;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void DebugTask(DebugStateTypeDef *state);
void DebugPrint(char *str);
void TestMsgSenderTask(void);
void BusTermOn(void);
void BusTermOff(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TestMsgSenderTask(void)
{
  static int32_t timestamp;
  if(HAL_GetTick() - timestamp >= 1000)
  {
    Device.Counter ++;
    CAN_TxHeaderTypeDef   txHeader;
    uint32_t              txMailbox;
    timestamp = HAL_GetTick();
    txHeader.StdId = 0x321;
    txHeader.ExtId = 0xFFFFFFF1;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_EXT;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x7, /*(uint8_t)TickGetMs()*/ 0x08};
    if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox) != HAL_OK)
    {
      DeviceErrLog("HAL_CAN_AddTxMessage");
    }
  }
}

/**
  * @brief  Rx Fifo 0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef   rxHeader;
  uint8_t               data[8];

  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_GetRxMessage");
  }
  if ((rxHeader.ExtId == UDS_RX_ADDR) && (rxHeader.IDE == CAN_ID_EXT) && (rxHeader.DLC == 8))
  {
    Iso15765IncomingStream(&Device.Transport, data, sizeof(data));
  }
    /*...*/
}

/**
  * @brief  Bus Write Callback
  */
uint8_t Iso15765BusWriteCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
  CAN_TxHeaderTypeDef   txHeader;
  uint32_t              txMailbox;
  txHeader.ExtId = UDS_TX_ADDR;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.DLC = 8;
  if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_AddTxMessage");
  }

  return ISO15765_OK;
}

/* UDS ------------------------------------------------------------------------*/
/**
  * @brief Request->Response Callback
  */
uint8_t Iso15765ReqRespCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{

  #define UDS_SID_DIAG_CTRL   0x10 /*DiagSesssionControl*/
  #define UDS_SID_CLEAR_DTC   0x14 /*ClearDiagnosticInformation*/
  #define UDS_SID_DTC_READ    0x19 /*ReadDTCInformationService*/
  #define UDS_SID_RDBI        0x22 /*ReadDtaByIdentifier*/
  #define UDS_SID_WDBI        0x2E /*WriteDataByIdentifier*/
  #define UDS_SID_RUTINE_CTRL 0x31 /*RutineControl*/


  uint8_t sid = data[0];
  #define DID_TEST    0x0001

  switch(sid)
  {
    /*** DiagSesssionControl ***/
    case UDS_SID_DIAG_CTRL:
    {
      DeviceDbgLog("UDS_SID_DIAG_CTRL, Reqtuest Length %d", size);
      uint8_t diagSessionType = data[1];
      uint8_t temp[]= {UDS_SID_DIAG_CTRL + 0x40, diagSessionType, 0x00, 0x32, 0x01, 0xF4, 0xAA};
      Iso15765Response(hnd, temp, sizeof(temp));
      break;
    }
    /*** ClearDiagnosticInformation ***/
    case UDS_SID_CLEAR_DTC:
    {
      DeviceDbgLog("UDS_SID_CLEAR_DTC, Reqtuest Length %d", size);
      uint8_t resp[] = {UDS_SID_CLEAR_DTC + 0x40 };
      Iso15765Response(hnd, resp, sizeof(resp));
      break;
    }
    /*** ReadDTCInformationService ***/
    case UDS_SID_DTC_READ:
    {
      DeviceDbgLog("UDS_SID_DTC_READ, Reqtuest Length %d", size);
      uint8_t subFunciton = data[1];
      uint8_t DTCstatusMask = data[2];
      if(subFunciton == 0x02  && DTCstatusMask == 0xFF)
      {
         Iso15765Response(hnd, SamplesStoreDtcRead0001, sizeof(SamplesStoreDtcRead0001));
      }
      else
      {
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
      }
      break;
    }
    /*** ReadDtaByIdentifier ***/
    case UDS_SID_RDBI:
    {
      DeviceDbgLog("UDS_SID_RDBI, Reqtuest Length %d", size);
      uint8_t did_msb = data[1];
      uint8_t did_lsb = data[2];
      uint16_t did =  did_msb << 8 | did_lsb;
      switch(did)
      {
        case DID_TEST:
        {
          uint8_t msbValue = 0x00;
          uint8_t lsbValue = 0x10;
          uint8_t temp[] = {UDS_SID_RDBI + 0x40, data[1], data[2], msbValue, lsbValue};
          Iso15765Response(hnd, temp, sizeof(temp));
          break;
        }
       // default:
       // {
       //   Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
       // }

        break;
      }
      data[0] = UDS_SID_RDBI + 0x40;
      Iso15765Response(hnd, data, size);
      break;
    }
    /*** WriteDataByIdentifier ***/
    case UDS_SID_WDBI:
    {
      uint8_t did_msb = data[1];
      uint8_t did_lsb = data[2];
      uint16_t did =  did_msb << 8 | did_lsb;
      DeviceDbgLog("UDS_SID_RDBI, Reqtuest Length %d", size);
      switch(did)
      {
        /*MASTER 2 */
        case 0x9281:
        {
          break;
        }

      }
      //default:
      //{
      //  Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
      //}
      data[0] = UDS_SID_WDBI + 0x40;
      Iso15765Response(hnd, data, size);
      break;
    }
    /*** RutineControl ***/
    case UDS_SID_RUTINE_CTRL:
    {
      if(data[1] == 0x01)
      {
        data[0] = UDS_SID_RUTINE_CTRL + 0x40;
        Iso15765Response(hnd, data, size);
      }
      else
      {
        uint8_t resp[] = {UDS_SID_RUTINE_CTRL + 0x40 };
        Iso15765Response(hnd, resp, sizeof(resp));
      }
      DeviceDbgLog("UDS_SID_RUTINE_CTRL, Reqtuest Length %d", size);
      break;
    }
    /*** Unknown ***/
    default:
    {
       Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT );
    }
  }
  return ISO15765_OK;
}


void DebugTask(DebugStateTypeDef *state)
{
  switch(*state)
  {
    case DBG_NONE: //0
    {
      break;
    }
    case DBG_RxIT://1
    {
//      if(HAL_UART_Receive_IT(&CMDLINE_UART, (uint8_t *)&CmdLine.InputBuffer.CharBuffer, 1) != HAL_OK)
//        DeviceErrLog("HAL_UART_RxCpltCallback.HAL_UART_Receive_IT");

      break;
    }
  }
  *state = DBG_NONE;
}
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  printf("%s, %s, Init OK.\r\n", "xx", DEVICE_NAME /*DEVICE_FW*/);
  printf("HAL_RCC_GetPCLK2Freq: %dHz\r\n",(int)HAL_RCC_GetPCLK2Freq());
  printf("SystemCoreClock: %dHz\r\n",(int)SystemCoreClock);
  printf("\r\n");

  /*** LiveLed ***/
  LiveLed.LedOffFnPtr = &LiveLedOff;
  LiveLed.LedOnFnPtr = &LiveLedOn;
  LiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&LiveLed);

  /*** Counter ***/
  Device.Counter = 0;

  Iso15765ServerInit(&Device.Transport, 0, 30);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LiveLedTask(&LiveLed);
    DebugTask(&DebugState);
    TestMsgSenderTask();
    Iso15765Task(&Device.Transport);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /*
   * Baudrate   BRP  SJW          BS1           BS2
   * 500000,    4,   CAN_SJW_3TQ, CAN_BS1_16TQ, CAN_BS2_4TQ,
   */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /*** Filter Init***/
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_ConfigFilter");
  }

  /*** Activate CAN RX notification ***/
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_ActivateNotification");
  }

  /*** Bus Termination***/
  BusTermOn();

/*** Start the CAN peripheral ***/
if (HAL_CAN_Start(&hcan1) != HAL_OK)
{
  DeviceErrLog("HAL_CAN_Start");
}
  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TERM_Pin|LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|LED_Y_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TERM_Pin LIVE_LED_Pin */
  GPIO_InitStruct.Pin = TERM_Pin|LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_Y_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_Y_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* printf -------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}


/* TERM ---------------------------------------------------------------------*/
void BusTermOn(void)
{
  HAL_GPIO_WritePin(TERM_GPIO_Port, TERM_Pin, GPIO_PIN_SET);
}
void BusTermOff(void)
{
  HAL_GPIO_WritePin(TERM_GPIO_Port, TERM_Pin, GPIO_PIN_RESET);
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
