/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "ASR.h"
#include "ACR.h"
#include "tim.h"



uint8_t motorChannel = 0;


CAN_FilterTypeDef sFilterConfig;


CAN_RxHeaderTypeDef can1RxHeader;
uint8_t can1RxData[8];
uint8_t can1RxFlg = 0;







/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


void CAN_Init()
{

	motorChannel = getChannel();


	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x2000 | motorChannel << 10;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xfc00;
	sFilterConfig.FilterMaskIdLow = 0x0006;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
	  Error_Handler();
	}


}


uint8_t getChannel()
{
	uint8_t ch = 0;

	ch |= !HAL_GPIO_ReadPin(CH_b0_GPIO_Port, CH_b0_Pin) << 0;
	ch |= !HAL_GPIO_ReadPin(CH_b1_GPIO_Port, CH_b1_Pin) << 1;
	ch |= !HAL_GPIO_ReadPin(CH_b2_GPIO_Port, CH_b2_Pin) << 2;

	return ch;
}




void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox1CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox2CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	union _rcdata{
		struct{
			float fval;
		};
		struct{
			uint8_t byte[4];
		};
	}controlRef;


	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxData);

	can1RxFlg = 1;

	if(((can1RxHeader.StdId & 0x1c) >> 2) == 0x01 && can1RxHeader.DLC == 0x4)
	{
		controlRef.byte[0] = can1RxData[0];
		controlRef.byte[1] = can1RxData[1];
		controlRef.byte[2] = can1RxData[2];
		controlRef.byte[3] = can1RxData[3];

		mainASR.omega_ref = controlRef.fval;

		timeoutReset();

	}

#if _APR_ENABLE_
	if(can1RxHeader.StdId == 0x008 && can1RxHeader.DLC == 0x4)
	{
		controlRef.byte[0] = can1RxData[0];
		controlRef.byte[1] = can1RxData[1];
		controlRef.byte[2] = can1RxData[2];
		controlRef.byte[3] = can1RxData[3];

		theta_ref = controlRef.fval;
	}
#endif


	HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);

}








/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
