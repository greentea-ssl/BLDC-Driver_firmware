

#include "canCom.h"

#include "main.h"

#include "ASR.h"
#include "APR.h"


extern CAN_HandleTypeDef hcan1;


uint8_t motorChannel = 0;


CAN_FilterTypeDef sFilterConfig;


CAN_RxHeaderTypeDef can1RxHeader;
uint8_t can1RxData[8];
uint8_t can1RxFlg = 0;




void CAN_Init()
{

	motorChannel = getChannel();


	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x4000 | motorChannel << 10;
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


	if(((can1RxHeader.StdId & 0x1c) >> 2) == 0x02 && can1RxHeader.DLC == 0x4)
	{
		controlRef.byte[0] = can1RxData[0];
		controlRef.byte[1] = can1RxData[1];
		controlRef.byte[2] = can1RxData[2];
		controlRef.byte[3] = can1RxData[3];

		mainAPR.theta_ref = controlRef.fval;

		timeoutReset();
	}


	HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);

}




