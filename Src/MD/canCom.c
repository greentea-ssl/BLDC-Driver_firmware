

#include "canCom.h"

#include "main.h"

#include "md_main.h"
#include "motorControl.h"
#include "encoder.h"


extern MD_Handler_t md_sys;

extern CAN_HandleTypeDef hcan1;

extern Encoder_TypeDef mainEncoder;


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
	//sFilterConfig.FilterIdHigh = 0x4000 | motorChannel << 10;
	sFilterConfig.FilterIdHigh = 0x300 << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x7ff << 5;
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

	int16_t Iq_ref_int = 0;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxData);

	can1RxFlg = 1;

	if(can1RxHeader.DLC != 8) return;


	switch(motorChannel)
	{
	case 0:
		Iq_ref_int = ((int16_t)can1RxData[1] << 8) | can1RxData[0]; break;

	case 1:
		Iq_ref_int = ((int16_t)can1RxData[3] << 8) | can1RxData[2]; break;

	case 2:
		Iq_ref_int = ((int16_t)can1RxData[5] << 8) | can1RxData[4]; break;

	case 3:
		Iq_ref_int = ((int16_t)can1RxData[7] << 8) | can1RxData[6]; break;

	default:
		return;
	}

	md_sys.motor.Iq_ref_pu_2q13 = Iq_ref_int * 8 / md_sys.motor.Init.I_base;


	timeoutReset(&md_sys);

	// send response
	sendToMain();

}


void sendToMain()
{

	uint32_t can1TxMailbox;
	CAN_TxHeaderTypeDef can1TxHeader;
	uint8_t can1TxData[8];

	int16_t Iq_int16, theta_uint16, omega_int16;

	can1TxHeader.StdId = 0x400 + motorChannel;
	can1TxHeader.ExtId = 0x00;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.DLC = 8;

	Iq_int16 = (int16_t)((int32_t)(md_sys.motor.Iq_pu_2q13 * md_sys.motor.Init.I_base) >> 3);
	theta_uint16 = mainEncoder.raw_Angle;
	omega_int16 = md_sys.motor.omega_q5;

	can1TxData[0] = 0;

	can1TxData[1] = Iq_int16 & 0xff;
	can1TxData[2] = (Iq_int16 >> 8) & 0xff;

	can1TxData[3] = theta_uint16 & 0xff;
	can1TxData[4] = (theta_uint16 >> 8) & 0xff;

	can1TxData[5] = omega_int16 & 0xff;
	can1TxData[6] = (omega_int16 >> 8) & 0xff;

	can1TxData[7] = 0;


	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxData, &can1TxMailbox);

	return;
}







