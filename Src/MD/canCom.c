

#include "canCom.h"

#include "main.h"
#include "md_main.h"
#include "parameters.h"

extern MD_Handler_t md_sys;


void SendResToMain();
void SendParamToMain();
void RecvCmdFromMain(uint8_t* canRxData);
void RecvMidiFromMain(uint8_t* canRxData);


void CAN_Init()
{

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//sFilterConfig.FilterIdHigh = 0x4000 | motorChannel << 10;
	sFilterConfig.FilterIdHigh = 0x300 << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x70F << 5;
	sFilterConfig.FilterMaskIdLow = 0x0006;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(md_sys.hcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_CAN_Start(md_sys.hcan) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(md_sys.hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
	  Error_Handler();
	}


}



void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == md_sys.hcan->Instance)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox1CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == md_sys.hcan->Instance)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox2CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == md_sys.hcan->Instance)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef canRxHeader;
	uint8_t canRxData[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, canRxData);

	if(canRxHeader.StdId == 0x300 && canRxHeader.DLC == 8)
	{
		RecvCmdFromMain(canRxData);
		return;
	}
	if(canRxHeader.StdId == 0x310 && canRxHeader.DLC == 0)
	{
		SendParamToMain();
		return;
	}
	if(canRxHeader.StdId == 0x320 && canRxHeader.DLC == 8)
	{
		RecvMidiFromMain(canRxData);
		return;
	}

}


void RecvCmdFromMain(uint8_t* canRxData)
{
	int16_t Iq_ref_int = 0;

	if(md_sys.motor.RunMode != MOTOR_MODE_CC_VECTOR)
	{
		md_sys.motor.RunMode = MOTOR_MODE_CC_VECTOR;
		Motor_Reset(&md_sys.motor);
	}

	switch(md_sys.motor_channel)
	{
	case 0:
		Iq_ref_int = ((int16_t)canRxData[1] << 8) | canRxData[0]; break;

	case 1:
		Iq_ref_int = ((int16_t)canRxData[3] << 8) | canRxData[2]; break;

	case 2:
		Iq_ref_int = ((int16_t)canRxData[5] << 8) | canRxData[4]; break;

	case 3:
		Iq_ref_int = ((int16_t)canRxData[7] << 8) | canRxData[6]; break;

	default:
		return;
	}

	md_sys.motor.Iq_ref_pu_2q13 = (int32_t)Iq_ref_int * 8 / md_sys.motor.Init.I_base;

	timeoutReset(&md_sys);

	// send response
	SendResToMain();

}

void RecvMidiFromMain(uint8_t* canRxData)
{
	int16_t Iq_ref_int = 0;

	md_sys.motor.RunMode = MOTOR_MODE_CV_MIDI;

	switch(md_sys.motor_channel)
	{
	case 0:
		md_sys.motor.MIDI_notenum = canRxData[0];
		md_sys.motor.MIDI_vel = canRxData[1];
		break;
	case 1:
		md_sys.motor.MIDI_notenum = canRxData[2];
		md_sys.motor.MIDI_vel = canRxData[3];
		break;
	case 2:
		md_sys.motor.MIDI_notenum = canRxData[4];
		md_sys.motor.MIDI_vel = canRxData[5];
		break;
	case 3:
		md_sys.motor.MIDI_notenum = canRxData[6];
		md_sys.motor.MIDI_vel = canRxData[7];
		break;
	default:
		return;
	}

	timeoutReset(&md_sys);

	// send response
	SendResToMain();
}


void SendResToMain()
{

	uint32_t canTxMailbox;
	CAN_TxHeaderTypeDef canTxHeader;
	uint8_t canTxData[8];

	int16_t Iq_int16, theta_uint16, omega_int16;

	canTxHeader.StdId = 0x400 + md_sys.motor_channel;
	canTxHeader.ExtId = 0x00;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = 8;

	Iq_int16 = (int16_t)((int32_t)(md_sys.motor.Iq_pu_2q13 * md_sys.motor.Init.I_base) >> 3);
	theta_uint16 = md_sys.encoder.raw_Angle;
	omega_int16 = md_sys.encoder.speedCalc.omega_q5_filtered;

	canTxData[0] = !HAL_GPIO_ReadPin(BR_FLT_GPIO_Port, BR_FLT_Pin);
	canTxData[0] |= (md_sys.calibration_is_running & 0x01) << 1;

	canTxData[1] = Iq_int16 & 0xff;
	canTxData[2] = (Iq_int16 >> 8) & 0xff;

	canTxData[3] = theta_uint16 & 0xff;
	canTxData[4] = (theta_uint16 >> 8) & 0xff;

	canTxData[5] = omega_int16 & 0xff;
	canTxData[6] = (omega_int16 >> 8) & 0xff;

	canTxData[7] = 0;


	HAL_CAN_ActivateNotification(md_sys.hcan, CAN_IT_TX_MAILBOX_EMPTY);

	HAL_CAN_AddTxMessage(md_sys.hcan, &canTxHeader, canTxData, &canTxMailbox);

	return;
}



void SendParamToMain()
{

	uint32_t canTxMailbox;
	CAN_TxHeaderTypeDef canTxHeader;
	uint8_t canTxData[8];

	uint16_t Irated_uint16;

	canTxHeader.StdId = 0x410 + md_sys.motor_channel;
	canTxHeader.ExtId = 0x00;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = 4;

	Irated_uint16 = (uint16_t)(CURRENT_RATING * 1024);

	canTxData[0] = (uint16_t)MOTOR_KV & 0xff;
	canTxData[1] = ((uint16_t)MOTOR_KV >> 8) & 0xff;
	canTxData[2] = Irated_uint16 & 0xff;
	canTxData[3] = (Irated_uint16 >> 8) & 0xff;

	HAL_CAN_ActivateNotification(md_sys.hcan, CAN_IT_TX_MAILBOX_EMPTY);

	HAL_CAN_AddTxMessage(md_sys.hcan, &canTxHeader, canTxData, &canTxMailbox);

	return;
}




