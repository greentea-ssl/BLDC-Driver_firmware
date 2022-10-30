


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "md_main.h"

#include "main.h"
#include "flash.h"
#include "pwm.h"
#include "parameters.h"
#include "encoder.h"
#include "currentSensor.h"
#include "drv8323.h"
#include "canCom.h"
#include "sin_t.h"
#include "motorControl.h"
#include "dump_int.h"
#include "intMath.h"
#include "led_blink.h"


#define DEBUG_PRINT_ENABLE 0

#define DUMP_DEBUG_ENABLE 1


#define  PRINT_HEX(x)  printf(#x " = %04x\n", (x))


void DRV_Setting(MD_Handler_t* h);

void MD_Calibration(MD_Handler_t* h);

uint8_t getChannel();

inline void LED_blink(LED_Blink_t* h);

inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...);

int32_t printFloat(float val);


void MD_Init(MD_Handler_t* h)
{

	uint8_t p_ch, ch;


	h->carrier_counter = 0;

	DRV_Init(&h->drv8323);

	Motor_Init(&h->motor);

	LED_Blink_Init(&h->led_blink, LD2_GPIO_Port, LD2_Pin, 100);

	h->timeoutEnable = 1;

#if DEBUG_PRINT_ENABLE

	printf("Hello\n");

#endif


	DRV_Setting(h);

	p_ch = getChannel();

	for(int count = 0; count < 6; count++)
	{
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	}

	ch = getChannel();

	h->motor_channel = ch;
	LED_Blink_SetBlinksNum(&h->led_blink, ch);

	CAN_Init();

	Encoder_Init(&h->encoder);

	CurrentSensor_Init(&h->currentSense);

	CurrentSensor_Start(&h->currentSense);

	PWM_Init(&h->pwm);

	PWM_Start(&h->pwm);

	h->pFlashData = (FlashStoredData_t*)Flash_load();
	if(ch != p_ch)
	{
		MD_Calibration(h);
	}
	const int cs_offset_err_limit = 512;
	const int theta_offset_limit = 8192;
	if(h->pFlashData->theta_offset >= theta_offset_limit ||
			abs(h->pFlashData->AD_Iu_offset_err) >= cs_offset_err_limit ||
			abs(h->pFlashData->AD_Iv_offset_err) >= cs_offset_err_limit ||
			abs(h->pFlashData->AD_Iw_offset_err) >= cs_offset_err_limit)
	{
		h->pFlashData->AD_Iu_offset_err = 0;
		h->pFlashData->AD_Iv_offset_err = 0;
		h->pFlashData->AD_Iw_offset_err = 0;
		h->pFlashData->theta_offset = 0;
	}
	h->motor.Init.AD_Iu_offset = h->pFlashData->AD_Iu_offset_err + 2048;
	h->motor.Init.AD_Iv_offset = h->pFlashData->AD_Iv_offset_err + 2048;
	h->motor.Init.AD_Iw_offset = h->pFlashData->AD_Iw_offset_err + 2048;
	h->motor.Init.theta_int_offset = h->pFlashData->theta_offset;
#if DEBUG_PRINT_ENABLE
	printf("AD_Iu_offset = %d\n", h->motor.Init.AD_Iu_offset);
	printf("AD_Iv_offset = %d\n", h->motor.Init.AD_Iv_offset);
	printf("AD_Iw_offset = %d\n", h->motor.Init.AD_Iw_offset);
	printf("theta_int_offset = %d\n", h->motor.Init.theta_int_offset);
#endif

	h->timeoutEnable = 1;
	h->timeoutCount = 0;

	h->motor.Vd_pu_2q13 = 0;
	h->motor.Vq_pu_2q13 = 0;
	h->motor.RunMode = MOTOR_MODE_CV_VECTOR;

	/* Start */
//	Motor_Reset(&h->motor);
//	h->motor.RunMode = MOTOR_MODE_CC_VECTOR;

}



void MD_Calibration(MD_Handler_t* h)
{

	/***** Current sense offset calibration *****/
	const int cal_sample_num = 1000;
	uint32_t sum_offset_err[3] = {0, 0, 0};
	for(int i = 0; i < cal_sample_num; i++)
	{
		HAL_Delay(1);
		sum_offset_err[0] += h->motor.AD_Iu - 2048;
		sum_offset_err[1] += h->motor.AD_Iv - 2048;
		sum_offset_err[2] += h->motor.AD_Iw - 2048;
	}
	h->pFlashData->AD_Iu_offset_err = sum_offset_err[0] / cal_sample_num;
	h->pFlashData->AD_Iv_offset_err = sum_offset_err[1] / cal_sample_num;
	h->pFlashData->AD_Iw_offset_err = sum_offset_err[2] / cal_sample_num;

	h->motor.Init.AD_Iu_offset = h->pFlashData->AD_Iu_offset_err + 2048;
	h->motor.Init.AD_Iv_offset = h->pFlashData->AD_Iv_offset_err + 2048;
	h->motor.Init.AD_Iw_offset = h->pFlashData->AD_Iw_offset_err + 2048;

	/***** Encoder offset calibration *****/

	h->motor.Igam_ref_pu_2q13 = (uint16_t)(5.0 / h->motor.Init.I_base * 8192);
	h->motor.Idel_ref_pu_2q13 = (uint16_t)(0.0 / h->motor.Init.I_base * 8192);

	h->motor.Init.theta_int_offset = 0;
	h->motor.theta_force_int = 0;
	h->motor.RunMode = MOTOR_MODE_CC_FORCE;

	HAL_Delay(1000);

	h->pFlashData->theta_offset = h->motor.theta_re_int & SIN_TBL_MASK;

	if (!Flash_store())
	{
#if DEBUG_PRINT_ENABLE
		printf("Failed to write flash\n");
#endif
	}

	h->motor.RunMode = MOTOR_MODE_CV_FORCE;

}



inline void MD_Update_SyncPWM(MD_Handler_t* h)
{

	Encoder_Refresh(&h->encoder);

	CurrentSensor_Refresh(&h->currentSense);

#if DUMP_DEBUG_ENABLE
	if(h->motor.RunMode == MOTOR_MODE_CC_VECTOR)
	{
		// 5A: 2731, 10A: 5461, 15A: 8192
		if(h->carrier_counter % 4 < 2)
		{
			h->motor.Id_ref_pu_2q13 = 0;
			h->motor.Iq_ref_pu_2q13 = 2731;
		}
		else
		{
			h->motor.Id_ref_pu_2q13 = 0;
			h->motor.Iq_ref_pu_2q13 = -2731;
		}
		h->carrier_counter++;
	}
	if(h->motor.RunMode == MOTOR_MODE_CV_VECTOR)
	{
		int period = 2000;
		if(h->carrier_counter < period)
		{
			h->motor.Vd_pu_2q13 = 0;
			h->motor.Vq_pu_2q13 = h->carrier_counter * 2;
		}
		else
		{
			h->motor.Vq_pu_2q13 = period * 2;
		}
//		h->motor.Vd_pu_2q13 = 0;
//		h->motor.Vq_pu_2q13 = 341;
		h->carrier_counter++;
	}
#endif

	h->motor.AD_Iu = h->currentSense.AD_Iu[0];
	h->motor.AD_Iv = h->currentSense.AD_Iv[0];
	h->motor.AD_Iw = h->currentSense.AD_Iw[0];
	h->motor.AD_Vdc = h->currentSense.AD_Vdc[0];
	h->motor.raw_theta_14bit = h->encoder.raw_Angle;

	// Motor Controller Update
	Motor_Update(&h->motor);

	h->pwm.duty_u = h->motor.duty_u;
	h->pwm.duty_v = h->motor.duty_v;
	h->pwm.duty_w = h->motor.duty_w;
	PWM_SetDuty(&h->pwm);

	if(!Dump_isFull())
	{
		if(h->motor.RunMode == MOTOR_MODE_CC_VECTOR || h->motor.RunMode == MOTOR_MODE_CV_VECTOR)
		{
			Dump_Update(h);
		}
	}


	Encoder_Request(&h->encoder);



#if 1
	if(h->timeoutEnable == 1)
	{
		// timeout control
		if(h->timeoutCount < TIMEOUT_MS * TIMEOUT_BASE_FREQ / 1000)
		{
			h->timeoutCount += 1;
		}
		else
		{
			h->motor.Id_ref_pu_2q13 = 0;
			h->motor.Iq_ref_pu_2q13 = 0;
			h->timeoutCount = 0;
			h->timeoutState = 1;
		}
	}
#endif

	LED_Blink_Update(&h->led_blink);
}



int MD_Update_Async(MD_Handler_t* h)
{

	HAL_Delay(10);

	// Reset CAN Error
	if(HAL_CAN_GetState(h->hcan) == 0x05)
	{
		HAL_CAN_Init(h->hcan);
		CAN_Init();
		HAL_CAN_ResetError(h->hcan);
	}

#if DUMP_DEBUG_ENABLE
	if(Dump_isFull())
	{
		return 1;
	}
#endif

	return 0; /* 0: continue, 1: End*/
}


void MD_End(MD_Handler_t* h)
{
	h->motor.Id_ref_pu_2q13 = 0;
	h->motor.Iq_ref_pu_2q13 = 0;

	// Gate Disable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

	PWM_Stop(&h->pwm);

	Dump_Print();

#if DEBUG_PRINT_ENABLE
	printf("Finished.\r\n");
#endif

}



uint8_t getChannel()
{
	uint8_t ch = 0;

	ch |= !HAL_GPIO_ReadPin(CH_b0_GPIO_Port, CH_b0_Pin) << 0;
	ch |= !HAL_GPIO_ReadPin(CH_b1_GPIO_Port, CH_b1_Pin) << 1;
	ch |= !HAL_GPIO_ReadPin(CH_b2_GPIO_Port, CH_b2_Pin) << 2;

	return ch;
}




void timeoutReset(MD_Handler_t* h)
{
	h->timeoutCount = 0;
	h->timeoutState = 0;

	if(h->timeoutState == 1)
	{
		h->timeoutState = 0;

		Motor_Reset(&h->motor);

		//startPWM(&htim8);

		// Gate Enable
		//HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);
	}
}





void DRV_Setting(MD_Handler_t* h)
{


	// Gate Enable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);

	// Current Sensing Auto Offset Calibration
	HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_RESET);



	/*************************************************/
	#if DEBUG_PRINT_ENABLE

	DRV_ReadData(&h->drv8323, ADDR_FaultStatus1);
	DRV_ReadData(&h->drv8323, ADDR_FaultStatus2);
	DRV_ReadData(&h->drv8323, ADDR_DriverControl);
	DRV_ReadData(&h->drv8323, ADDR_GateDrive_HS);
	DRV_ReadData(&h->drv8323, ADDR_GateDrive_LS);
	DRV_ReadData(&h->drv8323, ADDR_OCP_Control);
	DRV_ReadData(&h->drv8323, ADDR_CSA_Control);

	printf("Initial register data.\r\n");

	PRINT_HEX(h->drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(h->drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(h->drv8323.Reg.DriverControl.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(h->drv8323.Reg.OCP_Control.word);
	PRINT_HEX(h->drv8323.Reg.CSA_Control.word);

	printf("-----------------------\r\n");

	#endif
	/*************************************************/


	#if 0
	DRV_ReadData(&drv8323, ADDR_GateDrive_HS);
	//drv8323.Reg.GateDrive_HS.IDRIVEP_HS = 0b1011; // 440mA
	//drv8323.Reg.GateDrive_HS.IDRIVEN_HS = 0b1011; // 880mA
	drv8323.Reg.GateDrive_HS.IDRIVEP_HS = 0b1001; // 330mA
	drv8323.Reg.GateDrive_HS.IDRIVEN_HS = 0b0111; // 380mA
	DRV_WriteData(&drv8323, ADDR_GateDrive_HS);

	DRV_ReadData(&drv8323, ADDR_GateDrive_LS);
	//drv8323.Reg.GateDrive_LS.IDRIVEP_LS = 0b1011; // 440mA
	//drv8323.Reg.GateDrive_LS.IDRIVEN_LS = 0b1011; // 880mA
	drv8323.Reg.GateDrive_LS.IDRIVEP_LS = 0b1001; // 330mA
	drv8323.Reg.GateDrive_LS.IDRIVEN_LS = 0b1001; // 380mA
	DRV_WriteData(&drv8323, ADDR_GateDrive_LS);
	#endif

	DRV_ReadData(&h->drv8323, ADDR_OCP_Control);
	h->drv8323.Reg.OCP_Control.TRETRY = 0b0; // VDS_OCP and SEN_OCP retry time is 4 ms
	h->drv8323.Reg.OCP_Control.DEAD_TIME = 0b01; // Dead Time : 100ns
	h->drv8323.Reg.OCP_Control.OCP_MODE = 0b00; // Overcurrent causes a latched fault
	h->drv8323.Reg.OCP_Control.OCP_DEG = 0b11; // Deglitch Time of 8us
	//drv8323.Reg.OCP_Control.VDS_LVL = 0b1001; // VDS = 0.75V -> ID = 75A
	h->drv8323.Reg.OCP_Control.VDS_LVL = 0b1111; // VDS = 1.88V -> ID = 75A
	DRV_WriteData(&h->drv8323, ADDR_OCP_Control);

	DRV_ReadData(&h->drv8323, ADDR_CSA_Control);
	//drv8323.Reg.CSA_Control.DIS_SEN = 0b1;	// Sense overcurrent fault is disabled
	//drv8323.Reg.CSA_Control.SEN_LVL = 0b00;	// Vsense = 0.25V -> 25A
	h->drv8323.Reg.CSA_Control.SEN_LVL = 0b11;	// Vsense = 1.0V -> 100A
	h->drv8323.Reg.CSA_Control.CSA_GAIN = 0b01;	// Amplifier Gain = 10V/V
	DRV_WriteData(&h->drv8323, ADDR_CSA_Control);

	#if 0
	DRV_ReadData(&drv8323, ADDR_DriverControl);
	drv8323.Reg.DriverControl.DIS_CPUV = 1;
	drv8323.Reg.DriverControl.DIS_GDF = 1;
	drv8323.Reg.DriverControl.OTW_REP = 1;
	DRV_WriteData(&drv8323, ADDR_DriverControl);
	#endif


	#if DEBUG_PRINT_ENABLE

	printf("Write data.\r\n");

	PRINT_HEX(h->drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(h->drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(h->drv8323.Reg.DriverControl.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(h->drv8323.Reg.OCP_Control.word);
	PRINT_HEX(h->drv8323.Reg.CSA_Control.word);

	printf("-----------------------\r\n");

	#endif

	DRV_ReadData(&h->drv8323, ADDR_DriverControl);
	h->drv8323.Reg.DriverControl.CLR_FLT = 1;	// Clear flt bit
	DRV_WriteData(&h->drv8323, ADDR_DriverControl);


	#if DEBUG_PRINT_ENABLE

	DRV_ReadData(&h->drv8323, ADDR_FaultStatus1);
	DRV_ReadData(&h->drv8323, ADDR_FaultStatus2);
	DRV_ReadData(&h->drv8323, ADDR_DriverControl);
	DRV_ReadData(&h->drv8323, ADDR_GateDrive_HS);
	DRV_ReadData(&h->drv8323, ADDR_GateDrive_LS);
	DRV_ReadData(&h->drv8323, ADDR_OCP_Control);
	DRV_ReadData(&h->drv8323, ADDR_CSA_Control);

	printf("Check register..\r\n");

	PRINT_HEX(h->drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(h->drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(h->drv8323.Reg.DriverControl.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(h->drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(h->drv8323.Reg.OCP_Control.word);
	PRINT_HEX(h->drv8323.Reg.CSA_Control.word);

	printf("-----------------------\r\n");

	#endif
}




inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...){
	int32_t TransStrLength;
	char TransStr[1024];

	va_list args;
	va_start(args, format);
	TransStrLength = vsprintf(TransStr, format, args);
	va_end(args);

	HAL_UART_Transmit(huart, (uint8_t*)TransStr, TransStrLength, 100);

	return TransStrLength;
}


int32_t printFloat(float val)
{
	int i;
	int charCount = 0;
	int first, temp;
	const int decNum = 4;
	int i_val;

	if(val < 0.0f)
	{
		i_val = val * pow(10.0, decNum) - 0.5;
		putchar('-');
		i_val *= -1;
		charCount += 1;
	}
	else
	{
		i_val = val * pow(10.0, decNum) + 0.5;
	}

	first = 0;
	for(i = 9; i >= 0; i--)
	{
		temp = (int)(i_val * pow(10.0, -1.0 * i)) % 10;

		if(i == decNum - 1)
		{
			if(first == 0)
			{
				first = 1;
				putchar('0');
			}
			putchar('.');
		}

		if(first == 1 || temp != 0)
		{
			putchar('0' + temp);
			charCount += 1;
			first = 1;
		}

	}

	return charCount;

}

