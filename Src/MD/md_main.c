


#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "md_main.h"

#include "main.h"
#include "flash.h"
#include "pwm.h"
#include "parameters.h"
#include "encoder.h"
#include "CurrentSensor.h"
#include "drv8323.h"
#include "canCom.h"
#include "sin_t.h"
#include "motorControl.h"
#include "dump_int.h"
#include "intMath.h"


extern MD_Handler_t md_sys;


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern CAN_HandleTypeDef hcan1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart2;



#define DEBUG_PRINT_ENABLE 1



#define  PRINT_HEX(x)  printf(#x " = %04x\n", (x))


void DRV_Setting();

inline void LED_blink(LED_Blink_t* h);

inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...);

int32_t printFloat(float val);


void MD_Init(MD_Handler_t* h)
{

	uint8_t p_ch, ch;


	h->led_blink.LED_blink_count = 0;
	h->led_blink.LED_blink_state = 0;
	h->led_blink.LED_blink_t_us = 0;
	h->led_blink.LED_blink_times = 0;
	h->led_blink.LED_blink_Ton_us = 50000;
	h->led_blink.LED_blink_Toff_us = 200000;
	h->led_blink.LED_blink_T_wait_us = 1000000;
	h->led_blink.LED_blink_Ts_us = 100;


	h->sequence = Seq_Init;

	h->carrier_counter = 0;

	DRV_Init();

	// Motor Handler initialize
	Motor_Init(&h->motor);

	h->timeoutEnable = 1;

#if DEBUG_PRINT_ENABLE

	printf("Hello\n");

#endif

	//printf("Hello SPI Gate Driver\n");

	DRV_Setting();


	/******** DEBUG ********/
	HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_RESET);


	p_ch = getChannel();

	int count;
	for(count = 0; count < 6; count++)
	{
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	}

	ch = getChannel();

	h->led_blink.LED_blink_times = ch;

	CAN_Init();

	Encoder_Init();


	CurrentSensor_Init();


	CurrentSensor_Start(&mainCS);

	PWM_Init();



	// Offset calibration
	#if 0
	float sum_uvw[3] = {0, 0, 0};
	for(count = 0; count < 1000; count++)
	{
	  HAL_Delay(1);
	  sum_uvw[0] += mainCS.V_Iu;
	  sum_uvw[1] += mainCS.V_Iv;
	  sum_uvw[2] += mainCS.V_Iw;
	}
	mainCS.Init.V_Iu_offset += sum_uvw[0] / 1000.0;
	mainCS.Init.V_Iv_offset += sum_uvw[1] / 1000.0;
	mainCS.Init.V_Iw_offset += sum_uvw[2] / 1000.0;
	#endif

	//HAL_Delay(1000);


	h->sequence = Seq_PosAdj;


	//h->motor.Init.theta_int_offset = setZeroEncoder((p_ch != ch)? 1: 0);
	h->pFlashData = (FlashStoredData_t*)Flash_load();
	if(ch != p_ch)
	{
		MD_Calibration(h);
	}
	h->motor.Init.theta_int_offset = h->pFlashData->theta_offset;

	printf("THETA_INT_OFFSET = %d\n", h->motor.Init.theta_int_offset);


	h->timeoutEnable = 1;
	h->timeoutCount = 0;


	h->sequence = Seq_Running; /* Start */

}



void MD_Calibration(MD_Handler_t* h)
{

	md_sys.motor.Igam_ref_pu_2q13 = (uint16_t)(5.0 / md_sys.motor.Init.I_base * 8192);
	md_sys.motor.Idel_ref_pu_2q13 = (uint16_t)(0.0 / md_sys.motor.Init.I_base * 8192);

	md_sys.motor.Init.theta_int_offset = 0;
	md_sys.motor.theta_force_int = 0;
	md_sys.motor.RunMode = MOTOR_MODE_CC_FORCE;

	HAL_Delay(1000);

	h->pFlashData->theta_offset = md_sys.motor.theta_re_int & SIN_TBL_MASK;

	if (!Flash_store())
	{
#if DEBUG_PRINT_ENABLE
		printf("Failed to write flash\n");
#endif
	}

	Motor_Reset(&md_sys.motor);
	md_sys.motor.RunMode = MOTOR_MODE_CC_VECTOR;

}



void MD_Update_SyncPWM(MD_Handler_t* h)
{

	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);


#if 1

	Encoder_Refresh(&mainEncoder);

	CurrentSensor_Refresh(&mainCS);


	if(h->sequence == Seq_Running || h->sequence == Seq_PosAdj)
	{

		h->motor.AD_Iu = mainCS.AD_Iu[0];
		h->motor.AD_Iv = mainCS.AD_Iv[0];
		h->motor.AD_Iw = mainCS.AD_Iw[0];
		h->motor.AD_Vdc = mainCS.AD_Vdc[0];
		h->motor.raw_theta_14bit = mainEncoder.raw_Angle;

		// Motor Controller Update
		Motor_Update(&h->motor);

#if 1
		htim8.Instance->CCR1 = h->motor.duty_u;
		htim8.Instance->CCR2 = h->motor.duty_v;
		htim8.Instance->CCR3 = h->motor.duty_w;
#endif

		h->carrier_counter++;

	}


	if(h->sequence == Seq_Running && !Dump_isFull())
	{


/*
		dump_record[dump_counter][0] = motor.Id_ref_pu_2q13;
		dump_record[dump_counter][1] = motor.Iq_ref_pu_2q13;
		dump_record[dump_counter][2] = motor.Id_pu_2q13;
		dump_record[dump_counter][3] = motor.Iq_pu_2q13;
*/

#if 0
		dump_record[dump_counter][0] = motor.Id_ref_pu_2q13;
		dump_record[dump_counter][1] = motor.Id_pu_2q13;
		dump_record[dump_counter][2] = motor.Vd_pu_2q13;
		dump_record[dump_counter][3] = motor.Id_error;
		dump_record[dump_counter][4] = motor.Id_error_integ.integ;
		dump_record[dump_counter][5] = motor.Vdc_pu_2q13;
#endif


#if 0
		dump_record[dump_counter][0] = motor.Id_ref_pu_2q13;
		dump_record[dump_counter][1] = motor.Id_pu_2q13;
		dump_record[dump_counter][2] = motor.Iq_pu_2q13;
		dump_record[dump_counter][3] = motor.Vd_pu_2q13;
		dump_record[dump_counter][4] = motor.Vq_pu_2q13;
		dump_record[dump_counter][5] = motor.Vu_pu_2q13;
		dump_record[dump_counter][6] = motor.Vv_pu_2q13;
		dump_record[dump_counter][7] = motor.Vw_pu_2q13;
#endif

#if 0
		dump_record[dump_counter][0] = htim8.Instance->CCR1;
		dump_record[dump_counter][1] = htim8.Instance->CCR2;
		dump_record[dump_counter][2] = htim8.Instance->CCR3;
		dump_record[dump_counter][3] = motor.Iu_pu_2q13;
		dump_record[dump_counter][4] = motor.Iv_pu_2q13;
		dump_record[dump_counter][5] = motor.Iw_pu_2q13;
#endif

#if 0
		dump_record[dump_counter][0] = htim8.Instance->CCR1;
		dump_record[dump_counter][1] = htim8.Instance->CCR2;
		dump_record[dump_counter][2] = htim8.Instance->CCR3;
		dump_record[dump_counter][3] = motor.AD_Iu;
		dump_record[dump_counter][4] = motor.AD_Iv;
		dump_record[dump_counter][5] = motor.AD_Iw;
		dump_record[dump_counter][6] = motor.AD_Vdc;
#endif


#if 0
		dump_record[dump_counter][0] = motor.Id_ref_pu_2q13;
		dump_record[dump_counter][1] = motor.Id_pu_2q13;
		dump_record[dump_counter][2] = motor.Iq_pu_2q13;
		dump_record[dump_counter][3] = motor.Vd_pu_2q13;
		dump_record[dump_counter][4] = motor.Vq_pu_2q13;
		dump_record[dump_counter][5] = motor.Iu_pu_2q13;
		dump_record[dump_counter][6] = motor.Iv_pu_2q13;
		dump_record[dump_counter][7] = motor.Iw_pu_2q13;
#endif

#if 0
		dump_record[dump_counter][0] = motor.Id_ref_pu_2q13;
		dump_record[dump_counter][1] = motor.Id_pu_2q13;
		dump_record[dump_counter][2] = motor.Vd_pu_2q13;
		dump_record[dump_counter][3] = motor.Id_error;
		dump_record[dump_counter][4] = motor.Id_error_integ.integ;
		dump_record[dump_counter][5] = motor.Iu_pu_2q13;
		dump_record[dump_counter][6] = motor.Iv_pu_2q13;
		dump_record[dump_counter][7] = motor.Iw_pu_2q13;
#endif


		if(dump_counter < DUMP_LENGTH)
		{
			dump_counter++;
		}

	}


	Encoder_Request(&mainEncoder);


#endif


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
			//stopPWM(&htim8);
			// Gate Disable
			//HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

			h->motor.Id_ref_pu_2q13 = 0;
			h->motor.Iq_ref_pu_2q13 = 0;

			h->timeoutCount = 0;
			h->timeoutState = 1;
		}
	}

#endif

	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);



	LED_blink(&h->led_blink);
}



int MD_Update_Async(MD_Handler_t* h)
{


	HAL_Delay(10);

	//printf("%d\r\n", mainEncoder.raw_Angle);
	//printf("%10d, %10d\n", motor.theta_m_int, motor.theta_re_int);
	//printf("%6d, %6d, %6d\n", motor.theta_re_int, motor.Va_pu_2q13, motor.Vb_pu_2q13);
	//printf("QVuvw: %6d, %6d, %6d, %6d\n", motor.theta_re_int, motor.Vu_pu_2q13, motor.Vv_pu_2q13, motor.Vw_pu_2q13);
	//printf("QAmp: %6d, %6d, %6d, %6d\n", motor.theta_re_int, motor.duty_u, motor.duty_v, motor.duty_w);

	//printf("0x%8x, 0x%8x\n", HAL_CAN_GetState(&hcan1), HAL_CAN_GetError(&hcan1));


	// Reset CAN Error
	if(HAL_CAN_GetState(&hcan1) == 0x05)
	{
		HAL_CAN_Init(&hcan1);
		CAN_Init();
		HAL_CAN_ResetError(&hcan1);
	}

	//printf("%d\r\n", motor.theta_re_int);

	//printf("%10d, %10d, %10d\r\n", motor.AD_Iu, motor.AD_Iv, motor.AD_Iw);
	//printf("%d\r\n", motor.Vdc_pu_2q13);
	//printf("%d,%d,%d,%d,%d,%d,%d\r\n", carrier_counter, motor.Vu_pu_2q13, motor.Vv_pu_2q13, motor.Vw_pu_2q13, motor.Iu_pu_2q13, motor.Iv_pu_2q13, motor.Iw_pu_2q13);
	//printf("%10d, %10d\r\n", motor.Ia_pu_2q13, motor.Ib_pu_2q13);
	//printf("%d,\t%10d,\t%10d\r\n", carrier_counter, motor.Id_pu_2q13, motor.Iq_pu_2q13);

	//printf("%10d, %10d, %10d\r\n", motor.Vd_pu_2q13,motor.Vq_pu_2q13, motor.Vdc_pu_2q13);

	//printf("%d, %d\r\n", integTest.integ, integTest.error);

	//printf("%d,%d,%d,%d\r\n", motor.duty_u, motor.duty_v, motor.duty_w, motor.Vdc_pu_2q13);

	//if(Dump_isFull()) printf("a");


	return 0; /* 0: continue, 1: End*/
}

void MD_End(MD_Handler_t* h)
{
	md_sys.motor.Id_ref_pu_2q13 = 0;
	md_sys.motor.Iq_ref_pu_2q13 = 0;


	//mainACR.Id_ref = 0.0f;
	//mainACR.Iq_ref = 0.0f;

	HAL_Delay(10);

	#if DEBUG_PRINT_ENABLE

	DRV_ReadData(&drv8323, ADDR_FaultStatus1);
	DRV_ReadData(&drv8323, ADDR_FaultStatus2);
	DRV_ReadData(&drv8323, ADDR_DriverControl);
	DRV_ReadData(&drv8323, ADDR_GateDrive_HS);
	DRV_ReadData(&drv8323, ADDR_GateDrive_LS);
	DRV_ReadData(&drv8323, ADDR_OCP_Control);
	DRV_ReadData(&drv8323, ADDR_CSA_Control);

	printf("Check register..\r\n");

	PRINT_HEX(drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(drv8323.Reg.DriverControl.word);
	PRINT_HEX(drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(drv8323.Reg.OCP_Control.word);
	PRINT_HEX(drv8323.Reg.CSA_Control.word);

	printf("-----------------------\r\n");

	#endif


	// Gate Disable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	stopPWM(&htim8);

	HAL_Delay(10);


	Dump_Print();

	/*
	printf("t, Id, Iq, Id_ref, Iq_ref, Vd_ref, Vq_ref, theta_re, omega\n");

	for(count = 0; count < DUMP_STEPS; count++)
	{

		printf("%.4e, ", count * DUMP_CYCLETIME);

		printf("%.4f, ", record[count].Id);
		printf("%.4f, ", record[count].Iq);
		printf("%.4f, ", record[count].Id_ref);
		printf("%.4f, ", record[count].Iq_ref);
		printf("%.4f, ", record[count].Vd_ref);
		printf("%.4f, ", record[count].Vq_ref);
		printf("%.4f, ", record[count].theta_re);
		printf("%.4f, ", record[count].omega);

		printf("\n");

	}
	*/

	printf("Finished.\r\n");
}



inline void LED_blink(LED_Blink_t* h)
{

	switch(h->LED_blink_state)
	{
	case 0: // OFF WAIT
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		if(h->LED_blink_t_us >= h->LED_blink_T_wait_us)
		{
			if(h->LED_blink_times > 0)
			{
				h->LED_blink_state = 1;
				h->LED_blink_count = 0;
			}

			h->LED_blink_t_us = 0;
		}
		break;

	case 1: // ON
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		if(h->LED_blink_t_us >= h->LED_blink_Ton_us)
		{
			h->LED_blink_count += 1;
			h->LED_blink_state = 2;
			h->LED_blink_t_us = 0;
		}
		break;

	case 2: // OFF
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		if(h->LED_blink_t_us >= h->LED_blink_Toff_us)
		{
			if(h->LED_blink_count < h->LED_blink_times)
				h->LED_blink_state = 1;
			else
				h->LED_blink_state = 0;

			h->LED_blink_t_us = 0;
		}
		break;

	default:

		break;
	}

	h->LED_blink_t_us += h->LED_blink_Ts_us;

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





void DRV_Setting()
{


	// Gate Enable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);

	// Current Sensing Auto Offset Calibration
	HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_RESET);



	/*************************************************/
	#if DEBUG_PRINT_ENABLE

	DRV_ReadData(&drv8323, ADDR_FaultStatus1);
	DRV_ReadData(&drv8323, ADDR_FaultStatus2);
	DRV_ReadData(&drv8323, ADDR_DriverControl);
	DRV_ReadData(&drv8323, ADDR_GateDrive_HS);
	DRV_ReadData(&drv8323, ADDR_GateDrive_LS);
	DRV_ReadData(&drv8323, ADDR_OCP_Control);
	DRV_ReadData(&drv8323, ADDR_CSA_Control);

	printf("Initial register data.\r\n");

	PRINT_HEX(drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(drv8323.Reg.DriverControl.word);
	PRINT_HEX(drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(drv8323.Reg.OCP_Control.word);
	PRINT_HEX(drv8323.Reg.CSA_Control.word);

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

	DRV_ReadData(&drv8323, ADDR_OCP_Control);
	drv8323.Reg.OCP_Control.TRETRY = 0b0; // VDS_OCP and SEN_OCP retry time is 4 ms
	drv8323.Reg.OCP_Control.DEAD_TIME = 0b01; // Dead Time : 100ns
	drv8323.Reg.OCP_Control.OCP_MODE = 0b00; // Overcurrent causes a latched fault
	drv8323.Reg.OCP_Control.OCP_DEG = 0b11; // Deglitch Time of 8us
	//drv8323.Reg.OCP_Control.VDS_LVL = 0b1001; // VDS = 0.75V -> ID = 75A
	drv8323.Reg.OCP_Control.VDS_LVL = 0b1111; // VDS = 1.88V -> ID = 75A
	DRV_WriteData(&drv8323, ADDR_OCP_Control);

	DRV_ReadData(&drv8323, ADDR_CSA_Control);
	//drv8323.Reg.CSA_Control.DIS_SEN = 0b1;	// Sense overcurrent fault is disabled
	//drv8323.Reg.CSA_Control.SEN_LVL = 0b00;	// Vsense = 0.25V -> 25A
	drv8323.Reg.CSA_Control.SEN_LVL = 0b11;	// Vsense = 1.0V -> 100A
	drv8323.Reg.CSA_Control.CSA_GAIN = 0b01;	// Amplifier Gain = 10V/V
	DRV_WriteData(&drv8323, ADDR_CSA_Control);

	#if 0
	DRV_ReadData(&drv8323, ADDR_DriverControl);
	drv8323.Reg.DriverControl.DIS_CPUV = 1;
	drv8323.Reg.DriverControl.DIS_GDF = 1;
	drv8323.Reg.DriverControl.OTW_REP = 1;
	DRV_WriteData(&drv8323, ADDR_DriverControl);
	#endif


	#if DEBUG_PRINT_ENABLE

	printf("Write data.\r\n");

	PRINT_HEX(drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(drv8323.Reg.DriverControl.word);
	PRINT_HEX(drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(drv8323.Reg.OCP_Control.word);
	PRINT_HEX(drv8323.Reg.CSA_Control.word);

	printf("-----------------------\r\n");

	#endif

	DRV_ReadData(&drv8323, ADDR_DriverControl);
	drv8323.Reg.DriverControl.CLR_FLT = 1;	// Clear flt bit
	DRV_WriteData(&drv8323, ADDR_DriverControl);


	#if DEBUG_PRINT_ENABLE

	DRV_ReadData(&drv8323, ADDR_FaultStatus1);
	DRV_ReadData(&drv8323, ADDR_FaultStatus2);
	DRV_ReadData(&drv8323, ADDR_DriverControl);
	DRV_ReadData(&drv8323, ADDR_GateDrive_HS);
	DRV_ReadData(&drv8323, ADDR_GateDrive_LS);
	DRV_ReadData(&drv8323, ADDR_OCP_Control);
	DRV_ReadData(&drv8323, ADDR_CSA_Control);

	printf("Check register..\r\n");

	PRINT_HEX(drv8323.Reg.FaultStatus1.word);
	PRINT_HEX(drv8323.Reg.FaultStatus2.word);
	PRINT_HEX(drv8323.Reg.DriverControl.word);
	PRINT_HEX(drv8323.Reg.GateDrive_HS.word);
	PRINT_HEX(drv8323.Reg.GateDrive_LS.word);
	PRINT_HEX(drv8323.Reg.OCP_Control.word);
	PRINT_HEX(drv8323.Reg.CSA_Control.word);

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

