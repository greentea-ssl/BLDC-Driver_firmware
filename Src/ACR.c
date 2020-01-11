

#include <math.h>

#include "ACR.h"
#include "main.h"

#include "string.h"

#include "ASR.h"


#include "tim.h"
#include "adc.h"
#include "spi.h"
#include "pwm.h"
#include "parameters.h"
#include "sin_t.h"
#include "sound.h"

#include "encoder.h"
#include "CurrentSensor.h"




volatile uint8_t ACR_enable = 0;


int soundCount = 0;


float msec = 0.0f;


ACR_TypeDef mainACR;



void ACR_Init()
{

	memset(&mainACR, 0x00, sizeof(mainACR));

	mainACR.Init.Kp = 0.1f;
	mainACR.Init.Ki = 400.0f;

	mainACR.Init.Id_limit = 15.0f;
	mainACR.Init.Iq_limit = 15.0f;

	mainACR.Init.Id_error_integ_limit = 1.0f;
	mainACR.Init.Iq_error_integ_limit = 1.0f;

	mainACR.Init.cycleTime = 100E-6;

	mainACR.Init.hEncoder = &mainEncoder;

	mainACR.Init.hCS = &mainCS;
	mainACR.Init.htim = &htim8;

}



void ACR_Start(ACR_TypeDef *hACR)
{

	hACR->enable = 1;
	ACR_Reset(hACR);

}


void ACR_Stop(ACR_TypeDef *hACR)
{

	hACR->enable = 0;
	ACR_Reset(hACR);

}



inline void ACR_Refresh(ACR_TypeDef *hACR)
{


	static float _Id_ref;
	static float _Iq_ref;

	static ACR_InitTypeDef *hACR_Init;

	hACR_Init = &hACR->Init;

	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_SET);

	/*
	hACR->Id_ref = 0.0f;
	hACR->Iq_ref = 0.5f + 0.75f * sin_table2[(int)((fmod(mainEncoder.theta * POLES + 4.14159f, 2.0f * M_PI) * 0.3183f + 0.5f) * 5000.0f)];
	*/

	Encoder_Refresh(hACR_Init->hEncoder);


	CurrentSensor_Refresh(&mainCS, sector_SVM);


	CurrentSensor_getIdq(&mainCS, &hACR->Id, &hACR->Iq, hACR_Init->hEncoder->cos_theta_re, hACR_Init->hEncoder->sin_theta_re);


	/*
	 * 強制転流
	 */
	if(hACR->forced_commute_enable)
	{
		/*
		float _forced_theta_re = fmodf(forced_theta * POLE_PAIRS, 2.0f * M_PI);

		if(_forced_theta_re < 0.0f)				forced_theta_re = _forced_theta_re + 2 * M_PI;
		else if(_forced_theta_re >= 2 * M_PI)	forced_theta_re = _forced_theta_re - 2 * M_PI;
		else									forced_theta_re = _forced_theta_re;

		*/

		hACR->forced_cos_theta_re = sin_table2[(int)((hACR->forced_theta_re * 0.3183f + 0.5f) * 5000.0f)];
		hACR->forced_sin_theta_re = sin_table2[(int)(hACR->forced_theta_re * 1591.54943f)];

		CurrentSensor_getIdq(&mainCS, &hACR->Id, &hACR->Iq, hACR->forced_cos_theta_re, hACR->forced_sin_theta_re);

	}
	else
	{

		CurrentSensor_getIdq(&mainCS, &hACR->Id, &hACR->Iq, hACR_Init->hEncoder->cos_theta_re, hACR_Init->hEncoder->sin_theta_re);

		// 旧実装
		// refreshEncoder();
	}



	// 旧実装
	//get_current_dq(&Id, &Iq, sector_SVM, cos_theta_re, sin_theta_re);


	if(hACR_Init->hEncoder->theta_re < M_PI)
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);



	/********** ACR (Auto Current Regulator) **********/

	if(hACR->enable /*&& soundCount == -1*/)
	{

		_Id_ref = hACR->Id_ref;
		//_Iq_ref = hACR->Iq_ref + 0.75f * sin_table2[(int)((fmod(mainEncoder.theta * POLES + 4.14159f, 2.0f * M_PI) * 0.3183f + 0.5f) * 5000.0f)];
		_Iq_ref = hACR->Iq_ref;


		if(_Id_ref < -hACR_Init->Id_limit)			_Id_ref = -hACR_Init->Id_limit;
		else if(_Id_ref > hACR_Init->Id_limit)		_Id_ref = hACR_Init->Id_limit;

		if(_Iq_ref < -hACR_Init->Iq_limit)			_Iq_ref = -hACR_Init->Iq_limit;
		else if(_Iq_ref > hACR_Init->Iq_limit)		_Iq_ref = hACR_Init->Iq_limit;


		hACR->Id_error = _Id_ref - hACR->Id;
		hACR->Iq_error = _Iq_ref - hACR->Iq;


		hACR->Id_error_integ += hACR_Init->cycleTime * 0.5f * (hACR->Id_error + hACR->p_Id_error);
		hACR->Iq_error_integ += hACR_Init->cycleTime * 0.5f * (hACR->Iq_error + hACR->p_Iq_error);


		if(hACR->Id_error_integ > hACR_Init->Id_error_integ_limit)
		{
			hACR->Id_error_integ = hACR_Init->Id_error_integ_limit;
		}
		else if(hACR->Id_error_integ < -1.0 * hACR_Init->Id_error_integ_limit)
		{
			hACR->Id_error_integ = -1.0 * hACR_Init->Id_error_integ_limit;
		}

		if(hACR->Iq_error_integ > hACR_Init->Iq_error_integ_limit)
		{
			hACR->Iq_error_integ = hACR_Init->Iq_error_integ_limit;
		}
		else if(hACR->Iq_error_integ < -1.0 * hACR_Init->Iq_error_integ_limit)
		{
			hACR->Iq_error_integ = -1.0 * hACR_Init->Iq_error_integ_limit;
		}


		hACR->p_Id_error = hACR->Id_error;
		hACR->p_Iq_error = hACR->Iq_error;

		/*
		// integral
		Id_error_integ_temp1 = Id_error + Id_error_integ_temp2;
		if(Id_error_integ_temp1 < -1000000.0) Id_error_integ_temp1 = -1000000.0;
		else if(Id_error_integ_temp1 > 1000000.0) Id_error_integ_temp1 = 1000000.0;
		Id_error_integ = ACR_cycleTime * 0.5f * (Id_error_integ_temp1 + Id_error_integ_temp2);
		Id_error_integ_temp2 = Id_error_integ_temp1;

		Iq_error_integ_temp1 = Iq_error + Iq_error_integ_temp2;
		if(Iq_error_integ_temp1 < -1000000.0) Iq_error_integ_temp1 = -1000000.0;
		else if(Iq_error_integ_temp1 > 1000000.0) Iq_error_integ_temp1 = 1000000.0;
		Iq_error_integ = ACR_cycleTime * 0.5f * (Iq_error_integ_temp1 + Iq_error_integ_temp2);
		Iq_error_integ_temp2 = Iq_error_integ_temp1;
		*/

		hACR->Vd_ref = hACR_Init->Kp * hACR->Id_error + hACR_Init->Ki * hACR->Id_error_integ;
		hACR->Vq_ref = hACR_Init->Kp * hACR->Iq_error + hACR_Init->Ki * hACR->Iq_error_integ;

	}

	/*
	if(soundCount < 66641)
	{
		hACR.Vq_ref += soSound[soundCount++] / 127.0f * 3.0;
	}
	*/


	/********* end of ACR **********/


	if(HAL_GPIO_ReadPin(BR_FLT_GPIO_Port, BR_FLT_Pin) == GPIO_PIN_RESET)
	{
		//HAL_NVIC_SystemReset();
	}


	if(hACR->forced_commute_enable)
	{
		setSVM_dq(&htim8, hACR->Vd_ref, hACR->Vq_ref, hACR->forced_cos_theta_re, hACR->forced_sin_theta_re);
	}
	else
	{
		setSVM_dq(&htim8, hACR->Vd_ref, hACR->Vq_ref, hACR_Init->hEncoder->cos_theta_re, hACR_Init->hEncoder->sin_theta_re);
	}




#if _ACR_DUMP_

	if(ACR_dump_count < ACR_DUMP_STEPS)
	{
		Id_dump[ACR_dump_count] = Id;
		Iq_dump[ACR_dump_count] = Iq;
		Id_ref_dump[ACR_dump_count] = Id_ref;
		Iq_ref_dump[ACR_dump_count] = Iq_ref;
		Vd_ref_dump[ACR_dump_count] = Vd_ref;
		Vq_ref_dump[ACR_dump_count] = Vq_ref;
		ACR_dump_count += 1;
	}

#endif


	Encoder_Request(hACR_Init->hEncoder);


	// Auto Speed Regulator launching
	ASR_prescalerCount += 1;
	if(ASR_prescalerCount >= ASR_prescale)
	{
		ASR_flg = 1;
		ASR_prescalerCount = 0;
	}

	msec += 0.1f;


	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

	return;
}



inline void ACR_Reset(ACR_TypeDef *hACR)
{

	hACR->Id_error_integ = 0.0f;
	hACR->Iq_error_integ = 0.0f;

	hACR->Id = hACR->Id_ref = 0.0f;
	hACR->Iq = hACR->Iq_ref = 0.0f;

	hACR->Vd_ref = 0.0f;
	hACR->Vq_ref = 0.0f;

}











