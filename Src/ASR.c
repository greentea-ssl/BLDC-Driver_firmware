

#include "ASR.h"

#include <math.h>
#include <string.h>

#include "parameters.h"

#include "ACR.h"

#include "encoder.h"

/*

volatile uint8_t ASR_enable = 0;


float Kp_ASR = 0.3;
float Ki_ASR = 20.0;



const float ASR_cycleTime = 1E-3;

float omega_limit = 1000000.0;

volatile float omega_ref = 0.0f;


volatile float omega_error = 0.0f;

volatile float omega_error_integ = 0.0f;

volatile float torque_ref = 0.0;

volatile float coggingIq = 0.0f;


int ASR_steps = 0;


int ASR_flg = 0;
int ASR_prescalerCount = 0;

volatile float omega = 0.0f;


volatile float p_theta = 0.0f;


float d_theta;

float _omega_ref;

float omega_error_integ_temp1 = 0.0f;
float omega_error_integ_temp2 = 0.0f;


*/


ASR_TypeDef mainASR;


void ASR_Init()
{
	memset(&mainASR, 0x00, sizeof(mainASR));

	mainASR.Init.Kp = 0.3f;
	mainASR.Init.Ki = 20.0f;
	mainASR.Init.omega_limit = 400.0f;
	mainASR.Init.omega_error_integ_limit = 10000.0f;
	mainASR.Init.cycleTime = 1E-3;
	mainASR.Init.prescaler = 10;

	mainASR.Init.hEncoder = &mainEncoder;
	mainASR.Init.hACR = &mainACR;

	mainASR.firstLaunch = 1;

	mainASR.omega = 0.0f;

}


void ASR_Start(ASR_TypeDef *hASR)
{

	hASR->enable = 1;
	ASR_Reset(hASR);

}

void ASR_Stop(ASR_TypeDef *hASR)
{

	hASR->enable = 0;
	ASR_Reset(hASR);

}


inline void ASR_prescaler(ASR_TypeDef *hASR)
{

	if(hASR->prescalerCount >= hASR->Init.prescaler)
	{
		hASR->launchFlg = 1;
		hASR->prescalerCount = 0;
	}
	else
	{
		hASR->prescalerCount += 1;
	}

}


inline void ASR_Refresh(ASR_TypeDef *hASR)
{

	static float d_theta;
	static float _omega_ref;
	static float torque_ref;

	static ASR_InitTypeDef *hASR_Init;

	hASR_Init = &hASR->Init;

	// プリスケーラリセット時のみ実行
	if(hASR->launchFlg == 0)
	{
		return;
	}
	hASR->launchFlg = 0;


	if(hASR->firstLaunch != 0)
	{
		d_theta = 0.0f;
		hASR->firstLaunch = 0;
	}
	else
	{
		d_theta = hASR_Init->hEncoder->theta - hASR->p_theta;
	}

	hASR->p_theta = hASR_Init->hEncoder->theta;

	if(d_theta < - M_PI)		d_theta += 2 * M_PI;
	else if(d_theta > M_PI)		d_theta -= 2 * M_PI;

	hASR->omega = hASR->omega * 0.5f + 0.5f * d_theta / hASR_Init->cycleTime;


	if(hASR->enable == 0)
	{
		return;
	}

	if(hASR->omega_ref < -hASR_Init->omega_limit)		_omega_ref = -hASR_Init->omega_limit;
	else if(hASR->omega_ref > hASR_Init->omega_limit)	_omega_ref = hASR_Init->omega_limit;
	else												_omega_ref = hASR->omega_ref;

	hASR->omega_error = _omega_ref - hASR->omega;

	// integral

	hASR->omega_error_integ += hASR_Init->cycleTime * 0.5 * (hASR->omega_error + hASR->p_omega_error);

	hASR->p_omega_error = hASR->omega_error;

	torque_ref = hASR_Init->Kp * hASR->omega_error + hASR_Init->Ki * hASR->omega_error_integ;

	hASR_Init->hACR->Id_ref = 0.0f;
	hASR_Init->hACR->Iq_ref = hASR->Iq_ref = KT * torque_ref;


	return;
}



inline void ASR_Reset(ASR_TypeDef *hASR)
{

	hASR->p_theta = 0.0f;

	hASR->firstLaunch = 1;

	hASR->omega_error_integ = 0.0f;

	hASR->omega = 0.0f;

	hASR->omega_ref = 0.0f;


}





