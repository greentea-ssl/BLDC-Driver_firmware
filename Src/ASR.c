

#include "ASR.h"

#include <math.h>
#include <string.h>

#include "parameters.h"

#include "ACR.h"

#include "encoder.h"


ASR_TypeDef mainASR;


void ASR_Init()
{
	memset(&mainASR, 0x00, sizeof(mainASR));

	mainASR.Init.Kp = 0.5f;
	mainASR.Init.Ki = 15.0f;
	mainASR.Init.omega_limit = 400.0f;
	mainASR.Init.omega_error_integ_limit = 1000.0f;
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

	hASR->prescalerCount += 1;

	if(hASR->prescalerCount >= hASR->Init.prescaler)
	{
		hASR->launchFlg = 1;
		hASR->prescalerCount = 0;
	}

}


inline void ASR_Refresh(ASR_TypeDef *hASR)
{

	static float d_theta;
	static float _omega_ref;
	static float torque_ref;

	static ASR_InitTypeDef *hASR_Init;

	// 有効時のみ実行
	if(hASR->enable == 0)
	{
		return;
	}

	// プリスケーラリセット時のみ実行
	if(hASR->launchFlg == 0)
	{
		return;
	}
	hASR->launchFlg = 0;


	hASR_Init = &hASR->Init;

	hASR->omega = hASR_Init->hEncoder->omega;

	// 速度制限
	if(hASR->omega_ref < -hASR_Init->omega_limit)		_omega_ref = -hASR_Init->omega_limit;
	else if(hASR->omega_ref > hASR_Init->omega_limit)	_omega_ref = hASR_Init->omega_limit;
	else												_omega_ref = hASR->omega_ref;

	// 速度偏差
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

	hASR->firstLaunch = 1;

	hASR->omega_error_integ = 0.0f;

	hASR->omega = 0.0f;

	hASR->omega_ref = 0.0f;


}





