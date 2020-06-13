

#include "APR.h"

#include <math.h>
#include <string.h>

#include "parameters.h"

#include "ASR.h"
#include "pwm.h"

#include "encoder.h"


APR_TypeDef mainAPR;


void APR_Init()
{
	memset(&mainAPR, 0x00, sizeof(mainAPR));

	mainAPR.Init.Kp = 0.1f;
	mainAPR.Init.Ki = 0.0f;
	mainAPR.Init.Kd = 0.0f;

	mainAPR.Init.theta_error_integ_limit = 10000.0f;
	mainAPR.Init.cycleTime = 1E-3;
	mainAPR.Init.prescaler = 10;

	mainAPR.Init.hEncoder = &mainEncoder;
	mainAPR.Init.hASR = &mainASR;
	mainAPR.Init.hACR = &mainACR;
	mainAPR.Init.htim = &htim8;

	mainAPR.MV_type = 0;


}


void APR_Start(APR_TypeDef *hAPR)
{

	hAPR->enable = 1;
	APR_Reset(hAPR);

}

void APR_Stop(APR_TypeDef *hAPR)
{

	hAPR->enable = 0;
	APR_Reset(hAPR);

}


inline void APR_prescaler(APR_TypeDef *hAPR)
{

	hAPR->prescalerCount += 1;

	if(hAPR->prescalerCount >= hAPR->Init.prescaler)
	{
		hAPR->launchFlg = 1;
		hAPR->prescalerCount = 0;
	}

}


inline void APR_Refresh(APR_TypeDef *hAPR)
{

	static APR_InitTypeDef *hAPR_Init;

	// 有効時のみ実行
	if(hAPR->enable == 0)
	{
		return;
	}

	// プリスケーラリセット時のみ実行
	if(hAPR->launchFlg == 0)
	{
		return;
	}
	hAPR->launchFlg = 0;


	hAPR_Init = &hAPR->Init;

	hAPR->theta = hAPR_Init->hEncoder->theta_multiturn;

	// 速度偏差
	hAPR->theta_error = hAPR->theta_ref - hAPR->theta;

	// integral
	hAPR->theta_error_integ += hAPR_Init->cycleTime * 0.5 * (hAPR->theta_error + hAPR->p_theta_error);

	hAPR->p_theta_error = hAPR->theta_error;

	if(hAPR->MV_type == 0)
	{
		// P-D型
		hAPR_Init->hASR->omega_ref = hAPR->omega_ref = hAPR_Init->Kp * hAPR->theta_error - hAPR_Init->Kd * hAPR_Init->hEncoder->omega;
	}
	else if(hAPR->MV_type == 2)
	{
		// I-PD型
		hAPR->Vq_ref =
				hAPR_Init->Kp * hAPR->theta_error
				+ hAPR_Init->Ki * hAPR->theta_error_integ
				- hAPR_Init->Kd * hAPR_Init->hEncoder->omega;

		setSVM_dq(&htim8, 0.0f, hAPR->Vq_ref, hAPR_Init->hEncoder->cos_theta_re, hAPR_Init->hEncoder->sin_theta_re);

	}


	return;
}



inline void APR_Reset(APR_TypeDef *hAPR)
{

	hAPR->theta = 0.0f;

	hAPR->theta_ref = 0.0f;

	hAPR->theta_error_integ = 0.0f;

	hAPR->p_theta_error = 0.0f;

}





