

#ifndef _APR_H_
#define _APR_H_



#include "encoder.h"
#include "ASR.h"


typedef struct
{

	float Kp;
	float Ki;
	float Kd;

	float cycleTime;

	float theta_error_integ_limit;

	Encoder_TypeDef *hEncoder;

	ASR_TypeDef *hASR;

	ACR_TypeDef *hACR;

	TIM_HandleTypeDef* htim;

	uint32_t prescaler;

}APR_InitTypeDef;


typedef struct
{

	APR_InitTypeDef Init;

	uint8_t enable;

	uint8_t MV_type; // 0: omega_ref ,1: Iq_ref, 2: Vq_ref

	float theta_ref;

	float theta;

	float theta_error;

	float p_theta_error;

	float theta_error_integ;

	float omega_ref;

	float Iq_ref;

	float Vq_ref;

	uint32_t prescalerCount;

	uint8_t firstLaunch;

	uint8_t launchFlg;

}APR_TypeDef;



extern APR_TypeDef mainAPR;


void APR_Init();


void APR_Start(APR_TypeDef *hAPR);

void APR_Stop(APR_TypeDef *hAPR);

void APR_prescaler(APR_TypeDef *hAPR);

void APR_Refresh(APR_TypeDef *hAPR);

void APR_Reset(APR_TypeDef *hAPR);



#endif /* _APR_H_ */
