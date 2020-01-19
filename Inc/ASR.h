

#ifndef _ASR_H_
#define _ASR_H_


#include "encoder.h"
#include "ACR.h"


typedef struct
{

	float Kp;
	float Ki;

	float cycleTime;

	float omega_limit;

	float omega_error_integ_limit;

	Encoder_TypeDef *hEncoder;

	ACR_TypeDef *hACR;

	uint32_t prescaler;

}ASR_InitTypeDef;


typedef struct
{

	ASR_InitTypeDef Init;

	uint8_t enable;

	float omega_ref;

	float omega;

	float omega_error;

	float p_omega_error;

	float omega_error_integ;

	float Iq_ref;

	uint32_t prescalerCount;

	uint8_t firstLaunch;

	uint8_t launchFlg;

}ASR_TypeDef;



extern ASR_TypeDef mainASR;


void ASR_Init();


void ASR_Start(ASR_TypeDef *hASR);

void ASR_Stop(ASR_TypeDef *hASR);

void ASR_prescaler(ASR_TypeDef *hASR);

void ASR_Refresh(ASR_TypeDef *hASR);

void ASR_Reset(ASR_TypeDef *hASR);



#endif /* _ASR_H_ */
