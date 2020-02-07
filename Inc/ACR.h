

#ifndef _ACR_H_
#define _ACR_H_

#include "encoder.h"
#include "CurrentSensor.h"
#include "tim.h"


typedef struct
{

	float Kp;
	float Ki;

	float cycleTime;

	float Id_limit;
	float Iq_limit;

	float Id_error_integ_limit;
	float Iq_error_integ_limit;

	Encoder_TypeDef *hEncoder;

	CurrentSensor_TypeDef *hCS;

	TIM_HandleTypeDef* htim;

}ACR_InitTypeDef;


typedef struct
{

	ACR_InitTypeDef Init;


	uint8_t enable;

	float Id_limitError, Iq_limitError;

	float Id_ref, Iq_ref;

	float Id, Iq;

	float Id_error, Iq_error;

	float p_Id_error, p_Iq_error;

	float Id_error_integ, Iq_error_integ;

	float Vd_ref, Vq_ref;

	uint8_t forced_commute_enable;

	float forced_theta_re;

	float forced_cos_theta_re;
	float forced_sin_theta_re;



}ACR_TypeDef;



extern ACR_TypeDef mainACR;



void ACR_Init();


void ACR_Start(ACR_TypeDef *hACR);

void ACR_Stop(ACR_TypeDef *hACR);


void ACR_Refresh(ACR_TypeDef *hACR);

void ACR_Reset(ACR_TypeDef *hACR);



#endif /* _ACR_H_ */
