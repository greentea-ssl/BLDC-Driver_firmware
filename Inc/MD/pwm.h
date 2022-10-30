
#ifndef _PWM_H_
#define _PWM_H_


#include "main.h"


typedef struct
{
	TIM_HandleTypeDef *htim;
	uint16_t duty_u;
	uint16_t duty_v;
	uint16_t duty_w;
}PWM_Handler_t;


void PWM_Init(PWM_Handler_t* h);

void PWM_Start(PWM_Handler_t* h);

void PWM_Stop(PWM_Handler_t* h);

void PWM_SetDuty(PWM_Handler_t* h);

void PWM_UpdateDuty(PWM_Handler_t* h);

void PWM_InjectCommonMode_MinMax(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period);

void PWM_InjectCommonMode_TwoPhaseUp(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period);

void PWM_InjectCommonMode_TwoPhaseLow(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period);

void PWM_InjectCommonMode_AwayFromSwitching(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period);

void PWM_InjectCommonMode_AwayFromSwitching_MinMaxInLow(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period);


#endif /* _PWM_H_ */

