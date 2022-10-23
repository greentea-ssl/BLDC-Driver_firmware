
#ifndef _PWM_H_
#define _PWM_H_


#include "main.h"


typedef struct
{
	TIM_HandleTypeDef *htim;
	float Vd_ref;
	float Vq_ref;
	int sector_SVM;
	float amp_u;
	float amp_v;
	float amp_w;
}PWM_Handler_t;


void PWM_Init(PWM_Handler_t* h);

void PWM_Start(PWM_Handler_t* h);

void PWM_Stop(PWM_Handler_t* h);

void setSVM_dq(PWM_Handler_t* h, float Vd_ref, float Vq_ref, float Vdc, float cos_theta_re, float sin_theta_re);



#endif /* _PWM_H_ */

