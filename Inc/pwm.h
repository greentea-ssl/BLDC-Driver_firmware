
#ifndef _PWM_H_
#define _PWM_H_


#include "main.h"



extern volatile float Vd_ref;
extern volatile float Vq_ref;


extern volatile int sector_SVM;


extern volatile float amp_u;
extern volatile float amp_v;
extern volatile float amp_w;


void PWM_Init();


void startPWM(TIM_HandleTypeDef *htim);

void stopPWM(TIM_HandleTypeDef *htim);


void setSVM(float ampl, float phase);

void setSVM_dq(TIM_HandleTypeDef *htim, float Vd_ref, float Vq_ref, float Vdc, float cos_theta_re, float sin_theta_re);



#endif /* _PWM_H_ */

