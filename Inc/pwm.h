
#ifndef _PWM_H_
#define _PWM_H_


#include "tim.h"



extern volatile float Vdc;


extern volatile float Vd_ref;
extern volatile float Vq_ref;


extern volatile int sector_SVM;





void startPWM(TIM_HandleTypeDef *htim);

void stopPWM(TIM_HandleTypeDef *htim);


void setSVM(float ampl, float phase);

void setSVM_dq(TIM_HandleTypeDef *htim, float Vd, float Vq, float cos_theta_re, float sin_theta_re);





#endif /* _PWM_H_ */

