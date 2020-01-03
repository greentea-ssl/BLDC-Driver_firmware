
#ifndef _PWM_H_
#define _PWM_H_



extern volatile float Vd_ref;
extern volatile float Vq_ref;


extern volatile int sector_SVM;





void setSVM(float ampl, float phase);

void setSVM_dq();





#endif /* _PWM_H_ */

