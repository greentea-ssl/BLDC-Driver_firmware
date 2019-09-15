

#ifndef _ASR_H_
#define _ASR_H_



#define _ASR_ENABLE_		1



extern volatile float omega_ref;


extern volatile float omega_errorf;

extern volatile float omega_error_integ;

extern volatile float torque_ref;




extern int ASR_flg;
extern int ASR_prescalerCount;


#define ASR_prescale	10



void speedControl();

void ASR_reset();



#endif
