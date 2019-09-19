

#ifndef _ASR_H_
#define _ASR_H_



extern volatile float omega_ref;


extern volatile float omega_errorf;

extern volatile float omega_error_integ;

extern volatile float torque_ref;




extern int ASR_flg;
extern int ASR_prescalerCount;


#define ASR_prescale	10


void ASR_Start();

void ASR_Stop();


void speedControl();

void ASR_Reset();



#endif
