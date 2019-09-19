
#ifndef _MODULATOR_H_
#define _MODULATOR_H_



extern volatile float Vd_ref;
extern volatile float Vq_ref;


extern volatile int sector_SVM;





void setSVM(float ampl, float phase);

void setSVM_dq();





#endif

