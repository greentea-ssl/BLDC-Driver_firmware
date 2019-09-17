

#ifndef _ACR_H_
#define _ACR_H_



extern const float ACR_cycleTime;

extern volatile float Id_ref;
extern volatile float Iq_ref;


extern volatile float Id;
extern volatile float Iq;


void ACR_Start();

void ACR_Stop();


void currentControl(void);

void ACR_Reset();



#endif
