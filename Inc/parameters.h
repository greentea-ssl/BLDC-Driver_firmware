

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_




// RealRotorAngle = MeasuredAngle + this
extern float theta_offset;

// RealElecAngle = MeasuredElecAngle + this
extern float theta_re_offset;

// For Motor

// Magnet Poles
#define POLES	22


#define 	KV	370.0

#define 	KE	(60.0f / (KV * 2 * M_PI))

#define 	KT	(POLES / 2 * KE)




// For Encoder

// Encoder Resolution
#define ENCODER_RESOL 16384



// For PWM

// PWM resolution
#define PWM_RESOL	8000.0f





#endif

