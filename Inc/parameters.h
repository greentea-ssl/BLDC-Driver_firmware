

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_



// Motor Select

//#define _MOTOR_QUANUM_MT4108_KV370_
#define _MOTOR_SUNNYSKY_V4006_KV320_




/******************* For Motor ****************************/

// Magnet Poles
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define POLE_PAIRS	11
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define POLE_PAIRS	12
#endif

// Kv constant [rpm/V]
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define 	KV	370.0
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define 	KV	320.0
#endif

// Electromotive force constant [V/(rad/s)]
#define 	KE	(60.0f / (KV * 2 * M_PI))

// Torque Constant [N*m/A]
#define 	KT	(POLE_PAIRS * KE)




/******************* For Encoder ****************************/

// Encoder Resolution
#define ENCODER_RESOL 16384



// For PWM

// PWM resolution
#define PWM_RESOL	8000.0f





#endif

