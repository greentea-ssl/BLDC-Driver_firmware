

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_


// Motor Select

//#define _MOTOR_QUANUM_MT4108_KV370_
//#define _MOTOR_SUNNYSKY_V4006_KV320_
#define _MOTOR_SUNNYSKY_V2806_KV400_


/******************* For Motor ****************************/

// Magnet Poles
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define POLE_PAIRS	11
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define POLE_PAIRS	12
#endif
#ifdef _MOTOR_SUNNYSKY_V2806_KV400_
#define POLE_PAIRS	7
#endif

// Kv constant [rpm/V]
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define 	MOTOR_KV	(370)
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define 	MOTOR_KV	(320)
#endif
#ifdef _MOTOR_SUNNYSKY_V2806_KV400_
#define 	MOTOR_KV	(400)
#endif

// Electromotive force constant [V/(mech.rad/s)]
#define 	MOTOR_KE	(60.0f / (MOTOR_KV * 2 * M_PI))

// Torque Constant [N*m/A]
#define 	MOTOR_KT	MOTOR_KE

#define MOTOR_psi	(MOTOR_KE / POLE_PAIRS)

// Motor parameters
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define MOTOR_R    (0.13)
#define MOTOR_Ld   (24.0E-6)
#define MOTOR_Lq   (42.5E-6)
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define MOTOR_R    (0.14)
#define MOTOR_Ld   (49E-6)
#define MOTOR_Lq   (62E-6)
#endif
#ifdef _MOTOR_SUNNYSKY_V2806_KV400_
#define MOTOR_R    (0.35)
#define MOTOR_Ld   (76.1E-6)
#define MOTOR_Lq   (76.1E-6)
#endif

// Current Limit
#ifdef _MOTOR_QUANUM_MT4108_KV370_
#define CURRENT_RATING	(15.0)
#endif
#ifdef _MOTOR_SUNNYSKY_V4006_KV320_
#define CURRENT_RATING	(15.0)
#endif
#ifdef _MOTOR_SUNNYSKY_V2806_KV400_
#define CURRENT_RATING	(10.0)
#endif


/******************* MIDI Output ********************/

#define MIDI_GAIN_Q5	(1) // 0~31


/******************* For PWM ************************/

#define NO_UPDATE_ON_BOTTOM 	0


/******************* For Encoder ****************************/

#define SPEED_CALC_BUF_SIZE		(14)


/******************* Timeout ***********************/


#define TIMEOUT_BASE_FREQ		10000

#define TIMEOUT_MS				200




#endif

