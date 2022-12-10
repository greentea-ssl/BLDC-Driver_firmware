
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include <stdint.h>

#include "parameters.h"

#include "intMath.h"


typedef struct
{
	uint16_t Pn;
	float R, Ld, Lq;
}MotorParam_TypeDef;


typedef struct
{

	float I_base;
	float V_base;

	uint16_t AD_Iu_offset, AD_Iv_offset, AD_Iw_offset;

	uint16_t PWM_PRR;

	uint16_t DutyRateLimit_q5;

	int32_t Gain_Iad2pu_s14;
	int32_t Gain_Vad2pu_s14;

	int32_t Gain_Ib_by_Vb_q10;

	int32_t Gain_dOmegaInt_to_omegaQ5_q8;

	float acr_omega;

	int32_t acr_Kp_q14;
	int32_t acr_Ki_q2;

	int32_t acr_limErrFB_gain_q10;

	int16_t Id_limit_pu_2q13;
	int16_t Iq_limit_pu_2q13;

	int16_t theta_int_offset;

}MotorInit_TypeDef;


typedef enum{
	MOTOR_MODE_CV_FORCE = 0,
	MOTOR_MODE_CC_FORCE = 1,
	MOTOR_MODE_CV_VECTOR = 2,
	MOTOR_MODE_CC_VECTOR = 3,
	MOTOR_MODE_CV_MIDI = 4,
}Motor_RunMode_Enum;


typedef struct
{
	MotorInit_TypeDef Init;

	MotorParam_TypeDef motorParam;

	Motor_RunMode_Enum RunMode;

	uint16_t AD_Iu, AD_Iv, AD_Iw, AD_Vdc;

	int16_t Iu_pu_2q13, Iv_pu_2q13, Iw_pu_2q13;
	int16_t Ia_pu_2q13, Ib_pu_2q13;
	int16_t Id_pu_2q13, Iq_pu_2q13;

	int16_t Vdc_pu_2q13;

	int16_t Vd_pu_2q13, Vq_pu_2q13;
	int16_t Va_pu_2q13, Vb_pu_2q13;
	int16_t Vu_pu_2q13, Vv_pu_2q13, Vw_pu_2q13;

	uint16_t duty_u, duty_v, duty_w;

	uint8_t sector;

	uint16_t raw_theta_14bit;
	int16_t theta_m_int, theta_re_int;

	int16_t p_theta_int_buf[SPEED_CALC_BUF_SIZE];
	int16_t p_theta_buf_count;
	int16_t omega_q5;

	// Forced commutation
	int8_t forced_commutation_enable;
	int32_t Igam_ref_pu_2q13, Idel_ref_pu_2q13;
	int16_t theta_force_int;

	// ACR
	int32_t Id_error, Iq_error;
	int32_t Id_ref_pu_2q13, Iq_ref_pu_2q13;

	// MIDI output
	uint32_t MIDI_periodTable_us[128];
	uint32_t MIDI_cycle_us;
	uint32_t MIDI_count_us;
	uint8_t MIDI_notenum;
	uint8_t MIDI_vel;

	int32_t Vd_limit_error, Vq_limit_error;

	IntInteg_TypeDef Id_error_integ, Iq_error_integ;


}Motor_TypeDef;




void Motor_Init(Motor_TypeDef *hMotor, uint16_t pwm_period);


void Motor_ADCUpdate(Motor_TypeDef *hMotor);


void Motor_Reset(Motor_TypeDef *hMotor);




#endif /* _MOTOR_CONTROL_H_ */

