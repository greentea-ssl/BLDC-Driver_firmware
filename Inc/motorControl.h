
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include <stdint.h>


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

	float acr_omega;

	int32_t acr_Kp_q14;
	int32_t acr_Ki_q2;

}MotorInit_TypeDef;



typedef struct
{
	MotorInit_TypeDef Init;

	MotorParam_TypeDef motorParam;

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


	int32_t Id_error, Iq_error;
	int32_t Id_ref_pu_2q13, Iq_ref_pu_2q13;

	IntInteg_TypeDef Id_error_integ, Iq_error_integ;


}Motor_TypeDef;




void Motor_Init(Motor_TypeDef *hMotor);


void Motor_Update(Motor_TypeDef *hMotor);







#endif /* _MOTOR_CONTROL_H_ */

