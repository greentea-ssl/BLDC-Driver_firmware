
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include <stdint.h>


typedef struct
{

	float I_base;
	float V_base;

	uint16_t AD_Iu_offset, AD_Iv_offset, AD_Iw_offset;

	int32_t Gain_Iad2pu_s14;
	int32_t Gain_Vad2pu_s14;


}MotorInit_TypeDef;



typedef struct
{
	MotorInit_TypeDef Init;

	uint16_t AD_Iu, AD_Iv, AD_Iw, AD_Vdc;

	int16_t Iu_pu_2q13, Iv_pu_2q13, Iw_pu_2q13;
	int16_t Ia_pu_2q13, Ib_pu_2q13;
	int16_t Id_pu_2q13, Iq_pu_2q13;

	int16_t Vd_pu_2q13, Vq_pu_2q13;
	int16_t Va_pu_2q13, Vb_pu_2q13;
	int16_t Vu_pu_2q13, Vv_pu_2q13, Vw_pu_2q13;

	uint16_t duty_u, duty_v, duty_w;

	uint16_t raw_theta_14bit;
	int16_t theta_m_int, theta_re_int;

}Motor_TypeDef;




void Motor_Init(Motor_TypeDef *hMotor);


void Motor_Update(Motor_TypeDef *hMotor);







#endif /* _MOTOR_CONTROL_H_ */

