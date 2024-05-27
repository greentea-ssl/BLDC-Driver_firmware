


#include "motor_control.h"

#include <md/parameters.h>
#include <md/pwm.h>
#include <md/sin_t.h>
#include <string.h>

/***** Private functions prototypes *****/

inline void uvw2ab(int16_t *a, int16_t *b, int16_t u, int16_t v, int16_t w)
{
	const int32_t C_uvw2ab_s14[2][3] = {{13377, -6689, -6689}, {0, 11585, -11585}};
	*a = (C_uvw2ab_s14[0][0] * u + C_uvw2ab_s14[0][1] * v + C_uvw2ab_s14[0][2] * w) >> 14;
	*b = (C_uvw2ab_s14[1][0] * u + C_uvw2ab_s14[1][1] * v + C_uvw2ab_s14[1][2] * w) >> 14;
}

inline void ab2uvw(int16_t *u, int16_t *v, int16_t *w, int16_t a, int16_t b)
{
	const int32_t C_ab2uvw_s14[3][2] = {{13377, 0}, {-6689, 11585}, {-6689, -11585}};
	*u = (C_ab2uvw_s14[0][0] * a + C_ab2uvw_s14[0][1] * b) >> 14;
	*v = (C_ab2uvw_s14[1][0] * a + C_ab2uvw_s14[1][1] * b) >> 14;
	*w = (C_ab2uvw_s14[2][0] * a + C_ab2uvw_s14[2][1] * b) >> 14;
}

inline void ab2dq(int16_t *d, int16_t *q, uint16_t theta_re, int16_t a, int16_t b)
{
	int32_t cos_theta_re = COS_Q14(theta_re & SIN_TBL_MASK);
	int32_t sin_theta_re = SIN_Q14(theta_re & SIN_TBL_MASK);
	*d = ( cos_theta_re * a + sin_theta_re * b) >> 14;
	*q = (-sin_theta_re * a + cos_theta_re * b) >> 14;
}

inline void dq2ab(int16_t *a, int16_t *b, uint16_t theta_re, int16_t d, int16_t q)
{
	int32_t cos_theta_re = COS_Q14(theta_re & SIN_TBL_MASK);
	int32_t sin_theta_re = SIN_Q14(theta_re & SIN_TBL_MASK);
	*a = (cos_theta_re * d - sin_theta_re * q) >> 14;
	*b = (sin_theta_re * d + cos_theta_re * q) >> 14;
}

inline void circular_limit(int16_t *dst_x, int16_t *dst_y, int16_t limit, int16_t src_x, int16_t src_y)
{
	int32_t r2 = (int32_t)src_x * (int32_t)src_x + (int32_t)src_y * (int32_t)src_y;
	int32_t limit2 = (int32_t)limit * (int32_t)limit;

	if(r2 <= limit2) return;

	int32_t dec = (limit << 14) / intSqrt(r2);

	*dst_x = (src_x * dec) >> 14;
	*dst_y = (src_y * dec) >> 14;
}

inline int32_t limitter_int32(int32_t val, int32_t min, int32_t max)
{
	if(val < min)
	{
		return min;
	}
	if(val > max)
	{
		return max;
	}
	return val;
}


void CurrentControl(Motor_TypeDef *hMotor)
{

	if(hMotor->RunMode == MOTOR_MODE_CC_VECTOR)
	{
		// Current limit
		if(hMotor->Id_ref_pu_2q13 < -hMotor->Init.Id_limit_pu_2q13)
			hMotor->Id_ref_pu_2q13 = -hMotor->Init.Id_limit_pu_2q13;
		else if(hMotor->Id_ref_pu_2q13 > hMotor->Init.Id_limit_pu_2q13)
			hMotor->Id_ref_pu_2q13 = hMotor->Init.Id_limit_pu_2q13;

		if(hMotor->Iq_ref_pu_2q13 < -hMotor->Init.Iq_limit_pu_2q13)
			hMotor->Iq_ref_pu_2q13 = -hMotor->Init.Iq_limit_pu_2q13;
		else if(hMotor->Iq_ref_pu_2q13 > hMotor->Init.Iq_limit_pu_2q13)
			hMotor->Iq_ref_pu_2q13 = hMotor->Init.Iq_limit_pu_2q13;

		hMotor->Id_error = hMotor->Id_ref_pu_2q13 - hMotor->Id_pu_2q13;
		hMotor->Iq_error = hMotor->Iq_ref_pu_2q13 - hMotor->Iq_pu_2q13;
	}
	else if(hMotor->RunMode == MOTOR_MODE_CC_FORCE)
	{
		// Current limit
		if(hMotor->Igam_ref_pu_2q13 < -hMotor->Init.Id_limit_pu_2q13)
			hMotor->Igam_ref_pu_2q13 = -hMotor->Init.Id_limit_pu_2q13;
		else if(hMotor->Igam_ref_pu_2q13 > hMotor->Init.Id_limit_pu_2q13)
			hMotor->Igam_ref_pu_2q13 = hMotor->Init.Id_limit_pu_2q13;

		if(hMotor->Idel_ref_pu_2q13 < -hMotor->Init.Iq_limit_pu_2q13)
			hMotor->Idel_ref_pu_2q13 = -hMotor->Init.Iq_limit_pu_2q13;
		else if(hMotor->Idel_ref_pu_2q13 > hMotor->Init.Iq_limit_pu_2q13)
			hMotor->Idel_ref_pu_2q13 = hMotor->Init.Iq_limit_pu_2q13;

		hMotor->Id_error = hMotor->Igam_ref_pu_2q13 - hMotor->Id_pu_2q13;
		hMotor->Iq_error = hMotor->Idel_ref_pu_2q13 - hMotor->Iq_pu_2q13;
	}

	int32_t Vd_limErrFB = hMotor->Vd_limit_error * hMotor->Init.acr_limErrFB_gain_q10 >> 10;
	int32_t Vq_limErrFB = hMotor->Vq_limit_error * hMotor->Init.acr_limErrFB_gain_q10 >> 10;

	IntInteg_Update(&hMotor->Id_error_integ, hMotor->Id_error - Vd_limErrFB);
	IntInteg_Update(&hMotor->Iq_error_integ, hMotor->Iq_error - Vq_limErrFB);

	hMotor->Vd_pu_2q13 = ((hMotor->Init.acr_Kp_q14 * hMotor->Id_error >> 14) + (hMotor->Init.acr_Ki_q2 * hMotor->Id_error_integ.integ >> 14))
			* hMotor->Init.Gain_Ib_by_Vb_q10 >> 10;
	hMotor->Vq_pu_2q13 = ((hMotor->Init.acr_Kp_q14 * hMotor->Iq_error >> 14) + (hMotor->Init.acr_Ki_q2 * hMotor->Iq_error_integ.integ >> 14))
			* hMotor->Init.Gain_Ib_by_Vb_q10 >> 10;
}

void Midi_Init(Motor_TypeDef *hMotor)
{
	for(int n = 0; n < 128; n++)
	{
		float f = 440.0f * powf(2.0, (n - 69) / 12.0f);
		hMotor->MIDI_periodTable_us[n] = (uint32_t)(1000000 / f + 0.5);
	}
	hMotor->MIDI_cycle_us = 25;
	hMotor->MIDI_count_us = 0;
	hMotor->MIDI_notenum = 69;
	hMotor->MIDI_vel = 0;
}

void Midi_Update(Motor_TypeDef *hMotor)
{
	uint32_t period_us = hMotor->MIDI_periodTable_us[hMotor->MIDI_notenum];

	// Counter update
	hMotor->MIDI_count_us += hMotor->MIDI_cycle_us;
	if(hMotor->MIDI_count_us >= period_us)
	{
		//hMotor->MIDI_count_us -= period_us; // 周波数に忠実
		hMotor->MIDI_count_us = 0; // 音質が良い
	}

	// Generate square wave
	if(hMotor->MIDI_count_us * 2 > period_us)
	{
		hMotor->Vd_pu_2q13 = 0;
		hMotor->Vq_pu_2q13 = -1 * hMotor->MIDI_vel * MIDI_GAIN_Q5;
	}
	else
	{
		hMotor->Vd_pu_2q13 = 0;
		hMotor->Vq_pu_2q13 = hMotor->MIDI_vel * MIDI_GAIN_Q5;
	}
}


void Limitter_Vdq(Motor_TypeDef *hMotor)
{
	int16_t Vd_bef_lim = hMotor->Vd_pu_2q13;
	int16_t Vq_bef_lim = hMotor->Vq_pu_2q13;

	// Circular voltage limit
	//	// Vdq_lim = Vdc / 2 * sqrt(3/2) * DutyLimitRate
	//	const int16_t Vdq_lim_pu_2q13 = ((int32_t)hMotor->Vdc_pu_2q13 * 5017 * hMotor->Init.DutyRateLimit_q5) >> (13 + 5);
	// Vdq_lim = Vdc / sqrt(2) * DutyLimitRate
	const int16_t Vdq_lim_pu_2q13 = ((int32_t)hMotor->Vdc_pu_2q13 * 5793 * hMotor->Init.DutyRateLimit_q5) >> (13 + 5);
	circular_limit(&hMotor->Vd_pu_2q13, &hMotor->Vq_pu_2q13, Vdq_lim_pu_2q13, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);

	hMotor->Vd_limit_error = Vd_bef_lim - hMotor->Vd_pu_2q13;
	hMotor->Vq_limit_error = Vq_bef_lim - hMotor->Vq_pu_2q13;
}


void UpdateSpeed(Motor_TypeDef *hMotor)
{

	int32_t delta_theta_int;

	delta_theta_int = hMotor->theta_m_int - hMotor->p_theta_int_buf[hMotor->p_theta_buf_count];


	if(delta_theta_int < -4096) delta_theta_int += 8192;
	else if(delta_theta_int > 4096) delta_theta_int -= 8192;

	hMotor->p_theta_int_buf[hMotor->p_theta_buf_count] = hMotor->theta_m_int;
	hMotor->p_theta_buf_count ++;
	if(hMotor->p_theta_buf_count > SPEED_CALC_BUF_SIZE - 1)
	{
		hMotor->p_theta_buf_count = 0;
	}

	/*
	hEncoder->prev_theta_buf[hEncoder->prev_theta_buf_count] = hEncoder->theta;
	hEncoder->prev_theta_buf_count ++;
	if(hEncoder->prev_theta_buf_count > SPEED_CALC_BUF_SIZE - 1)
	{
		hEncoder->prev_theta_buf_count = 0;
	}
	*/

	// ( (delta_theta_int / 8192 * 2 * M_PI)[rad] * 10000[Hz] )[rad/s] * 32
	hMotor->omega_q5 = (int16_t)((delta_theta_int * hMotor->Init.Gain_dOmegaInt_to_omegaQ5_q8) >> 8);
	//hMotor->omega_q5 = (int16_t)hMotor->p_theta_m_int;


}



void Motor_Init(Motor_TypeDef *hMotor, uint16_t pwm_period)
{


	hMotor->RunMode = MOTOR_MODE_CV_FORCE;


	/***** Motor parameters *****/

	hMotor->motorParam.Pn = POLE_PAIRS;
	hMotor->motorParam.R = MOTOR_R;
	hMotor->motorParam.Ld = MOTOR_Ld;
	hMotor->motorParam.Lq = MOTOR_Lq;

	/***** Data convention setting *****/

	hMotor->Init.I_base = CURRENT_RATING;
	hMotor->Init.V_base = 24.0;

	hMotor->Init.AD_Iu_offset = 2084;
	hMotor->Init.AD_Iv_offset = 2067;
	hMotor->Init.AD_Iw_offset = 2077;

	hMotor->Init.PWM_PRR = pwm_period;

	hMotor->Init.DutyRateLimit_q5 = 32; //  = 0.9 * 32

	//hMotor->Init.Gain_Iad2pu_s14 = 16384/*shift:14bit*/ / 4096.0/*ADC Range*/ * 3.3/*V_ref*/ * -10.0/*[A/V]*/ / hMotor->Init.I_base * 8192/*q.13*/;
	hMotor->Init.Gain_Iad2pu_s14 = 16384/*shift:14bit*/ / 4096.0/*ADC Range*/ * 3.3/*V_ref*/ * 10.0/*[A/V]*/ / hMotor->Init.I_base * 8192/*q.13*/;

	hMotor->Init.Gain_Vad2pu_s14 = 16384/*shift:14bit*/ / 4096.0/*ADC Range*/ * 3.3/*V_ref*/ * 12.5385f/*[V/V]*/ / hMotor->Init.V_base * 8192/*q.13*/;

	hMotor->Init.Gain_Ib_by_Vb_q10 = hMotor->Init.I_base / hMotor->Init.V_base * 1024;



	hMotor->Init.Gain_dOmegaInt_to_omegaQ5_q8 = (62831.85 + 0.5) / SPEED_CALC_BUF_SIZE;



	/***** ACR Setting *****/

	hMotor->Init.acr_omega = 2000;

	float acr_Kp = hMotor->Init.acr_omega * hMotor->motorParam.Lq;
	float acr_Ki = hMotor->Init.acr_omega * hMotor->motorParam.R;

	hMotor->Init.acr_Kp_q14 = acr_Kp * 16384;
	hMotor->Init.acr_Ki_q2 = acr_Ki * 4;

	IntInteg_Init(&hMotor->Id_error_integ, 12, 268435456 / 20000, 32768);
	IntInteg_Init(&hMotor->Iq_error_integ, 12, 268435456 / 20000, 32768);

	hMotor->Init.acr_limErrFB_gain_q10 = hMotor->Init.V_base / hMotor->Init.I_base / acr_Kp * 1024;

	hMotor->Init.Id_limit_pu_2q13 = 8192;
	hMotor->Init.Iq_limit_pu_2q13 = 8192;

	hMotor->Init.theta_int_offset = 0;

	/***** State Initialization *****/

	hMotor->Id_error = 0;
	hMotor->Iq_error = 0;
	hMotor->Id_ref_pu_2q13 = 0;
	hMotor->Iq_ref_pu_2q13 = 0;

	hMotor->Vd_pu_2q13 = 0;
	hMotor->Vq_pu_2q13 = 0;

	hMotor->theta_force_int = 0;
	hMotor->Igam_ref_pu_2q13 = 0;
	hMotor->Idel_ref_pu_2q13 = 0;

	Midi_Init(hMotor);

	hMotor->Vd_limit_error = 0;
	hMotor->Vq_limit_error = 0;

	hMotor->sector = 0;

	memset(hMotor->p_theta_int_buf, 0x00, sizeof(hMotor->p_theta_int_buf));
	hMotor->p_theta_buf_count = 0;
	hMotor->omega_q5 = 0;

	hMotor->duty_u = hMotor->Init.PWM_PRR / 2;
	hMotor->duty_v = hMotor->Init.PWM_PRR / 2;
	hMotor->duty_w = hMotor->Init.PWM_PRR / 2;

}


void Motor_ADCUpdate(Motor_TypeDef *hMotor)
{

	hMotor->theta_m_int = (hMotor->raw_theta_14bit >> 1) & SIN_TBL_MASK;
	hMotor->theta_re_int = ( ( ((uint32_t)hMotor->raw_theta_14bit * hMotor->motorParam.Pn) >> 1 ) - hMotor->Init.theta_int_offset) & SIN_TBL_MASK;

	UpdateSpeed(hMotor);

	hMotor->Iu_pu_2q13 = ( ((int32_t)hMotor->AD_Iu - (int32_t)hMotor->Init.AD_Iu_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;
	hMotor->Iv_pu_2q13 = ( ((int32_t)hMotor->AD_Iv - (int32_t)hMotor->Init.AD_Iv_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;
	hMotor->Iw_pu_2q13 = ( ((int32_t)hMotor->AD_Iw - (int32_t)hMotor->Init.AD_Iw_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;
	hMotor->Vdc_pu_2q13 = ( (int32_t)hMotor->AD_Vdc * (int32_t)hMotor->Init.Gain_Vad2pu_s14 ) >> 14;

	// Current source selection
	if(hMotor->duty_u > hMotor->duty_v && hMotor->duty_u > hMotor->duty_w)
	{
		hMotor->Iu_pu_2q13 = - hMotor->Iv_pu_2q13 - hMotor->Iw_pu_2q13; // Sector: 6,1
	}
	else if(hMotor->duty_v > hMotor->duty_u && hMotor->duty_v > hMotor->duty_w)
	{
		hMotor->Iv_pu_2q13 = - hMotor->Iw_pu_2q13 - hMotor->Iu_pu_2q13; // Sector: 2,3
	}
	else
	{
		hMotor->Iw_pu_2q13 = - hMotor->Iu_pu_2q13 - hMotor->Iv_pu_2q13; // Sector: 4,5
	}

	// UVW => ab
	uvw2ab(&hMotor->Ia_pu_2q13, &hMotor->Ib_pu_2q13, hMotor->Iu_pu_2q13, hMotor->Iv_pu_2q13, hMotor->Iw_pu_2q13);

	// MIDI出力モードの時はキャリア同期タイミングでduty更新をする
	if(hMotor->RunMode == MOTOR_MODE_CV_MIDI)
	{
		return;
	}

	switch(hMotor->RunMode)
	{
	case MOTOR_MODE_CV_FORCE:
		ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_force_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);
		Limitter_Vdq(hMotor);
		dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_force_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
		break;
	case MOTOR_MODE_CV_VECTOR:
		ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_re_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);
		Limitter_Vdq(hMotor);
		dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_re_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
		break;
	case MOTOR_MODE_CC_FORCE:
		ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_force_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);
		CurrentControl(hMotor);
		Limitter_Vdq(hMotor);
		dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_force_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
		break;
	case MOTOR_MODE_CC_VECTOR:
		ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_re_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);
		CurrentControl(hMotor);
		Limitter_Vdq(hMotor);
		dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_re_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
		break;
	}

	ab2uvw(&hMotor->Vu_pu_2q13, &hMotor->Vv_pu_2q13, &hMotor->Vw_pu_2q13, hMotor->Va_pu_2q13, hMotor->Vb_pu_2q13);

	int32_t Gain_Vref2duty_s14 = ((int32_t)hMotor->Init.PWM_PRR << 14) / hMotor->Vdc_pu_2q13;
	int32_t duty_u = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vu_pu_2q13 * Gain_Vref2duty_s14) >> 14);
	int32_t duty_v = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vv_pu_2q13 * Gain_Vref2duty_s14) >> 14);
	int32_t duty_w = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vw_pu_2q13 * Gain_Vref2duty_s14) >> 14);

	PWM_InjectCommonMode_AwayFromSwitching_MinMaxInLow(&duty_u, &duty_v, &duty_w, hMotor->Init.PWM_PRR);

	// Duty Limit
	hMotor->duty_u = limitter_int32(duty_u, 0, hMotor->Init.PWM_PRR);
	hMotor->duty_v = limitter_int32(duty_v, 0, hMotor->Init.PWM_PRR);
	hMotor->duty_w = limitter_int32(duty_w, 0, hMotor->Init.PWM_PRR);
}

void Motor_PWMUpdate(Motor_TypeDef *hMotor)
{

	if(hMotor->RunMode == MOTOR_MODE_CV_MIDI)
	{
		ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_re_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);

		Midi_Update(hMotor);

		Limitter_Vdq(hMotor);

		dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_re_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
		ab2uvw(&hMotor->Vu_pu_2q13, &hMotor->Vv_pu_2q13, &hMotor->Vw_pu_2q13, hMotor->Va_pu_2q13, hMotor->Vb_pu_2q13);

		int32_t Gain_Vref2duty_s14 = ((int32_t)hMotor->Init.PWM_PRR << 14) / hMotor->Vdc_pu_2q13;
		int32_t duty_u = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vu_pu_2q13 * Gain_Vref2duty_s14) >> 14);
		int32_t duty_v = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vv_pu_2q13 * Gain_Vref2duty_s14) >> 14);
		int32_t duty_w = (hMotor->Init.PWM_PRR >> 1) + (((int32_t)hMotor->Vw_pu_2q13 * Gain_Vref2duty_s14) >> 14);

		PWM_InjectCommonMode_AwayFromSwitching_MinMaxInLow(&duty_u, &duty_v, &duty_w, hMotor->Init.PWM_PRR);

		// Duty Limit
		hMotor->duty_u = limitter_int32(duty_u, 0, hMotor->Init.PWM_PRR);
		hMotor->duty_v = limitter_int32(duty_v, 0, hMotor->Init.PWM_PRR);
		hMotor->duty_w = limitter_int32(duty_w, 0, hMotor->Init.PWM_PRR);
	}

}


void Motor_Reset(Motor_TypeDef *hMotor)
{

	hMotor->Id_error = 0;
	hMotor->Iq_error = 0;
	hMotor->Id_ref_pu_2q13 = 0;
	hMotor->Iq_ref_pu_2q13 = 0;

	hMotor->Igam_ref_pu_2q13 = 0;
	hMotor->Idel_ref_pu_2q13 = 0;
	hMotor->theta_force_int = 0;

	hMotor->Vd_limit_error = 0;
	hMotor->Vq_limit_error = 0;

	hMotor->sector = 0;

	hMotor->duty_u = hMotor->Init.PWM_PRR / 2;
	hMotor->duty_v = hMotor->Init.PWM_PRR / 2;
	hMotor->duty_w = hMotor->Init.PWM_PRR / 2;

	IntInteg_Reset(&hMotor->Id_error_integ);
	IntInteg_Reset(&hMotor->Iq_error_integ);

}




