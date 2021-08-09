


#include "motorControl.h"

#include "sin_t.h"

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


/**********/


void Motor_Init(Motor_TypeDef *hMotor)
{

	hMotor->Init.I_base = 15.0;
	hMotor->Init.V_base = 24.0;

	hMotor->Init.AD_Iu_offset = 2048;
	hMotor->Init.AD_Iv_offset = 2048;
	hMotor->Init.AD_Iw_offset = 2048;

	hMotor->Init.Gain_Iad2pu_s14 = 4096/*shift:14bit*/ / 4096/*ADC Range*/ * 3.3/*V_ref*/ * -10.0/*[A/V]*/ / hMotor->Init.I_base * 8192/*q.13*/;

}


void Motor_Update(Motor_TypeDef *hMotor)
{

	hMotor->theta_m_int = (hMotor->raw_theta_14bit >> 1) & SIN_TBL_MASK;
	hMotor->theta_re_int = ( ( ((uint32_t)hMotor->raw_theta_14bit * 7) >> 1 ) - 1480) & SIN_TBL_MASK;

	hMotor->Iu_pu_2q13 = ( (hMotor->AD_Iu - hMotor->Init.AD_Iu_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;
	hMotor->Iv_pu_2q13 = ( (hMotor->AD_Iv - hMotor->Init.AD_Iv_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;
	hMotor->Iw_pu_2q13 = ( (hMotor->AD_Iw - hMotor->Init.AD_Iw_offset) * hMotor->Init.Gain_Iad2pu_s14 ) >> 14;

	//uvw2ab(&hMotor->Ia_pu_2q13, &hMotor->Ib_pu_2q13, hMotor->Iu_pu_2q13, hMotor->Iv_pu_2q13, hMotor->Iw_pu_2q13);
	//ab2dq(&hMotor->Id_pu_2q13, &hMotor->Iq_pu_2q13, hMotor->theta_re_int, hMotor->Ia_pu_2q13, hMotor->Ib_pu_2q13);


	hMotor->Vd_pu_2q13 = 0;
	hMotor->Vq_pu_2q13 = 500;

	dq2ab(&hMotor->Va_pu_2q13, &hMotor->Vb_pu_2q13, hMotor->theta_re_int, hMotor->Vd_pu_2q13, hMotor->Vq_pu_2q13);
	ab2uvw(&hMotor->Vu_pu_2q13, &hMotor->Vv_pu_2q13, &hMotor->Vw_pu_2q13, hMotor->Va_pu_2q13, hMotor->Vb_pu_2q13);

	hMotor->duty_u = 1000 - (((int32_t)hMotor->Vu_pu_2q13 * 3000) >> 14);
	hMotor->duty_v = 1000 - (((int32_t)hMotor->Vv_pu_2q13 * 3000) >> 14);
	hMotor->duty_w = 1000 - (((int32_t)hMotor->Vw_pu_2q13 * 3000) >> 14);

}



/***** Private functions *****/




