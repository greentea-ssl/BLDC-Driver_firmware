

#include <math.h>

#include "ACR.h"
#include "main.h"

#include "ASR.h"

#include "adc.h"
#include "spi.h"
#include "modulator.h"





float Kp_ACR = 0.1;
float Ki_ACR = 400.0;

const float ACR_cycleTime = 100E-6;



float Id_limit = 15.0f;
float Iq_limit = 15.0f;


volatile float Id_ref = 0.0f;
volatile float Iq_ref = 0.0f;


volatile float Id = 0.0;
volatile float Iq = 0.0;


volatile float Id_error = 0.0f;
volatile float Iq_error = 0.0f;

volatile float Id_error_integ = 0.0f;
volatile float Iq_error_integ = 0.0f;



volatile float _Id_ref;
volatile float _Iq_ref;


volatile float Id_error_integ_temp1 = 0.0f;
volatile float Id_error_integ_temp2 = 0.0f;
volatile float Iq_error_integ_temp1 = 0.0f;
volatile float Iq_error_integ_temp2 = 0.0f;




inline void currentControl(void)
{



	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_SET);

	refreshEncoder();


#if 0
	// calculate sin(theta_re), cos(theta_re)
	if(forced_commute_state > 0)
	{

		_forced_theta_re = fmodf(forced_theta * POLES / 2, 2.0f * M_PI);

		if(_forced_theta_re < 0.0f)				forced_theta_re = _forced_theta_re + 2 * M_PI;
		else if(_forced_theta_re >= 2 * M_PI)	forced_theta_re = _forced_theta_re - 2 * M_PI;
		else									forced_theta_re = _forced_theta_re;

		cos_theta_re = sin_table2[(int)((forced_theta_re * 0.3183f + 0.5f) * 5000.0f)];
		sin_theta_re = sin_table2[(int)(forced_theta_re * 1591.54943f)];
	}
#endif


	get_current_dq(&Id, &Iq, sector_SVM, cos_theta_re, sin_theta_re);


	if(theta_re < M_PI)
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);


	/********** ACR (Auto Current Regulator) **********/

#if _ACR_ENABLE_

	if(Id_ref < -Id_limit)		_Id_ref = -Id_limit;
	else if(Id_ref > Id_limit)	_Id_ref = Id_limit;
	else						_Id_ref = Id_ref;

	if(Iq_ref < -Iq_limit)		_Iq_ref = -Iq_limit;
	else if(Iq_ref > Iq_limit)	_Iq_ref = Iq_limit;
	else						_Iq_ref = Iq_ref;


#if 0
	if(forced_commute_state > 0)
	{
		Id_error = forced_I_gamma_ref - Id;
		Iq_error = forced_I_delta_ref - Iq;
	}
	else
	{
#endif
		Id_error = _Id_ref - Id;
		Iq_error = _Iq_ref - Iq;
#if 0
	}
#endif


	// integral
	Id_error_integ_temp1 = Id_error + Id_error_integ_temp2;
	if(Id_error_integ_temp1 < -1000000.0) Id_error_integ_temp1 = -1000000.0;
	else if(Id_error_integ_temp1 > 1000000.0) Id_error_integ_temp1 = 1000000.0;
	Id_error_integ = ACR_cycleTime * 0.5f * (Id_error_integ_temp1 + Id_error_integ_temp2);
	Id_error_integ_temp2 = Id_error_integ_temp1;

	Iq_error_integ_temp1 = Iq_error + Iq_error_integ_temp2;
	if(Iq_error_integ_temp1 < -1000000.0) Iq_error_integ_temp1 = -1000000.0;
	else if(Iq_error_integ_temp1 > 1000000.0) Iq_error_integ_temp1 = 1000000.0;
	Iq_error_integ = ACR_cycleTime * 0.5f * (Iq_error_integ_temp1 + Iq_error_integ_temp2);
	Iq_error_integ_temp2 = Iq_error_integ_temp1;


	Vd_ref = Kp_ACR * Id_error + Ki_ACR * Id_error_integ;
	Vq_ref = Kp_ACR * Iq_error + Ki_ACR * Iq_error_integ;

#endif

	/********* end of ACR **********/


	if(HAL_GPIO_ReadPin(BR_FLT_GPIO_Port, BR_FLT_Pin) == GPIO_PIN_RESET)
	{
		HAL_NVIC_SystemReset();
	}


	setSVM_dq();


#if _ACR_DUMP_

	if(ACR_dump_count < ACR_DUMP_STEPS)
	{
		Id_dump[ACR_dump_count] = Id;
		Iq_dump[ACR_dump_count] = Iq;
		Id_ref_dump[ACR_dump_count] = Id_ref;
		Iq_ref_dump[ACR_dump_count] = Iq_ref;
		Vd_ref_dump[ACR_dump_count] = Vd_ref;
		Vq_ref_dump[ACR_dump_count] = Vq_ref;
		ACR_dump_count += 1;
	}

#endif



	requestEncoder();


	// Auto Speed Regulator launching
	ASR_prescalerCount += 1;
	if(ASR_prescalerCount >= ASR_prescale)
	{
		ASR_flg = 1;
		ASR_prescalerCount = 0;
	}



	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

	return;
}



inline void ACR_reset()
{

	Id_error_integ_temp1 = 0.0f;
	Id_error_integ_temp2 = 0.0f;
	Iq_error_integ_temp1 = 0.0f;
	Iq_error_integ_temp2 = 0.0f;


	Id = Id_ref = 0.0f;
	Iq = Iq_ref = 0.0f;

	Vd_ref = 0.0f;
	Vq_ref = 0.0f;

}











