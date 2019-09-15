

#include "ASR.h"

#include <math.h>
#include "parameters.h"

#include "spi.h"
#include "ACR.h"





float Kp_ASR = 0.3;
float Ki_ASR = 20.0;



const float ASR_cycleTime = 1E-3;

float omega_limit = 1000000.0;

volatile float omega_ref = 0.0f;


volatile float omega_error = 0.0f;

volatile float omega_error_integ = 0.0f;

volatile float torque_ref = 0.0;



int ASR_steps = 0;


int ASR_flg = 0;
int ASR_prescalerCount = 0;

volatile float omega = 0.0f;


volatile float p_theta = 0.0f;


float d_theta;

float _omega_ref;

float omega_error_integ_temp1 = 0.0f;
float omega_error_integ_temp2 = 0.0f;





inline void speedControl()
{



	  if(ASR_steps <= 0)
	  {
		  d_theta = 0.0f;
	  }
	  else
	  {
		  d_theta = theta - p_theta;
	  }
	  ASR_steps += 1;

	  p_theta = theta;

	  if(d_theta < - M_PI)		d_theta += 2 * M_PI;
	  else if(d_theta > M_PI)	d_theta -= 2 * M_PI;

	  omega = omega * 0.5 + 0.5 * d_theta / ASR_cycleTime;


#if _ASR_ENABLE_


	  if(omega_ref < -omega_limit)		_omega_ref = -omega_limit;
	  else if(omega_ref > omega_limit)	_omega_ref = omega_limit;
	  else								_omega_ref = omega_ref;

	  omega_error = _omega_ref - omega;

	  // integral
	  omega_error_integ_temp1 = omega_error + omega_error_integ_temp2;
	  if(omega_error_integ_temp1 < -6.0 / ASR_cycleTime)
	  {
		  omega_error_integ_temp1 = -6.0 / ASR_cycleTime;
	  }
	  else if(omega_error_integ_temp1 > 6.0 / ASR_cycleTime)
	  {
		  omega_error_integ_temp1 = 6.0 / ASR_cycleTime;
	  }
	  omega_error_integ = ASR_cycleTime * 0.5f * (omega_error_integ_temp1 + omega_error_integ_temp2);
	  omega_error_integ_temp2 = omega_error_integ_temp1;


	  torque_ref = Kp_ASR * omega_error + Ki_ASR * omega_error_integ;

	  Id_ref = 0.0f;
	  Iq_ref = KT * torque_ref;


#endif




	return;
}



inline void ASR_reset()
{

	p_theta = 0.0f;

	omega_error_integ_temp1 = 0.0f;
	omega_error_integ_temp2 = 0.0f;

	omega = omega_ref = 0.0f;

	ASR_steps = 0;

}





