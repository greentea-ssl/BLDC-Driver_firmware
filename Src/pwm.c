

#include <pwm.h>
#include "parameters.h"



TIM_HandleTypeDef htim8;



volatile float amp_u = 0.0;
volatile float amp_v = 0.0;
volatile float amp_w = 0.0;



// reference vectors for SVM
const float refVector[6+1][2] = {
	{ 1.000,  0.000},
	{ 0.500,  0.866},
	{-0.500,  0.866},
	{-1.000,  0.000},
	{-0.500, -0.866},
	{ 0.500, -0.866},
	{ 1.000,  0.000},
};



volatile float Vd_ref = 0.0f;
volatile float Vq_ref = 0.0f;


volatile int sector_SVM = 0;


void PWM_Init()
{

	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);


	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);
	//HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_TRIGGER);

	htim8.Instance->CCR4 = 7900;

	startPWM(&htim8);

}


inline void startPWM(TIM_HandleTypeDef *htim)
{


	// 3phase PWM Starting
	HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start_IT(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(htim, TIM_CHANNEL_3);

	// Gate Enable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);

}



inline void stopPWM(TIM_HandleTypeDef *htim)
{

	// Gate Disable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

/*
	// 3phase PWM Stopping
	HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Stop_IT(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop_IT(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop_IT(htim, TIM_CHANNEL_3);
*/

}


inline void setSVM_dq(TIM_HandleTypeDef *htim, float Vd_ref, float Vq_ref, float cos_theta_re, float sin_theta_re)
{

	static float cross0 = 0.0;
	static float cross1 = 0.0;
	static float duty[3] = {0};
	static float x1, y1, x2, y2;
	static float x, y;
	static float vect1, vect2;


	x = Vd_ref * cos_theta_re - Vq_ref * sin_theta_re;
	y = Vd_ref * sin_theta_re + Vq_ref * cos_theta_re;

	cross0 = refVector[0][0] * y - refVector[0][1] * x;
	cross1 = refVector[1][0] * y - refVector[1][1] * x;

	if(cross0 >= 0)
	{
		if(cross1 <= 0)				sector_SVM = 0;
		else if(cross0 >= cross1)	sector_SVM = 1;
		else						sector_SVM = 2;
	}
	else
	{
		if(cross1 >= 0)				sector_SVM = 3;
		else if(cross0 <= cross1)	sector_SVM = 4;
		else						sector_SVM = 5;
	}

	x1 = refVector[sector_SVM][0];
	y1 = refVector[sector_SVM][1];
	x2 = refVector[sector_SVM + 1][0];
	y2 = refVector[sector_SVM + 1][1];

	vect1 = (y2 * x - x2 * y) / ((x1 * y2 - y1 * x2) * VDC);
	vect2 = (-y1 * x + x1 * y) / ((x1 * y2 - y1 * x2) * VDC);

	switch(sector_SVM)
	{
	case 0: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[2] + vect2; 	duty[0] = duty[1] + vect1;  break;
	case 1: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[2] + vect1; 	duty[1] = duty[0] + vect2; 	break;
	case 2: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[0] + vect2; 	duty[1] = duty[2] + vect1; 	break;
	case 3: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[0] + vect1; 	duty[2] = duty[1] + vect2; 	break;
	case 4: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[1] + vect2; 	duty[2] = duty[0] + vect1; 	break;
	case 5: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[1] + vect1; 	duty[0] = duty[2] + vect2; 	break;
	}


	if(duty[0] < -1.0f) duty[0] = -1.0f; else if (duty[0] > 1.0f) duty[0] = 1.0f;
	if(duty[1] < -1.0f) duty[1] = -1.0f; else if (duty[1] > 1.0f) duty[1] = 1.0f;
	if(duty[2] < -1.0f) duty[2] = -1.0f; else if (duty[2] > 1.0f) duty[2] = 1.0f;

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, htim->Init.Period * (1.0f - (amp_u = duty[0])));
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, htim->Init.Period * (1.0f - (amp_v = duty[1])));
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, htim->Init.Period * (1.0f - (amp_w = duty[2])));


	return;
}


#if 0
inline static void setSVM(float ampl, float phase){
	//float rate[3] = {0};
	//float _duty[3] = {0};
	static float duty[3] = {0};
	static float x1, y1, x2, y2;
	static float x, y;
	static float vect1, vect2;

	// reference vectors for SVM
	const float refVector[6+1][2] = {
			{ 1.000,  0.000},
			{ 0.500,  0.866},
			{-0.500,  0.866},
			{-1.000,  0.000},
			{-0.500, -0.866},
			{ 0.500, -0.866},
			{ 1.000,  0.000},
	};

	// Select two from 6 vector
	int sector = (int)(phase * 0.95493f);



	x = sin_table2[(int)((phase * 0.3183f + 0.5f) * 5000.0f)];
	y = sin_table2[(int)(phase * 1591.54943f)];


	x1 = refVector[sector][0];
	y1 = refVector[sector][1];
	x2 = refVector[sector + 1][0];
	y2 = refVector[sector + 1][1];


	vect1 = ampl / (x1 * y2 - y1 * x2) * (y2 * x - x2 * y);
	vect2 = ampl / (x1 * y2 - y1 * x2) * (-y1 * x + x1 * y);

	switch(sector)
	{
	case 0: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[2] + vect2; 	duty[0] = duty[1] + vect1;  break;
	case 1: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[2] + vect1; 	duty[1] = duty[0] + vect2; 	break;
	case 2: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[0] + vect2; 	duty[1] = duty[2] + vect1; 	break;
	case 3: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[0] + vect1; 	duty[2] = duty[1] + vect2; 	break;
	case 4: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[1] + vect2; 	duty[2] = duty[0] + vect1; 	break;
	case 5: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[1] + vect1; 	duty[0] = duty[2] + vect2; 	break;
	}


	if(duty[0] < -1.0f) duty[0] = -1.0f; else if (duty[0] > 1.0f) duty[0] = 1.0f;
	if(duty[1] < -1.0f) duty[1] = -1.0f; else if (duty[1] > 1.0f) duty[1] = 1.0f;
	if(duty[2] < -1.0f) duty[2] = -1.0f; else if (duty[2] > 1.0f) duty[2] = 1.0f;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200.0f * (1.0f - (amp_u = duty[0])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200.0f * (1.0f - (amp_v = duty[1])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200.0f * (1.0f - (amp_w = duty[2])));


/*
	x = sin_t(phase + M_PI * 0.5);
	y = sin_t(phase);

	float x1 = refVector[sector][0];
	float y1 = refVector[sector][1];
	float x2 = refVector[sector + 1][0];
	float y2 = refVector[sector + 1][1];

	float vect1 = ampl / (x1 * y2 - y1 * x2) * (y2 * x - x2 * y);
	float vect2 = ampl / (x1 * y2 - y1 * x2) * (-y1 * x + x1 * y);

	if(sector%2 != 0){
		rate[1] = vect2;
		rate[2] = vect1;
	}
	else{
		rate[1] = vect1;
		rate[2] = vect2;
	}

	rate[0] = 1.0 - rate[1] - rate[2];


	_duty[0]   = rate[0] * 0.5 + rate[1] + rate[2];
	_duty[1]   = rate[0] * 0.5 + rate[2];
	_duty[2]   = rate[0] / 2.0;

	switch(sector){
		case 0: duty[0] = _duty[0]; duty[1] = _duty[1]; duty[2] = _duty[2]; break;
		case 1: duty[0] = _duty[1]; duty[1] = _duty[0]; duty[2] = _duty[2]; break;
		case 2: duty[0] = _duty[2]; duty[1] = _duty[0]; duty[2] = _duty[1]; break;
		case 3: duty[0] = _duty[2]; duty[1] = _duty[1]; duty[2] = _duty[0]; break;
		case 4: duty[0] = _duty[1]; duty[1] = _duty[2]; duty[2] = _duty[0]; break;
		case 5: duty[0] = _duty[0]; duty[1] = _duty[2]; duty[2] = _duty[1]; break;
	}

	if(duty[0] < -1.0f) duty[0] = -1.0; else if (duty[0] > 1.0) duty[0] = 1.0;
	if(duty[1] < -1.0f) duty[1] = -1.0; else if (duty[1] > 1.0) duty[1] = 1.0;
	if(duty[2] < -1.0f) duty[2] = -1.0; else if (duty[2] > 1.0) duty[2] = 1.0;

	setPWM(duty);

*/

	return;

}
#endif








