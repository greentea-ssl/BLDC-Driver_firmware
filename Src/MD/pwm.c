

#include "pwm.h"
#include "parameters.h"


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


void PWM_Init(PWM_Handler_t* h)
{

	__HAL_TIM_CLEAR_FLAG(h->htim, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(h->htim, TIM_IT_UPDATE);


	HAL_TIM_GenerateEvent(h->htim, TIM_EVENTSOURCE_UPDATE);
	//HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_TRIGGER);


	  // Timer Init
	__HAL_TIM_SET_AUTORELOAD(h->htim, 8000);

	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, 4000);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, 4000);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, 4000);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_4, h->htim->Init.Period - 1);

	startPWM(h);

}


inline void startPWM(PWM_Handler_t* h)
{

	// 3phase PWM Starting
	HAL_TIM_PWM_Start_IT(h->htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(h->htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(h->htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(h->htim, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start_IT(h->htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(h->htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(h->htim, TIM_CHANNEL_3);

	// Gate Enable
	HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);

}



inline void stopPWM(PWM_Handler_t* h)
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


inline void setSVM_dq(PWM_Handler_t* h, float Vd_ref, float Vq_ref, float Vdc, float cos_theta_re, float sin_theta_re)
{

	static float cross0 = 0.0;
	static float cross1 = 0.0;
	static float duty[3] = {0};
	static float x1, y1, x2, y2;
	static float x, y;
	static float vect1, vect2;
	static int sector_SVM;

	h->Vd_ref = Vd_ref;
	h->Vq_ref = Vq_ref;

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

	vect1 = (y2 * x - x2 * y) / ((x1 * y2 - y1 * x2) * Vdc);
	vect2 = (-y1 * x + x1 * y) / ((x1 * y2 - y1 * x2) * Vdc);

	switch(sector_SVM)
	{
	case 0: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[2] + vect2; 	duty[0] = duty[1] + vect1;  break;
	case 1: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[2] + vect1; 	duty[1] = duty[0] + vect2; 	break;
	case 2: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[0] + vect2; 	duty[1] = duty[2] + vect1; 	break;
	case 3: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[0] + vect1; 	duty[2] = duty[1] + vect2; 	break;
	case 4: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[1] + vect2; 	duty[2] = duty[0] + vect1; 	break;
	case 5: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[1] + vect1; 	duty[0] = duty[2] + vect2; 	break;
	}

	if(duty[0] < 0.0f) duty[0] = 0.0f; else if (duty[0] > 1.0f) duty[0] = 1.0f;
	if(duty[1] < 0.0f) duty[1] = 0.0f; else if (duty[1] > 1.0f) duty[1] = 1.0f;
	if(duty[2] < 0.0f) duty[2] = 0.0f; else if (duty[2] > 1.0f) duty[2] = 1.0f;

	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, h->htim->Init.Period * (1.0f - duty[0]));
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, h->htim->Init.Period * (1.0f - duty[1]));
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, h->htim->Init.Period * (1.0f - duty[2]));

	h->sector_SVM = sector_SVM;
	h->amp_u = duty[0];
	h->amp_v = duty[1];
	h->amp_w = duty[2];

	return;
}








