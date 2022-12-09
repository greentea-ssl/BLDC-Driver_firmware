

#include "pwm.h"
#include "parameters.h"
#include "main.h"

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

	//__HAL_TIM_SET_AUTORELOAD(h->htim, pwm_period);
	// ARR set in generated code

	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, h->htim->Init.Period >> 1);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, h->htim->Init.Period >> 1);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, h->htim->Init.Period >> 1);
	__HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_4, h->htim->Init.Period - 1);

}


inline void PWM_Start(PWM_Handler_t* h)
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



inline void PWM_Stop(PWM_Handler_t* h)
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


inline void PWM_SetDuty(PWM_Handler_t* h)
{
#if NO_UPDATE_ON_BOTTOM
	if(!__HAL_TIM_IS_TIM_COUNTING_DOWN(h->htim))
	{
#endif
		h->htim->Instance->CCR1 = h->duty_u;
		h->htim->Instance->CCR2 = h->duty_v;
		h->htim->Instance->CCR3 = h->duty_w;
#if NO_UPDATE_ON_BOTTOM
	}
#endif
	return;
}


void PWM_UpdateDuty(PWM_Handler_t* h)
{
	h->htim->Instance->CCR1 = h->duty_u;
	h->htim->Instance->CCR2 = h->duty_v;
	h->htim->Instance->CCR3 = h->duty_w;
	return;
}



inline void PWM_InjectCommonMode_MinMax(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period)
{
	int32_t middle;
	if(*duty_u > *duty_v)
	{
		if(*duty_v > *duty_w) middle = *duty_v; // U > V > W
		else if(*duty_w > *duty_u) middle = *duty_u; // W > U > V
		else middle = *duty_w; // U > W > V
	}
	else
	{
		if(*duty_u > *duty_w) middle = *duty_u; // V > U > W
		else if(*duty_w > *duty_v) middle = *duty_v; // W > V > U
		else middle = *duty_w; // V > W > U
	}
	middle = (middle - (period >> 1)) >> 1;
	*duty_u += middle;
	*duty_v += middle;
	*duty_w += middle;
	return;
}


inline void PWM_InjectCommonMode_TwoPhaseUp(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period)
{
	int32_t min, max;
	if(*duty_u > *duty_v)
	{
		if(*duty_v > *duty_w) // U > V > W
		{
			max = *duty_u;
			min = *duty_w;
		}
		else if(*duty_w > *duty_u) // W > U > V
		{
			max = *duty_w;
			min = *duty_v;
		}
		else // U > W > V
		{
			max = *duty_u;
			min = *duty_v;
		}
	}
	else
	{
		if(*duty_u > *duty_w) // V > U > W
		{
			max = *duty_v;
			min = *duty_w;
		}
		else if(*duty_w > *duty_v) // W > V > U
		{
			max = *duty_w;
			min = *duty_u;
		}
		else // V > W > U
		{
			max = *duty_v;
			min = *duty_u;
		}
	}
	*duty_u += period - max;
	*duty_v += period - max;
	*duty_w += period - max;
	return;
}



void PWM_InjectCommonMode_TwoPhaseLow(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period)
{

	int32_t min, max;
	if(*duty_u > *duty_v)
	{
		if(*duty_v > *duty_w) // U > V > W
		{
			max = *duty_u;
			min = *duty_w;
		}
		else if(*duty_w > *duty_u) // W > U > V
		{
			max = *duty_w;
			min = *duty_v;
		}
		else // U > W > V
		{
			max = *duty_u;
			min = *duty_v;
		}
	}
	else
	{
		if(*duty_u > *duty_w) // V > U > W
		{
			max = *duty_v;
			min = *duty_w;
		}
		else if(*duty_w > *duty_v) // W > V > U
		{
			max = *duty_w;
			min = *duty_u;
		}
		else // V > W > U
		{
			max = *duty_v;
			min = *duty_u;
		}
	}
	*duty_u -= min;
	*duty_v -= min;
	*duty_w -= min;
	return;
}


void PWM_InjectCommonMode_AwayFromSwitching(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period)
{
	int32_t min, mid, max;
	if(*duty_u > *duty_v)
	{
		if(*duty_v > *duty_w) // U > V > W
		{
			max = *duty_u;
			mid = *duty_v;
			min = *duty_w;
		}
		else if(*duty_w > *duty_u) // W > U > V
		{
			max = *duty_w;
			mid = *duty_u;
			min = *duty_v;
		}
		else // U > W > V
		{
			max = *duty_u;
			mid = *duty_w;
			min = *duty_v;
		}
	}
	else
	{
		if(*duty_u > *duty_w) // V > U > W
		{
			max = *duty_v;
			mid = *duty_u;
			min = *duty_w;
		}
		else if(*duty_w > *duty_v) // W > V > U
		{
			max = *duty_w;
			mid = *duty_v;
			min = *duty_u;
		}
		else // V > W > U
		{
			max = *duty_v;
			mid = *duty_w;
			min = *duty_u;
		}
	}
	if(max - mid > period - (max - min))
	{
		*duty_u += period - max;
		*duty_v += period - max;
		*duty_w += period - max;
	}
	else
	{
		*duty_u -= min;
		*duty_v -= min;
		*duty_w -= min;
	}
	return;
}



void PWM_InjectCommonMode_AwayFromSwitching_MinMaxInLow(int32_t* duty_u, int32_t* duty_v, int32_t* duty_w, int32_t period)
{
	int32_t min, mid, max;
	uint16_t threthold = (period * 3) >> 2;
	if(*duty_u > *duty_v)
	{
		if(*duty_v > *duty_w) // U > V > W
		{
			max = *duty_u;
			mid = *duty_v;
			min = *duty_w;
		}
		else if(*duty_w > *duty_u) // W > U > V
		{
			max = *duty_w;
			mid = *duty_u;
			min = *duty_v;
		}
		else // U > W > V
		{
			max = *duty_u;
			mid = *duty_w;
			min = *duty_v;
		}
	}
	else
	{
		if(*duty_u > *duty_w) // V > U > W
		{
			max = *duty_v;
			mid = *duty_u;
			min = *duty_w;
		}
		else if(*duty_w > *duty_v) // W > V > U
		{
			max = *duty_w;
			mid = *duty_v;
			min = *duty_u;
		}
		else // V > W > U
		{
			max = *duty_v;
			mid = *duty_w;
			min = *duty_u;
		}
	}
	if(max - min < threthold)
	{
		int32_t Vcom = (mid - (period >> 1)) >> 1;
		*duty_u += Vcom;
		*duty_v += Vcom;
		*duty_w += Vcom;
	}
	else if(max - mid > period - (max - min))
	{
		*duty_u += period - max;
		*duty_v += period - max;
		*duty_w += period - max;
	}
	else
	{
		*duty_u -= min;
		*duty_v -= min;
		*duty_w -= min;
	}
	return;
}


