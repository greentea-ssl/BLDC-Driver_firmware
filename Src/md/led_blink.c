
#include <md/led_blink.h>
#include "main.h"


void LED_Blink_Update_ChannelMode(LED_Blink_t* h);
void LED_Blink_Update_CalibrationMode(LED_Blink_t* h);


void LED_Blink_Init(LED_Blink_t* h, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Ts_us)
{

	h->init.GPIOx = GPIOx;
	h->init.GPIO_Pin = GPIO_Pin;
	h->init.Ton_us = 50000;
	h->init.Toff_us = 400000;
	h->init.T_wait_us = 1000000;
	h->init.Ts_us = Ts_us;
	h->init.mode = LED_BLINK_MODE_CHANNEL;

	h->count = 0;
	h->state = LED_STATE_WAIT_ON;
	h->t_us = 0;
	h->times = 0;

}

void LED_Blink_ResetState(LED_Blink_t* h)
{
	h->count = 0;
	h->state = LED_STATE_WAIT_ON;
	h->t_us = 0;
	h->times = 0;
}

void LED_Blink_SetBlinksNum(LED_Blink_t* h, uint32_t blinksNum)
{
	h->times = blinksNum;
}


void LED_Blink_Update(LED_Blink_t* h)
{

	switch(h->init.mode)
	{
	case LED_BLINK_MODE_CALIBRATION:
		LED_Blink_Update_CalibrationMode(h);
		break;

	case LED_BLINK_MODE_CHANNEL:
		LED_Blink_Update_ChannelMode(h);
		break;

	case LED_BLINK_MODE_CONT_ON:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_SET);
		break;

	case LED_BLINK_MODE_CONT_OFF:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_RESET);
		break;

	default:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_RESET);
		break;
	}

}


void LED_Blink_Update_ChannelMode(LED_Blink_t* h)
{
	switch(h->state)
	{
	case LED_STATE_WAIT_ON:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_SET);
		if(h->t_us >= h->init.T_wait_us)
		{
			h->state = LED_STATE_WAIT_OFF;
			h->count = 0;
			h->t_us = 0;
		}
		break;

	case LED_STATE_WAIT_OFF:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_RESET);
		if(h->t_us >= h->init.Toff_us)
		{
			if(h->times > 0)
			{
				h->state = LED_STATE_ON;
			}
			else
			{
				h->state = LED_STATE_WAIT_ON;
			}
			h->count = 0;
			h->t_us = 0;
		}
		break;

	case LED_STATE_ON:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_SET);
		if(h->t_us >= h->init.Ton_us)
		{
			h->count += 1;
			h->state = LED_STATE_OFF;
			h->t_us = 0;
		}
		break;

	case LED_STATE_OFF:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_RESET);
		if(h->t_us >= h->init.Toff_us)
		{
			if(h->count < h->times)
			{
				h->state = LED_STATE_ON;
			}
			else
			{
				h->state = LED_STATE_WAIT_ON;
			}
			h->t_us = 0;
		}
		break;

	default:
		h->state = LED_STATE_WAIT_OFF;
		break;
	}
	h->t_us += h->init.Ts_us;
}


void LED_Blink_Update_CalibrationMode(LED_Blink_t* h)
{
	switch(h->state)
	{
	case LED_STATE_OFF:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_RESET);
		if(h->t_us >= h->init.Ton_us)
		{
			h->state = LED_STATE_ON;
			h->t_us = 0;
		}
		break;

	case LED_STATE_ON:
		HAL_GPIO_WritePin(h->init.GPIOx, h->init.GPIO_Pin, GPIO_PIN_SET);
		if(h->t_us >= h->init.Ton_us)
		{
			h->state = LED_STATE_OFF;
			h->t_us = 0;
		}
		break;
	default:
		h->state = LED_STATE_OFF;
		break;
	}
	h->t_us += h->init.Ts_us;
}


