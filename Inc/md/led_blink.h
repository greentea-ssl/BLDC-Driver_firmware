
#ifndef _LED_BLINK_H_
#define _LED_BLINK_H_


#include "main.h"


typedef enum
{
	LED_BLINK_MODE_CHANNEL,
	LED_BLINK_MODE_CALIBRATION,
	LED_BLINK_MODE_CONT_ON,
	LED_BLINK_MODE_CONT_OFF,
}LED_Blink_Mode_t;


typedef struct
{

	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint32_t Ton_us;
	uint32_t Toff_us;
	uint32_t T_wait_us;
	uint32_t Ts_us;
	LED_Blink_Mode_t mode;

}LED_Blink_Init_t;


typedef enum
{
	LED_STATE_WAIT_ON,
	LED_STATE_WAIT_OFF,
	LED_STATE_ON,
	LED_STATE_OFF,
}LED_Blink_State_t;


typedef struct
{
	LED_Blink_Init_t init;

	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;

	uint32_t count;
	LED_Blink_State_t state;
	uint32_t t_us;
	uint32_t times;

}LED_Blink_t;



void LED_Blink_Init(LED_Blink_t* h, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Ts_us);

void LED_Blink_ResetState(LED_Blink_t* h);

void LED_Blink_SetBlinksNum(LED_Blink_t* h, uint32_t blinksNum);

void LED_Blink_Update(LED_Blink_t* h);



#endif /* _LED_BLINK_H_ */


