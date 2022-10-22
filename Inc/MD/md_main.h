
#ifndef _MD_MAIN_H_
#define _MD_MAIN_H_

#include <stdio.h>

#include "motorControl.h"

typedef enum {
	Seq_Init = 0,
	Seq_PosAdj = 1,
	Seq_Running = 2,
}Sequense_TypeDef;


typedef struct
{
	uint32_t LED_blink_count;
	uint32_t LED_blink_state;
	uint32_t LED_blink_t_us;
	uint32_t LED_blink_times;
	uint32_t LED_blink_Ton_us;
	uint32_t LED_blink_Toff_us;
	uint32_t LED_blink_T_wait_us;
	uint32_t LED_blink_Ts_us;
}LED_Blink_t;



typedef struct
{

	Sequense_TypeDef sequence;

	uint32_t carrier_counter;

	Motor_TypeDef motor;

	uint8_t timeoutEnable;
	uint32_t timeoutCount;

	// 1: timeout
	uint8_t timeoutState;

	LED_Blink_t led_blink;


}MD_Handler_t;


void MD_Init(MD_Handler_t* h);

void MD_Update_SyncPWM(MD_Handler_t* h);

int MD_Update_Async(MD_Handler_t* h);

void MD_End(MD_Handler_t* h);



#endif /* _MD_MAIN_H_ */





