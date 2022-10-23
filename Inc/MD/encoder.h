

#ifndef _ENCODER_H_
#define _ENCODER_H_


#include "main.h"

#include "parameters.h"


typedef struct
{
	float theta_offset;

	float theta_re_offset;

	float cycleTime;

	SPI_HandleTypeDef* hspi;

	GPIO_TypeDef* SPI_NSS_Port;

	uint16_t SPI_NSS_Pin;


}Encoder_InitTypeDef;



typedef struct
{

	Encoder_InitTypeDef Init;


	// Rotor mechanical position
	float theta;

	int32_t turnCount;

	float theta_multiturn;

	uint16_t raw_Angle;

	float p_theta;

	// Rotor Speed
	float omega;

	float prev_theta_buf[SPEED_CALC_BUF_SIZE];

	int prev_theta_buf_count;


	float firstLaunch;

	float cycleTime;

	// Rotor electrical position
	float theta_re;


	float cos_theta_re;
	float sin_theta_re;

	// Sensing Data
	uint8_t spi2txBuf[2];
	uint8_t spi2rxBuf[2];

}Encoder_TypeDef;



extern Encoder_TypeDef mainEncoder;



void Encoder_Init();

void Encoder_Request(Encoder_TypeDef *hEncoder);

int Encoder_Refresh(Encoder_TypeDef *hEncoder);



#endif /* _ENCODER_H_ */




