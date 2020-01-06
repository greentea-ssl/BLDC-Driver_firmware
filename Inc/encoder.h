

#ifndef _ENCODER_H_
#define _ENCODER_H_


#include "spi.h"



typedef struct
{
	float theta_offset;

	float theta_re_offset;


	SPI_HandleTypeDef* hspi;

	GPIO_TypeDef* SPI_NSS_Port;

	uint16_t SPI_NSS_Pin;


}Encoder_InitTypeDef;



typedef struct
{

	Encoder_InitTypeDef Init;


	// Rotor mechanical position
	float theta;

	// Rotor electrical position
	float theta_re;


	float cos_theta_re;
	float sin_theta_re;

	uint8_t forced_commute_enable;

	// Sensing Data
	volatile uint8_t spi2txBuf[2];
	volatile uint8_t spi2rxBuf[2];

}Encoder_TypeDef;



extern Encoder_TypeDef mainEncoder;



void Encoder_Init();


void setZeroEncoder(uint8_t exe);


void Encoder_Request(Encoder_TypeDef *hEncoder);

int Encoder_Refresh(Encoder_TypeDef *hEncoder);





#endif /* _ENCODER_H_ */




