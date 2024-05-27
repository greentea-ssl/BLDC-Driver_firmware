

#ifndef _ENCODER_H_
#define _ENCODER_H_


#include "main.h"
#include "parameters.h"

#define SPEED_MAF_MAX_NUM 32

typedef struct
{
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* SPI_NSS_Port;
	uint16_t SPI_NSS_Pin;
}Encoder_InitTypeDef;

typedef struct
{
	float Fs_enc;
	uint16_t MAF_period;
	uint8_t firstSample;
	uint32_t Gain_CountToSpeedQ21;
	uint16_t p_rawAngle;
	int16_t diff;
	int16_t omega_q5;
	int16_t omega_q5_filtered;
	int16_t MAF_buf[SPEED_MAF_MAX_NUM];
	uint32_t MAF_elem_en;
	int32_t MAF_sum;
	uint16_t MAF_cursor;
	uint16_t MAF_buf_size;
}SpeedCalc_TypeDef;

typedef struct
{
	Encoder_InitTypeDef Init;
	SpeedCalc_TypeDef speedCalc;
	uint16_t raw_Angle;
	uint8_t spi2txBuf[2];
	uint8_t spi2rxBuf[2];
}Encoder_TypeDef;


void Encoder_Init(Encoder_TypeDef *hEncoder);

int Encoder_Update(Encoder_TypeDef *hEncoder);



#endif /* _ENCODER_H_ */
