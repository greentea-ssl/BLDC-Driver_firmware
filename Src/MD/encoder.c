

#include "encoder.h"


#include <string.h>

#include "motorControl.h"
#include "parameters.h"
#include "math.h"
#include "flash.h"
#include "md_main.h"
#include "sin_t.h"


extern SPI_HandleTypeDef hspi2;

extern MD_Handler_t md_sys;


Encoder_TypeDef mainEncoder;



void Encoder_Init()
{

	int count;

	mainEncoder.Init.hspi = &hspi2;
	mainEncoder.Init.SPI_NSS_Port = SPI2_NSS_GPIO_Port;
	mainEncoder.Init.SPI_NSS_Pin = SPI2_NSS_Pin;

	mainEncoder.Init.theta_offset = 0.0f;
	mainEncoder.Init.theta_re_offset = -3.0723f;
	mainEncoder.Init.cycleTime = 100E-6;

	mainEncoder.theta = 0.0f;
	mainEncoder.theta_re = 0.0f;
	mainEncoder.cos_theta_re = 1.0f;
	mainEncoder.sin_theta_re = 0.0f;

	for(count = 0; count < SPEED_CALC_BUF_SIZE; count++)
	{
		mainEncoder.prev_theta_buf[count] = 0;
	}

	mainEncoder.prev_theta_buf_count = 0;

	mainEncoder.firstLaunch = 1;

	// SPI Interrupt Setting
	__HAL_SPI_ENABLE_IT(mainEncoder.Init.hspi, SPI_IT_TXE | SPI_IT_RXNE);


}


inline void Encoder_Request(Encoder_TypeDef *hEncoder)
{

	// Reading Encoder for next sampling
	hEncoder->spi2txBuf[0] = 0xff;
	hEncoder->spi2txBuf[1] = 0xff;

	HAL_GPIO_WritePin(hEncoder->Init.SPI_NSS_Port, hEncoder->Init.SPI_NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive_IT(hEncoder->Init.hspi, hEncoder->spi2txBuf, hEncoder->spi2rxBuf, 1);

}


inline int Encoder_Refresh(Encoder_TypeDef *hEncoder)
{

	static uint16_t rawData;
	static uint16_t parity;

	// Reading RX Data from SPI Encoder
	HAL_GPIO_WritePin(hEncoder->Init.SPI_NSS_Port, hEncoder->Init.SPI_NSS_Pin, GPIO_PIN_SET);

	rawData = (hEncoder->spi2rxBuf[1] << 8) | hEncoder->spi2rxBuf[0];

#if 0
	parity = 0;
	for(i = 0; i < 16; i++)
	{
		parity += (rawData >> i) & 0x01;
	}
	if((parity & 0x01) != 0) return -1;
#else
	parity = rawData;
	parity ^= parity >> 8;
	parity ^= parity >> 4;
	parity ^= parity >> 2;
	parity ^= parity >> 1;
	if((parity & 0x01) != 0) return -1;
#endif

	hEncoder->raw_Angle = rawData & 0x3FFF;

	return 0;

}





