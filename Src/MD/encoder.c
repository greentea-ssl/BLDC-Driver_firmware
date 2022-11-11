

#include "encoder.h"

#include <string.h>

#include "parameters.h"
#include "math.h"
#include "sin_t.h"


void Encoder_Request(Encoder_TypeDef *hEncoder);
int Encoder_Refresh(Encoder_TypeDef *hEncoder);

void SpeedCalc_Init(SpeedCalc_TypeDef* h, float Fs_enc, int MAF_period);
void SpeedCalc_Update(SpeedCalc_TypeDef* h, uint16_t angle_14bit);


void Encoder_Init(Encoder_TypeDef *hEncoder)
{
	SpeedCalc_Init(&hEncoder->speedCalc, 11.25E+3, 20);

	// SPI Interrupt Setting
	__HAL_SPI_ENABLE_IT(hEncoder->Init.hspi, SPI_IT_TXE | SPI_IT_RXNE);
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

	parity = rawData;
	parity ^= parity >> 8;
	parity ^= parity >> 4;
	parity ^= parity >> 2;
	parity ^= parity >> 1;
	if((parity & 0x01) != 0) return -1;

	hEncoder->raw_Angle = rawData & 0x3FFF;

	return 0;
}

int Encoder_Update(Encoder_TypeDef *h)
{
	int ret;

	ret = Encoder_Refresh(h);
	Encoder_Request(h);
	if(ret != 0) return -1;

	SpeedCalc_Update(&h->speedCalc, h->raw_Angle);

	return 0;
}



void SpeedCalc_Init(SpeedCalc_TypeDef* h, float Fs_enc, int MAF_period)
{
	h->Fs_enc = Fs_enc;
	h->MAF_period = MAF_period;

	h->firstSample = 1;
	h->p_rawAngle = 0;
	h->omega_q5 = 0;
	memset(h->MAF_buf, 0x00, sizeof(h->MAF_buf));
	h->MAF_sum = 0;
	h->MAF_buf_size = h->MAF_period;
	h->MAF_cursor = 0;
	h->MAF_elem_en = 0xffffffff;

	h->Gain_CountToSpeedQ21 = 2*M_PI / 16384 * h->Fs_enc * powf(2,21) + 0.5f;
}

inline void SpeedCalc_Update(SpeedCalc_TypeDef* h, uint16_t angle_14bit)
{
	if(h->firstSample != 0)
	{
		h->p_rawAngle = angle_14bit;
		h->firstSample = 0;
		return;
	}
	h->diff = (angle_14bit << 2) - (h->p_rawAngle << 2);
	h->diff = h->diff >> 2;
	h->p_rawAngle = angle_14bit;

	if((h->MAF_elem_en & (1 << h->MAF_cursor)) != 0)
	{
		h->MAF_sum -= h->MAF_buf[h->MAF_cursor];
		h->MAF_buf_size--;
	}
	if(h->diff != 0)
	{
		h->omega_q5 = (h->Gain_CountToSpeedQ21 * h->diff) >> 16;
		h->MAF_buf[h->MAF_cursor] = h->omega_q5;
		h->MAF_sum += h->MAF_buf[h->MAF_cursor];
		h->MAF_buf_size++;
		h->MAF_elem_en |= 1 << h->MAF_cursor;
	}
	else
	{
		h->MAF_buf[h->MAF_cursor] = 0;
		h->MAF_elem_en &= ~(1 << h->MAF_cursor);
	}
	h->MAF_cursor++;
	if(h->MAF_cursor >= h->MAF_period)
	{
		h->MAF_cursor = 0;
	}
	if(h->MAF_buf_size > 0)
	{
		h->omega_q5_filtered = h->MAF_sum / h->MAF_buf_size;
	}

}



