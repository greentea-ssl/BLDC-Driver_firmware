

#include "encoder.h"


#include <string.h>

#include "motorControl.h"


extern SPI_HandleTypeDef hspi2;


extern Motor_TypeDef motor;


#include "parameters.h"

#include "math.h"
#include "flash.h"
#include "motorControl.h"

#include "sin_t.h"



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
	mainEncoder.forced_commute_enable = 0;
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


#if 1

uint16_t setZeroEncoder(uint8_t exe)
{

	uint16_t ret_theta_offset = 0;

	uint16_t *flash_data;

	flash_data = (uint16_t*)Flash_load();

	if(exe == 0)
	{

		memcpy(&ret_theta_offset, flash_data, 2);

		return ret_theta_offset & SIN_TBL_MASK;
	}

	motor.Igam_ref_pu_2q13 = (uint16_t)(5.0 / motor.Init.I_base * 8192);
	motor.Idel_ref_pu_2q13 = (uint16_t)(0.0 / motor.Init.I_base * 8192);

	motor.Init.theta_int_offset = 0;
	motor.theta_force_int = 0;
	motor.RunMode = MOTOR_MODE_CC_FORCE;

	HAL_Delay(1000);

	ret_theta_offset = motor.theta_re_int & SIN_TBL_MASK;

	memcpy(flash_data, &ret_theta_offset , 2);

	if (!Flash_store())
	{
#if DEBUG_PRINT_ENABLE
		printf("Failed to write flash\n");
#endif
	}


#if DEBUG_PRINT_ENABLE
	printf("flash_data:%d\n", *flash_data);
#endif

	Motor_Reset(&motor);
	motor.RunMode = MOTOR_MODE_CC_VECTOR;

	return ret_theta_offset;

}

#endif


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





