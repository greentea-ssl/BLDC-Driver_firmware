

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

	uint32_t *flash_data;

	flash_data = (uint32_t*)Flash_load();

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

	memcpy(flash_data, &motor.Init.theta_int_offset , 2);

	if (!Flash_store())
	{
#if DEBUG_PRINT_ENABLE
		printf("Failed to write flash\n");
#endif
	}


#if DEBUG_PRINT_ENABLE
	printf("flash_data:%lu\n", *flash_data);
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
	static uint16_t parity, i;
	static float _theta;
	static float _theta_re;
	static float d_theta;
	static float speed_calc_d_theta;

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

#if 0

	_theta = (float)hEncoder->raw_Angle / (float)ENCODER_RESOL * 2.0f * M_PI + hEncoder->Init.theta_offset;

	if(_theta < 0.0f)			hEncoder->theta = _theta + 2 * M_PI;
	else if(_theta >= 2 * M_PI)	hEncoder->theta = _theta - 2 * M_PI;
	else						hEncoder->theta = _theta;


	// 差分角度計算，初めのステップは速度ゼロとする
	if(hEncoder->firstLaunch != 0)
	{
		d_theta = 0.0f;
		hEncoder->firstLaunch = 0;
	}
	else
	{
		d_theta = hEncoder->theta - hEncoder->p_theta;
	}
	hEncoder->p_theta = hEncoder->theta;

	// Unwrapping Process
	if(d_theta < - M_PI)
	{
		d_theta += 2 * M_PI;
		hEncoder->turnCount += 1;
	}
	else if(d_theta > M_PI)
	{
		d_theta -= 2 * M_PI;
		hEncoder->turnCount += -1;
	}

	speed_calc_d_theta = hEncoder->theta - hEncoder->prev_theta_buf[hEncoder->prev_theta_buf_count];

	if(speed_calc_d_theta < - M_PI)
	{
		speed_calc_d_theta += 2 * M_PI;
	}
	else if(speed_calc_d_theta > M_PI)
	{
		speed_calc_d_theta -= 2 * M_PI;
	}

	hEncoder->omega = speed_calc_d_theta / (hEncoder->Init.cycleTime * SPEED_CALC_BUF_SIZE);

	hEncoder->prev_theta_buf[hEncoder->prev_theta_buf_count] = hEncoder->theta;
	hEncoder->prev_theta_buf_count ++;
	if(hEncoder->prev_theta_buf_count > SPEED_CALC_BUF_SIZE - 1)
	{
		hEncoder->prev_theta_buf_count = 0;
	}

#if 0
	// 速度計算，LPF付き
	hEncoder->omega = hEncoder->omega * SPEED_LPF_COEFF + d_theta / hEncoder->Init.cycleTime * (1.0f - SPEED_LPF_COEFF);
#endif

	// マルチターン角度更新
	hEncoder->theta_multiturn = hEncoder->theta + 2.0f * M_PI * hEncoder->turnCount;

	// 電気角取得
	_theta_re = fmodf((float)hEncoder->raw_Angle / (float)ENCODER_RESOL * 2.0f * M_PI * POLE_PAIRS, 2.0f * M_PI) + hEncoder->Init.theta_re_offset;

	if(_theta_re < 0.0f)			hEncoder->theta_re = _theta_re + 2 * M_PI;
	else if(_theta_re >= 2 * M_PI)	hEncoder->theta_re = _theta_re - 2 * M_PI;
	else							hEncoder->theta_re = _theta_re;

	hEncoder->cos_theta_re = sin_table2[(int)((hEncoder->theta_re * 0.3183f + 0.5f) * 5000.0f)];
	hEncoder->sin_theta_re = sin_table2[(int)(hEncoder->theta_re * 1591.54943f)];

#endif

	return 0;

}





