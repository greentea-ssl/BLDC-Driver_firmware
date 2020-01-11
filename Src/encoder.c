

#include "encoder.h"

#include "spi.h"


#include "parameters.h"
#include "sin_t.h"
#include "ACR.h"
#include "math.h"
#include "tim.h"
#include "flash.h"


uint32_t *flash_data;


Encoder_TypeDef mainEncoder;



void Encoder_Init()
{

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


	mainEncoder.firstLaunch = 1;

	// SPI Interrupt Setting
	__HAL_SPI_ENABLE_IT(mainEncoder.Init.hspi, SPI_IT_TXE | SPI_IT_RXNE);


}


#if 1

void setZeroEncoder(uint8_t exe)
{

	const int32_t forced_commute_steps = 2000;



	volatile uint32_t forced_commute_count = 0;

	const float forced_I_gamma_ref = 8.0f;
	const float forced_I_delta_ref = 0.0f;

	volatile float sensed_theta_re_error;

	volatile float sensed_theta_error;
	volatile float sensed_theta_error_sum = 0.0f;
	volatile float sensed_theta_error_ave = 0.0f;


	flash_data = (uint32_t*)Flash_load();

	if(exe == 0)
	{

		memcpy(&mainEncoder.Init.theta_re_offset, flash_data, 4);

		printf("flash_data:%d\n", mainEncoder.Init.theta_re_offset * 100000);
		printf(" theta_re_offset = %d\n", (int)(mainEncoder.Init.theta_re_offset * 100000));
		return;
	}


	mainACR.Id_ref = forced_I_gamma_ref;
	mainACR.Iq_ref = forced_I_delta_ref;

	mainEncoder.Init.theta_re_offset = 0.0f;

	mainACR.forced_theta_re = 0.0f;

	mainACR.forced_commute_enable = 1;

	HAL_Delay(1000);


	mainEncoder.Init.theta_re_offset = 0.0f - mainEncoder.theta_re;

	mainACR.Id_ref = 0.0f;
	mainACR.Iq_ref = 0.0f;


	while(mainEncoder.Init.theta_re_offset < -M_PI)	mainEncoder.Init.theta_re_offset += 2.0f * M_PI;
	while(mainEncoder.Init.theta_re_offset > M_PI)	mainEncoder.Init.theta_re_offset -= 2.0f * M_PI;


	printf(" theta_re_offset = %d -- ", (int)(mainEncoder.Init.theta_re_offset * 100000));
	HAL_Delay(1);
	printf(" theta_re_offset = %d\n", (int)(mainEncoder.Init.theta_re_offset * 100000));
	HAL_Delay(1);
	printf(" theta_re_offset(4) = %d -- ", (int)(mainEncoder.Init.theta_re_offset * 10000));
	HAL_Delay(1);
	printf(" theta_re_offset(4) = %d\n", (int)(mainEncoder.Init.theta_re_offset * 10000));
	HAL_Delay(1);

	printf("(theta_re_offset < 1.0f) = %d\n", (int)(mainEncoder.Init.theta_re_offset < 1.0f));

	printf("(theta_re_offset > -1.0f) = %d\n", (int)(mainEncoder.Init.theta_re_offset > -1.0f));


	memcpy(flash_data, &mainEncoder.Init.theta_re_offset, 4);

	if (!Flash_store())
	{
		printf("Failed to write flash\n");
	}

	printf("flash_data:%lu\n", *flash_data);


	mainACR.forced_commute_enable = 0;

	ACR_Reset(&mainACR);



#if 0

	requestEncoder();

	for(forced_commute_count = 0; forced_commute_count < forced_commute_steps; forced_commute_count++)
	{
		HAL_Delay(1);
		timeoutReset();
		refreshEncoder();
		sensed_theta_error = forced_theta - theta;
		while(sensed_theta_error < -M_PI)	sensed_theta_error += 2.0f * M_PI;
		while(sensed_theta_error > M_PI)	sensed_theta_error -= 2.0f * M_PI;
		sensed_theta_error_sum += sensed_theta_error;
		forced_theta = forced_commute_count * 2.0f * M_PI / forced_commute_steps;
		forced_commute_count += 1;

		requestEncoder();
	}

	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);

	for(forced_commute_count = 0; forced_commute_count < forced_commute_steps; forced_commute_count++)
	{
		HAL_Delay(1);
		timeoutReset();
		refreshEncoder();
		sensed_theta_error = forced_theta - theta;
		while(sensed_theta_error < -M_PI)	sensed_theta_error += 2.0f * M_PI;
		while(sensed_theta_error > M_PI)	sensed_theta_error -= 2.0f * M_PI;
		sensed_theta_error_sum += sensed_theta_error;
		forced_theta = (forced_commute_steps - forced_commute_count - 1) * 2.0f * M_PI / forced_commute_steps;
		forced_commute_count += 1;

		requestEncoder();
	}


	theta_offset = sensed_theta_error_sum * 0.5f / forced_commute_steps;

	theta_re_offset = fmod(theta_offset * POLE_PAIRS, 2.0f * M_PI);
	while(theta_re_offset < -M_PI)	theta_re_offset += 2.0f * M_PI;
	while(theta_re_offset > M_PI)	theta_re_offset -= 2.0f * M_PI;

	printf("theta_offset = %d\n", (int)(theta_offset * 100000));

	sensed_theta_error_sum = 0.0f;

	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);

	requestEncoder();

	for(forced_commute_count = 0; forced_commute_count < forced_commute_steps; forced_commute_count++)
	{
		HAL_Delay(1);
		timeoutReset();
		refreshEncoder();
		sensed_theta_error = forced_theta - theta;
		while(sensed_theta_error < -M_PI)	sensed_theta_error += 2.0f * M_PI;
		while(sensed_theta_error > M_PI)	sensed_theta_error -= 2.0f * M_PI;
		sensed_theta_error_sum += sensed_theta_error;
		forced_theta = forced_commute_count * 2.0f * M_PI / forced_commute_steps;
		forced_commute_count += 1;

		requestEncoder();
	}

	sensed_theta_error_ave = sensed_theta_error_sum * 1.0f / forced_commute_steps;

	printf("error_ave = %d\n", (int)(sensed_theta_error_ave * 100000));


	ACR_Reset();

	forced_commute_enable = 0;


#endif


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

	static uint16_t angle_raw = 0;
	static float _theta;
	static float _theta_re;
	static float d_theta;

	// Reading RX Data from SPI Encoder
	HAL_GPIO_WritePin(hEncoder->Init.SPI_NSS_Port, hEncoder->Init.SPI_NSS_Pin, GPIO_PIN_SET);

	angle_raw = (hEncoder->spi2rxBuf[1] & 0x3f) << 8 | hEncoder->spi2rxBuf[0];

	_theta = (float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI + hEncoder->Init.theta_offset;

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

	// 速度計算，LPF付き
	hEncoder->omega = hEncoder->omega * SPEED_LPF_COEFF + d_theta / hEncoder->Init.cycleTime * (1.0f - SPEED_LPF_COEFF);

	// マルチターン角度更新
	hEncoder->theta_multiturn = hEncoder->theta + 2.0f * M_PI * hEncoder->turnCount;

	// 電気角取得
	_theta_re = fmodf((float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI * POLE_PAIRS, 2.0f * M_PI) + hEncoder->Init.theta_re_offset;

	if(_theta_re < 0.0f)			hEncoder->theta_re = _theta_re + 2 * M_PI;
	else if(_theta_re >= 2 * M_PI)	hEncoder->theta_re = _theta_re - 2 * M_PI;
	else							hEncoder->theta_re = _theta_re;

	hEncoder->cos_theta_re = sin_table2[(int)((hEncoder->theta_re * 0.3183f + 0.5f) * 5000.0f)];
	hEncoder->sin_theta_re = sin_table2[(int)(hEncoder->theta_re * 1591.54943f)];


	return 0;

}





