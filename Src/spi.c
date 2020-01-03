/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

#include "parameters.h"
#include "sin_t.h"
#include "ACR.h"
#include "math.h"
#include "tim.h"
#include "flash.h"



// Sensing Data
volatile uint8_t spi2txBuf[2] = {0};
volatile uint8_t spi2rxBuf[2] = {0};

float theta_offset = 0.0f;

float theta_re_offset = 0.0f; //-3.0723f;


// Rotor rotation angle
volatile float theta = 0.0f;

// Rotor electrical position
volatile float theta_re = 0.0f;


volatile float cos_theta_re = 1.0;
volatile float sin_theta_re = 0.0;


uint32_t *flash_data;


uint16_t angle_raw = 0;



volatile uint8_t forced_commute_enable = 0;




/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration    
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration    
    PB2     ------> SPI3_MOSI
    PA15     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10     ------> SPI2_SCK 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* SPI2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();
  
    /**SPI3 GPIO Configuration    
    PB2     ------> SPI3_MOSI
    PA15     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void SPI_Init()
{


	  // SPI Interrupt Setting
	  __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_TXE | SPI_IT_RXNE);

}



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

		memcpy(&theta_re_offset, flash_data, 4);

		printf("flash_data:%d\n", theta_re_offset * 100000);
		printf(" theta_re_offset = %d\n", (int)(theta_re_offset * 100000));
		return;
	}


	Id_ref = forced_I_gamma_ref;
	Iq_ref = forced_I_delta_ref;

	forced_commute_enable = 1;


	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);
	timeoutReset();	HAL_Delay(100);

	requestEncoder();
	HAL_Delay(5);
	refreshEncoder();
	HAL_Delay(5);
	requestEncoder();
	HAL_Delay(5);
	refreshEncoder();

	theta_re_offset = 0.0f - theta_re;

	while(theta_re_offset < -M_PI)	theta_re_offset += 2.0f * M_PI;
	while(theta_re_offset > M_PI)	theta_re_offset -= 2.0f * M_PI;


	printf(" theta_re_offset = %d -- ", (int)(theta_re_offset * 100000));
	HAL_Delay(1);
	printf(" theta_re_offset = %d\n", (int)(theta_re_offset * 100000));
	HAL_Delay(1);
	printf(" theta_re_offset(4) = %d -- ", (int)(theta_re_offset * 10000));
	HAL_Delay(1);
	printf(" theta_re_offset(4) = %d\n", (int)(theta_re_offset * 10000));
	HAL_Delay(1);

	printf("(theta_re_offset < 1.0f) = %d\n", (int)(theta_re_offset < 1.0f));

	printf("(theta_re_offset > -1.0f) = %d\n", (int)(theta_re_offset > -1.0f));


	memcpy(flash_data, &theta_re_offset, 4);

	if (!Flash_store())
	{
		printf("Failed to write flash\n");
	}

	printf("flash_data:%lu\n", *flash_data);



	ACR_Reset();

	forced_commute_enable = 0;


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

	theta_re_offset = fmod(theta_offset * POLES / 2, 2.0f * M_PI);
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




inline void requestEncoder()
{


	// Reading Encoder for next sampling
	spi2txBuf[0] = 0xff;
	spi2txBuf[1] = 0xff;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi2, spi2txBuf, spi2rxBuf, 1);


}


inline int refreshEncoder()
{

	uint16_t angle_raw = 0;
	float _theta;
	float _theta_re;

	// Reading RX Data from SPI Encoder
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	angle_raw = (spi2rxBuf[1] & 0x3f) << 8 | spi2rxBuf[0];

	_theta = (float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI + theta_offset;

	if(_theta < 0.0f)			theta = _theta + 2 * M_PI;
	else if(_theta >= 2 * M_PI)	theta = _theta - 2 * M_PI;
	else						theta = _theta;

	_theta_re = fmodf((float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI * POLES / 2, 2.0f * M_PI) + theta_re_offset;

	if(_theta_re < 0.0f)			theta_re = _theta_re + 2 * M_PI;
	else if(_theta_re >= 2 * M_PI)	theta_re = _theta_re - 2 * M_PI;
	else							theta_re = _theta_re;

	cos_theta_re = sin_table2[(int)((theta_re * 0.3183f + 0.5f) * 5000.0f)];
	sin_theta_re = sin_table2[(int)(theta_re * 1591.54943f)];


	return 0;

}



#if 0

// SPI RX CallBack function
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi)
{


}

#endif




/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
