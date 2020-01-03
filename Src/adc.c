/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */



const float Vref_AD = 3.3f;

const int32_t AD_Range = 4096;

volatile uint16_t AD_Iu[1] = {0};
volatile uint16_t AD_Iv[1] = {0};
volatile uint16_t AD_Iw[1] = {0};


int32_t ADI_Offset = 2048;

volatile float V_Iu = 0.0f;
volatile float V_Iv = 0.0f;
volatile float V_Iw = 0.0f;


float V_Iu_offset = 1.67497551f;
float V_Iv_offset = 1.67578125f;
float V_Iw_offset = 1.67819822f;


const float Gain_currentSense = -10.0f; // 1 / ( R * OPAmpGain) [A / V]

const float Gain_Vdc_sense = 12.538f;

volatile float Iu = 0.0;
volatile float Iv = 0.0;
volatile float Iw = 0.0;


volatile float Vdc = 20.0f;


// Moving Average Filter
#define _MAF_ENABLE_	0

// Median Filter
#define _MEDF_ENABLE_	1

#define N_MAF_I			2


#define N_MEDF_I		3



/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PA0-WKUP     ------> ADC1_IN0 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC3 GPIO Configuration    
    PA1     ------> ADC3_IN1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA2_Stream1;
    hdma_adc3.Init.Channel = DMA_CHANNEL_2;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_NORMAL;
    hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc3);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PA0-WKUP     ------> ADC1_IN0 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN4 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PA1     ------> ADC3_IN1 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


void ADC_Init(void)
{

	// ADC Starting


	HAL_ADC_Start_DMA(&hadc1, AD_Iu, 1);
	HAL_ADC_Start_DMA(&hadc2, AD_Iv, 1);
	HAL_ADC_Start_DMA(&hadc3, AD_Iw, 1);



/*
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
*/

}



void get_current_dq(float *Id, float *Iq, int SVM_sector, float cos_theta_re, float sin_theta_re)
{

	// Moving Average Filter
#if _MAF_ENABLE_

	static int32_t pos_MAF_I = 0;

	static int32_t AD_Iu_buf[N_MAF_I] = {0};
	static int32_t AD_Iv_buf[N_MAF_I] = {0};
	static int32_t AD_Iw_buf[N_MAF_I] = {0};

	static int32_t AD_Iu_MAF = 0;
	static int32_t AD_Iv_MAF = 0;
	static int32_t AD_Iw_MAF = 0;

#endif

#if _MEDF_ENABLE_

	static int32_t pos_MEDF_I = 0;

	static int32_t AD_Iu_buf[N_MEDF_I] = {0};
	static int32_t AD_Iv_buf[N_MEDF_I] = {0};
	static int32_t AD_Iw_buf[N_MEDF_I] = {0};

	static int32_t AD_Iu_MEDF = 0;
	static int32_t AD_Iv_MEDF = 0;
	static int32_t AD_Iw_MEDF = 0;

#endif




	// Read ADC
/*
	AD_Iu[0] = HAL_ADC_GetValue(&hadc1);
	AD_Iv[0] = HAL_ADC_GetValue(&hadc2);
	AD_Iw[0] = HAL_ADC_GetValue(&hadc3);
*/


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Iu, 1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)AD_Iv, 1);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)AD_Iw, 1);


#if _MAF_ENABLE_

	AD_Iu_buf[pos_MAF_I] = (int32_t)AD_Iu[0];
	AD_Iv_buf[pos_MAF_I] = (int32_t)AD_Iv[0];
	AD_Iw_buf[pos_MAF_I] = (int32_t)AD_Iw[0];


	AD_Iu_MAF += AD_Iu_buf[pos_MAF_I];
	AD_Iv_MAF += AD_Iv_buf[pos_MAF_I];
	AD_Iw_MAF += AD_Iw_buf[pos_MAF_I];

	// Writing position Update
	pos_MAF_I += 1;
	if(pos_MAF_I >= N_MAF_I)
	{
		pos_MAF_I = 0;
	}

	V_Iu = (float)AD_Iu_MAF / (N_MAF_I * AD_Range) * Vref_AD - V_Iu_offset;
	V_Iv = (float)AD_Iv_MAF / (N_MAF_I * AD_Range) * Vref_AD - V_Iv_offset;
	V_Iw = (float)AD_Iw_MAF / (N_MAF_I * AD_Range) * Vref_AD - V_Iw_offset;

	AD_Iu_MAF -= AD_Iu_buf[pos_MAF_I];
	AD_Iv_MAF -= AD_Iv_buf[pos_MAF_I];
	AD_Iw_MAF -= AD_Iw_buf[pos_MAF_I];

#endif

#if _MEDF_ENABLE_

	AD_Iu_buf[pos_MEDF_I] = (int32_t)AD_Iu[0];
	AD_Iv_buf[pos_MEDF_I] = (int32_t)AD_Iv[0];
	AD_Iw_buf[pos_MEDF_I] = (int32_t)AD_Iw[0];


	pos_MEDF_I += 1;
	if(pos_MEDF_I >= N_MEDF_I)
	{
		pos_MEDF_I = 0;
	}

	AD_Iu_MEDF = median3(AD_Iu_buf);
	AD_Iv_MEDF = median3(AD_Iv_buf);
	AD_Iw_MEDF = median3(AD_Iw_buf);

	V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - V_Iu_offset;
	V_Iv = (float)AD_Iv_MEDF / AD_Range * Vref_AD - V_Iv_offset;
	V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - V_Iw_offset;


#endif



#if !_MAF_ENABLE_ && !_MEDF_ENABLE_

	V_Iu = (float)AD_Iu[0] / AD_Range * Vref_AD - V_Iu_offset;
	V_Iv = (float)AD_Iv[0] / AD_Range * Vref_AD - V_Iv_offset;
	V_Iw = (float)AD_Iw[0] / AD_Range * Vref_AD - V_Iw_offset;

#endif

	//Vdc = Vdc * 0.9 + 0.1 * (float)AD_Iu[1] / AD_Range * Vref_AD * Gain_Vdc_sense;

	switch(SVM_sector)
	{
	case 0: case 5:
		Iv = V_Iv * Gain_currentSense;
		Iw = V_Iw * Gain_currentSense;
		Iu = - Iv - Iw;
		break;

	case 1: case 2:
		Iw = V_Iw * Gain_currentSense;
		Iu = V_Iu * Gain_currentSense;
		Iv = - Iw - Iu;
		break;

	case 3: case 4:
		Iu = V_Iu * Gain_currentSense;
		Iv = V_Iv * Gain_currentSense;
		Iw = - Iu - Iv;
		break;
	}


	*Id = 0.8165f * (Iu * cos_theta_re + Iv * (-0.5f * cos_theta_re + 0.855f * sin_theta_re) + Iw * (-0.5f * cos_theta_re - 0.855f * sin_theta_re));
	*Iq = 0.8165f * (-Iu * sin_theta_re + Iv * (0.5f * sin_theta_re + 0.855f * cos_theta_re) + Iw * (0.5f * sin_theta_re - 0.855f * cos_theta_re));



	return;

}



extern int32_t median3(int32_t *buf)
{

	if(buf[0] < buf[1])
	{
		if(buf[2] < buf[0])			return buf[0];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[1];
	}
	else
	{
		if(buf[2] < buf[1])			return buf[1];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[0];
	}

	return 0;
}




/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
