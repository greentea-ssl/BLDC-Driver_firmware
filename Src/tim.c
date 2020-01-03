/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

#include "ACR.h"
#include "ASR.h"



volatile float amp_u = 0.0;
volatile float amp_v = 0.0;
volatile float amp_w = 0.0;


volatile uint32_t timeoutCount = 0;

// 1: timeout
volatile uint8_t timeoutState = 0;


/* USER CODE END 0 */

TIM_HandleTypeDef htim8;

/* TIM8 init function */
void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 8000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim8);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspPostInit 0 */

  /* USER CODE END TIM8_MspPostInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration    
    PA7     ------> TIM8_CH1N
    PB0     ------> TIM8_CH2N
    PB1     ------> TIM8_CH3N
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM8_MspPostInit 1 */

  /* USER CODE END TIM8_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /* TIM8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



void TIM_Init()
{


	  // Setting Timer Interrupts
	  /*
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC1);
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC2);
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC3);
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC4);
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_COM);
	  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);*/
	  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
	  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);


	  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);
	  //HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_TRIGGER);


	  startPWM();


}



inline void startPWM()
{


	// 3phase PWM Starting
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);

}



inline void stopPWM()
{

	// 3phase PWM Stopping
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_3);

}



void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM8)
	{

		if(!__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{

			currentControl();

			// timeout control
			if(timeoutCount < TIMEOUT_MS * PWM_FREQ / 1000)
			{
				timeoutCount += 1;
			}
			else
			{
				stopPWM();
				timeoutCount = 0;
				timeoutState = 1;
			}

		}


	}

}


inline void timeoutReset()
{
	timeoutCount = 0;
	if(timeoutState == 1)
	{
		timeoutState = 0;
		ASR_Reset();
		ACR_Reset();
		startPWM();
	}
}



inline void setPWM(const float *duty){

	if(duty[0] <= 1.0 && duty[0] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200 * (1.0 - (amp_u = duty[0])));
	if(duty[1] <= 1.0 && duty[1] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200 * (1.0 - (amp_v = duty[1])));
	if(duty[2] <= 1.0 && duty[2] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200 * (1.0 - (amp_w = duty[2])));

	return 0;
}




/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
