/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "parameters.h"
#include "ACR.h"
#include "ASR.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */







#define _APR_ENABLE_		0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/********** ACR DUMP DEBUG **********/

#if _ACR_DUMP_


float Id_dump[ACR_DUMP_STEPS] = {0.0f};
float Iq_dump[ACR_DUMP_STEPS] = {0.0f};
float Id_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Iq_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Vd_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Vq_ref_dump[ACR_DUMP_STEPS] = {0.0f};


int ACR_dump_count = 0;

#endif


/********** ASR DUMP DEBUG **********/

#if _ASR_DUMP_

#define ASR_DUMP_STEPS		2000


float omega_dump[ASR_DUMP_STEPS] = {0.0f};
float omega_ref_dump[ASR_DUMP_STEPS] = {0.0f};
float torque_ref_dump[ASR_DUMP_STEPS] = {0.0f};

int ASR_dump_count = 0;

#endif




/********** for Magnetic Rotary Encoder **********/




/********** Forced commutation **********/



#define _FC_DUMP_	0



/********** for ADC **********/




/********** for PWM Output **********/




/********** for ACR (Auto Current Regulator) **********/





/********** for ASR (Auto Speed Regulator) **********/




/********** APR **********/

float Kp_APR = 50.0f;
float Kd_APR = 0.02f;


const float APR_cycleTime = 1E-3;

volatile float theta_ref = 0.0f;


volatile float theta_error = 0.0f;

volatile float theta_error_diff = 0.0f;




/********** CAN **********/




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...);

int32_t printFloat(float val);


#if 0

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

#endif



int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	float phase = 0.0f;


	int count = 0;

	uint8_t p_ch, ch;

	/********** for ASR ***********/



  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */




  //UartPrintf(&huart2, "Hello world\n");

  printf("Hello\n");


  // Gate Enable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);


  // Current Sensing Auto Offset Calibration
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_RESET);


  /******** DEBUG ********/

  HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_RESET);


  p_ch = getChannel();


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);


  ch = getChannel();



  ADC_Init();

  //CAN_Init();


  HAL_Delay(100);


  TIM_Init();

  SPI_Init();


  ACR_Start();

  setZeroEncoder((p_ch != ch)? 1: 0);



  //while(1);

  ASR_Start();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(ASR_flg == 1)
	  {
		  HAL_GPIO_TogglePin(DB2_GPIO_Port, DB2_Pin);


		  omega_ref = 50.0 * sin(phase);

		  phase += 1E-3 * 2 * M_PI * 0.2;

		  if(phase > 2 * M_PI)
		  {
			  phase -= 2 * M_PI;
		  }





#if _FC_DUMP_
		  else
		  {
			  break;
		  }

#endif



#if _ACR_DUMP_

#if 0

		  if(Iq_ref <= 0.0f)
			  Iq_ref = 5.0f;
		  else
			  Iq_ref = -5.0f;

#endif

		  if(ACR_dump_count >= ACR_DUMP_STEPS)
			  break;


#endif



#if _ASR_DUMP_

		  if(ASR_dump_count % 1000 < 500)
		  {
			  omega_ref = 50.0f;
		  }
		  else
		  {
			  omega_ref = -50.0f;
		  }

		  if(ASR_dump_count >= ASR_DUMP_STEPS)
			  break;

#endif




		  /********** APR (Auto Position Regulator) **********/

#if _APR_ENABLE_

		  theta_error = theta_ref - theta;

		  if(theta_error < - M_PI)		theta_error += 2 * M_PI;
		  else if(theta_error > M_PI)	theta_error -= 2 * M_PI;

		  theta_error_diff = (theta_error - p_theta_error) / APR_cycleTime;

		  p_theta_error = theta_error;

		  omega_ref = Kp_APR * theta_error + Kd_APR * theta_error_diff;

#endif

		  /********** ASR (Auto Speed Regulator) **********/


		  speedControl();


#if _ASR_DUMP_

		  if(ASR_dump_count < ASR_DUMP_STEPS)
		  {
			  omega_dump[ASR_dump_count] = omega;
			  omega_ref_dump[ASR_dump_count] = omega_ref;
			  torque_ref_dump[ASR_dump_count] = torque_ref;
			  ASR_dump_count += 1;
		  }

#endif



		  /********** end of ASR **********/


		  ASR_flg = 0;

	  }




  }


  Id_ref = 0.0f;
  Iq_ref = 0.0f;

  HAL_Delay(10);

  // Gate Disable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

  HAL_Delay(10);

  stopPWM();

  HAL_Delay(10);

#if _FC_DUMP_

  printFloat(theta_re_offset);
  printf("\n");

  printf("forcedTheta[rad], measuredTheta[rad]\n");

  for(count = 0; count < FORCED_COMMUTE_STEPS; count++)
  {
	  printFloat(count * 2.0f * M_PI / FORCED_COMMUTE_STEPS);
	  printf(", ");
	  printFloat(sensedTheta_f[count]);
	  printf(", ");
	  printFloat(sensedTheta_b[count]);
	  printf("\n");
  }


#endif



#if _ACR_DUMP_

  printf("time[s], Id[A], Iq[A], Id*[A], Iq*[A], Vd*[V], Vq*[V]\n");

  for(count = 0; count < ACR_DUMP_STEPS; count++)
  {

	  printFloat(count * ACR_cycleTime);
	  printf(", ");

	  printFloat(Id_dump[count]);
	  printf(", ");
	  printFloat(Iq_dump[count]);
	  printf(", ");

	  printFloat(Id_ref_dump[count]);
	  printf(", ");
	  printFloat(Iq_ref_dump[count]);
	  printf(", ");

	  printFloat(Vd_ref_dump[count]);
	  printf(", ");
	  printFloat(Vq_ref_dump[count]);
	  printf("\n");

	  HAL_Delay(1);

  }

#endif

#if _ASR_DUMP_

  printf("time[s], ω[rad/s], ???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?*[rad/s], Torque*[N・m]\n");

  for(count = 0; count < ASR_DUMP_STEPS; count++)
  {
	  printFloat(count * ASR_cycleTime);
	  printf(", ");

	  printFloat(omega_dump[count]);
	  printf(", ");

	  printFloat(omega_ref_dump[count]);
	  printf(", ");

	  printFloat(torque_ref_dump[count]);
	  printf("\n");

  }

#endif




  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */







inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...){
	int32_t TransStrLength;
	char TransStr[1024];

	va_list args;
	va_start(args, format);
	TransStrLength = vsprintf(TransStr, format, args);
	va_end(args);

	HAL_UART_Transmit(huart, (uint8_t*)TransStr, TransStrLength, 100);

	return TransStrLength;
}


int32_t printFloat(float val)
{
	int i;
	int charCount = 0;
	int first, temp;
	const int decNum = 4;
	int i_val;

	if(val < 0.0f)
	{
		i_val = val * pow(10.0, decNum) - 0.5;
		putchar('-');
		i_val *= -1;
		charCount += 1;
	}
	else
	{
		i_val = val * pow(10.0, decNum) + 0.5;
	}

	first = 0;
	for(i = 9; i >= 0; i--)
	{
		temp = (int)(i_val * pow(10.0, -1.0 * i)) % 10;

		if(i == decNum - 1)
		{
			if(first == 0)
			{
				first = 1;
				putchar('0');
			}
			putchar('.');
		}

		if(first == 1 || temp != 0)
		{
			putchar('0' + temp);
			charCount += 1;
			first = 1;
		}

	}

	return charCount;

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
